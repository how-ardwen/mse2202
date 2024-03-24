/*
MSE 2202 Project - V1 Funnel
March 7th, 2024
Howard Wen
*/

// libraries
#include <Adafruit_NeoPixel.h>                                                 // LED library
#include <Wire.h>                                                              // wire library for I2C
#include "Adafruit_TCS34725.h"                                                 // color sensor library

// debug
#define DEBUG_COLOR                                                            // debug color sensors

// function compile declarations
void setMotor(int dir, int pwm, int chanA, int chanB);
void stopMotor();
void clearEncoders();
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void ARDUINO_ISR_ATTR buttonISR();
void ARDUINO_ISR_ATTR timerISR();

// structures
struct Encoder {
  const int chanA;                                                             // channel A pin
  const int chanB;                                                             // channel B pin
  long pos;                                                                    // store current encoder position
};

// PIN CONSTANTS
  // motors
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define WINDMILL_MOTOR_PIN  8                                                  // windmill motor pin
#define WINDMILL_MOTOR_CHAN 4                                                  // windmill motor channel
  // ultrasonic sensor
#define USENSOR_TRIG        39                                                 // ultrasonic sensor trigger pin
#define USENSOR_ECHO        40                                                 // ultrasonic sensor echo pin
  // color sensor (TCS34725)
#define SDA                 47                                                 // I2C data pin
#define SCL                 48                                                 // I2C clock pin
#define TCSLED              14                                                 // color sensor LED
  // servos
#define SERVO_DEPOSIT_PIN   41                                                 // deposit servo motor pin
#define SERVO_DEPOSIT_CHAN  5                                                  // deposit servo motor channel
#define SERVO_SORT_PIN      42                                                 // sort servo motor pin
#define SERVO_SORT_CHAN     6                                                  // sort servo motor channel
  // inputs
#define PUSH_BUTTON         0                                                  // push button pin number
#define DEBOUNCE            250                                                // push button debounce
  // LEDs
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use

// constants
  // servos
const int cDepositStore = 2000;                                                // deposit storing value (2000/16383 = 12.2%)
const int cDepositDump = 1250;                                                 // deposit dumping value (1250/16383 = 7.6%)
const int cSortGreen = 400;                                                    // sorting value if green
const int cSortNotGreen = 625;                                                 // sorting value if not green
const int cSortMiddle = 500;                                                   // sorting value to test gem color
const int cServoPWMfreq = 50;                                                  // servo frequency
const int cServoPWMRes = 14;                                                   // servo resolution
  // motors
const int cPWMFreq = 20000;                                                    // PWM frequency
const int cPWMRes = 8;                                                         // PWM resolution
// TO DO - change cPWMMin
const int cPWMMin = pow(2, 8)/4;                                               // min PWM (quarter) ARBITRARY
const int cPWMMax = pow(2, 8) - 1;                                             // max PWM (full bar)
const int cNumMotors = 2;                                                      // number of motors
const int cINPinA[] = {LEFT_MOTOR_A, RIGHT_MOTOR_A};                           // left and right motor A pins
const int cINChanA[] = {0,1};                                                  // left and right motor A ledc channels
const int cINPinB[] = {LEFT_MOTOR_B, RIGHT_MOTOR_B};                           // left and right motor B pins
const int cINChanB[] = {2,3};                                                  // left and right motor B ledc channels
  // timer
const int cTimer1ID = 0;                                                       // timer 1 (1 of 4 timers with IDs from 0 to 3)
const int cSweepTime = 15000000;                                               // 15,000,000 ticks (15 seconds)
const int cPrescaler = 80;                                                     // prescaler (80 MHz => 1 MHz)

// objects
  // left and right encoder structures initialized with position 0
Encoder encoder[] = {{ENCODER_LEFT_A, ENCODER_LEFT_B, 0},
                     {ENCODER_RIGHT_A, ENCODER_RIGHT_B, 0}};

  // initialize smart LED object
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800); // smart LED

  // initialize TCS object with 2.4ms integration and gain of 4
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_16X);
bool tcsFlag = false;

// variables
  // navigation
int driveSpeed;                                                                // speed for motor
int robotStage = 0;                                                            // tracks robot stage: 0 = stop, 1 = collecting, 2 = depositing
int driveStage = 0;                                                            // tracks driving pattern, 0 = stop, 1 = forward, 2 = turning
int turnNum = 0;                                                               // track number of turns
  // time variables
unsigned long prevTime;                                                        // previous time for arduino
unsigned long currTime;                                                        // current time for arduino
unsigned int timeDiff;                                                         // difference between current and previous time
int msCounter;                                                                 // count for ms
int oneSecondCounter;                                                          // count for 1 second
bool oneSecondPassed;                                                          // 1 second passed flag
int twoSecondCounter;                                                          // count for 2 second
bool twoSecondPassed;                                                          // 2 second passed flag
  // ultrasonic detector variables
float usDuration;                                                              // ultrasonic sensor duration (us)
float usDistance;                                                              // ultrasonic sensor distance (cm)
int usMode;                                                                    // track ultrasonic sensor mode (0 is stopped, 1 is sending, 2 is receiving)
  // push button debounce
volatile unsigned long buttonTime = 0;                                         // track button pressed time
volatile unsigned long lastButtonTime = 0;                                     // track prev button pressed time
uint32_t numberPresses = 0;                                                    // track # of times pressed
bool pressed = false;                                                          // button flag
  // color sensor
bool isGreen;                                                                  // track whether current gem is green
bool prevGreen = false;                                                        // track whether prev test had a green gem
  // timer interrupt
hw_timer_t* pTimer = NULL;                                                     // timer pointer
bool returnHome = false;                                                       // determine whether it needs to return from

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // set up motors and encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttachPin(cINPinA[k], cINChanA[k]);                                     // set up L/R motor A pin and set to corresponding channel {0,1}
    ledcSetup(cINChanA[k], cPWMFreq, cPWMRes);                                  // set up channel with PWM freq and resolution
    ledcAttachPin(cINPinB[k], cINChanB[k]);                                     // set up L/R motor B pin and set to corresponding channel {2,3}
    ledcSetup(cINChanB[k], cPWMFreq, cPWMRes);                                  // set up channel with PWM freq and resolution

    pinMode(encoder[k].chanA, INPUT);                                           // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                                           // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  // set up windmill motor
  ledcAttachPin(WINDMILL_MOTOR_PIN, WINDMILL_MOTOR_CHAN);                        // set up pin and channel
  ledcSetup(WINDMILL_MOTOR_CHAN, cPWMFreq, cPWMRes);                             // set up channel with PWM freq and resolution
  // pinMode(WINDMILL_MOTOR_PIN, OUTPUT);                                           // set up motor pin output

  // set up ultrasonic sensor
  pinMode(USENSOR_TRIG, OUTPUT);
  pinMode(USENSOR_ECHO, INPUT);

  // set up tcs color sensor
  Wire.setPins(SDA, SCL);                                                        // set up i2c pins
  pinMode(TCSLED, OUTPUT);                                                       // set tcs LED as an output pin
  digitalWrite(TCSLED, HIGH);
  if (tcs.begin()) {
    Serial.println("TCS connection found");
    tcsFlag = true;
  } else {
    Serial.println("TCS connection not found, try again");
    tcsFlag = false;
  }

  // set up sorting servo pins
  pinMode(SERVO_SORT_PIN, OUTPUT);                                               // set up sorting servo motor pin
  ledcAttachPin(SERVO_SORT_PIN, SERVO_SORT_CHAN);                                // set up sorting servo motor channel
  ledcSetup(SERVO_SORT_CHAN, cServoPWMfreq, cServoPWMRes);                       // set up channel with PWM freq and resolution
  ledcWrite(SERVO_SORT_CHAN, cSortMiddle);                                       // move sorting arm to middle

  // set up deposit servo pins
  pinMode(SERVO_DEPOSIT_PIN, OUTPUT);                                            // set up servo motor pin
  ledcAttachPin(SERVO_DEPOSIT_PIN, SERVO_DEPOSIT_CHAN);                          // set up servo motor channel
  ledcSetup(SERVO_DEPOSIT_CHAN, cServoPWMfreq, cServoPWMRes);                    // set up channel with PWM freq and resolution
  ledcWrite(SERVO_DEPOSIT_CHAN, cDepositStore);                                  // move deposit bin to storage position

  // set up push button
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  attachInterrupt(PUSH_BUTTON, buttonISR, FALLING);

  // set up timer alarm
  pTimer = timerBegin(cTimer1ID, cPrescaler, true);                              // initialize esp32 timer1, with 1 microsecond clock, counting up
  timerStop(pTimer);                                                             // stop the timer after initialization
  timerAttachInterrupt(pTimer, &timerISR, true);                                 // attach timer interrupt to timer, edge enabled
  timerAlarmWrite(pTimer, sweepTime, false);                                     // set timer to go off after 15 seconds, no reload

  // declare previous time as 0 and current time as 0
  prevTime = 0;
  currTime = 0;
  // declare drive speed
  driveSpeed = 200;
  // declare us mode
  usMode = 0;

  // Set up SmartLED
  SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
  SmartLEDs.clear();                                                          // clear pixel
  SmartLEDs.setPixelColor(0,SmartLEDs.Color(255,0,0));                        // set pixel colors to green
  SmartLEDs.setBrightness(15);                                                // set brightness of heartbeat LED
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}

void loop() {
  // if push button is pressed and robot is currently stopped
  if (pressed) {
    timerWrite(pTimer, 0);                                                      // reset timer
    timerStart(pTimer);                                                         // start timer
    Serial.printf("Starting %d second timer\n", sweepTime/1000000);

    pressed = false;                                                            // reset button flag
  }

  if (returnHome) {
    Serial.printf("%d second timer up!\n", sweepTime/1000000);
    timerStop(pTimer);
    returnHome = false;
  }

  // clear time passed flags
  oneSecondPassed = false;
  twoSecondPassed = false;
}

// motor function
void setMotor(int dir, int pwm, int chanA, int chanB) {
  // take direction as argument
  switch (dir) {
    // if direction is 1, forwards
    case 1:
      Serial.println("setting motor forwards");
      ledcWrite(chanA, pwm);                                                  // set front channel to pwm
      ledcWrite(chanB, 0);                                                    // set backwards channel to 0
      break;
    // if direction is -1, backwards
    case -1:
      ledcWrite(chanA, 0);                                                    // set front channel to 0
      ledcWrite(chanB, pwm);                                                  // set backwards channel to pwm
      break;
    // if direction is 0, stop
    case 0:
      ledcWrite(chanA, 0);                                                    // set front channel to 0
      ledcWrite(chanB, 0);                                                    // set backwards channel to 0
      break;
  }
}

// stop motor
void stopMotor() {
  ledcWrite(cINChanA[0], 0);
  ledcWrite(cINChanB[0], 0);
  ledcWrite(cINChanA[1], 0);
  ledcWrite(cINChanB[1], 0);
}

// clear encoders
void clearEncoders() {
  encoder[0].pos = 0;
  encoder[1].pos = 0;
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
   Encoder* s = static_cast<Encoder*>(arg);                                  // cast pointer to static structure
  
   int b = digitalRead(s->chanB);                                            // read state of channel B
   if (b > 0) {                                                              // high, leading channel A
      s->pos++;                                                              // increase position
   }
   else {                                                                    // low, lagging channel A
      s->pos--;                                                              // decrease position
   }
}

// button isr
void ARDUINO_ISR_ATTR buttonISR() {
  buttonTime = millis();
  if (buttonTime - lastButtonTime > DEBOUNCE) {
    numberPresses++;
    pressed = true;
    lastButtonTime = buttonTime;
  }
}

// timer isr
void ARDUINO_ISR_ATTR timerISR() {
  returnHome = true;
}