/*
MSE 2202 Project - V1 Funnel
March 7th, 2024
Howard Wen
*/

// library
#include <Adafruit_NeoPixel.h>

// function compile declarations
void setMotor(int dir, int pwm, int chanA, int chanB);
void ARDUINO_ISR_ATTR encoderISR(void* arg);

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
  // sensors
#define USENSOR_TRIG        39                                                 // ultrasonic sensor trigger pin
#define USENSOR_ECHO        40                                                 // ultrasonic sensor echo pin
  // inputs
#define PUSH_BUTTON         0                                                  // push button pin number
  // LEDs
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use


// constants
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

Encoder encoder[] = {{ENCODER_LEFT_A, ENCODER_LEFT_B, 0},
                     {ENCODER_RIGHT_A, ENCODER_RIGHT_B, 0}};                   // left and right encoder structures initialized with position 0
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800); // LED

// variables
int pbDebounce;                                                                // debounce timer for push button
int driveSpeed;                                                                // speed for motor
int robotStage;                                                                // tracks robot stage: 0 = stop, 1 = driving
  // time variables
unsigned long prevTime;                                                        // previous time for arduino
unsigned long currTime;                                                        // current time for arduino
unsigned int timeDiff;                                                         // difference between current and previous time
// int fiveHundredMSCounter                                                       // count for 500 ms
// bool fiveHundredMSPassed;                                                      // 500 ms passed flag
int oneSecondCounter;                                                          // count for 1 second
bool oneSecondPassed;                                                          // 1 second passed flag
int twoSecondCounter;                                                          // count for 2 second
bool twoSecondPassed;                                                          // 2 second passed flag
  // ultrasonic detector variables
float usDuration;                                                              // ultrasonic sensor duration (us)
float usDistance;                                                              // ultrasonic sensor distance (cm)
int usMode;                                                                    // track ultrasonic sensor mode (0 is stopped, 1 is sending, 2 is receiving)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // set up motors and encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttachPin(cINPinA[k], cINChanA[k]);                                     // set up L/R motor A pin and set to corresponding channel {0,1}
    ledcSetup(cINChanA[k], cPWMFreq, cPWMRes);                                  // set up channel with PWM freq and resolution
    ledcAttachPin(cINPinB[k], cINChanB[k]);                                     // set up L/R motor B pin and set to corresponding channel {2,3}
    ledcSetup(cINChanB[k], cPWMFreq, cPWMRes);                                  // set up channel with PWM freq and resolution

    pinMode(encoder[k].chanA, INPUT);                                        // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                                        // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  // set up ultrasonic sensor
  pinMode(USENSOR_TRIG, OUTPUT);
  pinMode(USENSOR_ECHO, INPUT);

  // set up push button
  pinMode(PUSH_BUTTON, INPUT_PULLUP);

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
  SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,255,0));                        // set pixel colors to 'off'
  SmartLEDs.setBrightness(15);                                                // set brightness of heartbeat LED
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}

void loop() {
  // get current time in microseconds (us)
  currTime = micros();
  timeDiff = currTime - prevTime;

  // ultrasonic code
  digitalWrite(USENSOR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(USENSOR_TRIG, LOW);
  usDuration = pulseIn(USENSOR_ECHO, HIGH);
  usDistance = 0.017 * usDuration;                                                      // calculate distance
  Serial.printf("Distance: %.2fcm\n", usDistance);

  // if past loop time is 1ms less than current loop time 
  if (timeDiff >= 1000) {
    // update previous time
    prevTime = currTime;

    // increment 1 second timer
    oneSecondCounter = oneSecondCounter + timeDiff/1000;
    // check to see if one second has passed
    if (oneSecondCounter >= 1000) {
      Serial.println("1 second has passed");
      // set one second passed flag to true
      oneSecondPassed = true;
      // reset counter
      oneSecondCounter = 0;
    }

    // increment 2 second timer
    twoSecondCounter = twoSecondCounter + timeDiff/1000;
    if (twoSecondCounter >= 2000) {
      Serial.println("2 seconds have passed");
      // set two second passed flag to true
      twoSecondPassed = true;
      // reset counter
      twoSecondCounter = 0;
    }
  }

  // if push button is pressed and robot is currently stopped
  if (!digitalRead(PUSH_BUTTON) && robotStage == 0) {
    Serial.println("Button pressed, moving to stage 1 and waiting two seconds");
    // set robot to stage 1
    robotStage = 1;
    // reset 2 second timer
    twoSecondPassed = false;
    twoSecondCounter = 0;

  }

  // if robot is in stage 1 (driving)
  if (robotStage == 1) {
    if (twoSecondPassed) {
      Serial.printf("2 seconds have passed, now movingat speed %d\n", driveSpeed);
      // start the motors to go forwards
      setMotor(1, driveSpeed, cINChanA[0], cINChanB[0]);
      setMotor(-1, driveSpeed, cINChanA[1], cINChanB[1]);
      // set stage
      robotStage = 2;
    }
  }

  if (robotStage == 2 && usDistance > 1 && usDistance < 5) {
    Serial.println("Robot has encountered obstacle, stopping");
    robotStage = 0;
    encoder[0].pos = 0;
    // start the motors to stop
    setMotor(0, 0, cINChanA[0], cINChanB[0]);
    setMotor(0, 0, cINChanA[1], cINChanB[1]);
  }

  // if (robotStage == 0 && twoSecondPassed) {
  //   Serial.println("stopping motors!");
  //   // start the motors to stop
  //   setMotor(0, 0, cINChanA[0], cINChanB[0]);
  //   setMotor(0, 0, cINChanA[1], cINChanB[1]);
  // }

  // clear time passed flags
  oneSecondPassed = false;
  twoSecondPassed = false;

  delay(1000);
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
