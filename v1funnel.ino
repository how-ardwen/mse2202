/*
MSE 2202 Project - V1 Funnel
March 7th, 2024
Howard Wen
*/

// libraries
#include <Adafruit_NeoPixel.h>                                                 // LED library
#include <Wire.h>                                                              // wire library for I2C
#include "Adafruit_TCS34725.h"                                                 // color sensor library
#include <WiFi.h>                                                              // wifi module

// debug
#define DEBUG_COLOR                                                            // debug color sensors

// function compile declarations
void setMotor(int dir, int pwm, int chanA, int chanB);
void stopMotors();
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
  // IR detector
#define IR_PIN              13                                                 // IR detector pin
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
const int cSortGreen = 300;                                                    // sorting value if green
const int cSortNotGreen = 500;                                                 // sorting value if not green
const int cSortMiddle = 320;                                                   // sorting value to test gem color
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
const int cSweepTime = 45000000;                                               // 15,000,000 ticks (15 seconds)
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
int robotStage = 0;                                                            // tracks robot stage: 0 = idle, 1 = collecting, 2 = depositing
int driveStage = 0;                                                            // tracks driving pattern, 0 = stop, 1 = forward, 2 = turning
int sortStage = 0;                                                             // tracks sort stage, 0 is reading gem color, 1 is moving arm, 2 is waiting
int depositStage = 0;                                                          // tracks deposit pattern, 0 = searching for IR, 1 = forward, 2 = turning around, 3 = depositing
int turnNum = 0;                                                               // track number of turns
  // time variables
unsigned long prevTime;                                                        // previous time for arduino
unsigned long currTime;                                                        // current time for arduino
unsigned int timeDiff;                                                         // difference between current and previous time
int msCounter;                                                                 // count for ms
int fiftyMsCounter;                                                            // count for 50 ms
bool fiftyMsPassed;                                                            // 50 millisecond timer
int sortingTimeCounter;
bool sortingTimePassed;
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
  // deposit servo
int depositServoPos;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // wifi setup
  WiFi.mode(WIFI_AP);
  WiFi.softAP("kuromi", "mse2202!");

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

  // set up IR detector
  pinMode(IR_PIN, INPUT);

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
  depositServoPos = cDepositStore;

  // set up push button
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  attachInterrupt(PUSH_BUTTON, buttonISR, FALLING);

  // set up timer alarm
  pTimer = timerBegin(0, cPrescaler, true);                                      // initialize esp32 timer1, with 1 microsecond clock, counting up
  timerStop(pTimer);                                                             // stop the timer after initialization
  timerAttachInterrupt(pTimer, &timerISR, true);                                 // attach timer interrupt to timer, edge enabled
  timerAlarmWrite(pTimer, cSweepTime, false);                                    // set timer to go off after 15 seconds, no reload
  timerAlarmEnable(pTimer);                                                      // enable timer alarm

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
  // get current time in microseconds (us)
  currTime = micros();
  timeDiff = currTime - prevTime;

  // if past loop time is 1ms less than current loop time 
  if (timeDiff >= 1000) {
    // update previous time
    prevTime = currTime;

    // increment ms counter
    msCounter = msCounter + 1;
    
    fiftyMsCounter++;
    if (fiftyMsCounter >= 50) {
      fiftyMsPassed = true;
      fiftyMsCounter = 0;
    }

    sortingTimeCounter++;
    if (sortingTimeCounter >= 100) {
      sortingTimePassed = true;
      sortingTimeCounter = 0;
    }

    // increment 1 second timer
    oneSecondCounter = oneSecondCounter + timeDiff/1000;
    // check to see if one second has passed
    if (oneSecondCounter >= 1000) {
      // Serial.println("1 second has passed");
      // set one second passed flag to true
      oneSecondPassed = true;
      // reset counter
      oneSecondCounter = 0;
    }

    // increment 2 second timer
    twoSecondCounter = twoSecondCounter + timeDiff/1000;
    if (twoSecondCounter >= 2000) {
      // Serial.println("2 seconds have passed");
      // set two second passed flag to true
      twoSecondPassed = true;
      // reset counter
      twoSecondCounter = 0;
    }
  }
  
  // if push button is pressed and robot is currently idle
  if (pressed && robotStage == 0) {
    Serial.println("Button pressed, moving to stage 1 and waiting two seconds");
    // set robot to stage 1
    robotStage = 1;
    // set drive to stage 1
    driveStage = 1;

    // reset 2 second timer
    twoSecondPassed = false;
    twoSecondCounter = 0;

    // activate timer alarm
    timerWrite(pTimer, 0);                                                      // reset timer
    timerStart(pTimer);                                                         // start timer
    Serial.printf("Starting %d second timer\n", cSweepTime/1000000);

    // clear encoders
    clearEncoders();

    ledcWrite(SERVO_DEPOSIT_CHAN, cDepositStore);                               // set deposit servo to storage position
    ledcWrite(WINDMILL_MOTOR_CHAN, driveSpeed);                                 // turn on windmill motor

    SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,255,0));                        // set pixel colors to green
    SmartLEDs.setBrightness(15);                                                // set brightness of heartbeat LED
    SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

    pressed = false;                                                            // reset button flag
  }

  // if robot is in collection mode
  if (robotStage == 1) {
    // if milisecond counter is a multiple of 500 (i.e. every 500ms, ping ultrasonic detector)
    if (msCounter % 500 == 0) {
      // ultrasonic code
      digitalWrite(USENSOR_TRIG, HIGH);
      delayMicroseconds(10);
      digitalWrite(USENSOR_TRIG, LOW);
      usDuration = pulseIn(USENSOR_ECHO, HIGH);
      usDistance = 0.017 * usDuration;                                               // calculate distance
      // Serial.printf("Distance: %.2fcm\n", usDistance);
      if (usDistance > 1 && usDistance < 5) {
        Serial.println("Robot has encountered obstacle, stopping");
        encoder[0].pos = 0;
        stopMotors();
        robotStage = 0;
      }
    }

    // if tcs is on and 1 second has passed and arm is in the middle (gem in FOV of color sensor)
    if (sortStage == 0 && oneSecondPassed && tcsFlag) {
      uint16_t r, g, b, c;                                                        // RGBC values from TCS
      tcs.getRawData(&r, &g, &b, &c);
      #ifdef DEBUG_COLOR
      Serial.printf("R: %d, G: %d, B: %d, C: %d\n", r, g, b, c);
      #endif
      isGreen = (g) > (r + 20) && (b) > (g - 10) && b < (g+10);
      if (isGreen) {
        Serial.println("is Green!");
        ledcWrite(SERVO_SORT_CHAN, cSortGreen);
      } else {
        ledcWrite(SERVO_SORT_CHAN, cSortNotGreen);
      }
      sortStage = 1;

      // reset time counter
      sortingTimePassed = false;
      sortingTimeCounter = 0;
    }

    if (sortStage == 1 && sortingTimePassed) {
      ledcWrite(SERVO_SORT_CHAN, cSortMiddle);
      sortStage = 0;

      
    }

    // wait two seconds before driving forwards
    if (twoSecondPassed && driveStage == 1) {
      Serial.printf("2 seconds have passed, now movingat speed %d\n", driveSpeed);
      // start the motors to go forwards
      setMotor(1, driveSpeed, cINChanA[0], cINChanB[0]);
      setMotor(-1, driveSpeed, cINChanA[1], cINChanB[1]);
      // set stage
      driveStage = 2;
    }

    // if driveStage == 2
    if (driveStage == 2 && encoder[0].pos >= 5000) {
      // even turn number, turn right
      if (turnNum % 2 == 0) {
        setMotor(0,0, cINChanA[0], cINChanB[0]);                                    // stop right motor
      }
      // odd number turns, turn left
      else {
        setMotor(0,0, cINChanA[1], cINChanB[1]);                                    // stop left motor
      }

      // clear encoders
      clearEncoders();

      turnNum++;                                                                    // increment turn number
      driveStage = 3;                                                               // set drive stage to 3
    }

    if (driveStage == 3 && (encoder[0].pos >= 1500 || encoder[1].pos <= -1500)) {
      if (turnNum % 2 == 0) {
        setMotor(-1, driveSpeed, cINChanA[1], cINChanB[1]);                         // start up left motor again
      }
      else {
        setMotor(1, driveSpeed, cINChanA[0], cINChanB[0]);                          // start up right motor again
      }

      clearEncoders();                                                              // clear encoders
      driveStage = 2;                                                               // set drive stage to 2
    }
  }

  // if alarm goes off to return home and deposit gems
  if (returnHome) {
    timerStop(pTimer);
    robotStage = 3;                                                                 // set to stage 3
    stopMotors();                                                                   // stop motors

    // reset 1 second timer
    oneSecondPassed = false;
    oneSecondCounter = 0;

    SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,255));                            // set pixel colors to blue
    SmartLEDs.setBrightness(15);                                                    // set brightness of heartbeat LED
    SmartLEDs.show();                                                               // send the updated pixel colors to the hardware
  }

  // if in depositing stage
  if (robotStage == 3) {
    // stage 0: stop 1 second before starting to spin 
    if (depositStage == 0) {
      Serial.println("spinning");
      setMotor(1, driveSpeed, cINChanA[0], cINChanB[0]);                          // set left motor to drive forwards
      setMotor(1, driveSpeed, cINChanA[1], cINChanB[1]);                          // set right motor to drive backwards
      depositStage = 1;                                                           // set deposit stage to 1 (going forwards)
    }

    // stage 1: Every 5ms while spinning, search for IR signal
    if (depositStage == 1 && msCounter % 5 == 0) {
      // search for signal
    }

    // stage 2: once IR signal is found, start moving forwards and 
    if (depositStage == 2) {
      setMotor(1, driveSpeed, cINChanA[0], cINChanB[0]);                          // set left motor to drive forwards
      setMotor(-1, driveSpeed, cINChanA[1], cINChanB[1]);                         // set right motor to drive forwards
      depositStage = 3;
    }

    // TO DO
    // stage 3: wait for ultrasonic detector to detect container 10 cm away
    if (depositStage == 3) {
      // ping ultrasonic every 50ms

      // if distance matches
      if (usDistance <= 10) {
        // stop motors
        // go to stage 4
      }
    }

    // stage 4: spin around
    if (depositStage == 4) {
      setMotor(1, driveSpeed, cINChanA[0], cINChanB[0]);                          // set left motor to drive forwards
      setMotor(1, driveSpeed, cINChanA[1], cINChanB[1]);                          // set right motor to drive backwards
      depositStage = 5;                                                           // set deposit stage to 5
      clearEncoders();
    }

    // stage 5: back in once left encoder has moved 500
    if (depositStage == 5 && encoder[0].pos >= 500) {
      setMotor(-1, driveSpeed, cINChanA[0], cINChanB[0]);                         // set left motor to drive backwards
      setMotor(1, driveSpeed, cINChanA[1], cINChanB[1]);                          // set right motor to drive backwards
      depositStage = 6;
      clearEncoders();
    }

    // stage 6, slowly engage servo to deposit gems once backed in 15cm (negative left encoder position because moving backwards)
    if (depositStage == 6 && encoder[0].pos <= -1500) {
      stopMotors();
      int split = (cDepositStore - cDepositDump)/10;
      if (fiftyMsPassed && depositServoPos > cDepositDump) {
        depositServoPos = depositServoPos - split;
        ledcWrite(SERVO_DEPOSIT_CHAN, depositServoPos);
      }
    }
  }

  // clear time passed flags
  fiftyMsPassed = false;
  sortingTimePassed = false;
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
void stopMotors() {
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