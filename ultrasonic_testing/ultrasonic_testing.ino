#define USENSOR_TRIG 39
#define USENSOR_ECHO 40

float usDuration, usDistance;
int usMode;                                                                    // 0 for stopped, 1 for sending, 2 for receiving

unsigned long prevTime;                                                        // previous time for arduino
unsigned long currTime;                                                        // current time for arduino
unsigned int timeDiff;                                                         // difference between current and previous time
int oneSecondCounter;                                                          // count for 1 second
bool oneSecondPassed;                                                          // 1 second passed flag
int twoSecondCounter;                                                          // count for 2 second
bool twoSecondPassed;                                                          // 2 second passed flag

void setup() {
  // put your setup code here, to run once:
  pinMode(USENSOR_TRIG, OUTPUT);
  pinMode(USENSOR_ECHO, INPUT);

  prevTime = 0;
  currTime = 0;
  usMode = 0;
}

void loop() {
  // get current time in microseconds (us)
  currTime = micros();
  timeDiff = currTime - prevTime;

  // if stopped, wait 0.5 seconds
  if (usMode == 0 && timeDiff >= 500000) {
    Serial.println("waited 0.5 seconds");
    // set to send mode
    usMode = 1;
    // update time
    prevTime = currTime;
  }

  // if in send mode, send high signal to trigger
  if (usMode == 1) {
    Serial.println("sending pulse");
    digitalWrite(USENSOR_TRIG, HIGH);
    usMode = 2;
    timeDiff = 0;
    delayMicroseconds(10);
    digitalWrite(USENSOR_TRIG, LOW);                                                      // turn off trigger
  }

  // if in receive mode and 10 us have passed
  if (usMode == 2) {
    usDuration = pulseIn(USENSOR_ECHO, HIGH);                                             // measure duration of pulse
    usDistance = 0.017 * usDuration;                                                      // calculate distance
    Serial.printf("Distance: %.2fcm\n", usDistance);

    // stop receiving
    usMode = 0;
    prevTime = currTime;
  }
}
