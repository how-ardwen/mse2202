# MSE 2202 Final Project

## Code Implementation Steps

- [x] Initial Upload
  - Create GitHub, invite everyone as collaborators
  - Start Arduino code and establish enough to get the robot moving forwards

- [X] Ultrasonic Sensor Testing and Implementation
  - Test and record ultrasonic sensor behaviour
  - Physically add sensors to robot
  - Implement code to get robot to stop once sensors detect something _x_ distance away

- [x] IR Testing and implementation
  - Test IR sensitivity and direction
  - Implement code to search for IR signal by spinning, before going towards it

- [ ] Finalize Navigation System
  - Get robot sweeping an area without colliding into anything and then returning to IR beacon once finished
  - Get containment raising arm spinning

- [x] Pigmentation Sensor Testing and Implementation
  - Test and record pigmentation behavior
  - Get LED to shine whatever color the pigment sensor detects 

- [x] Finalize Sorting System
  - Implement code to get sensors to trigger servo motor. During testing phase, if green, servo turns left, if not green, servo turns right
  - Finish implementing code into robot, refine servo motor timing as well
  - Implement code for final deposit into collection container

## Code Explanation

### Setup

1. Begins Serial at 115200 baud
2. Sets up each motor with corresponding channel and sstuff

### Loop

Constantly updates time in microseconds
- Flag for 50ms
- Flag for 1 second
- Flag for 2 seconds

Variable Explanations
- robotStage keeps track of stage robot is in.
  - 0 for idle: waiting for button press
  - 1 for collection: moves around in snake pattern sorting gems
  - 2 for depositing: scouts for IR beacon, returns to base and deposits
- driveStage keeps track of how the robot drives
  - 0 for stopped
  - 1 for forwards motion
  - 2 for turning 180 left or right
- depositStage keeps track of which deposit step is taking place
  - 0 is stopped
  - 1 is searching for IR
  - 2 is for forwards driving
  - 3 is ultrasonic detector
  - 4 is for turning around for rear to face container
  - 5 is for servo motor to deposit gems
