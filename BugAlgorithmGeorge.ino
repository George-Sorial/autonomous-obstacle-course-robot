//Version by George A A Sorial  
// 8/12/2025

//Define L298N Dual H-Bridge Motor Controller Pins
#include <Servo.h>
#define LFSensor_0 A0
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4 
Servo head; 
const int SERVO_PIN = 10;
const int Trig = 3; 
const int Echo = 2; 
const int distancelimit = 12; // 12cm obstacle detection
#define FAST_SPEED 150
#define MID_SPEED 140
#define SLOW_SPEED  130     //back speed
#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
#define RightMotorDirPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
#define RightMotorDirPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
#define LeftMotorDirPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
#define LeftMotorDirPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 


enum Mode { LINE_FOLLOW, WALL_FOLLOW }; //to allow the robot to switch from modes 
Mode mode = LINE_FOLLOW;

/*motor control*/
void go_Advance(void)  //Forward
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
}
void go_Left(int t=0)  //Turn left
{
  digitalWrite(RightMotorDirPin1, HIGH);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,HIGH);
  digitalWrite(LeftMotorDirPin2,LOW);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,HIGH);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,HIGH);
  analogWrite(speedPinL,200);
  analogWrite(speedPinR,200);
  delay(t);
}
void stop_Stop()    //Stop
{
  digitalWrite(RightMotorDirPin1, LOW);
  digitalWrite(RightMotorDirPin2,LOW);
  digitalWrite(LeftMotorDirPin1,LOW);
  digitalWrite(LeftMotorDirPin2,LOW);
}

void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}


void setup()
{
  pinMode(RightMotorDirPin1, OUTPUT); 
  pinMode(RightMotorDirPin2, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
 
  pinMode(LeftMotorDirPin1, OUTPUT);
  pinMode(LeftMotorDirPin2, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();//stop move  

  Serial.begin(9600);   // initialize serial for debugging

  pinMode(Trig, OUTPUT); 
  pinMode(Echo, INPUT);
  head.attach(SERVO_PIN); 
  head.write(90);

}

int watch(){
  // Ultrasonic distance measurement using HC-SR04.
  // The sensor emits a 40 kHz pulse (Trig HIGH), and pulseIn() measures
  // the time for the echo to return. Distance is computed using the
  // time-of-flight equation: d = (t * speed_of_sound) / 2.
  // The constant 0.01657 converts microseconds to centimetres:
  // 343 m/s = 0.0343 cm/µs, and divide by 2 for the round trip.
  // This provides real-time closed-loop distance sensing for obstacle detection.
  long echo_distance;
  digitalWrite(Trig,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig,HIGH);
  delayMicroseconds(15);
  digitalWrite(Trig,LOW);
  echo_distance=pulseIn(Echo,HIGH);
  echo_distance=echo_distance*0.01657; //how far away is the object in cm
  //Serial.println((int)echo_distance);
  return round(echo_distance);
}

char sensor[5];

// Reads five IR reflectance sensors and converts them into a 5-bit pattern.
// A '1' indicates black (line), and '0' indicates white (background).
// The bit pattern encodes the lateral position of the line relative to the robot.
// This provides closed-loop feedback for correcting steering in line-following mode.
String read_sensor_values()
{   int sensorvalue=32;
  sensor[0]= !digitalRead(LFSensor_0);
 
  sensor[1]=!digitalRead(LFSensor_1);
 
  sensor[2]=!digitalRead(LFSensor_2);
 
  sensor[3]=!digitalRead(LFSensor_3);
 
  sensor[4]=!digitalRead(LFSensor_4);
  sensorvalue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
  
  String sen_str= String(sensorvalue,BIN);
  sen_str=sen_str.substring(1,6);


  return sen_str;
}

// Flags used for Bug-style navigation logic.
// wallHit = robot first encounters an obstacle ("hit point").
// goalHit = robot has reached the goal line ("goal condition").
// These markers allow the robot to distinguish between moving along
// the main path and following obstacle boundaries.

bool goalHit = false;
bool wallHit = false;

// Wall-following behaviour (right-hand rule):
// The robot attempts to maintain a target distance from the right wall.
// Control decisions are based on feedback from ultrasonic measurements,
// making this behaviour fully closed-loop. If the robot drifts too close,
// it steers left; if too far, it steers right. When the front is blocked,
// the robot performs a right rotation to continue following the boundary.
// This behaviour approximates the boundary-following stage of Bug 2.

void wallFollow() {
  stop_Stop();

  // The servo-mounted ultrasonic sensor performs directional scanning.
  // 45°, 90°, and 135° correspond to right, forward, and left orientations.
  // This allows the robot to estimate obstacle positions on multiple sides,
  // which is essential for determining how to follow a wall or avoid obstacles.
  // By reading distance at multiple angles, the robot forms a simple
  // situational awareness model without requiring a full 360° lidar.

  // Scan front
  head.write(90); // center
  delay(50);
  long frontDist = watch();

  // Scan left (45°)
  head.write(45);
  delay(50);
  long leftDist = watch();

  // Scan right (135°)
  head.write(135);
  delay(50);
  long rightDist = watch();

  // Return servo to center
  head.write(90);


  // --- Closed-loop logic ---

  int targetDist = 12;   // ideal distance from wall (cm)
  int tolerance  = 3;    // +- cm allowed


  // Right turns here use fixed timing (e.g., go_Right(200)),
  // which is open-loop. The rotation angle depends on battery voltage,
  // motor torque, and friction. As batteries discharge, turn angles become
  // smaller, reducing reliability. This is a known limitation of open-loop
  // timing control, since no sensor is used to measure heading directly.

  // FRONT BLOCKED → turn right (clockwise)
  if (frontDist < targetDist + tolerance) {
    stop_Stop();
    go_Right(200);
    wallHit = true;
    return;
  }

  // Too far from right wall → rotate right to hug the wall
  if (rightDist > targetDist + tolerance) {
    go_Right(80);
    return;
  }

  //  Too close to right wall → rotate left
  if (rightDist < targetDist - tolerance) {
    go_Left(80);
    return;
  }

  //  Path clear + distance good → drive forward
  go_Advance();
  set_Motorspeed(150,150);
}

// High-level navigation controller.
// This function selects either line-following or wall-following based on
// sensor conditions. It fuses ultrasonic distance measurements with
// IR line-sensor data to decide behaviour. This implements a reactive
// hybrid navigation system combining closed-loop feedback (sensors)
// with mode-based decision making (state machine).


void auto_tracking(){
  String Sensor_val= read_sensor_values();
  bool onLine = (Sensor_val != "00000" && Sensor_val != "11111");
  bool goalLine = (Sensor_val == "11111");
  int frontDist = watch();

  // The navigation system switches between LINE_FOLLOW and WALL_FOLLOW modes.
  // This implements a simplified Bug-2 style behaviour:
  //  - LINE_FOLLOW acts as movement along the "m-line" (desired path).
  //  - WALL_FOLLOW activates when an obstacle is detected.
  // The robot records a "hit point" when switching to WALL_FOLLOW,
  // and a "leave point" when the line is detected again.
  // This ensures the robot resumes goal-directed motion after bypassing
  // an obstacle, consistent with Bug 2’s theoretical behaviour,
  // assuming the line corresponds to the true m-line.

  if(mode == LINE_FOLLOW){
    if (frontDist < distancelimit || Sensor_val == "00000"){// "00000" means all sensors see white, so the robot has lost the line.
      mode = WALL_FOLLOW; // record "hit point"
      wallHit = true;
    }
  }

  
  else if (mode == WALL_FOLLOW) {
    // If line appears again, switch back to line mode
    if (onLine && wallHit) {
      mode = LINE_FOLLOW; // "leave point" reached
      wallHit = false;
    }
  }
  if (mode == WALL_FOLLOW) {
    wallFollow();
    return;   // skip line logic
  }

  // ================================
  // LINE FOLLOWING MODE
  // ================================

  // Different bit patterns correspond to the line being detected on the left,
  // right, centre, or at multiple sensors. The robot adjusts motor speeds
  // to steer proportionally in the direction needed to recenter on the line.
  // This implements a simple feedback controller using discrete inputs,
  // producing reactive closed-loop steering behaviour.

  if (Sensor_val=="10000" )
  { 
    //The black line is in the left of the car, need  left turn 
    go_Left();  //Turn left
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (Sensor_val=="10100"  || Sensor_val=="01000" || Sensor_val=="01100" || Sensor_val=="11100"  || Sensor_val=="10010" || Sensor_val=="11010")
  {
    go_Advance();  //Turn slight left
    set_Motorspeed(0,FAST_SPEED);
  }
  if (Sensor_val=="00001"  ){ //The black line is  on the right of the car, need  right turn 
    go_Right();  //Turn right
    set_Motorspeed(FAST_SPEED,FAST_SPEED);
  }
  if (Sensor_val=="00011" || Sensor_val=="00010"  || Sensor_val=="00101" || Sensor_val=="00110" || Sensor_val=="00111" || Sensor_val=="01101" || Sensor_val=="01111"   || Sensor_val=="01011"  || Sensor_val=="01001")
  {
    go_Advance();  //Turn slight right
    set_Motorspeed( FAST_SPEED,0);
  }
 
  if (Sensor_val=="11111"){
    stop_Stop();   //The car front touch stop line, need stop
    set_Motorspeed(0,0);
    mode = LINE_FOLLOW;
    goalHit = true;
  }
}


  void loop()
  { 
    auto_tracking();
  } //end of loop
 
