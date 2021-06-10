/* YourDuino / KUFY EBONG  Robot Control
  - WHAT IT DOES Controls a Keyes robot in multiple modes
  - SEE the comments after "//" on each line below

  - V1.01 16 August 2019
  - V1.01 18 September 2019: corrected autoDrive() func
  - V1.02 26 September 2019: Terry rearranged Pin definitions etc..
  - V1.05 30 September 2019: fixes by Kufy.. And fixed float assigment NULL.
  - V1.06 21 October   2019: Kufy added motor speed correction
  - V1.07 23 October   2019: Kufy added edge detection, changed THRESHOLD_DISTANCE and PAUSE_INTERVAL

   Questions: kuffykeys@gmail.com , terry@yourduino.com

*/


/*--------------( Import needed libraries )-------------------*/
#include <SimpleKalmanFilter.h>
#include <Servo.h>

/*-----( Declare Constants  )-----*/
#define DIRECTION_FORWARD 0  // For use with motor controls
#define DIRECTION_REVERSE 1
#define LOOP_TIME 100       // Refresh time to apply the closed-loop control signal to the motor. The frequency of the system is about 10Hz (f = 1/T, T = 0.1s)
#define THRESHOLD_DIST 45
#define EDGE_DIST 6
#define BUZZ_INTERVAL 600
#define LIGHT_THRESHOLD 800
#define PAUSE_INTERVAL 600


/*-------------( Declare  Pin Numbers )-----------------------*/
const byte LEFT_OPTOINTERRUPTER_PIN  =    2;  // The optoInterrupters for encoder discs
const byte RIGHT_OPTOINTERRUPTER_PIN =    3;

volatile unsigned int counter_left;
volatile unsigned int counter_right;

const byte LEFT_MOTOR_DIRECTION_PIN   =  4 ; // Motor Control Pins Direction and PWM Speed
const byte LEFT_MOTOR_POWER_PIN       =  5 ;
const byte RIGHT_MOTOR_POWER_PIN      =  6 ;
const byte RIGHT_MOTOR_DIRECTION_PIN  =  7 ;


//ultrasonic sensor parameters
const byte echoPin = 8;
const byte trigPin = 9;

const byte echoPinEdge = 12;
const byte trigPinEdge = 11;

const byte servoPin = 10;
const byte buzzerPin = 13; //meant for obstacle alert

const byte leftLine = A1;   // PhotoSensors in Line Follower
const byte  middleLine = A2;
const byte rightLine = A3;
const byte ldrPin = A4;

/*--------------------(flags for different motor speeds)---------------*/
bool slowSpeed = false, mediumSpeed = true, fastSpeed = false;

int leftMotorDirection  = 0;  // Forward zero power
int leftMotorPower      = 0;

int rightMotorDirection = 0;
int rightMotorPower     = 0;

static byte countSpeedLeft = 0;     // variable to store count speed of left motor in counts per 0.1s
static byte countSpeedRight = 0;    // variable to store count speed of right motor in counts per 0.1s

static byte powerLeft = 127;        // power to be applied to the left motor
static byte powerRight = 127;       // power to be applied to the right

static byte correctedLeftPower = 127;  // variables to remember the corrected speed when if the car comes to rest
static byte correctedRightPower = 127;

static byte lowerLimit = 3;
static byte upperLimit = 4;


/*************positions of servo *****************/
byte lookLeft = 170, lookRight = 17, lookAhead = 90;


/*************maybe create flags for the state of the robots movement declare these variables when the autoDrive state is entered*******************/

unsigned long prevMillis1 = 0l, prevMillis2 = 0l, prevMillis3 = 0l, prevMillis4 = 0l, prevMillis5 = 0l, prevMillis6 = 0l, currentMillis = 0l;
unsigned long prevBuzzMillis = 0l, currentBuzzMillis = 0l;

unsigned long currentMillis_Motor = 0l;   // variable to keep track of current time for motor speed update
unsigned long previousMillis_Motor = 0l;  // variable to keep track of previous time for motor speed update

/*----------------------( Declare Variables )--------------------------------*/

/***********flags for the millis state  ********************/
bool state1 = false, state2 = false, state3 = false, state4 = false, state5 = false, state6 = false;

bool resetBuzz = true, reset1 = true, reset2 = true, reset3 = true, reset4 = true, reset5 = true, reset6 = true;

int dist = 0;
int dist1 = 0;
int obstacleThreshold = 35; //Threshold distance where the robot sees object in front of it as an obstacle
int estimated_value = 0;


/*------( flags to hold the different robot modes )-----*/

bool manMode = false;
bool lineMode = false;
bool autoMode = false;
bool followMode = false;
bool StopMode = false;
bool frontMode = false;
bool backMode = false;
bool leftMode = false;
bool rightMode = false;
bool lightMode = false;

/*-----( variable to hold data from Serial )-----*/

char val = ' ';
char val1 = ' ';

/*-----( Declare objects )-----*/
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);
Servo servo;          //create servo object to control a servo, max = 8 servos

void setup()   /**************************** SETUP: RUNS ONCE *******************/
{
  pinMode(leftLine, INPUT);
  pinMode(middleLine, INPUT);
  pinMode(rightLine, INPUT);

  pinMode(ldrPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(servoPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPinEdge, OUTPUT);
  pinMode(echoPinEdge, INPUT);

  pinMode(LEFT_OPTOINTERRUPTER_PIN, INPUT);
  pinMode(RIGHT_OPTOINTERRUPTER_PIN, INPUT);

  pinMode(LEFT_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_POWER_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_POWER_PIN , OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION_PIN, OUTPUT);

  // turn on the optointerrupter LED
  digitalWrite(LEFT_OPTOINTERRUPTER_PIN, HIGH);
  digitalWrite(RIGHT_OPTOINTERRUPTER_PIN, HIGH);

  // attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_OPTOINTERRUPTER_PIN), countpulse_left, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_OPTOINTERRUPTER_PIN), countpulse_right, RISING);

  servo.attach(servoPin);
  servo.write(lookAhead); //default central position

  Serial.begin(9600);

}//--(end setup )---


void loop()   /******************** LOOP: RUNS CONSTANTLY *******************/
{

  //  delay(10);
  //  buzzer();
  //  delay(500);
//  int lightIntensity1 = analogRead(ldrPin);
//  Serial.println(lightIntensity1);
  //  autoDrive();
  //  Serial.println(detectEdge());
  //  Serial.println(obstacle());


  currentMillis_Motor = millis();


  if (currentMillis_Motor - previousMillis_Motor >= LOOP_TIME)
  {
    countSpeedLeft = counter_left;
    countSpeedRight = counter_right;

    counter_left = 0;
    counter_right = 0;

    //increase speeds to meet lowerLimit
    if (countSpeedLeft < lowerLimit)
    {
      powerLeft += 1;
    }

    if (countSpeedRight < lowerLimit)
    {
      powerRight += 1;
    }

    // increase speeds to reach upper limit
    if (countSpeedLeft > upperLimit)
    {
      powerLeft -= 1;
    }

    if (countSpeedRight > upperLimit)
    {
      powerRight -= 1;
    }

    //Set MAX power to be 250
    if (powerLeft > 250)
    {
      powerLeft = 250;
    }

    if (powerRight > 250)
    {
      powerRight = 250;
    }

    //set MIN power to be 0
    if (powerLeft <= 0)
    {
      powerLeft = 10;
    }

    if (powerRight <= 0)
    {
      powerRight = 0;
    }

    previousMillis_Motor = currentMillis_Motor;  // reset previousMillis

    //debugging
    //    Serial.println("powerLeft: " + String(powerLeft));
    //    Serial.println("powerRight: " + String(powerRight));
    //    Serial.println("SpeedLeft: "  + String(countSpeedLeft) + " count/0.1s");
    //    Serial.println("SpeedRight: "  + String(countSpeedRight) + " count/0.1s" + '\n');
  }

  if (countSpeedLeft >= lowerLimit && countSpeedLeft < upperLimit)
  {
    correctedLeftPower = powerLeft;
  }

  if (countSpeedRight >= lowerLimit && countSpeedRight < upperLimit)
  {
    correctedRightPower = powerRight;
  }

  /********this prevents the correction algorithm from running while the car is at rest*****/
  if (countSpeedLeft == 0)
  {
    powerLeft = correctedLeftPower;
  }

  if (countSpeedRight == 0)
  {
    powerRight = correctedRightPower;
  }


  if (Serial.available())
  {
    val = Serial.read();
    switch (val)
    {

      case 'x':
        manMode = true;
        lineMode = false;
        autoMode = false;
        followMode = false;
        lightMode = false;
        servo.write(lookAhead);
        break;

      case 'f':
        manMode = false;
        lineMode = false;
        autoMode = false;
        followMode = true;
        lightMode = false;
        servo.write(lookAhead);
        break;

      case 'T':
        manMode = false;
        lineMode = true;
        autoMode = false;
        followMode = false;
        lightMode = false;
        servo.write(lookAhead);
        break;

      case 't':
        manMode = false;
        lineMode = false;
        autoMode = false;
        followMode = false;
        lightMode = true;
        servo.write(lookAhead);
        break;

      case 'A':
        manMode = false;
        lineMode = false;
        autoMode = true;
        followMode = false;
        lightMode = false;

        dist1 = obstacle();
        state1 = true;
        state2 = false; state3 = false; state4 = false; state5 = false; state6 = false;
        reset1 = true; reset2 = true; reset3 = true; reset4 = true; reset5 = true; reset6 = true;

      case 'S':
        StopMode = true;
        frontMode = false;
        backMode = false;
        leftMode = false;
        rightMode = false;
        lightMode = false;
        break;

      case 'F':
        StopMode = false;
        frontMode = true;
        backMode = false;
        leftMode = false;
        rightMode = false;
        lightMode = false;
        break;

      case 'B':
        StopMode = false;
        frontMode = false;
        backMode = true;
        leftMode = false;
        rightMode = false;
        lightMode = false;
        break;

      case 'L':
        StopMode = false;
        frontMode = false;
        backMode = false;
        leftMode = true;
        rightMode = false;
        lightMode = false;
        break;

      case 'R':
        StopMode = false;
        frontMode = false;
        backMode = false;
        leftMode = false;
        rightMode = true;
        lightMode = false;
        break;

      case 'X':
        slowSpeed = true, mediumSpeed = false, fastSpeed = false;
        break;

      case 'Y':
        slowSpeed = false, mediumSpeed = true, fastSpeed = false;
        break;

      case 'Z':
        slowSpeed = false, mediumSpeed = false, fastSpeed = true;
        break;

      default:
        StopMode = true;
        frontMode = false;
        backMode = false;
        leftMode = false;
        rightMode = false;
        lightMode = false;
        servo.write(lookAhead);
        break;
    } // END Switch
  }// END If Serial Available

  if (manMode)
  {
    if (StopMode) {
      Stop();
    }

    if (frontMode) {
      front();
    }

    if (backMode) {
      back();
    }

    if (leftMode) {
      left();
    }

    if (rightMode) {
      right();
    }
  }// END If ManMode

  if (lineMode)
  {
    trackLine();
  }

  if (autoMode) {
    autoDrive();
  }

  if (followMode) {
    follower();
  }

  if (lightMode) {
    detectLight();
  }

  if (slowSpeed) {
    lowerLimit = 3;
    upperLimit = 4;
  }

  if (mediumSpeed) {
    lowerLimit = 4;
    upperLimit = 5;
  }

  if (fastSpeed) {
    lowerLimit = 5;
    upperLimit = 6;
  }

}//--(end main loop )---



/*-----( Declare User-written Functions )-----*/

void front()
{
  if (detectEdge() >= EDGE_DIST && manMode || detectEdge() >= EDGE_DIST && followMode || detectEdge() >= EDGE_DIST && lineMode)
  {
    Stop();
  }

  else
  {
    runLeftMotor(DIRECTION_FORWARD, powerLeft);
    runRightMotor(DIRECTION_FORWARD, powerRight);
  }
}

void back()
{
  runLeftMotor(DIRECTION_REVERSE, powerLeft);
  runRightMotor(DIRECTION_REVERSE, powerRight);
}

void Stop()
{
  stopLeftMotor();
  stopRightMotor();
}

void left()
{
  runLeftMotor(DIRECTION_REVERSE, powerLeft);
  runRightMotor(DIRECTION_FORWARD, powerRight);
}

void right()
{
  runLeftMotor(DIRECTION_FORWARD, powerLeft);
  runRightMotor(DIRECTION_REVERSE, powerRight);
}

void trackLine()
{
  if (digitalRead(middleLine))
  {
    if (digitalRead(rightLine) && !(digitalRead(leftLine)))
    {
      front();
    }
    else if (!(digitalRead(rightLine)) && digitalRead(leftLine))
    {
      front();
    }
    else
    {
      front();
    }
  }
  else
  {
    if (digitalRead(rightLine) && !(digitalRead(leftLine)))
    {
      left();
    }
    else if (!(digitalRead(rightLine)) && digitalRead(leftLine))
    {
      right();
    }
    else
    {
      Stop();
    }
  }
}// END TrackLine

int obstacle()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPin, LOW);
  dist = pulseIn(echoPin, HIGH);
  delayMicroseconds(10000);
  dist = dist / 58;

  //smoothening algorithm here for ultrasonic
  //  estimated_value = simpleKalmanFilter.updateEstimate(dist);
  //  Serial.println(estimated_value);
  //  return estimated_value;
  return dist;

}//END obstacle


int detectEdge()
{
  int estimate, distEdge;
  digitalWrite(trigPinEdge, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinEdge, HIGH);
  delayMicroseconds(12);
  digitalWrite(trigPinEdge, LOW);
  distEdge = pulseIn(echoPinEdge, HIGH);
  delayMicroseconds(10000);
  distEdge = distEdge / 58.00;

  //smoothening algorithm here for ultrasonic
  //  estimate = simpleKalmanFilter.updateEstimate(distEdge);

  //  Serial.println(estimated_value);
  return distEdge;

}//END obstacle


void autoDrive()
{

  if (state1 == true)
  {
    servo.write(lookAhead);
    if (detectEdge() < EDGE_DIST) {
      front();

      dist1 = obstacle();
      state2 =  false; state3 = false; state4 = false; state5 = false; state6 =  false;

      //for debugging
      Serial.println("looking ahead + moving forward + read dist1_______state1");
    }

    if (detectEdge() >= EDGE_DIST)
    {
      right();
    }
  }


  if (dist1 < THRESHOLD_DIST && !state3 && !state4)
  {
    state1 = false; state2 =  true; state3 = false; state4 = false; state5 = false; state6 =  false;

    if (state2 == true && !state3 && !state4)
    {
      Stop();
      servo.write(lookLeft);
      unsigned long currentMillis2 = millis();
      reset2 = false;
      reset1 = true; reset3 = true; reset4 = true; reset5 = true; reset6 = true;
      float dist2 = 0.0;;


      //for debugging
      Serial.println ("Stop + lookLeft___________state2");

      if (currentMillis2 - prevMillis2 >= PAUSE_INTERVAL)
      {
        dist2 = obstacle();
        state2 = false;
        reset2 = true;

        //for debugging
        Serial.println("read left distance + leave state 2-----------------------------------------------------(dist2)" + String(dist2));

        if (dist2 > THRESHOLD_DIST && !state1)
        {
          state1 = false; state2 =  false; state3 = true; state4 = false; state5 = false; state6 =  false;
        }

        if (dist2 < THRESHOLD_DIST && !state5)
        {
          state1 = false; state2 =  false; state3 = false; state4 = true; state5 = false; state6 =  false;
        }
      }
    }
  }


  if (state3 == true)
  {
    servo.write(lookAhead);
    left();
    unsigned long currentMillis3 = millis();
    reset3 = false;
    reset1 = true; reset2 = true; reset4 = true; reset5 = true; reset6 = true;

    //for debugging
    Serial.println("lookAhead + move left __________state3");

    if (currentMillis3 - prevMillis3 >= PAUSE_INTERVAL)
    {
      state1 = true;
      state2 =  false; state3 = false; state4 = false; state5 = false; state6 =  false;
      reset1 = false; reset2 = true; reset3 = true; reset4 = true; reset5 = true; reset6 = true;

      //for debugging
      Serial.println("leave state3 and proceed to state 1");
    }
  }

  if (state4 == true)
  {
    Stop();
    servo.write(lookRight);
    unsigned long currentMillis4 = millis();
    reset4 = false;

    float dist3 = 0;

    //for debugging
    Serial.println("Stop + lookRight __________state4");

    if (currentMillis4 - prevMillis4 >= PAUSE_INTERVAL)
    {
      dist3  = obstacle();
      state4 = false;
      reset4 = true;

      //for debugging
      Serial.println("read right distance and left state 4-----------------------------------------------------(dist3)" + String(dist3));

      if (dist3 > THRESHOLD_DIST && !state1)
      {
        state1 = false; state2 =  false; state3 = false; state4 = false; state5 = true; state6 =  false;
      }

      if (dist3  < THRESHOLD_DIST && !state1)
      {
        state1 = false; state2 =  false; state3 = false; state4 = false; state5 = false; state6 =  true;
      }
    }
  }


  if (state5 == true)
  {
    servo.write(lookAhead);
    right();
    unsigned long currentMillis5 = millis();
    reset5 = false;

    //for debugging
    Serial.println("lookAhead + move right____________________state5");

    if (currentMillis - prevMillis5 >= PAUSE_INTERVAL)
    {
      state5 = false;
      reset5 = true;
      state1 = true;

      //for debugging
      Serial.println("left state 5 to state 1");
    }
  }


  if (state6 == true)
  {
    back();
    servo.write(lookAhead);
    unsigned long currentMillis6 = millis();
    reset6 = false;

    //for debugging
    Serial.println("moveBackward + lookAhead  ___________________state6");

    if (currentMillis6 - prevMillis6 >= PAUSE_INTERVAL)
    {
      state6 = false;
      reset6 = true;
      state2 = true;

      //for debugging
      Serial.println("left  state 6 to state 2 to scan again and see where to go");
    }
  }


  if (reset1)
    prevMillis1 = millis();

  if (reset2)
    prevMillis2 = millis();

  if (reset3)
    prevMillis3 = millis();

  if (reset4)
    prevMillis4 = millis();

  if (reset5)
    prevMillis5 = millis();

  if (reset6)
    prevMillis6 = millis();




  else
  {
    state1 = true; state2 =  false; state3 = false; state4 = false; state5 = false; state6 =  false;
    dist1 = obstacle();
    Serial.println("the else statement has happened");
  }


  //for debugging
  Serial.println("nothing happening here-----------------------------------------------------(dist1)" + String(dist1));
}//END Autodrive


void detectLight()
{
  int lightIntensity = analogRead(ldrPin);
  currentBuzzMillis = millis();

  if (resetBuzz)
  {
    prevBuzzMillis = millis();
  }


  if (lightIntensity > LIGHT_THRESHOLD)
  {
    if (detectEdge() < EDGE_DIST)
    {
      front();
    }
    resetBuzz = false;

    if (currentBuzzMillis - prevBuzzMillis >= BUZZ_INTERVAL)
    {
      digitalWrite (buzzerPin, LOW);
      resetBuzz = true;

      //for debugging
      Serial.println("resetBUZZZZZ_________________________________________________________________________");
    }

    else
    {
      digitalWrite (buzzerPin, HIGH);
    }

    //for debugging
    Serial.println("detected light ahead_____________________________________________________moving forward");

  }

  else {
    Stop();
    digitalWrite (buzzerPin, LOW);

    //for debugging
    Serial.println("Stopped because light is not up to threshold or at an edge");
  }
}


void follower()
{
 int distObst = obstacle();

  if (distObst < 100 && distObst > 30) {
    //This condition is to move forward the robot when the object is in the range of 100 to 30 centimeters.
    front();
    Serial.println("condition1");
  }

  if (distObst < 30 && distObst > 20) {
    //This condition is to make the robot stable when the object is in the range of 20 to 30 centimeters.
    Stop();
    Serial.println("condition2");
  }

  if (distObst < 20 && distObst > 2) {
    //This condition is to move backward the robot when the object is in the range of 2 to 20 centimeters.
    back();
    Serial.println("condition3");
  }

  if (distObst > 100) {
    //This condition is to stop the robot when the object is in the out of range i.e greater than 100 centimeters.
    Stop();

    Serial.println("condition4");
  }
//
//  else  
//  {
//    Stop();
//    Serial.println("conditionELSE");
//  }
}// END Follower

//motor control functions
void runLeftMotor(int LMdirection, int LMpower)
{
  leftMotorDirection  = LMdirection ;
  leftMotorPower      = LMpower ;

  if (leftMotorDirection == DIRECTION_FORWARD)
  {
    digitalWrite(LEFT_MOTOR_DIRECTION_PIN, HIGH);
    analogWrite(LEFT_MOTOR_POWER_PIN, (255 - leftMotorPower));
  }
  else
  {
    digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
    analogWrite(LEFT_MOTOR_POWER_PIN,  leftMotorPower );
  }
}
//END RunLeftMotor

void stopLeftMotor()
{
  digitalWrite(LEFT_MOTOR_DIRECTION_PIN, LOW);
  analogWrite(LEFT_MOTOR_POWER_PIN, 0);
}
//END stopLeftMotor


void runRightMotor(int RMdirection, int RMpower)
{
  rightMotorDirection  = RMdirection ;
  rightMotorPower      = RMpower ;

  if (rightMotorDirection == DIRECTION_FORWARD)
  {
    digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, HIGH);
    analogWrite(RIGHT_MOTOR_POWER_PIN, (255 - rightMotorPower));
  }
  else
  {
    digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
    analogWrite(RIGHT_MOTOR_POWER_PIN,  rightMotorPower );
  }
}
//END RunrightMotor

void stopRightMotor()
{
  digitalWrite(RIGHT_MOTOR_DIRECTION_PIN, LOW);
  analogWrite(RIGHT_MOTOR_POWER_PIN, 0);
}
//END stoprightMotor

//Interrupt Service Routines
void countpulse_left() {
  counter_left++;
}

void countpulse_right() {
  counter_right++;
}

//*********( THE END )***********
