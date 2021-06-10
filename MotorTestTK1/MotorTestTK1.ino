/* YourDuino Physical Computing Kit Tests
  - SEE the comments after "//" on each line below
  - CONNECTIONS:
   -
   -
  - V1.00 01/10/19
   Questions: terry@yourduino.com */

/*-----( Import needed libraries )-----*/

/*-------------( Declare  Pin Numbers )-----------------------*/
const byte LEFT_OPTOINTERRUPTER_PIN  =    2;  // The optoInterrupters for encoder discs
const byte RIGHT_OPTOINTERRUPTER_PIN =    3;

const byte LEFT_MOTOR_DIRECTION_PIN   =  4 ; // Motor Control Pins Direction and PWM Speed
const byte LEFT_MOTOR_POWER_PIN       =  5 ;
const byte RIGHT_MOTOR_POWER_PIN      =  6 ;
const byte RIGHT_MOTOR_DIRECTION_PIN  =  7 ;

const byte ULTRASONIC_ECHO_PIN        = 8 ;   //ultrasonic sensor parameters
const byte ULTRASONIC_TRIGGER_PIN     = 9 ;

const byte UNUSED10_PIN               = 10 ;  // Unused
const byte SERVO_PIN                  = 11 ;
const byte UNUSED12_PIN               = 12 ;  // Unused
const byte BUZZER_PIN                 = 13 ; //meant for obstacle alert

const byte LEFT_LINE_FOLLOW           = A0 ;   // PhotoSensors in Line Follower mode
const byte MIDDLE_LINE_FOLLOW         = A1 ;
const byte RIGHT_LINE_FOLLOW          = A2 ;

const byte LIGHT_SENSOR_PIN           = A3 ;   // For Light-following mode

// NOTE: Pins A4,A5 reserved for I2C communication (LCD etc.)

/*-----( Declare Constants  )-----*/
#define DIRECTION_FORWARD 0  // For use with motor controls
#define DIRECTION_REVERSE 1

/*-----( Declare objects )-----*/
/*-----( Declare Variables )-----*/
int leftMotorDirection  = 0;  // Forward zero power
int leftMotorPower      = 0;

int rightMotorDirection = 0;
int rightMotorPower     = 0;



void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(115200);
  Serial.println("TESTING TESTING ");
  pinMode(LEFT_LINE_FOLLOW, INPUT);
  pinMode(MIDDLE_LINE_FOLLOW, INPUT);
  pinMode(RIGHT_LINE_FOLLOW, INPUT);
  pinMode(LIGHT_SENSOR_PIN, INPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(ULTRASONIC_TRIGGER_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  pinMode(LEFT_MOTOR_DIRECTION_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_POWER_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_POWER_PIN , OUTPUT);
  pinMode(RIGHT_MOTOR_DIRECTION_PIN, OUTPUT);

}//--(end setup )---


void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
//  Serial.println("StartLoop ");
//  delay(8000);
  
//  Serial.println("Left motor Forward ");
//  runLeftMotor(DIRECTION_FORWARD, 100);
//  delay(5000);
  stopLeftMotor();
//  delay(1000);
//  Serial.println("Left motor Reverse ");
//  runLeftMotor(DIRECTION_REVERSE, 100);
//  delay(5000);
//  stopLeftMotor();
//  
//  Serial.println("Left motor STOP ");
//
//  Serial.println("right motor Forward ");
//  runRightMotor(DIRECTION_FORWARD, 100);
//  delay(5000);
  stopRightMotor();
//  delay(1000);
//  Serial.println("Right motor Reverse ");
//  runRightMotor(DIRECTION_REVERSE, 100);
//  delay(5000);
//  stopRightMotor();
//  
//  Serial.println("Right motor STOP ");
//  
//  Serial.println("EndLoop ");
//  delay(2000);
}//--(end main loop )---

/*-----( Declare User-written Functions )-----*/

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




//*********( THE END )***********
