/*-----( Declare Constants  )-----*/
#define DIRECTION_FORWARD 0  // For use with motor controls
#define DIRECTION_REVERSE 1
#define LOOP_TIME 100       // Refresh time to apply the closed-loop control signal to the motor. The frequency of the system is about 10Hz (f = 1/T, T = 0.1s)
#define LOWER_LIMIT 3       // Speed lower limit
#define UPPER_LIMIT 4       // Speed upper limit 

/*-------------( Declare  Pin Numbers )-----------------------*/
const byte LEFT_OPTOINTERRUPTER_PIN  =    2;  // The optoInterrupters for encoder discs
const byte RIGHT_OPTOINTERRUPTER_PIN =    3;

volatile unsigned int counter_left;
volatile unsigned int counter_right;

const byte LEFT_MOTOR_DIRECTION_PIN   =  4 ; // Motor Control Pins Direction and PWM Speed
const byte LEFT_MOTOR_POWER_PIN       =  5 ;
const byte RIGHT_MOTOR_POWER_PIN      =  6 ;
const byte RIGHT_MOTOR_DIRECTION_PIN  =  7 ;

int leftMotorDirection  = 0;  // Forward zero power
int leftMotorPower      = 0;

int rightMotorDirection = 0;
int rightMotorPower     = 0;

unsigned long currentMillis = 0;   // variable to keep track of current time
unsigned long previousMillis = 0;  // variable to keep track of previous time


static byte countSpeedLeft = 0;     // variable to store count speed of left motor in counts per 0.1s
static byte countSpeedRight = 0;    // variable to store count speed of right motor in counts per 0.1s

static byte powerLeft = 127;        // power to be applied to the left motor
static byte powerRight = 127;       // power to be applied to the right motor


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("TESTING TESTING ");

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

}

void loop() {
  // put your main code here, to run repeatedly:

  runRightMotor(DIRECTION_FORWARD, powerRight);
  runLeftMotor(DIRECTION_FORWARD, powerLeft);
  currentMillis = millis();


  //  runLeftMotor(DIRECTION_REVERSE, 100);
  //  runRightMotor(DIRECTION_REVERSE, 100);

  //  Serial.print("LEFT_COUNT is "  + String(counter_left) + '\n');
  //    Serial.println("RIGHT_COUNT is "  + String(counter_right) + '\n');


  if (currentMillis - previousMillis >= LOOP_TIME)
  {
    countSpeedLeft = counter_left;
    countSpeedRight = counter_right;

    counter_left = 0;
    counter_right = 0;

    //increase speeds to meet LOWER_LIMIT
    if (countSpeedLeft < LOWER_LIMIT)
    {
      powerLeft += 2;
    }

    if (countSpeedRight < LOWER_LIMIT)
    {
      powerRight += 2;
    }

    // increase speeds to reach upper limit
    if (countSpeedLeft > UPPER_LIMIT)
    {
      powerLeft -= 2;
    }

    if (countSpeedRight > UPPER_LIMIT)
    {
      powerRight -= 2;
    }

    //Set MAX power to be 254
    if (powerLeft > 254)
    {
      powerLeft = 254;
    }

    if (powerRight > 254)
    {
      powerRight = 254;
    }

    //set MIN power to be 0
    if (powerLeft <= 0)
    {
      powerLeft = 0;
    }

    if (powerRight <= 0)
    {
      powerRight = 0;
    }

    previousMillis = currentMillis;  // reset previousMillis

    //debugging
    Serial.println("powerLeft: " + String(powerLeft));
    Serial.println("powerRight: " + String(powerRight));
    Serial.println("SpeedLeft: "  + String(countSpeedLeft) + " count/0.1s");
    Serial.println("SpeedRight: "  + String(countSpeedRight) + " count/0.1s" + '\n');
  }
}


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

void countpulse_left() {
  counter_left++;
}

void countpulse_right() {
  counter_right++;
}
