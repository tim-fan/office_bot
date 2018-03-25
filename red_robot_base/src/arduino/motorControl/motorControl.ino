// Sketch produces an interface for controlling robot motors, over rosserial

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;

int leftMotorEnablePin = 11;
int leftMotorCtrlPin1 = A5;
int leftMotorCtrlPin2 = 12; 
int rightMotorEnablePin = 10;
int rightMotorCtrlPin1 = 6;
int rightMotorCtrlPin2 = 3;

void setMotorSpeed(float motorSpeed, int ctrlPin1, int ctrlPin2, int enablePin){
  
    digitalWrite(13, HIGH-digitalRead(13));   // blink the led

    //map from desired speed to pwm value
    //from rough test, motor spins at 16 rad/sec
    //when pwm is fully on (255)
    int pwmVal = map(abs(motorSpeed), 0, 16, 0, 255);
    pwmVal = constrain(pwmVal, 0, 255);

    //ctrl pins set motor direction
    bool ctrl1 = true;
    bool ctrl2 = false;
    if (motorSpeed < 0) {
        ctrl1 = !ctrl1;
        ctrl2 = !ctrl2;
    }

    digitalWrite(ctrlPin1, ctrl1);
    digitalWrite(ctrlPin2, ctrl2);
    analogWrite(enablePin, pwmVal);
    //digitalWrite(enablePin, pwmVal > 100);
}

void setLeftMotorSpeed( const std_msgs::Float32& motorSpeed){
  setMotorSpeed(motorSpeed.data, leftMotorCtrlPin1, leftMotorCtrlPin2, leftMotorEnablePin);
}

void setRightMotorSpeed( const std_msgs::Float32& motorSpeed){
  setMotorSpeed(motorSpeed.data, rightMotorCtrlPin1, rightMotorCtrlPin2, rightMotorEnablePin);
}


ros::Subscriber<std_msgs::Float32> subLeft("left_motor_speed", setLeftMotorSpeed );
ros::Subscriber<std_msgs::Float32> subRight("right_motor_speed", setRightMotorSpeed );

void setup()
{

  pinMode(leftMotorEnablePin, OUTPUT);
  pinMode(leftMotorCtrlPin1, OUTPUT);
  pinMode(leftMotorCtrlPin2, OUTPUT);
  pinMode(rightMotorEnablePin, OUTPUT);
  pinMode(rightMotorCtrlPin1, OUTPUT);
  pinMode(rightMotorCtrlPin2, OUTPUT);

  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.subscribe(subLeft);
  nh.subscribe(subRight);
}

void loop()
{
  nh.spinOnce();
  delay(50);
}
