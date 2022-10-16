#include "main.h"

pros::ADIGyro gyro('B',0.91);
void setDrive(int left, int right){
  driveLeftBack = left;
  driveLeftFront = left;
  driveRightBack = right;
  driveRightFront = right;
}
void setDriveMotors() {
  int leftJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int rightJoystick = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  if (abs(leftJoystick) < 10){
    leftJoystick = 0;
  }
  if (abs(rightJoystick) < 10){
    rightJoystick = 0;
  }

  setDrive(leftJoystick,rightJoystick);
}

void resetDriveEncoders(){
    driveLeftBack.tare_position();
    driveLeftFront.tare_position();
    driveRightBack.tare_position();
    driveRightFront.tare_position();

}
double avgDriveEncoderValue(){
  return (fabs(driveLeftFront.get_position()) + fabs(driveLeftBack.get_position()) + fabs(driveRightFront.get_position()) + fabs(driveRightBack.get_position()))/4;
}
void translate(int units, int voltage){
  int direction = abs(units);
  resetDriveEncoders();
  gyro.reset();

  while (avgDriveEncoderValue() < abs(units)){
    setDrive(voltage*direction + gyro.get_value(),voltage*direction-gyro.get_value());
    pros::delay(10);
  }

  setDrive(-10*direction,-10*direction);
  pros::delay(50); //change depending on weight of robot
  setDrive(0,0);

}
