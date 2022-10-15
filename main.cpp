#include "main.h"
#include <vector>
#include <array>
#include <iostream>
using namespace std;
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	driveLeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightBack.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveLeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveRightFront.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intakeLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	intakeRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	angler.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

	pros::ADIGyro gyro('B',0.91);
	pros::delay(2000);





}


/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void topLeftCorner(){
	translate(-500,100);
}

pros::Rotation rotation_sensor(12);
pros::Rotation rotation_sensor2(11);
pros::Rotation rotation_sensor3(10);
array<double,3> odometry(double absx, double absy, double robotAngle){

	// odometry uses robot's first starting point as origin.
	// field will use new coordinate system where the robot facing directly up is theta = 0, robot facing directly down is theta = 180, and robot facing directly left is theta = 270.
	int rotation1 = rotation_sensor2.get_position();
	int rotation2 = rotation_sensor.get_position();
	int rotation3 = rotation_sensor3.get_position();

	double left_arc = rotation1 * (3.141592653589793238462643383279502884197*2.75);
	double right_arc = rotation2 * (3.141592653589793238462643383279502884197*2.75);
	double horizontal_arc = rotation3 * (3.141592653589793238462643383279502884197*2.75);

	double tr = 4.75; //inches for all
	double tl = 4.75;
	double tb = 6;

	double angle = (left_arc - right_arc)/(tr + tl);//angle is only change in robot's global angle. We must add the change to the robot's global angle.
	double difference = angle - robotAngle;

	double x,y;
	if (left_arc == right_arc){
		x = horizontal_arc;
		y = left_arc;
	}
	else{
		y = 2*((right_arc/difference)+tr)*(sin(angle/2));
		x = 2*((horizontal_arc/difference)+tb)*(sin(angle/2));
	}
	double avg = robotAngle + difference/2;

	double r = sqrt(pow(x,2)+pow(y,2));
	double theta = atan(y/x) + (avg*-1);

	double diffx = r*cos(theta);
	double diffy = r*sin(theta);
	std:: array<double,3> array;
	robotAngle = angle;
	array[0] = diffx;
	array[1] = diffy;
	array[2] = robotAngle;
	return array;
}
array<double,3> goToPosition(double destx, double desty, double curx, double cury, double curAngle){
	double a = destx - curx;
	double b = desty- cury;
	double angle; // angle that the robot needs to turn to
	if (a > 0){
		angle = 90 - atan(b/a); //had a hard time deriving these formulas for the angle to turn to
	}
	else if (a < 0){
		angle = 270 - atan(b/a); //had a hard time deriving these formulas for the angle to turn to
	}
	//check how much robot needs to turn from the right
	double rightTurn = angle - curAngle;
	if (rightTurn < 0) rightTurn += 360;
	//check how much robot needs to turn from the left
	double leftTurn = curAngle-angle;
	if (leftTurn < 0) leftTurn += 360;
	if(rightTurn <= leftTurn){
		setDrive(5,-5);
		while (true){
			double difference = angle-curAngle;
			if (difference < 0) difference += 360;
			if (difference < 3){ //robot's angle doesn't have to match the desired angle
				//because program takes several ms to run each line, during that time, the robot's angle might equal the desired angle
				// but by the time program checks it, it is missed; the angle will be slightly greater or less than the desired angle
				break;
			}
			std:: array<double,3> newAngle = odometry(curx,cury,curAngle);
			curx += newAngle[0];
			cury += newAngle[1];
			curAngle = newAngle[2];
		}
	}
	else{
		setDrive(-5,5);
		while (true){
			double difference = curAngle-angle;
			if (difference < 0) difference += 360;
			if (difference < 3){
				// same principle as above
				break;
			}
			std:: array<double,3> newAngle = odometry(curx,cury,curAngle);
			curx += newAngle[0];
			cury += newAngle[1];
			curAngle = newAngle[2];
		}
	}
	setDrive(2,2);
	while (true){ //use distance formula to calculate distance from destination
		std:: array<double,3> pos = odometry(curx,cury,curAngle);
		curx += pos[0];
		cury += pos[1];
		curAngle = pos[2];
		double distance = sqrt(pow(destx-curx,2)+pow(desty-cury,2));
		if (distance < 1){ //because of the offset created while turning, robot might miss destination by a couple units.
			//maximum margin of error is 1
			break;
		}
	}
	setDrive(0,0); //stop robot
	std:: array<double,3> array;
	array[0] = curx;
	array[1] = cury;
	array[2] = curAngle;
	return array;
}
double x,y, robotAngle = 0; // when game starts, robot is at 0,0 and has an angle of 0. This is all relative to its starting positio
void autonomous() {
	std:: array<double,3> pos = goToPosition(20,20,x,y,robotAngle);
	x = pos[0];
	y = pos[1];
	robotAngle = pos[2];

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	while (true){
		setDriveMotors();
		setIntakeMotors();
		setLiftMotor();
		setAnglerMotor();

		pros::delay(10);

	}
}
