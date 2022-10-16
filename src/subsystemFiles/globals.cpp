#include "main.h"

pros::Motor lift(1,pros::E_MOTOR_GEARSET_36,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor angler(2,pros::E_MOTOR_GEARSET_36,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intakeLeft(3,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor intakeRight(4,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveLeftBack(5,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveLeftFront(6,pros::E_MOTOR_GEARSET_18,false,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightFront(7,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor driveRightBack(8,pros::E_MOTOR_GEARSET_18,true,pros::E_MOTOR_ENCODER_COUNTS);


pros::Controller controller(pros::E_CONTROLLER_MASTER);
