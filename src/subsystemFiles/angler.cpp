#include "main.h"

pros::ADIPotentiometer anglerPot('A');

void setAngler(int power){
  angler = power;
}

void setAnglerMotor(){
  int anglerPower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN));
  setAngler(anglerPower);
}
