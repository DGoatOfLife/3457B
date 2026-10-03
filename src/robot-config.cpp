#include "vex.h"
#include "Lift.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller;

Lift lift = Lift();

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);


//Add your devices below, and don't forget to do the same in robot-config.h:
motor_group Scoring = motor_group(ScoringMotorA, ScoringMotorB);
motor leftMotorA = motor(PORT10, ratio6_1, true);
motor leftMotorB = motor(PORT11, ratio6_1, true);
motor rightMotorA = motor(PORT3, ratio6_1, false);
motor rightMotorB = motor(PORT20, ratio6_1, false);
motor Preload = motor(PORT7, ratio18_1, false);
motor ScoringMotorA = motor(PORT21, ratio6_1, true);
motor ScoringMotorB = motor(PORT19, ratio6_1, false);

void vexcodeInit( void ) {
  // nothing to initialize
}