#include "vex.h"
#include "Lift.h"

using namespace vex;
competition Competition;

controller Controller1 = controller(primary);

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

// TODO: Fix.
bool matchloading = false;

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftMotorA, leftMotorB),

//Right Motors:
motor_group(rightMotorA, rightMotorB),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT18,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.9,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
// 6, -> just to center of wheels
7.5, // to right side of dt

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
3,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  odom_constants();
  chassis.Gyro.calibrate();
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void printCoords() {
  chassis.set_coordinates(0, 0, 0);

  // calibrate gyro
  chassis.Gyro.calibrate();
  while (true) {
    Brain.Screen.clearScreen();
    if (chassis.Gyro.isCalibrating()) {
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Calibrating gyro...");
    } else {
      break;
    }
    
    vex::wait(20, msec);
  }

  while (true) {
    double x = chassis.get_X_position();
    double y = chassis.get_Y_position();
    double l_pos = chassis.DriveL.position(vex::deg);
    double r_pos = chassis.DriveR.position(vex::deg);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("x: %.2f", x);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("y: %.2f", y);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("l: %.2f", l_pos);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("r: %.2f", r_pos);
    vex::wait(20, msec);
  }
}

void autonomous() {
  Seft();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // vexcodeInit();
  // default_constants();

  // printCoords();

  // -------------------------------

  // LiftRot.resetPosition();

  vexcodeInit();
  default_constants();
  while (1) {
    // Adjust LIFT TARGETS
    Controller1.ButtonA.pressed([]() {
      Lift::setLiftTarget(Lift::A);
    });
    Controller1.ButtonB.pressed([]() {
      Lift::setLiftTarget(Lift::B);
    });
    Controller1.ButtonX.pressed([]() {
      Lift::setLiftTarget(Lift::C);
    });
    Controller1.ButtonY.pressed([]() {
      Lift::setLiftTarget(Lift::D);
    });
    Controller1.ButtonUp.pressed([]() {
      Lift::setLiftTarget(Lift::E);
    });
    Controller1.ButtonDown.pressed([]() {
      Lift::setLiftTarget(Lift::F);
    });
    
    Lift::handleControllerInput(
      Controller1.ButtonL2.pressing(),
      Controller1.ButtonL1.pressing()
    );

    // Controller1.ButtonDown.pressed([]{
    //   if (matchloading == false){
    //     matchloading = true;
    //   } else if (matchloading == true) {
    //     matchloading=false;
    //   }
    // });
    // if (matchloading==true){
    //   useTarget=true;
    //   setLiftTarget(G);
    //   Scoring.spin(reverse, 10, percent);
    //   chassis.drive_distance(3, chassis.get_absolute_heading(), 70, 127);
    //   wait(300, msec);
    //   Scoring.spin(reverse, 85, percent); 
    // }

    // TODO: Implement.
    // Controller1.ButtonLeft.pressed([] {
    //   LiftRot.resetPosition();
    // });

    // check the ButtonR1/ButtonR2 status to control Scoring
    if (Controller1.ButtonL1.pressing()) {
      Scoring.spin(forward, 100, percent);
    } else if (Controller1.ButtonL2.pressing()) {
      Scoring.spin(reverse,100, percent);
    } else {
      Scoring.spin(reverse,4.5, percent);
    }

    // // check the ButtonX/ButtonB status to control Toggle
    // if (Controller1.ButtonX.pressing()) {
    //   Toggle.spin(forward, 100, percent);
    // } else if (Controller1.ButtonB.pressing()) {
    //   Toggle.spin(reverse, 100, percent);
    // } else {
    //   Toggle.stop();
    // }

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
