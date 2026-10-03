#include "vex.h"
#include "Lift.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  // kP 1 kD 0
  //
  chassis.set_drive_constants(12, 1, 0, 0, 0);
  // TODO: Not tuned.
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(3, 400, 4000);
  // TODO: Not tuned.
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void full_test(){
  // chassis.drive_distance(24);
  chassis.set_coordinates(0, 0, 0);
  chassis.drive_to_point(0, 24);
  // chassis.turn_to_angle(-45);
  // chassis.drive_distance(-36);
  // chassis.right_swing_to_angle(-90);
  // chassis.drive_distance(24);
  // chassis.turn_to_angle(0);
}

// ---------------------------------------

void setLiftTarget(double target) {
  Lift::setLiftTarget(target);
}
void Seft() {
  chassis.right_swing_to_angle(-18.08001438944787543);
  chassis.drive_distance(15);
  wait(1, sec);
  Preload.spinFor(forward, 600, msec);
  chassis.drive_distance(-3);
  chassis.left_swing_to_angle(90);
};

void Blue_Left_Odom() {
  // vex::thread coords = vex::thread(printCoords, nullptr);

  chassis.set_coordinates(0,0,0);
  Lift::spin(reverse, 30, percent);
  chassis.drive_to_point(0, 8.24);
  Lift::resetPosition();
  Lift::stop(hold);
  Scoring.spin(forward, 75, percent);
  wait(100, msec);
  setLiftTarget(Lift::D);
  wait(425, msec);
  Scoring.stop(coast);
  setLiftTarget(Lift::HOME);
  chassis.drive_to_point(0, 6);
  chassis.turn_to_point(-8.95, -9,82);
  chassis.drive_to_point(-8.95, -9.82);
  chassis.turn_to_point(-4.068, -7.42, 90);
  chassis.drive_to_point(-4.068, -7.42);
  chassis.drive_to_point(-8.95, -9.82);
  chassis.drive_to_point(-4.068, -7.42);
  chassis.turn_to_point(-13.91, 32.68);
  setLiftTarget(Lift::D);
  vex::wait(250, msec);
  Scoring.spinFor(reverse, 850, msec);
  chassis.drive_to_point(-13.91, 32.68);
  setLiftTarget(Lift::C);
  chassis.turn_to_point(-7.24, 26.95);
  chassis.drive_to_point(-7.24, 26.95);
}

void Blue_Left() {
  chassis.set_coordinates(0, 0, 133.34);
  Lift::spin(reverse, 30, percent);
  chassis.drive_distance(6.7865);
  Lift::resetPosition();
  Lift::stop(hold);
  Scoring.spin(forward, 75, percent);
  wait(100, msec);
  setLiftTarget(Lift::D);
  wait(425, msec);
  Scoring.stop(coast);
  chassis.drive_distance(-2.375);
  setLiftTarget(Lift::HOME);
  chassis.right_swing_to_angle(200);
  chassis.drive_distance(-2.4);
  chassis.left_swing_to_angle(90);
  chassis.drive_distance(8.7);
  chassis.drive_distance(-11.4);
  chassis.drive_distance(11.4);
  chassis.drive_distance(-11.4);
  chassis.drive_distance(18);
  chassis.left_swing_to_angle(157);
  setLiftTarget(Lift::D);
  chassis.drive_distance(15.125);
  Scoring.spin(reverse, 65, percent);
  setLiftTarget(Lift::A);
  wait(950, msec);
  Scoring.stop(coast);
  chassis.right_swing_to_angle(242);
  chassis.drive_distance(9.345);
  setLiftTarget(Lift::D);
  Scoring.spin(forward, 100, percent);
  wait(100, msec);
  setLiftTarget(Lift::C);
}

void Greft() {
  chassis.set_coordinates(0, 0, 0);
  Lift::spin(reverse, 30, percent);
  chassis.drive_distance(2.15);
  chassis.turn_to_point(-8, 11.69);
  chassis.drive_to_point(-8.17, 11.54);
  Lift::resetPosition();
  Lift::stop(hold);
  Scoring.spin(forward, 95, percent);
  setLiftTarget(Lift::D);
  wait(250, msec);
  Scoring.stop(coast);
  chassis.drive_distance(-9.345); 
  chassis.turn_to_angle(41.567); //works until here
  chassis.drive_distance(9.975);
  chassis.drive_stop(hold);
  Scoring.spin(reverse, 100, percent);
  setLiftTarget(Lift::A);
  wait(1, sec);
  Scoring.stop(coast);
  chassis.turn_to_point(-13, 24.5);
  setLiftTarget(Lift::D);
// chassis.drive_to_point(-13, 24.5);
// Scoring.spin(reverse, 75, percent);
// setLiftTarget(Lift::C);
//Scoring.stop(coast); //around 9 seconds after this point
}
// ------------------------------------


// void Blue_solo_awp() {
//   Scoring.spin(reverse, 2, percent);
//   chassis.drive_distance(4.72648);
//   chassis.drive_distance(-5);
//   chassis.drive_distance(4.6543);
//   chassis.drive_distance(-5);  
//   chassis.right_swing_to_angle(-60);
//   chassis.drive_distance(7.82);
//   Scoring.spin(forward, 60, percent);
//   wait(905, msec);
//   Scoring.stop(coast);
//   chassis.drive_distance(-8.25);
//   chassis.right_swing_to_angle(0);
//   chassis.drive_distance(17);
//   chassis.right_swing_to_angle(-60.25);
//   Lift.spin(forward, 60, percent);
//   chassis.drive_distance(11);
//   wait(245, msec);
//   Lift.stop(hold);
//   Lift.spin(reverse, 85, percent);
//   Scoring.spin(reverse, 100, percent);
//   wait(725, msec);
//   Scoring.stop(coast);
//   chassis.left_swing_to_angle(-141.5);
//   Lift.spin(forward, 35, percent);
//   wait(250, msec);
//   Lift.stop(hold);
//   chassis.drive_distance(11.5);
//   wait(350 ,msec);
//   Lift.spin(reverse, 85, percent);
//   wait(225, msec);
//   Scoring.spin(forward, 45, percent);
//   Lift.spin(forward, 65, percent);
//   wait(225, msec);
//   Lift.stop(hold);
// }

// void Red_solo_awp(){

// }

// /**
//  * The expected behavior is to return to the start position.
//  */

// void Blue_Right(){
//   chassis.right_swing_to_angle(-56);
//   chassis.drive_distance(6.69);
//   Scoring.spin(forward, 30, percent);
//   wait(650, msec);
//   Scoring.stop(coast);
// }

// void Red_Right(){

// }

// /**
//  * The expected behavior is to return to the start angle, after making a complete turn.
//  */

// void turn_test(){
//   chassis.turn_to_angle(5);
//   chassis.turn_to_angle(30);
//   chassis.turn_to_angle(90);
//   chassis.turn_to_angle(225);
//   chassis.turn_to_angle(0);
// }

// /**
//  * Should swing in a fun S shape.
//  */

// void swing_test(){
//   chassis.left_swing_to_angle(90);
//   chassis.right_swing_to_angle(0);
// }

// /**
//  * Doesn't drive the robot, but just prints coordinates to the Brain screen 
//  * so you can check if they are accurate to life. Push the robot around and
//  * see if the coordinates increase like you'd expect.
//  */

// void odom_test(){
//   chassis.set_coordinates(0, 0, 0);
//   while(1){
//     Brain.Screen.clearScreen();
//     Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
//     Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
//     Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
//     Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
//     Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
//   }
// }

// /**
//  * Should end in the same place it began, but the second movement
//  * will be curved while the first is straight.
//  */

// void tank_odom_test(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.turn_to_point(24, 24);
//   chassis.drive_to_point(24,24);
//   chassis.drive_to_point(0,0);
//   chassis.turn_to_angle(0);
// }

// /**
//  * Drives in a square while making a full turn in the process. Should
//  * end where it started.
//  */

// void holonomic_odom_test(){
//   odom_constants();
//   chassis.set_coordinates(0, 0, 0);
//   chassis.holonomic_drive_to_pose(0, 18, 90);
//   chassis.holonomic_drive_to_pose(18, 0, 180);
//   chassis.holonomic_drive_to_pose(0, 18, 270);
//   chassis.holonomic_drive_to_pose(0, 0, 0);
// }