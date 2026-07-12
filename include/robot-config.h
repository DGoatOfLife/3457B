using namespace vex;
#include "JAR-Template/PID.h"

extern brain Brain;
extern motor_group Lift;

// To set up a motor called LeftFront here, you'd use
// extern motor LeftFront;
extern rotation LiftRot;
extern double liftTarget;
extern double target;
extern PID liftPID;
extern bool useTarget;
extern double HOME;
// Intake Ready Pos
extern double A;
// Auton pos
extern double Auton;
// Non-Middle goals.
extern double B;
extern double C;
extern double D;
// Middle goal.
extern double E;
extern double F;
// Matchloading pos
extern double G;


// Add your devices below, and don't forget to do the same in robot-config.cpp:

extern motor_group Scoring;

void setLiftTarget(double target);

void  vexcodeInit( void );