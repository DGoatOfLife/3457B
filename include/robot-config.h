using namespace vex;
#include "JAR-Template/PID.h"

extern brain Brain;

// To set up a motor called LeftFront here, you'd use
// extern motor LeftFront;
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



extern motor leftMotorA;
extern motor leftMotorB;
extern motor rightMotorA;
extern motor rightMotorB;
extern motor Preload;
extern motor ScoringMotorA;
extern motor ScoringMotorB;
extern motor_group Scoring;

void setLiftTarget(double target);

void  vexcodeInit( void );