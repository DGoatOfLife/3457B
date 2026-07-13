#include "Lift.h"

int ROT_PORT = 16;

motor LiftMotorA = motor(PORT13, ratio36_1, true);
motor LiftMotorB = motor(PORT1, ratio36_1, false);

bool Lift::useTarget = false;
double Lift::liftTarget = Lift::HOME;

// Initialize sensors & motors
motor_group Lift::motors(LiftMotorA, LiftMotorB);
rotation Lift::liftRot(PORT16, true);
PID Lift::liftPID(0, 3, 0, 2, 0);
vex::thread Lift::liftThread = vex::thread(Lift::threadEntry, nullptr);

void Lift::threadEntry(void* object) {
    Lift* self = static_cast<Lift*>(object);
    return self->liftUpdateLoop();
}

void Lift::spin(
    vex::directionType dir,
    int pct,
    vex::percentUnits units
) {
    Lift::motors.spin(dir, pct, units);
}

void Lift::resetPosition() {
    Lift::liftRot.resetPosition();
}

void Lift::stop(vex::brakeType mode) {
    Lift::motors.stop(mode);
}

void Lift::liftUpdateLoop() {
    while (true) {
        double currentPos = liftRot.position(rev);
        double liftError = currentPos - liftTarget;
        double liftPower = -liftPID.compute(liftError) * 100;
        if (useTarget) {
            if (liftPower < 5 && liftPower > -5) {
                motors.stop(hold);
            } else {
                motors.spin(forward, liftPower, percent);
            }
        }

        wait(20, msec);
    }
}

void Lift::setLiftTarget(double target) {
    useTarget = true;
    liftTarget = target;
}

void Lift::handleControllerInput(
    bool L2_pressing,
    bool L1_pressing
) {
    if (L2_pressing) {
        Lift::useTarget = false;
        motors.spin(reverse, 85, percent);
    } else if (L1_pressing) {
        Lift::useTarget = false;
        motors.spin(forward, 65, percent);
    } else {
        if (!useTarget) {
            Lift::motors.stop(hold);
        }
    }
}