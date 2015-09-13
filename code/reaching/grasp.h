// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ARMSKILLS__
#define __ARMSKILLS__

#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif
#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include "ArmActuator.h"
#include "HandActuator.h"
#include <iCub/kinematics/gsl_aux.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

// namespace iCub {
//     namespace contrib {
//         class demoAff;
//     }
// }

// using namespace iCub::contrib;

/**
 *
 * baltazar reaching and grasping routines class
 *
 * \see icub_demoaff
 *
 */
class ArmSkills {

private:

    int state; // Current state of the FSM
    ArmActuator *arm;
    HandActuator *hand;
    gsl_matrix* Lp;
    gsl_vector* V;
    gsl_vector* Vaux;
    gsl_vector* qdot;

    double prevHandPos[4];
    double prevArmPos[6];
public:

    ArmSkills();
    ~ArmSkills();

// Re-entrant functions. They implement an internal state machine that returns 1 when the operation is completed
// Sequential motion to go 
int GoHome();
// Visual servoing cycle
int VisualServoing(double xerr, double yerr, double height, char orient);
// Stop the arm
int StopArm();
// pre-programmed grasp
int Grasp(double wristrot);
int GraspVertical();
// pre-programmed ungrasp
int Ungrasp();
int UngraspVertical();
// pre-programmed tap
int Tap();
// pre-programmed untap
int FinalTap();

int Touch();
int FinalTouch();
int MoveArm(double *pos);
int MoveHand(double *posHand); 
int MoveHandVel(double *velHand);
    int MovingHand();
    int MovingArm();
};


#endif
