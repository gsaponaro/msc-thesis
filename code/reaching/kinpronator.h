
#include "iCub/kinematics/robmatrix.h"
#include "iCub/kinematics/auxfuncs.h"

int ikpronator(double *pos, double t3e, double *pl, double* th, RobMatrix *T);
int ikpospronator(double *pos, double t3e, double* th, RobMatrix *T);
int ikorientationpronator( double *pl, double* th, RobMatrix T04);
//double min( double a, double b);
