#ifndef REACHING_H
#define REACHING_H

#define MAX_SOL 50	// for inverse kinematics solver

// Manuel Lopes' APIs:
#include "ArmActuator.h"
#include "grasp.h"
#include "HandActuator.h"
#include "kinpronator.h"

#include <ace/OS.h>
#include <yarp/String.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <cv.h>		// definition of CV_PI

#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

using namespace yarp::os;
using namespace yarp::sig;

int cininv(double *pos, double solutions[6][MAX_SOL], float n1, float n2, float n3);
void min_energy(int num_sols, double solutions[6][MAX_SOL], double joints[6]);
void dkinematics (double *juntas, int njoint, double res[3]);

class GraspingWithPFModule : public yarp::os::Module
{
private:
	// name of port where we receive the target pose from stereo vision module
	yarp::String target_port_name;

	int safety_threshold;	// thresholds in cm, so not to touch the object
	float sho_to_vis;		// horizontal offset between shoulder and vision ref. frames (in metres)

	ArmSkills arm;

	// variables used to control the evolution of the reaching FSM
	yarp::os::Semaphore _mutex;
	bool interact_flag;


public:
	virtual bool open(yarp::os::Searchable &config);
	virtual bool updateModule();
	virtual bool close();
	virtual bool respond(const Bottle &command, Bottle &reply);

};

#endif // REACHING_H