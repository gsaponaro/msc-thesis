#include <ace/OS.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include "graspingWithPF.h"

using namespace yarp::os;

int main(int argc, char *argv[])
{
	// create a specific Network object, rather than using Network::init().
	// this way, automatic cleanup by the means of Network::fini() will be smoother
	Network network;
	Time::turboBoost();

	GraspingWithPFModule g;
	return g.runModule(argc, argv);
}