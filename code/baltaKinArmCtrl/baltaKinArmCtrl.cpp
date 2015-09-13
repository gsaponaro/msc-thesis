#include "baltaKinArmCtrl.h"

using namespace yarp::os;
using namespace yarp::sig;

bool BaltaKinArmCtrlModule::open(yarp::os::Searchable &config)
{

	return true;
}

bool BaltaKinArmCtrlModule::updateModule()
{
	// until this function returns false, it will be called forever by runModule()


	return true;
}

bool BaltaKinArmCtrlModule::close()
{
	ACE_OS::printf("\nClosing baltaKinArmCtrl module...\n");


	// is this necessary?
	//Network::fini();

	return true;
}
