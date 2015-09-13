#include "Controller.h"

// 2008-09-04: begin fix because the right eye encoder is broken
#define RIGHT_EYE_BROKEN 0
#define DEFAULT_RIGHT_VERG 0
// end fix

using namespace yarp::os;
using namespace yarp::sig;

Controller::Controller(yarp::os::ConstString name)
: Device(name) // call superclass constructor
{
}

void Controller::setLV(float a)
{
	lv = a;
}

void Controller::setRV(float a)
{
	if(RIGHT_EYE_BROKEN) // force rv to zero
		rv = DEFAULT_RIGHT_VERG;
	else
		rv = a;
}

void Controller::setP(float a)
{
	p = a;
}

void Controller::setT(float a)
{
	t = a;
}

void Controller::update_angles(Vector *data)
{
	if (data == NULL)
		return;
	setLV( (float) (*data)[0] );
	setRV( (float) (*data)[1] );
	setT( (float) (*data)[2] );
	setP( (float) (*data)[3] );
}

float Controller::getLV()
{
	return lv;
}

float Controller::getRV()
{
	return (float) rv;
}

float Controller::getP()
{
	return (float) p;
}

float Controller::getT()
{
	return (float) t;
}