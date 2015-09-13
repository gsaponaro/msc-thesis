#include "Device.h"

using namespace yarp::os;

Device::Device(yarp::os::ConstString name) // constructor
{
	setName(name);
}

Device::~Device() // destructor
{
}


void Device::setName(yarp::os::ConstString n)
{
	name = n;
}


yarp::os::ConstString Device::getName() const
{
	return name;
}

bool Device::open(yarp::os::ConstString name)
{
	return port.BufferedPort<Vector>::open(name.c_str());
}

void Device::close()
{
	port.BufferedPort<Vector>::close();
}

Vector *Device::read(bool shouldWait)
{
	return port.BufferedPort<Vector>::read(shouldWait);
}
