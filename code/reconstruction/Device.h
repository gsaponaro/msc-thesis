#ifndef DEVICE_H
#define DEVICE_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ConstString.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

class Device
{
	yarp::os::ConstString name;
	BufferedPort<Vector> port;

public:
	
	Device(yarp::os::ConstString name);
	~Device();
	void setName(yarp::os::ConstString name);
	yarp::os::ConstString getName() const;
	bool open(yarp::os::ConstString name);
	void close();
	Vector *read(bool shouldWait);
};

#endif // DEVICE_H