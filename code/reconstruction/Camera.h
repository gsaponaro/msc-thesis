// simple subtype of Device. not to be confused with <iCub/camera.h>

#ifndef CAMERA_H
#define CAMERA_H

#include <ace/OS.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "Device.h"

using namespace yarp::os;
using namespace yarp::sig;

class Camera : public Device
{
public:
	Camera(yarp::os::ConstString name);
	void print();
};

#endif // CAMERA_H