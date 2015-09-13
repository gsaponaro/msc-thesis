#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ace/OS.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Vector.h>

#include "Device.h"

using namespace yarp::os;
using namespace yarp::sig;

class Controller : public Device
{
	// 2008-06-26: corrected the order of tilt (3rd value) and pan (4th)
	// junction angles: left vergence, right vergence, tilt, pan
	float lv, rv, t, p;

public:
	Controller::Controller(yarp::os::ConstString name);
	void setLV(float a);
	void setRV(float a);
	void setP(float a);
	void setT(float a);
	void update_angles(Vector *data);
	float getLV();
	float getRV();
	float getP();
	float getT();
};

#endif // CONTROLLER_H