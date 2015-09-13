#include "Camera.h" // simple subtype of Device. not to be confused with <iCub/camera.h>

using namespace yarp::os;

Camera::Camera(yarp::os::ConstString name)
: Device(name) // call superclass constructor
{
}

void Camera::print()
{
	// this is a blocking cycle to be used for debugging
	while(true) {
		Vector *text = read(true);
		if (text == NULL)
			continue;
		ACE_OS::printf("%s pt1: %d %d, pt2: %d %d, angle: %f\n", getName().c_str(),
			(int) (*text)[0],
			(int) (*text)[1],
			(int) (*text)[2],
			(int) (*text)[3],
			(*text)[4]
		);
	}
}