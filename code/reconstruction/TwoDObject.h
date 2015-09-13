#ifndef TWODOBJECT_H
#define TWODOBJECT_H

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <cv.h>

#include "Camera.h"

using namespace yarp::os;
using namespace yarp::sig;

class TwoDObject
{
	CvPoint pt1, pt2; // extremities of the major axis of the ellipse
	float angle;
public:
	void setPt1(CvPoint p);
	void setPt2(CvPoint p);
	void setAngle(float a);
	void synch_points(Camera cam);
	void update_points(Vector *data);
	CvPoint getPt1();
	CvPoint getPt2();
	float getAngle();
};

#endif // TWODOBJECT_H