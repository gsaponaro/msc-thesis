#include "Camera.h"
#include "TwoDObject.h"

void TwoDObject::setPt1(CvPoint p)
{
	pt1 = p;
}

void TwoDObject::setPt2(CvPoint p)
{
	pt2 = p;
}

void TwoDObject::setAngle(float a)
{
	angle = a;
}

CvPoint TwoDObject::getPt1()
{
	return pt1;
}

CvPoint TwoDObject::getPt2()
{
	return pt2;
}

float TwoDObject::getAngle()
{
	return angle;
}

void TwoDObject::synch_points(Camera cam)
{
	// this is a blocking cycle to be used for debugging
	while(true) {
		Vector *text = cam.Camera::read(true);
		if (text == NULL)
			return;
		setPt1(cvPoint( (int) (*text)[0], (int) (*text)[1] ));
		setPt2(cvPoint( (int) (*text)[2], (int) (*text)[3] ));
		setAngle( (float) (*text)[4] );
	}
}

void TwoDObject::update_points(Vector *data)
{
	if (data == NULL)
		return;
	setPt1(cvPoint( (int) (*data)[0], (int) (*data)[1] ));
	setPt2(cvPoint( (int) (*data)[2], (int) (*data)[3] ));
	setAngle( (float) (*data)[4] );
}