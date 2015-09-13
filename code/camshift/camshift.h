#ifndef CAMSHIFT_H
#define CAMSHIFT_H

#include <ace/OS.h>
#include <yarp/String.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Property.h>
#include <yarp/os/Terminator.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/Vector.h>

#include <cv.h>
#include <highgui.h>

using namespace yarp::os;
using namespace yarp::sig;

int camshift(int argc, char *argv[]);
void on_mouse( int event, int x, int y, int flags, void* param );
CvScalar hsv2rgb( float hue );
void name_ports(yarp::os::ConstString port_prefix, yarp::os::ConstString obj_name, yarp::os::ConstString cam,
				BufferedPort<ImageOf<PixelRgb> > &inPort,
				BufferedPort<ImageOf<PixelMono> > &outPort,
				BufferedPort<Bottle> &inRoiPort,
				BufferedPort<Vector> &outRoiPort,
				BufferedPort<Vector> &outObjPort,
				Terminee **terminee);

#endif // CAMSHIFT_H