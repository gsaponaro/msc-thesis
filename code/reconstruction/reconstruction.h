#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H

#include <ace/OS.h>
#include <yarp/String.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

#include "Camera.h" // simple subtype of Device. not to be confused with <iCub/camera.h>
#include "Controller.h"
#include "Device.h"
#include "TwoDObject.h"

float deg2rad(float d);

class ReconstructionModule : public yarp::os::Module
{
private:
	yarp::String left_port_name, right_port_name, ctrl_port_name;	// names of input ports
	yarp::String recon_port_name;									// name of output port
	float cos_lv,		// cosine of left vergence
		  sin_lv,
		  cos_rv,
		  sin_rv,
		  cos_p,		// ...of pan
		  sin_p,
		  cos_t,		// ...of tilt
		  sin_t;
	float b,			// half of baseline
		  head_height,
		  head_depth;
	// cam parameters
	float F;			// focal distance
	int W;				// width of resolution
	int H;				// height of resolution
	float pixel_res;	// pixel size
public:
	virtual bool open(yarp::os::Searchable &config);
	virtual bool updateModule();
	virtual bool close();
};

#endif // RECONSTRUCTION_H