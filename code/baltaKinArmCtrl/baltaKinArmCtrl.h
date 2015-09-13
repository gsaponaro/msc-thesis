#ifndef BALTAKINARMCTRL_H
#define BALTAKINARMCTRL_H

#include <ace/OS.h>
#include <yarp/String.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <cv.h>		// definition of CV_PI

#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>

using namespace yarp::os;
using namespace yarp::sig;


class BaltaKinArmCtrlModule : public yarp::os::Module
{
private:


public:

};

#endif // BALTAKINARMCTRL_H