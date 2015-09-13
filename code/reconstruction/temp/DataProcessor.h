#ifndef DATAPROCESSOR_H
#define DATAPROCESSOR_H

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include "Camera.h"
#include "TwoDObject.h"

using namespace yarp::os;
using namespace yarp::sig;

class DataProcessor
: public TypedReaderCallback<Vector>
{
public:
     virtual void onRead(Vector &text);
};

#endif DATAPROCESSOR_H