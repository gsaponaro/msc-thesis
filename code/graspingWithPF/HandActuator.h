#ifndef HANDACTUATOR_H
#define HANDACTUATOR_H

#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Terminator.h>
#include <yarp/String.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>


using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;


#define VOCAB_HELP VOCAB4('h','e','l','p')
#define VOCAB_QUIT VOCAB4('q','u','i','t')

#define Pi 3.1415927


class HandActuator
{

	private: 
		Property options;
		PolyDriver dd;
		
		IVelocityControl *vel;
		IEncoders *enc;
		IPidControl *pid;
		IAmplifierControl *amp;
		IControlLimits *lim;	

	public:
		IPositionControl *pos;

		HandActuator();
		~HandActuator();
		
		//velocidade em rad/s
		int move(int joint, double speed);
		//@position units -  rads
		int moveto(int joint, double position);

		//@reading units - rads
		bool getEncoderValue(int joint, double reading[1]);
		bool getEncodersValues(double readings[4]);

		bool close_board();
		
};

#endif HANDACTUATOR_H