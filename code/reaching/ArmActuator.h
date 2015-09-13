#ifndef ARMACTUATOR_H
#define ARMACTUATOR_H

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


class ArmActuator
{

	private: 
		Property options;
		PolyDriver dd;
		IPositionControl *pos;
		IVelocityControl *vel;
		IEncoders *enc;
		IPidControl *pid;
		IAmplifierControl *amp;
		IControlLimits *lim;
		double pan;
		double tilt;

	public:
		ArmActuator();
		~ArmActuator();
		
		// Set accelerations of all joints
		int setAccelerations(double acc);

		int setJointVel(int joint, double speed);

		//velocidade em rad/s
		int move(int joint, double speed);
		//@position units -  rads
		int moveto(int joint, double position);
		int moveto(int joint, double position, double speed);
		int moveto(int joint, double position, double speed, double acc);


		//@reading units - rads
		bool getEncoderValue(int joint, double reading[1]);
		bool getEncodersValues(double readings[6]);

		bool close_board();
		
};

#endif ARMACTUATOR_H