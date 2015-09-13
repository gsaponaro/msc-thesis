// Manuel Lopes' API

#include "HandActuator.h"
#include <stdio.h>
//#include <conio.h>




HandActuator::HandActuator()
{

//-----------Initialization requirements for motor control ----------------------------
	Network::init();
	Time::turboBoost();
	
	if (!options.check("device")) {
		ACE_OS::printf("Adding device remote_controlboard\n");
		options.put("device", "remote_controlboard");
	}

	//yarp::String s("");	s += "/controlboard"; options.put("local", s.c_str());
	//s.clear();	s += "/controlboard";	options.put("remote", s.c_str());

	yarp::String s("");	s += "/hand"; options.put("local", s.c_str());
	s.clear();	s += "/BaltaHand";	options.put("remote", s.c_str());


    // create a device to connect to the remote robot server.
    //PolyDriver dd(options);
		dd.open(options);
    if (!dd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();        
    }


    bool ok;
    ok = dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);

    if (!ok) {
        ACE_OS::printf("Problems acquiring interfaces\n");
     }
//-----------------------------------------------------------------------------


	int jnts = 0;
    pos->getAxes(&jnts);
	ACE_OS::printf("HandActuator: Working with %d axes\n", jnts);

}

HandActuator::~HandActuator()
{
	double position[2];
	position[0]=0.0;
	position[1]=0.0;
	

}


bool HandActuator::close_board()
{
	
	pos->setRefSpeed(0,170);
	pos->setRefSpeed(1,170);
	pos->setRefSpeed(2,170);
	pos->setRefSpeed(3,170);	
	pos->positionMove(0,0);
	pos->positionMove(1,0);
	pos->positionMove(2,0);
	pos->positionMove(3,0);
	   
    
    dd.close();
    Network::fini();

	return true;
}


int HandActuator::moveto(int joint, double position)
{
	pos->setRefSpeed(joint,170);
	pos->positionMove(joint, position*180/Pi);
		
	return 1;
}


bool HandActuator::getEncoderValue(int joint, double reading[1]){

	double *temp = new double[1];
	ACE_OS::memset(temp, 0, sizeof(double)*1);
	enc->getEncoder(joint, temp);
	reading[0]=temp[0]*Pi/180;

	return true;
	
}


//Note working on the device driver level
bool HandActuator::getEncodersValues(double readings[6]){


	double *q = new double[6];
	ACE_OS::memset(q, 0, sizeof(double)*6);
	enc->getEncoders(q);
	printf("\n\n\nencoders: ->%f    ->%f\n\n", q[0], q[1]);

	readings[0]=q[0]*Pi/180;
	readings[1]=q[1]*Pi/180;
	readings[2]=q[2]*Pi/180;
	readings[3]=q[3]*Pi/180;
	
	return true;
}

int HandActuator::move(int joint, double speed)
{

	vel->velocityMove(joint, speed*180/Pi);
	
	return 1;
}




