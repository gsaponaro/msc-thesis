#include "ArmActuator.h"
#include <stdio.h>




ArmActuator::ArmActuator()
{

//-----------Initialization requirements for motor control ----------------------------
	Network::init();
	Time::turboBoost();


//	Property options;
	
	if (!options.check("device")) {
		ACE_OS::printf("Adding device remote_controlboard\n");
		options.put("device", "remote_controlboard");
	}

	yarp::String s("");	s += "/baltaArm"; options.put("local", s.c_str());
	s.clear();	s += "/baltaArm";	options.put("remote", s.c_str());


    // create a device to connect to the remote robot server.
    //PolyDriver dd(options);
	dd.open(options);
    if (!dd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();        
    }

    /*IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IPidControl *pid;
    IAmplifierControl *amp;
    IControlLimits *lim;*/

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
	ACE_OS::printf("ArmActuator: Working with %d axes\n", jnts);

}

ArmActuator::~ArmActuator()
{
	double position[2];
	position[0]=0.0;
	position[1]=0.0;
	

}


bool ArmActuator::close_board()
{
	/*
	pos->setRefSpeed(0,20);
	pos->setRefSpeed(1,20);
	//pos->setRefSpeed(2,20);
	//pos->setRefSpeed(3,20);
	pos->setRefSpeed(4,20);
	pos->setRefSpeed(5,20);
	pos->setRefSpeed(6,20);
	pos->setRefSpeed(7,20);
	
	pos->positionMove(0,0);
	pos->positionMove(1,0);
	//pos->positionMove(2,0);
	//pos->positionMove(3,0);
	pos->positionMove(4,0);
	pos->positionMove(5,0);
	pos->positionMove(6,0);
	pos->positionMove(7,0);
*/
        
    
    dd.close();
    Network::fini();

	return true;
}

int ArmActuator::setAccelerations(double acc)
{
	pos->setRefAcceleration( 0, acc);
	pos->setRefAcceleration( 1, acc);
	pos->setRefAcceleration( 4, acc);
	pos->setRefAcceleration( 5, acc);
	pos->setRefAcceleration( 6, acc);
	pos->setRefAcceleration( 7, acc);

	return 1;
}

int ArmActuator::setJointVel(int joint, double speed)
{
  pos->setRefSpeed(joint,speed);
  
  return 1;
}

int ArmActuator::moveto(int joint, double position, double speed, double acc)
{	
	pos->setRefAcceleration( joint, acc);
	pos->setRefSpeed(joint,speed);
	pos->positionMove(joint, position*180/Pi);
//	pos->setRefAcceleration( joint, 100);
	
	return 1;
}

int ArmActuator::moveto(int joint, double position, double speed)
{
	pos->setRefSpeed(joint,speed);
	pos->positionMove(joint, position*180/Pi);
	
	return 1;
}

int ArmActuator::moveto(int joint, double position)
{
	pos->setRefSpeed(joint,30);
	pos->positionMove(joint, position*180/Pi);
	
	return 1;
}


bool ArmActuator::getEncoderValue(int joint, double reading[1]){

	double *temp = new double[1];
	ACE_OS::memset(temp, 0, sizeof(double)*1);
	enc->getEncoder(joint, temp);
	reading[0]=temp[0]*Pi/180;
	//reading[1]=reading[1];
//	printf("\nleitura %f temp: %f junta %d\n", reading[0], temp[0], joint);


	return true;
	
}

bool ArmActuator::getEncodersValues(double readings[6]){


	double *q = new double[8];
	ACE_OS::memset(q, 0, sizeof(double)*8);
	enc->getEncoders(q);
	printf("\n\n\nencoders: -> %lf %lf %lf %lf %lf %lf \n\n", q[0], q[1],q[4],q[5],q[6],q[7]);

	readings[0]=q[0]*Pi/180;
	readings[1]=q[1]*Pi/180;
	//readings[2]=q[2]*Pi/180;
	//readings[3]=q[3]*Pi/180;
	readings[2]=q[4]*Pi/180;
	readings[3]=q[5]*Pi/180;
	readings[4]=q[6]*Pi/180;
	readings[5]=q[7]*Pi/180;

	return true;
}

int ArmActuator::move(int joint, double speed)
{

	vel->velocityMove(joint, speed*180/Pi);
	
	return 1;
}




