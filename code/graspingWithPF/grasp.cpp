#include <yarp/os/all.h>

#include <iostream>
#include <stdio.h>

#include "grasp.h"
//#include "YARPBaltaEyes.h"

#include "iCub/kinematics/robot.h"
#include "kinpronator.h"

#include "iCub/kinematics/gsl_aux.h"



using namespace std;
using namespace yarp::os;


#define Pi 3.1415927
#define SUBSAMP 2
#define byte unsigned char
#define NUMOFCYCLESFOREFFECTS	40
int counteffects = 0;

double pos_or[1];
int incontact = 0;
int inshape = 0;
int incolor =0;
int insize = 0;


double handpos[2];
double objpos[2];	//x,y

double w_thumb;
double h_thumb;

Robot* R;

int sta;

int addrf[]={0,1,4,5,6,7};
double pos[6];
//double th[6];
double posr[]={0,0,0,0,0,0};

double height = 18;
int mouseParam=5;


double posrhighold[]={-0.430670,-0.371390,-0.566986,-1.850798,-0.989939,0.413941};
double posrhighold2[]={-0.063799, -0.639885, -0.472467, -1.865219, -0.976618, 0.027124};

//double posrhigh[]={0.134821,-0.620928,0.026227,-2.065404,-0.922200,-0.281146};
double posrhigh[]={-0.48, -0.48, -0.26, -1.36, -0.92, -0.04};

double posr5[]={-0.14,-0.09,0,-1.3,0,0};


int addr;

#undef main
//#define _WINSOCKAPI_


ArmSkills::ArmSkills(){

	// allocate space for gsl structures needed in visual servoing
	Lp = gsl_matrix_calloc( 3, 2);  
	gsl_matrix_set( Lp, 1, 1, 1);
	gsl_matrix_set( Lp, 2, 0, -1);  
	gsl_print_matrix( Lp, "Lp");
	V = gsl_vector_calloc( 6 );  
	Vaux = gsl_vector_calloc( 3 );  
	qdot = gsl_vector_calloc( 6 );
	
	// Initialize the arm and hand wrappers
	printf("Here\n");
	arm=new ArmActuator();
	printf("hand\n");
	hand=new HandActuator();
	printf("after constructors\n");
	// Initialize kinematic structure
	R = new Robot(6);
	R->SetJoint(0,0,0,0,0);
	R->SetJoint(1,Pi/2,0,Pi/2,0);
	R->SetJoint(2,Pi/2,0,Pi/2,29.13);
	R->SetJoint(3,Pi/2,2.82,0,0);
	R->SetJoint(4,-Pi/2,-2.18,Pi/2,26.95);
	R->SetJoint(5,Pi/2,0,0,0);          


	arm->getEncoderValue(0,&pos[0]);
	arm->getEncoderValue(1,&pos[1]);
	arm->getEncoderValue(4,&pos[2]);
	arm->getEncoderValue(5,&pos[3]);
	arm->getEncoderValue(6,&pos[4]);
	arm->getEncoderValue(7,&pos[5]);
	arm->moveto(addrf[0],pos[0]);
	arm->moveto(addrf[1],pos[1]);
	arm->moveto(addrf[2],pos[2]);
	arm->moveto(addrf[3],pos[3]);
	arm->moveto(addrf[4],pos[4]);
	arm->moveto(addrf[5],pos[5]);

	// Initialize arm joints vels and  accelerations
	arm->setJointVel(0,100);
	arm->setJointVel(1,100);
	arm->setJointVel(4,100);
	arm->setJointVel(5,100);
	arm->setJointVel(6,100);
	arm->setJointVel(7,100);

	arm->setAccelerations(100);
	
	  
	hand->pos->setRefSpeed(0,200);
	hand->pos->setRefSpeed(1,200);
	hand->pos->setRefSpeed(2,200);
	hand->pos->setRefSpeed(3,200);

	hand->pos->setRefAcceleration(0,1000);
	hand->pos->setRefAcceleration(1,1000);
	hand->pos->setRefAcceleration(2,1000);
	hand->pos->setRefAcceleration(3,1000);
	
}

int isfar( double a, double b)
{
	if( ((a-b)*(a-b)) < 0.00001 ) // for notarrived this should be 0.1 due to the motor's backlashes
		return 0;
	else
		return 1;
}


int notarrived(ArmActuator *arm, HandActuator *hand, double finalpos[])
{
double readings;
int ret = 0;

	if( arm != NULL)
	{
		
		for(int cnt=0;cnt<6;cnt++)
			if(finalpos[cnt] != -10000 )
			{
				arm->getEncoderValue( addrf[cnt], &readings);
				ret += isfar( readings,finalpos[cnt] );
				if(ret)
					return ret;
			}
	}
	else
	{
		
		for(int cnt=0;cnt<4;cnt++)
			if(finalpos[cnt] != -10000 )
			{
				hand->getEncoderValue( cnt, &readings);

				ret += isfar( readings,finalpos[cnt] );
				if(ret)
					return ret;
			}
	}

	return ret;
}


int moving(ArmActuator *arm, HandActuator *hand)
{
double readings;
static double prevread[10];
int ret = 0;

	if( arm == NULL && hand == NULL)
	{
		for(int cnt=0;cnt<10;cnt++)
			prevread[cnt]=-10000;
		return -1;
	}

	yarp::os::Time::delay(0.1);

	if( arm != NULL)
	{
		
		for(int cnt=0;cnt<6;cnt++)
		{
				arm->getEncoderValue( addrf[cnt], &readings);
				ret += isfar( readings,prevread[cnt] );
				prevread[cnt]=readings;
				if(ret)
				{					
					return ret;
				}
		}
	}
	if (hand != NULL)
	{
		
		for(int cnt=0;cnt<4;cnt++)
		{
				hand->getEncoderValue( cnt, &readings);
				ret += isfar( readings,prevread[6+cnt] );
				prevread[cnt+6]=readings;
				if(ret)
					return ret;
		}
	}

	return ret;
}

int ArmSkills::MovingHand(){
  int ret =0;
  double readings;

  for(int cnt=0;cnt<4;cnt++)
    {
      hand->getEncoderValue( cnt, &readings);
      ret += isfar( readings,prevHandPos[cnt] );
      prevHandPos[cnt]=readings;
    } 	
  return ret; 
}

int ArmSkills::MovingArm(){
  int ret =0;
  double readings;

  for(int cnt=0;cnt<6;cnt++)
    {
      arm->getEncoderValue( addrf[cnt], &readings);
      ret += isfar( readings,prevArmPos[cnt] );
      prevArmPos[cnt]=readings;
    }
  return ret;
}

int ArmSkills::GoHome() {

	int addr;
	static int homeState=0;

	switch(homeState){
		case 0:
			addr=1;	arm->moveto(addr, -1.5);
			addr=0;	arm->moveto(addr, -1.5);
			homeState=2;
			moving(NULL,NULL);
			break;
		case 2:
		  if (moving(arm,NULL)==0) {
		                moving(NULL,NULL);
				addr=6;	arm->moveto(addr, 0);
				addr=7;	arm->moveto(addr, 0);
				addr=4;	arm->moveto(addr, 0);
				addr=5;	arm->moveto(addr, 0);
				addr=0;	arm->moveto(addr, 0);
				homeState=3;
			}
			break;
		case 3:
		  if (moving(arm,NULL)==0) {
		                moving(NULL,NULL);
				addr=1;	arm->moveto(addr, 0);
				
				homeState=4;
			}
			break;
		case 4:
		  if (moving(arm,NULL)==0) {
				homeState=0;
				return 1;
			}
			break;
	}

	return 0;

}

//CLOSE

//VISUAL SERVOING
// orient = 'h' for horizontal
// orient = 'v' for vertical
int ArmSkills::VisualServoing(double xerr, double yerr, double height,char orient){

	double pos[6];
	
	if (xerr*xerr<0.0001 && yerr*yerr<0.0001) {
	  arm->move(0, 0.0 );
	  arm->move(1, 0.0 );
	  arm->move(4, 0.0 );
	  arm->move(5, 0.0 );
	  
	  return 1;
	    }
	// Read the encoders
	arm->getEncoderValue(0,&pos[0]);
	arm->getEncoderValue(1,&pos[1]);
	arm->getEncoderValue(4,&pos[2]);
	arm->getEncoderValue(5,&pos[3]);
	arm->getEncoderValue(6,&pos[4]);
	arm->getEncoderValue(7,&pos[5]);
	
	//call visual servoing	
	printf("Servoing\n");
	
	RobMatrix posact = R->fk(pos);
	//controller visual servoing
	gsl_vector* de = gsl_vector_calloc( 2 );
	gsl_vector_set( de, 0, xerr);gsl_vector_set( de, 1, yerr);
	
	//compute velocity Lp * de
	gsl_blas_dgemv( CblasNoTrans, 5, Lp, de, 0, Vaux);
	gsl_vector_free( de );
	gsl_vector_set( V, 0, gsl_vector_get( Vaux, 0));
	gsl_vector_set( V, 1, gsl_vector_get( Vaux, 1));
	gsl_vector_set( V, 2, gsl_vector_get( Vaux, 2));
	gsl_print_vector( V, "V");
	
	//include obstacle avoidance for joint 2
	float obst = 0;
	double *Ja;
	
	Ja = (double*)malloc( 6*6*sizeof(double) );
	R->Jacob0( pos, Ja);
		
	if(pos[1]>-0.2)
	{
		obst = (float) ( -Ja[1] * (pos[1]) );
		printf("avoidance %f\n", obst);
	}
	free( Ja );

	//include redundancy for height with the projector PLp
	gsl_vector_set( V, 0, gsl_vector_get( Vaux, 0) + height - (posact.M[0][3]) + obst );
	
	R->V2Qdot( pos, V, qdot, 1e-3);
	gsl_print_vector( qdot, "qdot");


	//arm.axismove(1, gsl_vector_get(qdot,0) );
	arm->move(0, gsl_vector_get(qdot,0) );
	arm->move(1, gsl_vector_get(qdot,1) );
	arm->move(4, gsl_vector_get(qdot,2) );
	arm->move(5, gsl_vector_get(qdot,3) );


	//hand orientation
	double pl[]={0,0,0,0,0,0};
	switch (orient)
	{
	case 'h':
		pl[0]=-1;
		break;
	case 'v':
		pl[2]=-1;
		break;
	}
    
	RobMatrix T04 = R->fk( pos, 4);
	int ret;
	//pos <> th
	ret = ikorientationpronator( pl, pos, T04);

	RobMatrix T06 = R->fk(pos);
	T06.print();

	printf("ikorient %d\n", ret);

	arm->moveto( 6, pos[4]);
	arm->moveto( 7, -(pos[5]));

	return 0;

}


int ArmSkills::StopArm(){
	arm->move(0, 0 );	
	arm->move(1, 0 );	
	arm->move(4, 0 );	
	arm->move(5, 0 );
	arm->move(6, 0 );	
	arm->move(7, 0 );

	return 1;
}



// preprogrammed GRASP for Baltazar
int ArmSkills::Grasp(double wristrot)
{
	int addr_hand;
	int addr_arm;
	double pos[1];
	static int graspstate = 0;

	if (graspstate == -1)
			graspstate = 0;
	switch( graspstate ) {
		case 0:
			//pulso
			moving( NULL, NULL);
			addr_arm = 6;
			arm->getEncoderValue(addr_arm,pos_or);
			arm->moveto(addr_arm, pos_or[0]+wristrot); //wristrot o valor padrao era 0.2
			printf("turn hand from %g %g degrees\n",pos_or[0],wristrot);
			addr_arm = 5;
			arm->getEncoderValue(5,pos);
			arm->moveto(addr_arm, pos[0]+0.02);
			addr_hand=3;	
			hand->moveto(addr_hand, -100*Pi/180);
			graspstate = 10;
   
		break;
		case 10:		
			if( !moving( arm, NULL) || !moving( NULL, hand) )
			{
				graspstate = 1;
				arm->move(0, 0 );	arm->move(1, 0 );	
				arm->move(4, 0 );	arm->move(5, 0 );	
				arm->move(6, 0 );	arm->move(7, 0 );
			}
		break;
		case 1:
			//fechar dedosve
			moving( NULL, NULL);
			printf("fechar dedos\n");	addr_hand=0;	hand->moveto(addr_hand, -500*Pi/180); // -450
			//while(moving( NULL, hand))
			//	yarp::os::Time::delay(0.1);
			//moving( NULL, NULL);
			addr_hand=1;	hand->moveto(addr_hand, 500*Pi/180);  // 500
			addr_hand=2;	hand->moveto(addr_hand, -250*Pi/180);
			graspstate = 11;
		break;
		case 11:
			if( !moving( NULL, hand) ) {
				graspstate = -1;
				StopArm();
				moving( NULL, NULL);
			}
		break;
	}
	return (graspstate==-1);
}


//finalGrasp
int ArmSkills::Ungrasp() {
	static int fgraspstate = 0;

	printf("deslargar\n");
	
	if (fgraspstate == -1)
		fgraspstate = 0;

	switch( fgraspstate) {
	case 0:
	//reorientar a m�o
	arm->moveto(6, 0);
	arm->getEncoderValue(5,pos);
	arm->moveto( 5, pos[0]-0.1);

	//arm->getEncoderValue(4,pos);
	//arm->moveto( 4, pos[0]+0.4);


	fgraspstate = 10;
	break;

	case 10:		

	if( !moving( arm, NULL) )
		fgraspstate = 1;
	break;

	case 1:

	arm->moveto(addrf[0],posrhigh[0]);	//arm->moveto(addrf[1],posrhigh[1]);
	//arm->moveto(addrf[2],posrhigh[2]);
	arm->moveto(addrf[3],posrhigh[3]-0.3);
	arm->moveto(addrf[4],posrhigh[4]);	arm->moveto(addrf[5],posrhigh[5]);

	fgraspstate = 11;
	break;

	case 11:		

	if( !moving( arm, NULL) )
		fgraspstate = 2;
	break;

	case 2:
		
	arm->moveto(addrf[1],posrhigh[1]);
	arm->moveto(addrf[2],posrhigh[2]);

	fgraspstate = 21;

	break;

	case 21:
	if( !moving( arm, NULL) )
		fgraspstate = 3;

	break;


	case 3:
		
	arm->moveto(addrf[3],posrhigh[3]);

	fgraspstate = 31;

	break;

	case 31:
	if( !moving( arm, NULL) )
		fgraspstate = 9;

	break;

	case 9:	

	//abrir
	printf("open hand\n");

	hand->moveto( 3, 0);	
	hand->moveto( 0, 0);	
	hand->moveto( 1, 0);
	hand->moveto( 2, 0);

	fgraspstate = 91;
	break;

	case 91:		

	if( !moving( NULL, hand) )
	{
		fgraspstate = -1;
		StopArm();
		moving( NULL, NULL);
	}
	break;
	
	}

	return (fgraspstate==-1);
}







//TAP
int ArmSkills::Tap()
{
	//int addr_hand;
	int addr_arm;
	double pos[1];
	static int statetap = 0;
	
	if (statetap == -1)
		statetap = 0;
	
	printf("statetap %d\n", statetap);
	
	switch( statetap ){
		case 0:
			// prepare tap
			addr_arm = 7;
			arm->getEncoderValue(addr_arm,pos);
			arm->moveto(addr_arm, +0.5);
			statetap=10;
		break;
		case 10:
			if( !moving( arm, NULL) ) {
				statetap = 1;
				moving( NULL, NULL);
			}
		break;

		case 1:
			//turn the wrist
			addr_arm = 6;
			arm->getEncoderValue(addr_arm,pos_or);
			arm->moveto(addr_arm, pos_or[0]+1.95);

			//double pos_orl;
			printf("moving 6\n");
			addr_arm=0;
			arm->getEncoderValue(addr_arm, pos);
			arm->moveto(addr_arm, pos[0]+10*Pi/180.0);

			statetap = 11;
		break;

		case 11:
			if( !moving( arm, NULL) ) {
				statetap = 2;
				moving( NULL, NULL);
			}
		break;

		case 2:
			//fa�er tap
			// TBD: increase speed to maximum
			printf("setup 1\n");
			addr_arm = 7;
			arm->getEncoderValue(addr_arm,pos);
			arm->moveto( addr_arm, -0.5, 180,2000);
			statetap = 21;	
		break;

		case 21:
			if( !moving( arm, NULL) )
			{
				statetap = -1;
				StopArm();
				moving( NULL, NULL);

			}
		break;
	}

	return (statetap==-1);
}



//finalTAP
int ArmSkills::FinalTap()
{
	static int fstatetap = 0;
	//int addr_hand;
	int addr_arm;
	//double pos[1];

	if (fstatetap == -1)
		fstatetap = 0;
	
	switch( fstatetap) {
	case 0:
		// Go back to the original position
		addr_arm = 7;
		arm->moveto(addr_arm, 0, 30,100);

		fstatetap = 10;
	break;

	case 10:		

		if( !moving( arm, NULL) )
			fstatetap = 1;
	break;

	case 1:
		arm->moveto(addrf[0],posrhigh[0]);	arm->moveto(addrf[1],posrhigh[1]);
		arm->moveto(addrf[2],posrhigh[2]);	arm->moveto(addrf[3],posrhigh[3]);
		arm->moveto(addrf[4],posrhigh[4]);	arm->moveto(addrf[5],posrhigh[5]);

		fstatetap = 11;
	break;

	case 11:		

		if( !moving( arm, NULL) ) {
			StopArm();
			fstatetap = -1;
		}
	break;
	}
	
	return (fstatetap==-1);
}



//Touch
int ArmSkills::Touch()
{
	//int addr_hand;
	int addr_arm;
	double pos[1];
	static int statetouch = 0;

	if (statetouch == -1)
		statetouch = 0;

	printf("statetouch %d\n", statetouch);

	switch( statetouch ){
	case 0:
		// do touch
		addr_arm = 7;
		arm->getEncoderValue(addr_arm,pos);
		arm->moveto(addr_arm, pos[0]-0.3);

		statetouch=10;
	break;

	case 10:
		if( !moving( arm, NULL) )
		{
			statetouch = -1;
			StopArm();
			moving( NULL, NULL);
		}
	break;
	}
	return (statetouch)==-1;
}


//finalTouch
int ArmSkills::FinalTouch()
{
	static int fstatetouch = 0;
	static int waitNum=0;
	//int addr_hand;
	int addr_arm;
	double pos[1];
	
	if (fstatetouch == -1){
		fstatetouch = 0;
		waitNum=0;
	}

	
	switch( fstatetouch) {
	case 0:
		waitNum++;
		if (waitNum>30){
			// Go back to the original position
			addr_arm = 7;
			arm->getEncoderValue(addr_arm,pos);
			arm->moveto(addr_arm, pos[0]+0.3);
			fstatetouch = 10;
		}
		else{
			fstatetouch=0;
		}

	break;

	case 10:		
		if( !moving( arm, NULL) ){
			fstatetouch = -1;
			StopArm();
		}
		break;
	}
	
	return (fstatetouch==-1);
}

int ArmSkills::MoveArm(double *posArm) {

  for (int i=0; i<6;i++) {
	arm->moveto(addrf[i],posArm[i]);
	prevArmPos[i]=-1000;
  }

  return 1;
}

int ArmSkills::MoveHand(double *posHand) {

  for (int i=0; i<4; i++) {
	hand->moveto(i,posHand[i]);
	prevHandPos[i]=-1000;
  } 
  return 1;
}

int ArmSkills::MoveHandVel(double *velHand) {

  for (int i=0; i<4; i++) {
	hand->move(i,velHand[i]);
	prevHandPos[i]=-1000;
  } 
  return 1;
}

//MAIN

ArmSkills::~ArmSkills()
{
	gsl_matrix_free(Lp);
	gsl_vector_free( V );
    	gsl_vector_free( Vaux );
    	gsl_vector_free( qdot ); 
	
	arm->move(0, 0 );	arm->move(1, 0 );	arm->move(4, 0 );	arm->move(5, 0 );
	hand->close_board();
	arm->close_board();
}



// preprogrammed GRASP for Baltazar
int ArmSkills::GraspVertical()
{
	int addr_hand;
	//int addr_arm;
	//double pos[1];
	static int graspstate = 0;

	if (graspstate == -1)
			graspstate = 1;
	switch( graspstate ) {
		case 0:
			// polegar
			addr_hand=3;	
			hand->moveto(addr_hand, -100*Pi/180);
			graspstate = 10;
   
		break;
		case 10:		
			if( !moving( NULL, hand) )
			{
				graspstate = 1;				
			}
		break;
		case 1:
			//fechar dedosve
			moving( NULL, NULL);
			printf("fechar dedos\n");	
			addr_hand=1;	hand->moveto(addr_hand, 550*Pi/180);  // 500
			addr_hand=0;	hand->moveto(addr_hand, -500*Pi/180); // -450			
			//while(moving( NULL, hand))
			//	yarp::os::Time::delay(0.1);
			//moving( NULL, NULL);			
			addr_hand=2;	hand->moveto(addr_hand, -450*Pi/180);
			graspstate = 11;
		break;
		case 11:
			if( !moving( NULL, hand) ) {
				graspstate = -1;
				StopArm();
				moving( NULL, NULL);
			}
		break;
	}
	return (graspstate==-1);
}

//finalGrasp
int ArmSkills::UngraspVertical() {
	static int fgraspstate = 0;

	printf("deslargar\n");
	
	if (fgraspstate == -1)
		fgraspstate = 1;

	switch( fgraspstate) {

	case 1:

	arm->moveto(addrf[0],posrhigh[0]);	//arm->moveto(addrf[1],posrhigh[1]);
	//arm->moveto(addrf[2],posrhigh[2]);
	arm->moveto(addrf[3],posrhigh[3]-0.3);
	arm->moveto(addrf[4],posrhigh[4]);	arm->moveto(addrf[5],posrhigh[5]);

	fgraspstate = 11;
	break;

	case 11:		

	if( !moving( arm, NULL) )
		fgraspstate = 2;
	break;

	case 2:
		
	arm->moveto(addrf[1],posrhigh[1]);
	arm->moveto(addrf[2],posrhigh[2]);

	fgraspstate = 21;

	break;

	case 21:
	if( !moving( arm, NULL) )
		fgraspstate = 3;

	break;


	case 3:
		
	arm->moveto(addrf[3],posrhigh[3]);

	fgraspstate = 31;

	break;

	case 31:
	if( !moving( arm, NULL) )
		fgraspstate = 9;

	break;

	case 9:	

	//abrir
	printf("open hand\n");

	hand->moveto( 3, 0);	
	hand->moveto( 0, 0);	
	hand->moveto( 1, 0);
	hand->moveto( 2, 0);

	fgraspstate = 91;
	break;

	case 91:		

	if( !moving( NULL, hand) )
	{
		fgraspstate = -1;
		StopArm();
		moving( NULL, NULL);
	}
	break;
	
	}

	return (fgraspstate==-1);
}


