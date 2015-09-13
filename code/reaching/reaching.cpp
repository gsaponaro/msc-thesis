/*
 * Copyright (C) 2008 Giovanni Saponaro <giovanni.saponaro@gmail.com>
 */

#include "reaching.h"

#define DEFAULT_CONFIG_FILE "reaching.ini"

using namespace yarp::os;
using namespace yarp::sig;

// global variables

BufferedPort<Vector> target_port;		// port where we receive target pose

// vectors to store data we receive in the form [ pt1.x pt1.y pt1.z centroid.x centroid.y centroid.z ] (visual ref. frame)
gsl_vector_float *target_pt1_vis;
gsl_vector_float *target_centroid_vis;
gsl_vector_float *target_or_vis;		// pt1 - pt2 difference, providing orientation
// corresponding vectors in the shoulder ref. frame
gsl_vector_float *target_pt1_sho;
gsl_vector_float *target_centroid_sho;
gsl_vector_float *target_or_sho;
gsl_vector_float *normalized_target_or_sho; // (pt1 - pt2) / target_or_sho = 2*(pt1-centroid) / target_or_sho
float target_or_sho_norm;				// ||target_or_sho||_2

// some predefined arm joint configurations (in radians) for Baltazar
double initArmPos[6] = {0.0, -1.2, 0.0,       -1.3   ,       0.0,    0.0};
//double initArmPos2[6] = {-0.48, -0.48, -0.26, -1.36, -0.92, -0.04};
double initArmPos2[6] = {-0.48, -0.48, 0.0, -1.36, -0.22, -0.04};

double curr_joints[6];				// last joint values that have been applied
double posm[3] = {-200,-200,-200};	// physical arm constraints

enum arm_state_enum
{
	INIT,		// state with initial joint positions encountered when starting program
	OUTSIDE1,	// largely outside of cameras view
	OUTSIDE2,	// barely outside of cameras view
	REACH		// Baltazar performs a reaching task (next state, if enter is pressed, is OUTSIDE1)
};
enum arm_state_enum arm_state;		// states of the FSM. user makes it evolve by pressing enter

float deg2rad(float d)
{
	return (float) (d / 180 * CV_PI);
}

float rad2deg(float r)
{
	return (float) (r * (180 / CV_PI));
}
void min_energy(int num_sols, double solutions[6][MAX_SOL], double min_joints[6])
{
	/*
	use energy minimization criterion to choose the inverse kinematics solution
	with the nearest arm joint configuration compared to the current one.

	num_sols   - number of solutions previously found by inv.kin.
	solutions  - bidimensional array containing the actual solutions previously found by inv.kin.
	min_joints - minimum energy solution
	*/
	int aux = 0, aux1 = -1;
	double e, E, Ea = -1, res[3];

	// weights of the joints according to the robot servo capabilities
	double W[6] = { 1, 1, 1.47, 1.89, 1.22, 1.43 };

	if(num_sols > 0) // cininv previously found some solutions to the inverse kinematics problem
	{
		int j;
		do
		{
			E = 999999;
			for(j = 0; j < num_sols; j++)
			{
				e = 0;
				for(int k = 0; k < 6; k++)
				{
					// Paulo's energy criterion:
					// min_{j \in cininv solutions} e = sum_{k=0}^6 W_k * |theta_k - theta_{cininv,k}^(j)|
					//e += W[k] * abs(min_joints[k] - solutions[k][j]); // 'min_joints' has the last values applied

					// Euclidean distance energy criterion:
					// min_{j \in cininv solutions} e = sum_{k=0}^6 W_k * ||theta_k-theta_{cininv,k}^j||_2
					//e += W[k] * pow((min_joints[k] - solutions[k][j]), 2); // we still have to apply the sqrt, outside the for
					e += W[k] * gsl_pow_2(min_joints[k] - solutions[k][j]); // we still have to apply the sqrt, outside the for
				} // end internal for
				e = sqrt(e); // completes Euclidean norm expression
				if(e < E || (e > Ea && aux1 != -1))
				{
					aux = 0;
					E = e;
					// update chosen joint solution
					for(int num = 0; num < 6; num++)
					{
						min_joints[num] = solutions[num][j];
					}
					dkinematics(min_joints, 3, res);	// res is expressed in cm
					//ACE_OS::printf("DEBUG: j=%d, dkinematics(min_joints,3,res) computed %.3f %.3f %.3f\n", j, res[0], res[1], res[2]);
					if(res[0]>posm[0] && res[1]>posm[1] && res[2]>posm[2])
					{
						Ea = e;
						aux = 1;
					}
				} // end if(e < E...
			} // end external for
			aux1 = 0;
		} while(aux==0 && j<num_sols); // end do-while
	} // end if(num_sols > 0)

	if(solutions[0][0] == -5)
		aux = 0;
	else if(num_sols<1 || aux==0) // position is not reachable
		//min_joints[0] = -4; // -229 degrees
		return;
}

bool ReachingModule::respond(const Bottle &command, Bottle &reply)
{
	/*
	handle user key press events. if he presses 'enter', then the reaching FSM will
	evolve, thus the robot arm will move. otherwise, nothing will happen.
	*/
	if (Module::respond(command, reply))
		return true;
	reply.clear(); // reply contains a "command not recognized" string from Module::respond()

	switch (command.get(0).asVocab())
	{
	case VOCAB1('\0'): // = enter key
		// let the FSM evolve when user presses the enter key
		_mutex.wait();
		interact_flag = true;
		_mutex.post();
	default:
		// do nothing, so the FSM doesn't evolve
		return false;	// critical failure
	}

	return true;		// no critical failure
}
bool ReachingModule::open(yarp::os::Searchable &config)
{
	interact_flag = false; // when this flag is true, the reaching FSM evolves
	arm_state = INIT;

	if (config.check("help", "if present, display user message")) {
		ACE_OS::printf("Call with --file <config_file.ini>.\n");
		return false;
	}
	yarp::os::ConstString config_file;
	if (!config.check("file")) {
		config_file = DEFAULT_CONFIG_FILE;
	} else {
		// use file provided by user in command line
		config_file = config.find("file").asString();
	}
	Property p;
	p.fromConfigFile(config_file.c_str(), true);

	safety_threshold = p.findGroup("REACHING").find("safety_threshold").asInt();
	sho_to_vis = (float) p.findGroup("REACHING").find("sho_to_vis").asDouble();

	target_port_name = p.findGroup("PORTS").find("port_prefix").asString();
	target_port_name += "/target";
	// now we have "/balta/reaching/target"

	// open YARP port where we will receive target object coordinates data
	target_port.open(target_port_name.c_str());

	// allocate data structures
	target_pt1_vis = gsl_vector_float_calloc(3);
	target_centroid_vis = gsl_vector_float_calloc(3);
	target_or_vis = gsl_vector_float_calloc(3);

	target_pt1_sho = gsl_vector_float_calloc(3);
	target_centroid_sho = gsl_vector_float_calloc(3);
	target_or_sho = gsl_vector_float_calloc(3);
	normalized_target_or_sho = gsl_vector_float_calloc(3);

	return true;
}

bool ReachingModule::updateModule()
{
	// until this function returns false, it will be called forever by runModule()

	// data continuously read (true=blocking) from input port
	Vector *target_data = target_port.read(true);

	// update coordinates, as received by the 3D reconstruction module
	// in the form [ pt1.x pt1.y pt1.z centroid.x centroid.y centroid.z ]

	gsl_vector_float_set( target_pt1_vis, 0, (float) (*target_data)[0] );
	gsl_vector_float_set( target_pt1_vis, 1, (float) (*target_data)[1] );
	gsl_vector_float_set( target_pt1_vis, 2, (float) (*target_data)[2] );

	gsl_vector_float_set( target_centroid_vis, 0, (float) (*target_data)[3] );
	gsl_vector_float_set( target_centroid_vis, 1, (float) (*target_data)[4] );
	gsl_vector_float_set( target_centroid_vis, 2, (float) (*target_data)[5] );

	// orientation = pt1 - pt2 = pt1 - (2*centroid-pt1) = 2*(pt1 - centroid)
	gsl_vector_float_set( target_or_vis, 0, 2*( gsl_vector_float_get(target_pt1_vis,0)-gsl_vector_float_get(target_centroid_vis,0) ) );
	gsl_vector_float_set( target_or_vis, 1, 2*( gsl_vector_float_get(target_pt1_vis,1)-gsl_vector_float_get(target_centroid_vis,1) ) );
	gsl_vector_float_set( target_or_vis, 2, 2*( gsl_vector_float_get(target_pt1_vis,2)-gsl_vector_float_get(target_centroid_vis,2) ) );
	/*
	ACE_OS::printf("target orientation (pt1 - pt2): [ X = %.3f, Y = %.3f, Z = %.3f ] (vision ref.)\n",
		gsl_vector_float_get(target_or_vis,0),
		gsl_vector_float_get(target_or_vis,1),
		gsl_vector_float_get(target_or_vis,2)
		);

	ACE_OS::printf("target centroid: [ X = %.3f, Y = %.3f, Z = %.3f ] (vision ref.)\n",
		gsl_vector_float_get(target_centroid_vis,0),
		gsl_vector_float_get(target_centroid_vis,1),
		gsl_vector_float_get(target_centroid_vis,2)
	);
	*/

	// transformations from visual frame to shoulder frame

	gsl_vector_float_set( target_pt1_sho, 0, -gsl_vector_float_get(target_pt1_vis,1) ); // x_sho = - y_vis
	gsl_vector_float_set( target_pt1_sho, 1, -gsl_vector_float_get(target_pt1_vis,2) ); // y_sho = - z_vis
	gsl_vector_float_set( target_pt1_sho, 2, gsl_vector_float_get(target_pt1_vis,0)+sho_to_vis ); // z_sho = x_vis + displacement

	gsl_vector_float_set( target_centroid_sho, 0, -gsl_vector_float_get(target_centroid_vis,1) ); // x_sho = - y_vis
	gsl_vector_float_set( target_centroid_sho, 1, -gsl_vector_float_get(target_centroid_vis,2) ); // y_sho = - z_vis
	gsl_vector_float_set( target_centroid_sho, 2, gsl_vector_float_get(target_centroid_vis,0)+sho_to_vis ); // z_sho = x_vis + displacement

	// either this or simply transform target_or_vis
	gsl_vector_float_set( target_or_sho, 0, 2*( gsl_vector_float_get(target_pt1_sho,0)-gsl_vector_float_get(target_centroid_sho,0) ) );
	gsl_vector_float_set( target_or_sho, 1, 2*( gsl_vector_float_get(target_pt1_sho,1)-gsl_vector_float_get(target_centroid_sho,1) ) );
	gsl_vector_float_set( target_or_sho, 2, 2*( gsl_vector_float_get(target_pt1_sho,2)-gsl_vector_float_get(target_centroid_sho,2) ) );

	ACE_OS::printf("target centroid: [ X = %.3f, Y = %.3f, Z = %.3f ] (shoulder ref.)\n",
		gsl_vector_float_get(target_centroid_sho,0),
		gsl_vector_float_get(target_centroid_sho,1),
		gsl_vector_float_get(target_centroid_sho,2)
	);

	/*
	// no need to reinvent the wheel like this -- use gsl_blas_snrm2 instead
	target_or_sho_norm = 0.0000001f;
	for(int i=0; i<3; i++)
		//target_or_sho_norm += pow((gsl_vector_float_get(target_pt1_sho, i) - gsl_vector_float_get(target_centroid_sho, i)), 2);
		target_or_sho_norm += (float) gsl_pow_2(gsl_vector_float_get(target_pt1_sho,i) - gsl_vector_float_get(target_centroid_sho,i));
	target_or_sho_norm = sqrt(target_or_sho_norm);
	target_or_sho_norm *= 2;
	ACE_OS::printf("DEBUG: target_or_sho_norm = %.3f\n", target_or_sho_norm);
	// target_or_sho_norm = sqrt( (pt1.x-pt2.x)^2-(pt1.y-pt2.y)^2-(pt1.z-pt2.z)^2 ) = 2 * sqrt( (pt1.x-cen.x)^2-(pt1.y-cen.y)^2-(pt1.z-cen.z)^2 )
	*/

	// norm of target orientation (shoulder ref. frame):
	target_or_sho_norm = gsl_blas_snrm2( target_or_sho );

	//ACE_OS::printf("DEBUG: target_or_sho_norm = %.3f\n", target_or_sho_norm);

	for(int i=0; i<3; i++)
		gsl_vector_float_set( normalized_target_or_sho, i, gsl_vector_float_get(target_or_sho,i)/target_or_sho_norm );
	ACE_OS::printf("DEBUG: normalized_target_or_sho = [ %.3f %.3f %.3f ]\n",
		gsl_vector_float_get(normalized_target_or_sho,0),
		gsl_vector_float_get(normalized_target_or_sho,1),
		gsl_vector_float_get(normalized_target_or_sho,2)
		);
	
	/*
	ACE_OS::printf("...whose sum of squares is %.3f\n",
		gsl_pow_2(gsl_vector_float_get(normalized_target_or_sho,0))
		+ gsl_pow_2(gsl_vector_float_get(normalized_target_or_sho,1))
		+ gsl_pow_2(gsl_vector_float_get(normalized_target_or_sho,2))
		);
	*/
	

	// NB: pos is expressed in centimetres in Manuel's module,
	// in metres (floor reference frame) in Paulo's armCalcs function,
	// --> in centimetres (shoulder reference frame) in Paulo's cininv function <--
	double pos[3];

	// convert from m to cm
	pos[0] = gsl_vector_float_get(target_centroid_sho,0) * 100;
	pos[1] = gsl_vector_float_get(target_centroid_sho,1) * 100;
	pos[2] = gsl_vector_float_get(target_centroid_sho,2) * 100;

	ACE_OS::printf("going towards pos = [ %.3f", pos[0]);

	/*
	pos[0] -= safety_threshold;
	ACE_OS::printf("-%d=%.3f", safety_threshold, pos[0]);
	*/
	
	ACE_OS::printf(" %.3f ", pos[1]);

	ACE_OS::printf("%.3f", pos[2]);
	pos[2] -= safety_threshold;
	ACE_OS::printf("-%d=%.3f ]\n", safety_threshold, pos[2]);

	// begin INVERSE KINEMATICS
	// ========================
	
	double solutions[6][MAX_SOL];
	for(int i=0; i<6; i++)
		for(int j=0; j<MAX_SOL; j++)
			solutions[i][j] = 0;
	double min_joints[6];	// minimum energy joint values
	int num_sols;
	//int theta;				// angle required to align hand with target (see Saponaro, Bernardino CLAWAR 2008 paper)

	//theta = 0;
	float n1=1, n2=0, n3=0;
	//float n1=0, n2=1, n3=0;
	//float n1=0, n2=0, n3=1;
	//float n1=cos(deg2rad(theta)), n2=0, n3=sin(deg2rad(theta));
	//float n1=0.148, n2=0., n3=0.989;
	/*
	float n1=gsl_vector_float_get(normalized_target_or_sho,0),
		  n2=gsl_vector_float_get(normalized_target_or_sho,1),
		  n3=gsl_vector_float_get(normalized_target_or_sho,2);
	*/
	
	ACE_OS::printf("DEBUG: trying to compute inv.kin. with normal [ %.3f %.3f %.3f ]",
		n1, n2, n3
		);

	ACE_OS::printf(" whose sum of squares is %.3f", 
		gsl_pow_2(n1) + gsl_pow_2(n2) + gsl_pow_2(n3)
		);

	ACE_OS::printf("\n");

	/*
		in Paulo Carreiras' functions, x: positive towards the floor, y: positive towards Baltazar's back (window)
		z: positive towards Baltazar's fictitious left arm
	*/
	num_sols = cininv(pos, solutions,
		n1, n2, n3
		);
	if(num_sols == -1)
	{
		ACE_OS::printf("*** FAILURE*** could NOT calculate any inv.kin. solution\n");
		// reset min_joints
		for(int i=0; i<6; i++)
			min_joints[i] = 0;
	}
	else if(num_sols == 1)
		ACE_OS::printf("*** WARNING *** calculated only one, possibly UNSTABLE, inv.kin. solution. Proceed with care!\n");
	else if(num_sols > 0)
		ACE_OS::printf("*** SUCCESS *** %d inv.kin. solutions found\n", num_sols);


	for (int i=0; i<num_sols; i++)
	{
		ACE_OS::printf("inv.kin. solution #%d: %.3f %.3f %.3f %.3f %.3f %.3f\n", i+1,
			rad2deg(solutions[0][i]), rad2deg(solutions[1][i]), rad2deg(solutions[2][i]),
			rad2deg(solutions[3][i]), rad2deg(solutions[4][i]), rad2deg(solutions[5][i])
			);
	}

	for(int i=0; i<6; i++)
		min_joints[i] = curr_joints[i];
	min_energy(num_sols, solutions, min_joints); // min_joints now contains the optimal inv.kin. solution

	// don't print default min_joints values if min_sols < 1: they are not significant
	if(num_sols >= 1)
		ACE_OS::printf("minimum energy solution is: %.3f %.3f %.3f %.3f %.3f %.3f\n",
			rad2deg(min_joints[0]), rad2deg(min_joints[1]), rad2deg(min_joints[2]),
			rad2deg(min_joints[3]), rad2deg(min_joints[4]), rad2deg(min_joints[5])
		);

	// end INVERSE KINEMATICS
	// ======================

	// begin REACHING FINITE STATE MACHINE
	// ===================================
	_mutex.wait();
	if(interact_flag && (arm_state==INIT || arm_state==REACH))
	{
		arm.MoveArm(initArmPos);	// position Baltazar's arm to a position out of camera sight
		/*
		while (arm.MovingArm()) {
			Time::delay(0.04);
		}
		*/
		for(int i=0; i<6; i++)		// update 'curr_joints' variable
			curr_joints[i] = initArmPos[i];
		arm_state = OUTSIDE1;
		interact_flag = false;
	}
	else if(interact_flag && arm_state==OUTSIDE1)
	{
		arm.MoveArm(initArmPos2);	// position Baltazar's arm to a position within camera sight
		for(int i=0; i<6; i++)
			curr_joints[i] = initArmPos2[i];
		arm_state = OUTSIDE2;
		interact_flag = false;
	}
	else if(interact_flag && arm_state==OUTSIDE2)
	{
		ACE_OS::printf("**** Going to REACH in 3 seconds ****\n");
		Time::delay(3);
		if (solutions[0][num_sols-1] != 0)
		{
			arm.MoveArm ( min_joints ); // reach, i.e., move arm in the vicinity of target
			for(int i=0; i<6; i++)
				curr_joints[i] = min_joints[i];
		}
		arm_state = REACH;
		interact_flag = false;
	}
	_mutex.post();
	// end REACHING FINITE STATE MACHINE
	// =================================

	ACE_OS::printf("DEBUG: current joint values are: %.3f %.3f %.3f %.3f %.3f %.3f\n",
			rad2deg(curr_joints[0]), rad2deg(curr_joints[1]), rad2deg(curr_joints[2]),
			rad2deg(curr_joints[3]), rad2deg(curr_joints[4]), rad2deg(curr_joints[5])
		);
	ACE_OS::printf("DEBUG: current arm state is %d\n\n", arm_state);

	return true;
}

bool ReachingModule::close()
{
	ACE_OS::printf("\nClosing open-loop reaching phase module...\n");

	// close YARP port
	target_port.close();

	// free data structures
	gsl_vector_float_free(target_pt1_vis);
	gsl_vector_float_free(target_centroid_vis);
	gsl_vector_float_free(target_or_vis);

	gsl_vector_float_free(target_pt1_sho);
	gsl_vector_float_free(target_centroid_sho);
	gsl_vector_float_free(target_or_sho);
	gsl_vector_float_free(normalized_target_or_sho);

	// is this necessary?
	//Network::fini();

	return true;
}
