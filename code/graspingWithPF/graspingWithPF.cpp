#include "graspingWithPF.h"

#define DEFAULT_CONFIG_FILE "graspingWithPF.ini"

using namespace yarp::os;
using namespace yarp::sig;

// global variables

BufferedPort<Vector> target_port;		// port where we receive target pose

gsl_vector_float *target_centroid_sho;

double initArmPos[6] = {0.0, -1.2, 0.0,       -1.3   ,       0.0,    0.0};
//double initArmPos2[6] = {-0.48, -0.48, -0.26, -1.36, -0.92, -0.04};
double initArmPos2[6] = {-0.48, -0.48, 0.0, -1.36, -0.22, -0.04};
double curr_joints[6];					// last joint values applied
double posm[3] = {-200,-200,-200}; // physical arm constraints

enum arm_state_enum
{
	INIT,
	OUTSIDE1,	// largely outside of cameras view
	//OUTSIDE2,	// barely outside of cameras view
	REACH
};
enum arm_state_enum arm_state;

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

	num_sols - number of solutions previously found by inv.kin.
	solutions - solutions previously found by inv.kin.
	min_joints - minimum energy solution
	*/
	int aux = 0, aux1 = -1;
	double e, E, Ea = -1, res[3];

	// weights of the joints according to robot servo capabilities
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

bool GraspingWithPFModule::respond(const Bottle &command, Bottle &reply)
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
	case VOCAB1('\0'):
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
bool GraspingWithPFModule::open(yarp::os::Searchable &config)
{
	interact_flag = false; // when it is true, the reaching FSM evolves
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

	target_port_name = p.findGroup("PORTS").find("port_prefix").asString();
	target_port_name += "/target";
	

	// open YARP port
	target_port.open(target_port_name.c_str());

	// allocate data structures

	target_centroid_sho = gsl_vector_float_calloc(3);


	return true;
}

bool GraspingWithPFModule::updateModule()
{
	// until this function returns false, it will be called forever by runModule()

	Vector *target_data = target_port.read(true);

	gsl_vector_float_set( target_centroid_sho, 0, (float) (*target_data)[0] );
	gsl_vector_float_set( target_centroid_sho, 1, (float) (*target_data)[1] );
	gsl_vector_float_set( target_centroid_sho, 2, (float) (*target_data)[2] );

	

	// pos is expressed in centimetres in Manuel's module,
	// in metres (floor reference frame) in Paulo's armCalcs function,
	// --> in centimetres (shoulder reference frame) in Paulo's cininv function <--
	double pos[3];

	pos[0] = gsl_vector_float_get(target_centroid_sho,0);
	pos[1] = gsl_vector_float_get(target_centroid_sho,1);
	pos[2] = gsl_vector_float_get(target_centroid_sho,2);

	ACE_OS::printf("going towards pos = [ %.3f", pos[0]);

	pos[0] -= safety_threshold;
	ACE_OS::printf("-%d=%.3f", safety_threshold, pos[0]);
	
	ACE_OS::printf(" %.3f ", pos[1]);

	int radius = 5;
	ACE_OS::printf("%.3f", pos[2]);
	
	pos[2] -= radius;
	ACE_OS::printf("-%d=%.3f ", radius, pos[2]);
	
	ACE_OS::printf(" ]\n");

	// begin INVERSE KINEMATICS
	// ========================
	
	double solutions[6][MAX_SOL];
	for(int i=0; i<6; i++)
		for(int j=0; j<MAX_SOL; j++)
			solutions[i][j] = 0;
	double min_joints[6];		// minimum energy joint values
	
	int num_sols;
	int theta = 0;


	//float n1=1, n2=0, n3=0;
	float n1=0, n2=0, n3=1;
	//float n1=cos(deg2rad(theta)), n2=0, n3=sin(deg2rad(theta));


	ACE_OS::printf("DEBUG: trying to compute inv.kin. with normal [ %.3f %.3f %.3f ]",
		n1, n2, n3
		);

	/*
	ACE_OS::printf(" whose sum of squares is %.3f", 
		gsl_pow_2(n1) + gsl_pow_2(n2) + gsl_pow_2(n3)
		);
	*/

	ACE_OS::printf("\n");
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

	// phase 1 of reaching: position Baltazar's arm to a default position out of camera sight
	// ======================================================================================

	_mutex.wait();
	if(interact_flag && (arm_state==INIT || arm_state==REACH))
	{
		arm.MoveArm(initArmPos);
		/*
		while (arm.MovingArm()) {
			Time::delay(0.04);
		}
		*/
		for(int i=0; i<6; i++) // update 'curr_joints' variable
			curr_joints[i] = initArmPos[i];
		arm_state = OUTSIDE1;
		interact_flag = false;
	}



	// phase 2 of reaching: move arm in the vicinity of target
	// =======================================================
	else if(interact_flag && arm_state==OUTSIDE1)
	{
		ACE_OS::printf("**** Going to REACH in 1 second ****\n");
		Time::delay(1);
		if (solutions[0][num_sols-1] != 0)
		{
			arm.MoveArm ( min_joints );
			for(int i=0; i<6; i++) // update 'curr_joints' variable
				curr_joints[i] = min_joints[i];
		}
		arm_state = REACH;
		interact_flag = false;
	}
	_mutex.post();

	ACE_OS::printf("DEBUG: current joint values are: %.3f %.3f %.3f %.3f %.3f %.3f\n",
			rad2deg(curr_joints[0]), rad2deg(curr_joints[1]), rad2deg(curr_joints[2]),
			rad2deg(curr_joints[3]), rad2deg(curr_joints[4]), rad2deg(curr_joints[5])
		);
	ACE_OS::printf("DEBUG: current arm state is %d\n\n", arm_state);

	return true;
}

bool GraspingWithPFModule::close()
{
	ACE_OS::printf("\nClosing open-loop reaching phase module...\n");

	// close YARP port
	target_port.close();

	// free data structures
	
	gsl_vector_float_free(target_centroid_sho);

	// is this necessary?
	//Network::fini();

	return true;
}
