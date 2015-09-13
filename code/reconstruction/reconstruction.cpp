/*
 * Copyright (C) 2008 Giovanni Saponaro <giovanni.saponaro@gmail.com>
 */

#include "reconstruction.h"

#define DEFAULT_CONFIG_FILE "reconstruct_target.ini"

using namespace yarp::os;
using namespace yarp::sig;

// global variables
gsl_vector_float *pt1;				// reconstructed pt1 (w.r.t. nose reference frame)
gsl_vector_float *pt2;				// "			 pt2
gsl_matrix_float *R_pt;				// nose-to-body rotation matrix
gsl_vector_float *t_pt;				// nose-to-body translation vector
gsl_vector_float *pt1_body;			// reconstructed pt1 (w.r.t. body)
gsl_vector_float *pt2_body;			// "			 pt2
gsl_vector_float *rec_diff;			// pt1_body-pt2_body difference, providing orientation
Camera left_cam("LEFT");
Camera right_cam("RIGHT");
Controller ctrl("HEAD CONTROL BOARD");
BufferedPort<Vector> recon_port;	// output port
TwoDObject left_o, right_o;			// the same tracked object, as seen from the two cameras
int delay_seconds;

float deg2rad(float d)
{
	return (float) (d / 180 * CV_PI);
}

bool ReconstructionModule::open(yarp::os::Searchable &config)
{
	if (config.check("help", "if present, display user message")) {
		ACE_OS::printf("Call with --file <config_file.ini>.\n");
		return false;
	}
	yarp::os::ConstString configFile;
	if (!config.check("file")) {
		configFile = DEFAULT_CONFIG_FILE;
	} else {
		// use file provided by user in command line
		configFile = config.find("file").asString();
	}

	Property p;
	p.fromConfigFile(configFile.c_str(), true);

	left_port_name.clear();
	left_port_name += p.findGroup("CAMSHIFT").find("port_prefix").asString();
	left_port_name += "/";
	left_port_name += p.findGroup("CAMSHIFT").find("name").asString(); // "target" or "hand"
	left_port_name += "/left/points";

	right_port_name.clear();
	right_port_name += p.findGroup("CAMSHIFT").find("port_prefix").asString();
	right_port_name += "/";
	right_port_name += p.findGroup("CAMSHIFT").find("name").asString(); // "target" or "hand"
	right_port_name += "/right/points";

	ctrl_port_name = p.findGroup("CAMSHIFT").find("port_prefix").asString();
	ctrl_port_name += "/";
	ctrl_port_name += p.findGroup("CAMSHIFT").find("name").asString(); // "target" or "hand"
	ctrl_port_name += "/head";

	// output port
	recon_port_name = p.findGroup("RECONSTRUCTION").find("out_port").asString();

	// refresh delay between two reconstruction cycles
	delay_seconds = p.findGroup("RECONSTRUCTION").find("delay").asInt();

	// open YARP ports

	// from each cam we will get 2 points
	left_cam.open(left_port_name.c_str());
	right_cam.open(right_port_name.c_str());
	
	// from head controller device we'll compute world-to-camera T matrices
	ctrl.open(ctrl_port_name.c_str());

	// output port
	recon_port.open(recon_port_name.c_str());


	b = (float) p.findGroup("HEAD").find("half_baseline").asDouble();
	// check if the following 2 values are correct (29.7, 27)!
	head_height = (float) p.findGroup("HEAD").find("height").asDouble();
	head_depth = (float) p.findGroup("HEAD").find("depth").asDouble();

	// get cam parameters. Baltazar uses Pointgrey Flea cams at 640x480 or 320x240 res
	F = (float) p.findGroup("CAMERA").find("focal_distance").asDouble();
	W = p.findGroup("CAMERA").find("width").asInt();
	H = p.findGroup("CAMERA").find("height").asInt();
	pixel_res = (float) (p.findGroup("CAMERA").find("pixel_res").asDouble() * 1e-6);

	// allocate data structures
	pt1 = gsl_vector_float_calloc(3);
	pt2 = gsl_vector_float_calloc(3);
	R_pt = gsl_matrix_float_alloc(3, 3);
	t_pt = gsl_vector_float_alloc(3);
	pt1_body = gsl_vector_float_alloc(3);
	pt2_body = gsl_vector_float_alloc(3);
	rec_diff = gsl_vector_float_alloc(3);

	return true;
}

bool ReconstructionModule::updateModule()
{
	// until this function returns false, it will be called forever by runModule()

	Vector *left_data = left_cam.read(true);
	Vector *right_data = right_cam.read(true);
	Vector *ctrl_data = ctrl.read(true);

	left_o.update_points(left_data);
	right_o.update_points(right_data);
	ctrl.update_angles(ctrl_data);

	ACE_OS::printf("%s:   pt1 %d,%d,\tpt2 %d,%d,\tangle %.3f\n", left_cam.getName().c_str(),
		left_o.getPt1().x, left_o.getPt1().y, left_o.getPt2().x, left_o.getPt2().y, left_o.getAngle());
	ACE_OS::printf("%s:   pt1 %d,%d,\tpt2 %d,%d,\tangle %.3f\n", right_cam.getName().c_str(),
		right_o.getPt1().x, right_o.getPt1().y, right_o.getPt2().x, right_o.getPt2().y, right_o.getAngle());
	ACE_OS::printf("%s:   lv %.3f, rv %.3f, pan %.3f, tilt %.3f\n", ctrl.getName().c_str(), ctrl.getLV(), ctrl.getRV(),
		ctrl.getP(), ctrl.getT());

	// update junction angles transmitted by head controller
	cos_lv = cos( deg2rad(ctrl.getLV()) );
	sin_lv = sin( deg2rad(ctrl.getLV()) );
	cos_rv = cos( deg2rad(ctrl.getRV()) );
	sin_rv = sin( deg2rad(ctrl.getRV()) );
	cos_p  = cos( deg2rad(ctrl.getP()) );
	sin_p  = sin( deg2rad(ctrl.getP()) );
	cos_t  = cos( deg2rad(ctrl.getT()) );
	sin_t  = sin( deg2rad(ctrl.getT()) );

	// 3D RECONSTRUCTION COMPUTATION
	// =============================
	// (see Samuel Ferreira's MSc thesis, paragraph 2.4.2)

	float X[2], Y[2], Z[2]; // {X[0],Y[0],Z[0]} will store reconstructed pt1 (w.r.t. nose)
							// {X[1],Y[1],Z[1]} will store reconstructed pt2 (w.r.t. nose)

	// first point
	float normalized_left_pt1_x = (left_o.getPt1().x - W / 2) * pixel_res;
	float normalized_right_pt1_x = (right_o.getPt1().x - W / 2) * pixel_res;
	float normalized_left_pt1_y = (left_o.getPt1().y - H / 2) * pixel_res;
	float normalized_right_pt1_y = (right_o.getPt1().y - H / 2) * pixel_res;

	float phi_l = normalized_left_pt1_x / F,
		  phi_r = normalized_right_pt1_x / F;

	Z[0] = (float)( 2*b /
							(
							tan( phi_l + deg2rad(ctrl.getLV()) )
							-
							tan( phi_r - deg2rad(ctrl.getRV()) )
							)
					);
	X[0] = (float)( Z[0] * (
							tan( phi_l + deg2rad(ctrl.getLV()) )
							+
							tan( phi_r - deg2rad(ctrl.getRV()) )
							)
						/ 2
					);
	Y[0] = (float)( (Z[0] / (2 * F))
						*
						(
						normalized_left_pt1_y / cos( deg2rad(ctrl.getLV()) )
						+
						normalized_right_pt1_y / cos( deg2rad(ctrl.getRV()) )
						)
					);

	//ACE_OS::printf("reconstructed pt1: [ X = %.3f, Y = %.3f, Z = %.3f ]\t\t(w.r.t. nose)\n", X[0], Y[0], Z[0]);

	// second point
	float normalized_left_pt2_x = (left_o.getPt2().x - W / 2) * pixel_res;
	float normalized_right_pt2_x = (right_o.getPt2().x - W / 2) * pixel_res;
	float normalized_left_pt2_y = (left_o.getPt2().y - H / 2) * pixel_res;
	float normalized_right_pt2_y = (right_o.getPt2().y - H / 2) * pixel_res;

	phi_l = normalized_left_pt2_x / F;
	phi_r = normalized_right_pt2_x / F;

	Z[1] = (float)( 2*b /
							(
							tan( phi_l + deg2rad(ctrl.getLV()) )
							-
							tan( phi_r - deg2rad(ctrl.getRV()) )
							)
					);
	X[1] = (float)( Z[1] * (
							tan( phi_l + deg2rad(ctrl.getLV()) )
							+
							tan( phi_r - deg2rad(ctrl.getRV()) )
							)
					  / 2
					  );
	Y[1] = (float)( (Z[1] / (2 * F))
						*
						(
						normalized_left_pt2_y / cos( deg2rad(ctrl.getLV()) )
						+
						normalized_right_pt2_y / cos( deg2rad(ctrl.getRV()) )
						)
					 );

	//ACE_OS::printf("reconstructed pt2: [ X =  %.3f, Y = %.3f, Z = %.3f ]\t\t(w.r.t. nose)\n", X[1], Y[1], Z[1]);

	// set the two 3D points we have reconstructed (w.r.t. nose)
	// 2008-06-26: inverted the signs of X and Y!
	gsl_vector_float_set( pt1, 0, - X[0] );
	gsl_vector_float_set( pt1, 1, - Y[0] );
	gsl_vector_float_set( pt1, 2, Z[0] );
	
	gsl_vector_float_set( pt2, 0, - X[1] );
	gsl_vector_float_set( pt2, 1, - Y[1] );
	gsl_vector_float_set( pt2, 2, Z[1] );

	// nose-to-body rotation matrix
	gsl_matrix_float_set( R_pt, 0, 0, cos_p );
	gsl_matrix_float_set( R_pt, 0, 1, - sin_p*sin_t );
	gsl_matrix_float_set( R_pt, 0, 2, cos_t*sin_p );
	gsl_matrix_float_set (R_pt, 1, 0, 0);
	gsl_matrix_float_set (R_pt, 1, 1, cos_t);
	gsl_matrix_float_set (R_pt, 1, 2, sin_t);
	gsl_matrix_float_set (R_pt, 2, 0, - sin_p);
	gsl_matrix_float_set (R_pt, 2, 1, - cos_p*sin_t);
	gsl_matrix_float_set (R_pt, 2, 2, cos_p*cos_t);

	ACE_OS::printf("values of nose-to-body rot. matrix: \n [ %.2f %.2f %.2f ; %.2f %.2f %.2f ; %.2f %.2f %.2f ]\n",
			gsl_matrix_float_get(R_pt,0,0),
			gsl_matrix_float_get(R_pt,0,1),
			gsl_matrix_float_get(R_pt,0,2),
			gsl_matrix_float_get(R_pt,1,0),
			gsl_matrix_float_get(R_pt,1,1),
			gsl_matrix_float_get(R_pt,1,2),
			gsl_matrix_float_get(R_pt,2,0),
			gsl_matrix_float_get(R_pt,2,1),
			gsl_matrix_float_get(R_pt,2,2)
		
		);

	// nose-to-body translation vector
	gsl_vector_float_set( t_pt, 0, 0 );
	gsl_vector_float_set( t_pt, 1, head_height );
	gsl_vector_float_set( t_pt, 2, head_depth );

	//ACE_OS::printf("reconstructed pt1: [ X = %.3f, Y = %.3f, Z = %.3f ]\t(w.r.t. nose)\n", X[0], Y[0], Z[0]);
	
	// compute matrix*vector product
	gsl_blas_sgemv( CblasNoTrans,	// type of operation: CblasNoTrans or CblasTrans or CblasConjTrans
		1.0,						// alpha coefficient
		R_pt,						// matrix
		pt1,						// vector
		0.0,						// beta coefficient
		pt1_body );					// result

	// add translation vector
	gsl_vector_float_add( pt1_body, t_pt );

	ACE_OS::printf("reconstructed pt1: [ X = %.3f, Y = %.3f, Z = %.3f ] (w.r.t. body)\n",
		gsl_vector_float_get(pt1_body,0),
		gsl_vector_float_get(pt1_body,1),
		gsl_vector_float_get(pt1_body,2)
		);

	//ACE_OS::printf("reconstructed pt2: [ X = %.3f, Y = %.3f, Z = %.3f ]\t(w.r.t. nose)\n", X[1], Y[1], Z[1]);

	// compute matrix*vector product then add translation (see above)
	gsl_blas_sgemv( CblasNoTrans, 1.0, R_pt, pt2, 0.0, pt2_body );
	gsl_vector_float_add( pt2_body, t_pt );

	ACE_OS::printf("reconstructed pt2: [ X = %.3f, Y = %.3f, Z = %.3f ] (w.r.t. body)\n",
		gsl_vector_float_get(pt2_body,0),
		gsl_vector_float_get(pt2_body,1),
		gsl_vector_float_get(pt2_body,2)
		);

	// difference between reconstructed pt1 and pt2, i.e., orientation
	gsl_vector_float_set( rec_diff, 0, gsl_vector_float_get(pt1_body,0)-gsl_vector_float_get(pt2_body,0) );
	gsl_vector_float_set( rec_diff, 1, gsl_vector_float_get(pt1_body,1)-gsl_vector_float_get(pt2_body,1) );
	gsl_vector_float_set( rec_diff, 2, gsl_vector_float_get(pt1_body,2)-gsl_vector_float_get(pt2_body,2) );
	ACE_OS::printf("difference: [ X = %.3f, Y = %.3f, Z = %.3f ]\n\n",
		gsl_vector_float_get(rec_diff,0),
		gsl_vector_float_get(rec_diff,1),
		gsl_vector_float_get(rec_diff,2)
		);

	// write reconstructed coordinates to port in the form [ pt1.x pt1.y pt1.z centroid.x centroid.y centroid.z ]
	Vector &recon_data = recon_port.prepare();
	recon_data.resize(6);
	recon_data[0] = gsl_vector_float_get(pt1_body,0); // pt1 x
	recon_data[1] = gsl_vector_float_get(pt1_body,1); // pt1 y
	recon_data[2] = gsl_vector_float_get(pt1_body,2); // pt1 z
	recon_data[3] = ( gsl_vector_float_get(pt1_body,0)+gsl_vector_float_get(pt2_body,0) ) / 2; // centroid x
	recon_data[4] = ( gsl_vector_float_get(pt1_body,1)+gsl_vector_float_get(pt2_body,1) ) / 2; // centroid y
	recon_data[5] = ( gsl_vector_float_get(pt1_body,2)+gsl_vector_float_get(pt2_body,2) ) / 2; // centroid z
	recon_port.write();

	Time::delay(delay_seconds);

	return true;
}

bool ReconstructionModule::close()
{
	ACE_OS::printf("\nClosing reconstruction module...\n");

	left_cam.close();
	right_cam.close();
	ctrl.close();
	recon_port.close();

	gsl_vector_float_free(pt1);
	gsl_vector_float_free(pt2);
	gsl_matrix_float_free(R_pt);
	gsl_vector_float_free(t_pt);
	gsl_vector_float_free(pt1_body);
	gsl_vector_float_free(pt2_body);
	gsl_vector_float_free(rec_diff);

	// is this necessary?
	//Network::fini();

	return true;
}