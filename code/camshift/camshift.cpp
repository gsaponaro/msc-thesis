/*
 * CAMSHIFT algorithm implementation, based on the OpenCV demo.
 *
 * Copyright (C) 2008 Giovanni Saponaro <giovanni.saponaro@gmail.com>
 */

#include "camshift.h"
#include "YarpTimer.h"

using namespace yarp::os;
using namespace yarp::sig;

#define DEFAULT_CONFIG_FILE "camshift_target.ini"

// image frame pointers
IplImage *imagetodisplay = 0;

// program mode control variables
int select_object = 0;	// 0: not selecting an object, 1: selecting an object
int track_object = 0;	// 0: idle, -1: object just selected, 1: object with valid color model 

CvPoint origin;
CvRect selection;

// timers
YarpTimer globalTimer;		// measures duration of the application
YarpTimer cycleTimer;		// measures duration of each image cycle
YarpTimer copyTimer;		// counts time required for image copy; done when frame changes
YarpTimer colorTimer;		// counts time required for color conversion; done when frame or ROI change
YarpTimer thresholdTimer;	// counts time required for thresholding ops; done when frame or ROI or
							// threshold change
YarpTimer modelTimer;		// counts time required to compute the histogram; done when obj selection changes
YarpTimer trackTimer;		// counts time required to compute backproject and CAMSHIFT tracking;
							// done when tracking is active and frame or threshold or ROI or object selection change 
YarpTimer readTimer;		// counts time required for polling and reading data from ports
YarpTimer writeTimer;		// counts time required to write data to the ports
YarpTimer interfaceTimer;	// counts time required for user interface (ROI selection)
YarpTimer drawTimer;		// counts time required for image overlay drawing
YarpTimer displayTimer;		// counts time required for image display


int main(int argc, char *argv[])
{
	// don't use Network::init() and ::fini(), but rather a specific Network object
	Network network;
	Time::turboBoost();

	yarp::os::ConstString config_file_name;
	yarp::os::ConstString cam; // "left" or "right"

	// parse command line for a --file option
	Property cmd_line;
	cmd_line.fromCommand(argc,argv);
	if ( !cmd_line.check("file") ) {
		ACE_OS::printf("No --file option provided. Assuming %s\n", DEFAULT_CONFIG_FILE);
		config_file_name = DEFAULT_CONFIG_FILE;
	} else {
		// use file provided by user with --file
		config_file_name = cmd_line.find("file").asString();
	}

	cam = cmd_line.find("cam").asString();
	if ( !cmd_line.check("cam") || (cam != "left" && cam != "right") )
	{
		ACE_OS::printf("\nUsage: \n\n\"%s --file config_file.ini --cam left\" or \"%s --file config_file.ini --cam right\".\n\n", argv[0], argv[0]);
		return EXIT_SUCCESS;
	}

	// parse configuration file
	Property p;
	p.fromConfigFile(config_file_name.c_str(), true);
	yarp::os::ConstString port_prefix = p.findGroup("CAMSHIFT").find("port_prefix").asString();
	yarp::os::ConstString obj_name = p.findGroup("CAMSHIFT").find("name").asString(); // what we will track

	ACE_OS::printf("The following ports will be created:\n\n");
	ACE_OS::printf("\t%s/%s/%s/img/i\tfor incoming images\n", port_prefix.c_str(), obj_name.c_str(), cam.c_str());
	ACE_OS::printf("\t%s/%s/%s/img/o\tfor outgoing (backprojected) images\n", port_prefix.c_str(), obj_name.c_str(), cam.c_str());
	ACE_OS::printf("\t%s/%s/%s/roi/i\tfor input selection\n", port_prefix.c_str(), obj_name.c_str(), cam.c_str());
	ACE_OS::printf("\t%s/%s/%s/roi/o\tfor output search region\n", port_prefix.c_str(), obj_name.c_str(), cam.c_str());
	ACE_OS::printf("\t%s/%s/%s/obj/o\tfor tracked object position, size, orientation\n", port_prefix.c_str(), obj_name.c_str(), cam.c_str());

	// create a histogram text file such as "histogram_target_left.txt"
	yarp::String filename;
	filename.clear();
	filename = "histogram_";
	filename += obj_name.c_str();
	filename += "_";
	filename += cam.c_str();
	filename += ".txt";
	FILE *fp = fopen(filename.c_str(),"w");

	// image frame pointers
	IplImage	*buffer = 0,
				*image = 0,			// copy of input BGR image frame
				*hsv = 0,			// image frame converted to 3-channel HSV frame
				*hue = 0,			// single-channel hue frame
				*mask = 0,			// mask to identify object pixels
				*backproject = 0,	// color probability frame
				*histimg = 0;		// frame to display color histogram
	CvHistogram *hist = 0;			// histogram of hue value distribution

	// program mode control variables
	int backproject_mode = 0;		// mode for histogram measurement
	int show_hist = 0;				// show histogram in histogram window

	CvRect track_window;
	CvBox2D track_box;
	CvConnectedComp track_comp;

	// other variables
	int hdims = 16;					// number of hue histogram bits
	float hranges_arr[] = {0,180};	// array with hue value limit
	float *hranges = hranges_arr;	// pointer to array with limits
	int vmin = 0,					// min hue value set by user
		vmax = 256,					// max hue value set by user
		smin = 210;					// saturation limit
	// extremities of the major axis of the ellipse
	CvPoint pt1, pt2;

	ImageOf<PixelRgb> *in = 0;
	ImageOf<PixelMono> out;
	Bottle *inRoi = 0;

	// create ports
	BufferedPort< ImageOf<PixelRgb> > inPort;
    BufferedPort< ImageOf<PixelMono> > outPort;
	BufferedPort<Bottle> inRoiPort;
	BufferedPort<Vector> outRoiPort;
	BufferedPort<Vector> outObjPort;
	Terminee *terminee = 0;
	name_ports(port_prefix, obj_name, cam, inPort, outPort, inRoiPort, outRoiPort, outObjPort, &terminee);



	while(in==0) {
		// read an img from the port.
		// argument is true, so that we wait until connection is established
		in = inPort.read(true);
	}

	// not needed anymore, we use yarpdev command line parameters
	//in->resize(320,240);
	

	// retrieve image dimensions
	const int width = in->width();
	const int height = in->height();




	yarp::String histogram_win_name;
	histogram_win_name.clear();
	histogram_win_name += "Histogram (";
	histogram_win_name += obj_name.c_str();
	histogram_win_name += ", ";
	histogram_win_name += cam.c_str();
	histogram_win_name += " cam)";

	yarp::String camshift_win_name;
	camshift_win_name.clear();
	camshift_win_name += "CAMSHIFT (";
	camshift_win_name += obj_name.c_str();
	camshift_win_name += ", ";
	camshift_win_name += cam.c_str();
	camshift_win_name += ")";
	
	// create GUI objects
	cvNamedWindow( histogram_win_name.c_str(), 1 );
    cvNamedWindow( camshift_win_name.c_str(), 1 );
    cvSetMouseCallback( camshift_win_name.c_str(), on_mouse, 0 );
    cvCreateTrackbar( "Vmin", camshift_win_name.c_str(), &vmin, 256, 0 );
    cvCreateTrackbar( "Vmax", camshift_win_name.c_str(), &vmax, 256, 0 );
    cvCreateTrackbar( "Smin", camshift_win_name.c_str(), &smin, 256, 0 );

	int _vmin = vmin, _vmax = vmax, _smin = smin;

	// allocate all the buffers
	buffer = cvCreateImage( cvSize(width,height), 8, 3 );
	image = cvCreateImage( cvSize(width,height), 8, 3 );
	imagetodisplay = cvCreateImage( cvSize(width,height), 8, 3 );
    hsv = cvCreateImage( cvSize(width,height), 8, 3 );
    hue = cvCreateImage( cvSize(width,height), 8, 1 );
    mask = cvCreateImage( cvSize(width,height), 8, 1 );
	// create 1-dimensional histogram with hdims bins (array with 1 element)
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
    histimg = cvCreateImage( cvSize(320,200), 8, 3 );
    cvZero( histimg );

	out.resize(width,height);
	backproject = (IplImage *)out.getIplImage();
	
	int bin_w, i, c;

	double refreshDelay = 10;
	double timeToRefresh = refreshDelay;

	bool newimage;
	bool newselection;
	bool onselection;
	bool ontracking;
	bool newtrackbar; // trackbar changed position
	bool overlaydisplay = false; 

	// elapsedStart = Time::now();
	while(!terminee->mustQuit())
	{                  
		globalTimer.start();
		readTimer.start();

		// second read of image. caution with width and height
		in = inPort.read(false);
		if(in != 0)
			cycleTimer.start();
		
		inRoi = inRoiPort.read(false);	// read ROI
		if((inRoi!=0)  && (inRoi->size() == 4))	// got a valid selection
		{
			track_object = -1;
			
			selection.x = inRoi->get(0).asInt();
			selection.y = inRoi->get(1).asInt();
			selection.width = inRoi->get(2).asInt();
			selection.height = inRoi->get(3).asInt();

			// check limits 
			if(selection.x < 0) selection.x = 0;
			if(selection.x >= width-1) selection.x = width-2;
			if(selection.y < 0) selection.y = 0;
			if(selection.y >= height-1) selection.y = height-2;
			if(selection.width <= 0) selection.width = 1;
			if(selection.height <= 0) selection.height = 1;
			if(selection.width+selection.x > width) selection.width = width - selection.x;
			if(selection.height+selection.y > height) selection.height = height - selection.y;
		}


		newimage = (in != 0);	
		onselection = ( select_object && selection.width > 0 && selection.height > 0 );
		newselection = (track_object == -1);
		ontracking = (track_object == 1);
		newtrackbar = ((_vmin != vmin) || (_vmax != vmax) || (_smin != smin ));

		if(newtrackbar)
		{ 
			_vmin = vmin;
			_vmax = vmax;
			_smin = smin;
		}

		readTimer.stop();

		if(newimage) // get image into local memory - take the chance to convert to RGB
		{
			readTimer.endlap();
			copyTimer.start();
			// get input image into OpenCV format
			IplImage *iplin = (IplImage *)in->getIplImage();
			// OpenCV display routines use BGR format
			cvCvtColor( iplin, buffer, CV_RGB2BGR); // buffer now contains the original RGB image
			copyTimer.stop();
			copyTimer.endlap();
		
			colorTimer.start();
			// convert to HSV
			cvCvtColor( buffer, hsv, CV_BGR2HSV );
			colorTimer.stop();
			colorTimer.endlap();
		}

		if( newimage || newtrackbar )
		{

			thresholdTimer.start();
			// thresholds the saturation channel - creates a mask frame indicating pixels with "good" saturation;
			// the mask frame has true values where HSV color components are within given limits:
			//		0		<	hue			<	180
			//		smin	<	saturation	<	256
			//		vmin	<	intensity	<	vmax
			cvInRangeS( hsv, cvScalar(0,_smin,MIN(_vmin,_vmax),0),
						cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

			// splits the hue channel from the color image, i.e. creates a frame with hue values only
			cvSplit( hsv, hue, 0, 0, 0 );

			thresholdTimer.stop();
			thresholdTimer.endlap();
		}
		

		// object being selected? then draw bounding box
		if( onselection )
		{
			drawTimer.start();
			// draw a translucid region in the selection
			cvCopy(buffer, imagetodisplay);
			cvSetImageROI( imagetodisplay, selection );
			cvXorS( imagetodisplay, cvScalarAll(255), imagetodisplay, 0 );
			cvResetImageROI( imagetodisplay );
			overlaydisplay = true;
			drawTimer.stop();
			drawTimer.endlap();
		}


		if( newselection ) // object just selected - create a color model of the object
		{
			modelTimer.start();
			float max_val = 0.0f;
			
			// computes hue histogram of pixels with good saturation in the region of interest 
			cvSetImageROI( hue, selection );
			cvSetImageROI( mask, selection );

			// calculates histogram of hue values in the masked pixels of ROI.
			// &hue points to an array of images, but there is only one image because
			// hist is 1-dimensional.  The 0 serves to clear the hist first.
			cvCalcHist( &hue, hist, 0, mask );

			// write histogram to file - for debugging
			for(int b = 0; b < hdims; b++)
				fprintf(fp, "%03.2f ", cvQueryHistValue_1D( hist, b ) );
			fprintf(fp, "\n");

			// get max value in hist.  Don't ask for min value or where.
			cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
			// rescale histogram such that maximum value is 255 (1.0)
			cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );

			// prepares full image tracking
			cvResetImageROI( hue );
			cvResetImageROI( mask );
			track_window = selection;	// save selection
			track_object = 1;			// flag mode initialization done
			
			// prepares histogram display
			cvZero( histimg );			// clear the frame for histogram graph
			bin_w = histimg->width / hdims;

			// draw an image of a histogram with hdims(= 16) colored columns.
			// Colors have the hue corresponding to the hue value represented by
			// columns.
			for( i = 0; i < hdims; i++ )
			{
				int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
				CvScalar color = hsv2rgb(i*180.f/hdims);
				cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
							 cvPoint((i+1)*bin_w,histimg->height - val),
							 color, -1, 8, 0 );
			}

			ontracking = true; // to process the next if-then-else
			modelTimer.stop();
			modelTimer.endlap();
		}

		if( ontracking && ( newimage || newselection || newtrackbar ) )
		{	
			// object is selected - enter tracking mode
			trackTimer.start();

			// segment image pixels with good match to the histogram.  For each position in
			// the hue frame, put in the corresponding position of the backproject frame
			// a number telling how often that hue value was in the reference spectrum used
			// to create the histogram
			cvCalcBackProject( &hue, backproject, hist );
			cvAnd( backproject, mask, backproject, 0 );		// clear positions not in the mask

			// Call CAMSHIFT to locate the set of pixels which most likely belong to the object
			// being tracked.  Initially search within the given window, but iterate to obtain
			// an improved window that cirumscribes the object.
			//
			// In short: search iteratively for the closest center of mass to the current search
			// window.
			//
			// Returns:
			//	track_window	- contains object bounding box
			//	track_box		- contains object size and orientation
			cvCamShift( backproject, track_window,
						cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
						&track_comp, &track_box );
			
			// initialize next search window by saving the improved window for the next frame
			track_window = track_comp.rect;
    
			// replace input image with backprojected frame in backproject mode
			if( backproject_mode )
				// display backproject image
				cvCvtColor( backproject, image, CV_GRAY2BGR );

			// upside down image?
			if( !imagetodisplay->origin )
			{
				//ACE_OS::printf("DEBUG: upside down image => changing sign of box angle\n");
				track_box.angle = -track_box.angle;
			}

			//printf("width: %f height: %f\n", track_box.size.width, track_box.size.height);
			//printf("x: %f y: %f\n", track_box.center.x, track_box.center.y);

			// testing for NAN's TO BE IMPROVED!!!!!!!!!!!!!!!!!!!!!!
			//if( (track_box.size.width > 10000) || (track_box.size.width < 1) )
			if (_isnan(track_box.size.width))
			{
				printf("Numeric error\n");
			}
			else
			{
				// draw an ellipse over the image to display

				cvCopy(buffer, imagetodisplay);
				cvEllipseBox( imagetodisplay, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );

				double cos_a = cos(track_box.angle / 180 * CV_PI),
					   sin_a = sin(track_box.angle / 180 * CV_PI);

				//ACE_OS::printf("angle: %f\n", track_box.angle);

				// calculate the extremities (approximating) the major axis of the ellipse
				pt1 = cvPoint( cvRound(track_box.center.x - cos_a*track_box.size.height/2),
									   cvRound(track_box.center.y + sin_a*track_box.size.width/2) );
				pt2 = cvPoint( cvRound(track_box.center.x + cos_a*track_box.size.height/2),
									   cvRound(track_box.center.y - sin_a*track_box.size.width/2) );
				// draw a segment approximating the major axis of the ellipse
				cvLine(imagetodisplay, pt1, pt2, CV_RGB(0,0,255), 2, CV_AA, 0);

				// draw two green circles on pt1 and centroid
				cvCircle(imagetodisplay,
						pt1,				// centre
						4,					// radius
						CV_RGB(0,255,0),	// colour
						2,					// thickness
						CV_AA,				// line type
						0);					// shift
				
				CvPoint centroid;
				centroid.x = (pt1.x + pt2.x) / 2;
				centroid.y = (pt1.y + pt2.y) / 2;
				cvCircle(imagetodisplay,
						centroid,
						4,
						CV_RGB(0,244,0),
						2,
						CV_AA,
						0);
				
				/* draw pt2
				cvCircle(imagetodisplay, pt2, 4, CV_RGB(0,255,0), 2, CV_AA, 0);
				*/

				overlaydisplay = true;
			};
			trackTimer.stop();
			trackTimer.endlap();


			// write data to ports
			writeTimer.start();

			Vector &roiData = outRoiPort.prepare();
			roiData.resize(4);
			roiData[0] = track_comp.rect.x;
			roiData[1] = track_comp.rect.y;
			roiData[2] = track_comp.rect.width;
			roiData[3] = track_comp.rect.height;

			Vector &objData = outObjPort.prepare();
			objData.resize(5);
			/*
			objData[0] = track_box.center.x;  
			objData[1] = track_box.center.y;
			objData[2] = track_box.size.width;
			objData[3] = track_box.size.height;
			*/			
			objData[0] = pt1.x;
			objData[1] = pt1.y;
			objData[2] = pt2.x;
			objData[3] = pt2.y;
			objData[4] = track_box.angle;			

			outRoiPort.write();
			outObjPort.write();

			ImageOf<PixelMono> &outMono = outPort.prepare();
			outMono.resize(width, height);
			outMono = out;
			outPort.write();

			writeTimer.stop();
			writeTimer.endlap();
		}
				
		displayTimer.start();
		// refresh image displays
		if(overlaydisplay)
			cvShowImage( camshift_win_name.c_str(), imagetodisplay );
		else
			cvShowImage( camshift_win_name.c_str(), buffer );

		cvShowImage( histogram_win_name.c_str(), histimg );
		c = cvWaitKey(1);
        if( c == 27 )
            break;
        switch( c )
        {
        case 'b':
            backproject_mode ^= 1;
            break;
        case 'c':
            track_object = 0;
			overlaydisplay = false;
            cvZero( histimg );
            break;
        case 'h':
            show_hist ^= 1;
            if( !show_hist )
                cvDestroyWindow( histogram_win_name.c_str() );
            else
                cvNamedWindow( histogram_win_name.c_str(), 1 );
            break;
        default:
            ;
        }

		displayTimer.stop();
		displayTimer.endlap();
		
		

		if( globalTimer.now() >= timeToRefresh)
		{
			timeToRefresh += refreshDelay;

			printf("Cycle %d; Time %03.2f Duration (ms) %03.2f Avg frame rate %03.2f\n", cycleTimer.cyc(), globalTimer.now(), cycleTimer.lastlap()*1000, cycleTimer.cyc()/globalTimer.now());
			printf("Copy   time (ms): %03.2f. (Average: %03.2f)\n", copyTimer.lastlap()*1000, copyTimer.avg()*1000);
			printf("Color  time (ms): %03.2f. (Average: %03.2f)\n", colorTimer.lastlap()*1000, colorTimer.avg()*1000);
			printf("Thresh time (ms): %03.2f. (Average: %03.2f)\n", thresholdTimer.lastlap()*1000, thresholdTimer.avg()*1000);
			printf("Model  time (ms): %03.2f. (Average: %03.2f)\n", modelTimer.lastlap()*1000, modelTimer.avg()*1000);
			printf("Track  time (ms): %03.2f. (Average: %03.2f)\n", trackTimer.lastlap()*1000, trackTimer.avg()*1000);
			printf("Read   time (ms): %03.2f. (Average: %03.2f)\n", readTimer.lastlap()*1000, readTimer.avg()*1000);
			printf("Write  time (ms): %03.2f. (Average: %03.2f)\n", writeTimer.lastlap()*1000, writeTimer.avg()*1000);
			printf("Interf.time (ms): %03.2f. (Average: %03.2f)\n", interfaceTimer.lastlap()*1000, interfaceTimer.avg()*1000);
			printf("Draw   time (ms): %03.2f. (Average: %03.2f)\n", drawTimer.lastlap()*1000, drawTimer.avg()*1000);
			printf("Disp.  time (ms): %03.2f. (Average: %03.2f)\n", displayTimer.lastlap()*1000, displayTimer.avg()*1000);
		}
		
		if(newimage) 
		{	
			globalTimer.stop();
			globalTimer.endlap();
			cycleTimer.stop();
			cycleTimer.endlap();
		}	
	}

	fclose(fp);

	// known issue: ports do not quit gracefully when user closes CAMSHIFT windows.
	// the only way to quit gracefully, at the moment, is to type this in a shell:
	//    yarp terminate /balta/camshift/{left,right}/quit
	delete terminee;

	inPort.close();
	outPort.close();
	inRoiPort.close();
	outRoiPort.close();
	outObjPort.close();

	return EXIT_SUCCESS;
}

void on_mouse( int event, int x, int y, int flags, void *param )
{
// on_mouse() - mouse callback routine, called when OpenCV GUI window is clicked.
//
// event -	reason why the routine is called
// x -		mouse x coordinate
// y -		mouse y coordinate
// flags -	state of mouse and other buttons
// param -	pointer to user-chosen data structure, used for return values.
//
// note:	communication with the main program is done via shared variables,
//			NOT via a data structure supplied by "param".  Access to the shared
//			data is NOT synchronous.
//
// The mouse is used for initial object selection.  The selection process begins
// when the mouse button is pressed and ends when the mouse button is released.
// During selection, select_button is 1, then it is set to 0 when the selection
// ends.  At the end of the selection, a new tracking mode is entered by setting
// track_object to -1.

	interfaceTimer.start();
    if( !imagetodisplay )
        return;

    if( imagetodisplay->origin )
        y = imagetodisplay->height - y;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);
        
        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, imagetodisplay->width );
        selection.height = MIN( selection.height, imagetodisplay->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

	// mouse button down indicates one selection corner; mouse button up
	// indicates the opposite corner and causes tracking mode to restart.
    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
		track_object = 0;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;		// end selection mode
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;	// begin new tracking
        break;
    }
	interfaceTimer.stop();
	interfaceTimer.endlap();
}


CvScalar hsv2rgb( float hue )
{
	// hsv2rgb - create a saturated color with given hue.

    int rgb[3], p, sector;
	// array with the 6 brightest fully-saturated colors:
    static const int sector_data[][3] =
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;

	// interpolate between neighbouring colors of the above color hexagon
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

void name_ports(yarp::os::ConstString port_prefix, yarp::os::ConstString obj_name, yarp::os::ConstString cam,
				BufferedPort<ImageOf<PixelRgb> > &inPort,
				BufferedPort<ImageOf<PixelMono> > &outPort,
				BufferedPort<Bottle> &inRoiPort,
				BufferedPort<Vector> &outRoiPort,
				BufferedPort<Vector> &outObjPort,
				Terminee **terminee)
{
	yarp::String inImageName = port_prefix.c_str();
	inImageName += "/";
	inImageName += obj_name.c_str();
	inImageName += "/";
	inImageName += cam.c_str();
	inImageName += "/img/i";

	yarp::String outImageName = port_prefix.c_str();
	outImageName += "/";
	outImageName += obj_name.c_str();
	outImageName += "/";
	outImageName += cam.c_str();
	outImageName += "/img/o";

	yarp::String roiInName = port_prefix.c_str();
	roiInName += "/";
	roiInName += obj_name.c_str();
	roiInName += "/";
	roiInName += cam.c_str();
	roiInName += "/roi/i";

	yarp::String roiOutName = port_prefix.c_str();
	roiOutName += "/";
	roiOutName += obj_name.c_str();
	roiOutName += "/";
	roiOutName += cam.c_str();
	roiOutName += "/roi/o";

	yarp::String objOutName = port_prefix.c_str();
	objOutName += "/";
	objOutName += obj_name.c_str();
	objOutName += "/";
	objOutName += cam.c_str();
	objOutName += "/obj/o";

	inPort.open(inImageName.c_str());
	outPort.open(outImageName.c_str());
	inRoiPort.open(roiInName.c_str());
	outRoiPort.open(roiOutName.c_str());
	outObjPort.open(objOutName.c_str());

	yarp::String term = port_prefix.c_str(); // ?
	term.clear();
	term += port_prefix.c_str();
	term += "/";
	term += obj_name.c_str();
	term += "/";
	term += cam.c_str();
	term += "/quit";

    *terminee = new Terminee(term.c_str());
    if(*terminee == 0) 
		ACE_OS::printf("Can't allocate terminator socket port\n");
    if (!((*terminee)->isOk())) 
		ACE_OS::printf("Failed to create terminator socket port\n");
}