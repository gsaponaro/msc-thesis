#ifndef YARPTIMER_H
#define YARPTIMER_H

#include <yarp/os/Time.h>

using namespace yarp;
using namespace yarp::os;

class YarpTimer 
{
private:
	double global_start, global_end, global_acc, lap_partial, lap_end, last_lap, lap_acc;
	int lap_counter;
public:
	YarpTimer();
	~YarpTimer();
	void reset();
	double now();
	void endlap();
	void start();
	void stop();
		
	// accessors
	int cyc();
	double lap();
	double lastlap();
	double tot();
	double avg();
};

#endif // YARPTIMER_H