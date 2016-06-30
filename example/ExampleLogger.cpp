#include <math.h> 
#include <vector>
#include <chrono>
#include <thread>

//Logger
#include <mrdplot/Logger.h>


//Simple logger example 
int main(int argc, char **argv)
{

	BatchLogger logger;
	double double_data = 0.0;
	double local_time = 0.0;
	std::vector<double> sine_waves(5, 0.0); //Create vector of 5 signals
	int count = 0;
	float sawtooth_wave = 0.0;
	float triangle_wave = 0.0;


	static const int log_time = 5;//Sec
	static const double period = 0.001;//1/Sec
	static const int max_count = (int)(log_time/period);
	std::cout << "Logging time is " << log_time << " sec" << std::endl;
	std::cout << "Period is       " << period << " 1/sec" <<std::endl;
	std::cout << "Sample count of " << max_count << std::endl;

	if (!logger.hasInited()) {
		logger.init(period);
	}

	// Add variables to be logged:
	logger.add_datapoint("double_data","s",&double_data);
	logger.add_datapoint("local_time","s",&local_time);
	logger.add_datapoint("count","s",&count);
	logger.add_datapoint("sine_wave_0","s",&sine_waves[0]);
	logger.add_datapoint("sine_wave_1","s",&sine_waves[1]);
	logger.add_datapoint("sine_wave_2","s",&sine_waves[2]);
	logger.add_datapoint("sine_wave_3","s",&sine_waves[3]);
	logger.add_datapoint("sine_wave_4","s",&sine_waves[4]);
	logger.add_datapoint("sawtooth_wave","s",&sawtooth_wave);
	logger.add_datapoint("triangle_wave","s",&triangle_wave);
	
	auto t_start = std::chrono::high_resolution_clock::now();

	while(1){

		//Forever increment value:
		double_data += 0.01;

		//Get local time
		auto t_latest = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> diff = t_latest - t_start;
		local_time = diff.count();
	
		//Random sine wave generation:
		//Amplitude*cos(2*M_PI*freq*time + phase);
		sine_waves[0] = 1.0*cos(2*M_PI*10*local_time);
		sine_waves[1] = 1.0*cos(2*M_PI*50*local_time);
		sine_waves[2] = 1.0*cos(2*M_PI*5*local_time);
		sine_waves[3] = 1.0*cos(2*M_PI*2*local_time);
		sine_waves[4] = 1.0*cos(2*M_PI*1*local_time);
		
		//Sawtooth wave generation:
		sawtooth_wave = abs((count % 100) - 1)*0.01;

		//Triangle wave generation:
		int tri_half_period = 240;
		triangle_wave = (1.0/tri_half_period) * 
			(tri_half_period - abs(count % (2*tri_half_period) - tri_half_period));
		
		//Capture data ( relis on loop happening at the right intervals)
		logger.saveData();

		//If number of max logs is reached, stop
		if(count == max_count) break; 
		count++;

		std::this_thread::sleep_for(std::chrono::microseconds(static_cast<long>(1000000*period)));

		if(count%100 == 0) std::cout << "Count " << count << " Time " << local_time << std::endl;
	}

	//Save log to file (with pre-fix "ctrl") in folder /logs/mrdplot
	logger.writeToMRDPLOT2("ctrl");
	return 0;
}

