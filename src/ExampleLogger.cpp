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
	std::vector<double> sine_waves(5, 0.0);
	int count = 0;
	float sawtooth_wave = 0.0;
	float triangle_wave = 0.0;


	static const double period = 0.001;//1/Sec
	static const int log_time = 5;//Sec
	static const int max_count = (int)(log_time/period);//Assume the topic is being published at 100 Hz
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


		double_data += 0.01;

		auto t_latest = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> diff = t_latest - t_start;
		local_time = diff.count();
		
		//Amplitude*cos(2*M_PI*freq*time + phase);
		sine_waves[0] = 1.0*cos(2*M_PI*10*local_time);
		sine_waves[1] = 1.0*cos(2*M_PI*100*local_time);
		sine_waves[2] = 1.0*cos(2*M_PI*5*local_time);
		sine_waves[3] = 1.0*cos(2*M_PI*2*local_time);
		sine_waves[4] = 1.0*cos(2*M_PI*1*local_time);
		
		//Sawtooth wave gen:
		sawtooth_wave = abs((count % 100) - 1);

		//Triangle wave gen:
		int triangle_period = 100;
		triangle_wave = (1.0/triangle_period) * 
			(triangle_period - abs(count % (2*triangle_period) - triangle_period));
		
		logger.saveData();


		if(count == max_count) break;
		count++;
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<long>(1000*period)));
		if(count%100 == 0) std::cout << "Count " << count << " Time " << local_time << std::endl;
	}

	logger.writeToMRDPLOT2("ctrl");
	return 0;
}

