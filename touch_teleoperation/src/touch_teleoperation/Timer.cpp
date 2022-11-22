/** Timer.cpp
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

#include "Timer.hpp"

/**
* @brief Default constructor of TechUserProxy
*/
Timer::Timer(const double& rate_){

	// Set the Timer rate
	this->setRate(rate_);

}

/**
* @brief Default destroyer of TechUserProxy
*/
Timer::~Timer(){}


/**
* @brief Set function of the timer rate variable
*/
void Timer::setRate(const double& r){

	this->rate = r;

}

/**
* @brief Get function of the current time instant
*/
void Timer::getCurTime(double& time){

#ifdef _WIN32
	LARGE_INTEGER time_i;
	QueryPerformanceCounter(&time_i);
	double time;
	time = (double)time_i.QuadPart;
#elif unix
#endif
}

/**
* @brief Get function of the current time instant
*/
void Timer::getCurTime(timespec& time){
#ifdef _WIN32
#elif unix
	/** int clock_gettime(clockid_t clk_id, timespec *tp);
	 *  CLOCK_REALTIME: system real time,from 1970
	 *  CLOCK_MONOTONIC: time from system start
	 *  CLOCK_PROCESS_CPUTIME_ID: process time 
	 *  CLOCK_THREAD_CPUTIME_ID: thread time
	*/
	timespec time_i = {0, 0};
	clock_gettime(CLOCK_REALTIME, &time_i);
	time.tv_sec = time_i.tv_sec;
	time.tv_nsec = time_i.tv_nsec;
#endif
}

/**
* @brief Sleep function
* @param sec the time to sleep in seconds
*/
void Timer::timeSleep(const double& sec){

#ifdef _WIN32
	Sleep((DWORD)1000 * sec); // Sleep(ms)
#elif unix
	sleep(sec);

#endif

}

/**
* @brief Get function of the elapsed time between two instants
* @param tic starting instant
* @param toc final instant
* @return the elapsed time
*/
double Timer::elapsedTime(const double& tic, const double& toc){

	double dt;

#ifdef _WIN32
	LARGE_INTEGER rate;
	QueryPerformanceFrequency(&rate);
	dt = (toc - tic)/(double)rate.QuadPart;
#elif unix
#endif

	return dt;
}

double Timer::elapsedTime(const timespec &tic, const timespec &toc){

	double dt;

#ifdef _WIN32
#elif unix
	dt = double(toc.tv_sec - tic.tv_sec) + double(toc.tv_nsec - tic.tv_nsec)/1000000000;
#endif
	return dt;
}

