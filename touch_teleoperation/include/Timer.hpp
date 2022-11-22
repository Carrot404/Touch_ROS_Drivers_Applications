/** Timer.hpp
 * 
 * This code is cloned from https://github.com/marcofer/portable-dvrk-simulator.git
 * and modified by Songjie Xiao (songjiexiao@zju.edu.cn) mainly for academic use.
 * 
 */

#ifndef TIMER_HPP_
#define TIMER_HPP_

#ifdef _WIN32
#include <Windows.h>
#elif unix
#include <unistd.h>
#include <time.h>
#endif

class Timer{

public:

	/**
	* @brief Default constructor of TechUserProxy with rate parameter
	* @param rate_ the rate of the Timer (default is 20.0)
	*/
	Timer(const double& rate_ = 20.0);

	/**
	* @brief Default destroyer of TechUserProxy
	*/
	~Timer();

	/**
	* @brief Set function of the timer rate variable
	*/
	void setRate(const double& r);

	/**
	* @brief Get function of the timer rate variable
	*/
	double getRate(){ return this->rate;}

	/**
	* @brief Get function of the current time instant
	*/
	void getCurTime(double& time);
	void getCurTime(timespec& time);

	/**
	* @brief Sleep function
	* @param sec the time to sleep in seconds
	*/
	void timeSleep(const double& sec);

	/**
	* @brief Get function of the elapsed time between two instants
	* @param tic starting instant
	* @param toc final instant
	* @return the elapsed time
	*/
	double elapsedTime(const double& tic, const double& toc);
	double elapsedTime(const timespec &tic, const timespec &toc);




private:

	double rate;			//!< The rate of the timer

};


#endif //TIMER_HPP_