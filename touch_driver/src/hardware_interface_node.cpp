/** hardware_interface_node.cpp
 * 
 * \brief ROS node for hardware_interface of 3D Systems Touch
 * \author Songjie Xiao (songjiexiao@zju.edu.cn)
 */

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>

#include <csignal>
#include <touch_driver/hardware_interface.h>

std::unique_ptr<touch_driver::HardwareInterface> g_hw_interface;

void signalHandler(int signum)
{
  std::cout << "touch_hardware_interface[INFO]: Interrupt signal (" << signum << ") received.\n";

  g_hw_interface.reset();

  exit(signum);
}

void *state_update(void *ptr)
{
    std::shared_ptr<GeomagicProxy> touchProxy((GeomagicProxy*) ptr);
    touchProxy->run();
    return nullptr;
}

int main(int argc, char** argv)
{
    // Set up ROS.
    ros::init(argc, argv, "touch_hardware_interface");
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // register signal SIGINT and signal handler
    signal(SIGINT, signalHandler);

    // Set up timers
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    g_hw_interface.reset(new touch_driver::HardwareInterface);

    if (!g_hw_interface->init(nh, nh_priv))
    {
        ROS_ERROR_STREAM("touch_hardware_interface[ERROR]: Could not correctly initialize touch_hardware_interface. Exiting");
        exit(1);
    }
    ROS_INFO_STREAM("touch_hardware_interface[INFO]: initialized touch_hardware_interface.");
    controller_manager::ControllerManager cm(g_hw_interface.get(), nh);

    // loop and update geoStatus
    pthread_t state_thread;
    pthread_create(&state_thread, NULL, state_update, (void*) g_hw_interface->getptrGeoProxy().get());

    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    // double expected_cycle_time = 1.0 / (static_cast<double>(g_hw_interface->getControlFrequency()));

    ros::Rate loop_rate(g_hw_interface->getControlFrequency());
    while (ros::ok())
    {
        // Receive current state from robot
        g_hw_interface->read(timestamp, period);

        // Get current time and elapsed time since last read
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
        stopwatch_last = stopwatch_now;

        cm.update(timestamp, period, g_hw_interface->shouldResetControllers());

        g_hw_interface->write(timestamp, period);

        loop_rate.sleep();

        // if (period.toSec() > expected_cycle_time)
        // {
        //     ROS_WARN_STREAM("Could not keep cycle rate of " << expected_cycle_time * 1000 << "ms");
        //     ROS_WARN_STREAM("Actual cycle time:" << period.toNSec() / 1000000.0 << "ms");
        // }
    }

    spinner.stop();
    ROS_INFO_STREAM("touch_hardware_interface[INFO]: Shutting down.");
    return 0;
}
