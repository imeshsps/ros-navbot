#include <chrono>
#include <functional>
#include <ros/callback_queue.h>
#include <my_robot_hw_interface.h>

void controlLoop(MyRobotHWInterface &hw, controller_manager::ControllerManager &cm, std::chrono::system_clock::time_point &last_time)
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw.read(elapsed);
    cm.update(ros::Time::now(), elapsed);
    hw.write();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_robot_base_node");

    MyRobotHWInterface hw;
    controller_manager::ControllerManager cm(&hw, hw.nh);

    double control_frequency;
    hw.private_nh.param<double>("control_frequency", control_frequency, 10.0);

    ros::CallbackQueue my_robot_queue;
    ros::AsyncSpinner my_robot_spinner(1, &my_robot_queue);

    std::chrono::system_clock::time_point last_time = std::chrono::system_clock::now();
    ros::TimerOptions control_timer(
        ros::Duration(1 / control_frequency), 
        std::bind(controlLoop, std::ref(hw), std::ref(cm), std::ref(last_time)), 
        &my_robot_queue);
    ros::Timer control_loop = hw.nh.createTimer(control_timer);
    my_robot_spinner.start();
    ros::spin();

    return 0;
}