/// \file  pid_controller_ros.h
///   \brief Contains node handle and message subscribers for a ROS node using PID algorithm.
///

#ifndef PID_CONTROLLER_ROS_H
#define PID_CONTROLLER_ROS_H

#include "pid_controller_base.h"

// ROS includes
#include "ros/ros.h"
#include "std_msgs/Float32.h"

/// \brief A class that implements PID control algorithm within ROS.
///
/// Base PID controller class is inherited. In addition, this class
/// implements ROS callbacks.
///
class PidControllerRos : public PidControllerBase
{
public:

    /// \brief Default constructor.
    ///
    /// Does nothing
    ///
    PidControllerRos();

    /// \brief Constructor with PID params.
    ///
    /// Calls the base constructor with the same params.
    ///
    /// \param kp Proportional gain.
    /// \param ki Integral gain.
    /// \param kd Derivative gain.
    ///
    PidControllerRos(double kp, double ki, double kd);



    /// \brief Default destructor.
    ///
    /// Does nothing.
    ///
    ~PidControllerRos();

    /// Returns the newest referent process value.
    double getReference(void);

    /// Returns the newest measured process value.
    ///
    double getMeasurement(void);

    /// ROS callback function of the referent process value.
    /// \param msg ROS msg containing referent value.
    ///
    void referenceCallback(const std_msgs::Float32::ConstPtr &msg);

    /// ROS callback function of the measured process value.
    /// \param msg ROS msg containing measured value.
    ///
    void measurementCallback(const std_msgs::Float32::ConstPtr &msg);

private:

    /// The newest referent value of the process value to be controlled.
    ///
    double reference_;

    /// The newest measured value of the process value to be controlled.
    ///
    double measurement_;

};

#endif // PID_CONTROLLER_ROS_H
