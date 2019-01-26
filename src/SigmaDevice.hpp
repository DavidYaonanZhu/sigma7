//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2016, Nearlab
    Refer to Readme.txt for full license text

    \author    <http://nearlab.polimi.it/>
    \author    Nima Enayati
    \version   2.0
*/
//==============================================================================

#ifndef SIGMADEVICE_HEADER_GUARD
#define SIGMADEVICE_HEADER_GUARD


//ROS
#include "ros/ros.h"
#include <tf_conversions/tf_kdl.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>

// Sigma
#include <dhdc.h>
#include <drdc.h>
#include <sensor_msgs/Joy.h>

//Filter
//#include "filters.h"
#include <Eigen/Dense>

using namespace Eigen;



namespace filters
{
class IIR
{

public:
  // [n] Elements to filter
  // [m] Filter Order
  // [b_coef] Filter denominator (size: (m+1)x1)
  // [a_coed] Filter Numerator  (size: mx1)
//  IIR(int n, int m, VectorXd b_coef, VectorXd a_coef);

  IIR(int n, int m, VectorXd b_coef, VectorXd a_coef)
      : N_elements_(n), order_(m), b_coef_(b_coef), a_coef_(a_coef)
  {
    Xmat_ = MatrixXd::Zero(N_elements_, order_ + 1);
    Ymat_ = MatrixXd::Zero(N_elements_, order_);
    flag_initialized = false;
  }

  ~IIR();

  VectorXd get_iir()
  {
    VectorXd iir = Xmat_ * b_coef_ - Ymat_ * a_coef_;
    // for (int i = 0; i < N_elements_; i++) {
    //   iir[i] = el_mat_.row(i).sum() / order_;
    // }
    return iir;
  }

  void shiftX(int n)
  {
    MatrixXd::Map(&Xmat_(0, n), N_elements_, ((order_ + 1) - n)) =
        MatrixXd::Map(&Xmat_(0, 0), N_elements_, ((order_ + 1) - n));
    MatrixXd::Map(&Xmat_(0, 0), N_elements_, n) = MatrixXd::Zero(N_elements_, n);
  }

  void shiftY(int n)
  {
    MatrixXd::Map(&Ymat_(0, n), N_elements_, (order_ - n)) =
        MatrixXd::Map(&Ymat_(0, 0), N_elements_, (order_ - n));
    MatrixXd::Map(&Ymat_(0, 0), N_elements_, n) = MatrixXd::Zero(N_elements_, n);
  }

  void addX(VectorXd new_vec)
  {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&Xmat_(0, 0), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM("filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" Mat Filter: " << el_mat_);
  }

  void addY(VectorXd new_vec)
  {
    if (new_vec.size() == N_elements_) {
      MatrixXd::Map(&Ymat_(0, 0), N_elements_, 1) = new_vec;
    } else {
      ROS_ERROR_STREAM("filter: New vector dimentions doesnt match filter dimensions");
      return;
    }
    // ROS_INFO_STREAM(" Mat Filter: " << el_mat_);
  }

  void init_full(VectorXd i_vector)
  {
    if (!flag_initialized) {
      for (int i = 0; i < order_ + 1; i++) {
        shiftX(1);
        addX(i_vector);
        if (i < order_) {
          shiftY(1);
          addY(get_iir());
        }
      }
      flag_initialized = true;
    }

  }

  VectorXd update(VectorXd new_vec)
  {
    if (!flag_initialized) {
      init_full(new_vec);
    }
    addX(new_vec);
    VectorXd out_filtered = get_iir();
    shiftX(1);
    shiftY(1);
    addY(out_filtered);

    return out_filtered;
  }

//  VectorXd get_iir();
//  void     shiftX(int n);
//  void     shiftY(int n);
//  void     init_full(VectorXd i_value);
//  void     addX(VectorXd);
//  void     addY(VectorXd);
//  VectorXd update(VectorXd);

private:
  int      N_elements_;
  int      order_;
  VectorXd a_coef_;
  VectorXd b_coef_;
  MatrixXd Xmat_;
  MatrixXd Ymat_;
  bool     flag_initialized;
}; // class end





  // ROS_INFO_STREAM(" Mat Initi: " << el_mat_);
}




//-----------------------------------------------------------------------
// Haptic device class
//-----------------------------------------------------------------------
class SigmaDevice {
public:

    // Constructor with node handle and a name space for the published topics
    SigmaDevice(ros::NodeHandle n, const std::string name_space);

    // Call back for the wrench subscriber. These are the forces and torques
    // that the sigma device will exert to the operator's hand.
    void WrenchCallback( const geometry_msgs::WrenchStampedConstPtr &msg);

    // Fetches all the measurements (Pose, Twist, button and pedal) from the
    // device
    int ReadMeasurementsFromDevice();

    // Publishing those 4 things in its name! Button and pedal will be
    // published only when they are pressed or released.
    void PublishPoseTwistButtonPedal();

    // Exert wrenches if any.
    void HandleWrench();



private:

    filters::IIR *force_filter;
    // Calibrates the device using the SDK commands
    int CalibrateDevice();

    //Force bias Falg
    bool force_bias_;
    VectorXd Force_bias_;

    double force_scale_;
private:

    // the id of the device. starts from zero and increments by one if
    // another device is connected
    static int id;

    // sigma can simulate a button with the gripper. THat is when yoy close
    // the gripper it resists a bit at the end and and springs back when you
    // release it.
    bool enable_gripper_button=0;

    // we can lock the orientation when the pedal is released. This is
    // useful for teleoperation
    bool lock_orient=0;


    // the orientation matric in the locked state
    double locked_orient[3]={0., 0., 0.};

    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TwistStamped twist_msg;
    geometry_msgs::WrenchStamped wrench;
    bool new_wrench_msg;
    std_msgs::Float32 gripper_angle;

    // the gripper button and pedal state and their previous state
    int buttons_state[2];
    int buttons_previous_state[2];

    sensor_msgs::Joy buttons_msg; // two elements, 0 is gripper button, 1
    // is pedal
    int pedal_previous_state;

    // publishers and subscribers
    ros::Publisher pub_pose;
    ros::Publisher pub_twist;
    ros::Publisher pub_gripper;
    ros::Publisher pub_buttons;
    ros::Subscriber sub_wrench;
};

#endif
