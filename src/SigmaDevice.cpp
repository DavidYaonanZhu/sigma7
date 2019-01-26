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

#include "SigmaDevice.hpp"
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

//#include "filters.h"

int SigmaDevice::id = -1;


//-----------------------------------------------------------------------
// Constructor
//-----------------------------------------------------------------------
SigmaDevice::SigmaDevice(ros::NodeHandle n, const std::string ns)
        : new_wrench_msg(false)
{
    id++;

    // setup the publishers and the subscriber
    pub_pose = n.advertise<geometry_msgs::PoseStamped>(ns+"/pose",1, 0);
    pub_twist = n.advertise<geometry_msgs::TwistStamped>(ns+"/twist", 1, 0);
    pub_gripper = n.advertise<std_msgs::Float32>(ns+"/gripper_angle", 1, 0);
    pub_buttons = n.advertise <sensor_msgs::Joy> (ns+"/buttons", 1, 0);

    std::string wrench_topic("/sigma/force_feedback");
    n.getParam("wrench_topic", wrench_topic);
    sub_wrench	= n.subscribe(wrench_topic, 1, &SigmaDevice::WrenchCallback,
                                this);

    // params
    n.param<bool>("enable_gripper_button", enable_gripper_button, 0);
    n.param<bool>("lock_orientation", lock_orient, 0);
    std::cout << enable_gripper_button;

    // calibrate the devices
    if(CalibrateDevice() == -1)
        ros::shutdown();

    buttons_msg.buttons.push_back(0);
    buttons_msg.buttons.push_back(0);

    // Filters
    Eigen::VectorXd a_coef(2);
    a_coef << -1.647459981, 0.7008967812;

    Eigen::VectorXd b_coef(3);
    b_coef << 0.01335920003,  0.02671840006, 0.01335920003;

    force_filter = new filters::IIR(6, 2, b_coef, a_coef);
    force_bias_ = false;
    Force_bias_ = VectorXd::Zero(6);
    force_scale_ = 0.1;
}


//------------------------------------------------------------------------------
// WrenchCallback
void SigmaDevice::WrenchCallback(
        const geometry_msgs::WrenchStampedConstPtr &msg) {
    //newDataDirect = true;
    wrench.wrench = msg->wrench;
    new_wrench_msg = true;
}

//------------------------------------------------------------------------------
// CalibrateDevice
int SigmaDevice::CalibrateDevice() {

    ROS_INFO("Calibrating device %i ...", id);
    //dhdOpen ();

    // open device //drdOpenID ((char)id)
    if (drdOpenID ((char)id) < 0) {
        ROS_ERROR("No device %i found. dhd says: %s", id, dhdErrorGetLastStr());
        dhdSleep (2.0);
        drdClose ((char)id);
        return -1;
    }
    //dhdOpen ();

    //Calibrate the device if it is not already calibrated;
    if(drdIsInitialized((char)id)){
        ROS_INFO("Device %i is already calibrated.", id);
    }
    else if(drdAutoInit((char)id)<0) {
        ROS_ERROR("Initialization of device %i failed. dhd says: (%s)", id,
                  dhdErrorGetLastStr ());
        dhdSleep(2.0);
    }

    // // center of workspace
    //double nullPose[DHD_MAX_DOF] = { 0.0, 0.0, 0.0, //base  (translations)
    //                                 0.0, 0.0, 0.0, //wrist (rotations)
    //                                 0.0 };         //gripper
    // //move to center
    //drdMoveTo (nullPose);

    // stop regulation (and leave force enabled)
    drdStop(true, (char)id);

    //int status;
    //ROS_INFO("mode (%c)",dhdGetStatus(&status,(char)id));

    //double version;
    //dhdGetVersion(&version,(char)id);
    //std::cout << version;

    // enable force

    //std::cout << dhdErrorGetLastStr ();
    //if(dhdEnableForce (DHD_ON, (char)id)==-1){
    //    ROS_INFO("(%s)",dhdErrorGetLastStr ());
    //}
    //dhdEnableForce (DHD_ON, (char)id);
    dhdEnableForce (DHD_ON, (char)id);
    //ROS_INFO("force (%s)",dhdErrorGetLastStr ());


    //dhdSetGravityCompensation(DHD_ON, (char)id);
    dhdSleep (0.2);

    //Enable the gripper button
    if(enable_gripper_button){
        dhdEmulateButton(DHD_ON, (char)id);
    }
    //dhdEmulateButton(DHD_ON, (char)id);
    //ROS_INFO("(%s)",dhdErrorGetLastStr ());
    //drdSleep(4.0);
    ROS_INFO("Device %i ready.", id);
    return 0;

}

int SigmaDevice::ReadMeasurementsFromDevice() {

    // -------------------------------
    // Pose
    double p[3];
    //double orient_m[3][3];
    double ox, oy, oz;
    //Reading the data from the device
    //dhdGetPositionAndOrientationFrame(&p[0], &p[1], &p[2], orient_m, (char)id); //This is original getting rotation matrix
    dhdGetPositionAndOrientationRad(&p[0], &p[1], &p[2], &ox, &oy, &oz, (char)id);
    //std::cout << p[0], p[1], p[2];

    //set regulation to roll pitch yaw to prevent unexpected UR motion
    std::cout << "roll" << ox <<std::endl;
    std::cout << "pitch" << oy <<std::endl;
    std::cout << "yaw" << oz <<std::endl;
    if (ox < -0.20) ox = -0.20;
    if (ox > 0.35)  ox = 0.35;
    if (oy < -0.50) oy = -0.50;
    if (oy > 0.60)  oy = 0.60;
    if (oz > 0.30)   oz = 0.30;
    std::cout << "roll_2" << ox <<std::endl;
    std::cout << "pitch_2" << oy <<std::endl;
    std::cout << "yaw_2" << oz <<std::endl;
    // convert to pose message
    //remapping device roll pitch yaw to ur cordinate. dev_roll->ur_pitch, dev_pitch->ur_roll dev_yaw->ur_yaw
    KDL::Rotation rot,
            rot_x(1,0,0,0,cos(oy),-sin(oy),0,sin(oy),cos(oy)),
            rot_y(cos(-ox),0,sin(-ox),0,1,0,-sin(-ox),0,cos(-ox)),
            rot_z(cos(oz),-sin(oz),0,sin(oz),cos(oz),0,0,0,1);
    //for (int r = 0; r < 3; ++r) {
    //    for (int c = 0; c < 3; ++c) { rot(r, c) = orient_m[r][c]; }
    //}
    rot = rot_x * rot_y *rot_z;

    //remapping cordinatesystem device_x to ur_y, device_y to -ur_x
    KDL::Rotation conv_rot_d2w(0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0);
    KDL::Rotation conv_rot_w2ur(0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0);
    rot = rot * conv_rot_d2w * conv_rot_w2ur;
    //Initial ur position init_x, init_y, init_z z=22cm 18cm gripper + 4cm wrist
    //double init_x = 0.17199; double init_y = 0.40659; double init_z = 0.12966;
    double init_x = 0.17446; double init_y = 0.40866; double init_z = 0.22214;

    tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(init_x+p[1],init_y-p[0],init_z+p[2])),
                     pose_msg.pose);
    //    tf::poseKDLToMsg(KDL::Frame(rot, KDL::Vector(10*p[0],10*p[1],10*p[2])),
    //pose_msg.pose);

    // stamp the msg
    pose_msg.header.stamp = ros::Time::now();
    //tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::PoseKDLToTF(KDL::Frame(rot, KDL::Vector(init_x+p[1],init_y-p[0],init_z+p[2])), transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "omega_7"));


    // -----------------------------
    // Twist
    double v[6];

    dhdGetLinearVelocity(&v[0], &v[1], &v[2], (char)id);
    dhdGetAngularVelocityRad(&v[3], &v[4], &v[5], (char)id);
    // convert to twist message
    twist_msg.twist.linear.x = v[0];
    twist_msg.twist.linear.y = v[1];
    twist_msg.twist.linear.z = v[2];
    twist_msg.twist.angular.x = v[3];
    twist_msg.twist.angular.y = v[4];
    twist_msg.twist.angular.z = v[5];
    // stamp the msg
    twist_msg.header.stamp = ros::Time::now();

    // ------------------------------
    // gripper
    double temp;
    dhdGetGripperAngleRad(&temp);
    gripper_angle.data = (float)temp;
    // ------------------------------
    // buttons
    // saving the previous states of gripper button and pedal
    for (int i = 0; i < 2; ++i) {
        buttons_previous_state[i] = buttons_state[i];
        buttons_state[i] = dhdGetButton(i, (char)id);
    }

    return 0;
}

void SigmaDevice::PublishPoseTwistButtonPedal() {

    pub_pose.publish(pose_msg);
    pub_twist.publish(twist_msg);

    // Publish gripper angle
    pub_gripper.publish(gripper_angle);

    // publish buttons when there is a change
    if((buttons_state[0] != buttons_previous_state[0]) ||
            (buttons_state[1] != buttons_previous_state[1]) ){

        // populate the message
        buttons_msg.buttons[0] = buttons_state[0];
        buttons_msg.buttons[1] = buttons_state[1];
        // publish it
        pub_buttons.publish(buttons_msg);
    }


}

void SigmaDevice::HandleWrench() {

    // should we use new_wrench_msg?
    if(true)//buttons_state[1] == 1)
    {
        Eigen::VectorXd Force_data;
        Force_data = Eigen::VectorXd::Zero(6);

        // Create a bias t=0
        if (!force_bias_)
        {
            Force_bias_[0] = wrench.wrench.force.y;
            Force_bias_[1] = -wrench.wrench.force.x;
            Force_bias_[2] = wrench.wrench.force.z;
            Force_bias_[3] = wrench.wrench.torque.y;
            Force_bias_[4] = -wrench.wrench.torque.x;
            Force_bias_[5] = wrench.wrench.torque.z;
            force_bias_ = true;
        }
        else
        {
            Force_data[0] = force_scale_*(wrench.wrench.force.y - Force_bias_[0]);
            Force_data[1] = force_scale_*(-wrench.wrench.force.x - Force_bias_[1]);
            Force_data[2] = force_scale_*(wrench.wrench.force.z -Force_bias_[2]) ;
            Force_data[3] = force_scale_*(wrench.wrench.torque.y -Force_bias_[3]);
            Force_data[4] = force_scale_*(-wrench.wrench.torque.x - Force_bias_[4]);
            Force_data[5] = force_scale_*(wrench.wrench.torque.z - Force_bias_[5]);

            force_filter->update(Force_data);

            Force_data = force_filter->get_iir();


            if (dhdSetForceAndTorqueAndGripperForce(Force_data[0],
                                                    Force_data[1],
                                                    Force_data[2],
                                                    Force_data[3],
                                                    Force_data[4],
                                                    Force_data[5],
                                                    0.0, (char)id) < DHD_NO_ERROR){
                printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
            }

    //        if (dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.y,
    //                                                -wrench.wrench.force.x,
    //                                                wrench.wrench.force.z,
    //                                                wrench.wrench.torque.y,
    //                                                -wrench.wrench.torque.x,
    //                                                wrench.wrench.torque.z,
    //                                                0.0, (char)id) < DHD_NO_ERROR){
    //            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
    //        }


            dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2]);
        }


//        Force_data[0] = wrench.wrench.force.y;
//        Force_data[1] = -wrench.wrench.force.x;
//        Force_data[2] = wrench.wrench.force.z;
//        Force_data[3] = wrench.wrench.torque.y;
//        Force_data[4] = -wrench.wrench.torque.x;
//        Force_data[5] = wrench.wrench.torque.z;

//        force_filter->update(Force_data);

//        Force_data = force_filter->get_iir();


//        if (dhdSetForceAndTorqueAndGripperForce(Force_data[0],
//                                                Force_data[1],
//                                                Force_data[2],
//                                                Force_data[3],
//                                                Force_data[4],
//                                                Force_data[5],
//                                                0.0, (char)id) < DHD_NO_ERROR){
//            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
//        }

////        if (dhdSetForceAndTorqueAndGripperForce(wrench.wrench.force.y,
////                                                -wrench.wrench.force.x,
////                                                wrench.wrench.force.z,
////                                                wrench.wrench.torque.y,
////                                                -wrench.wrench.torque.x,
////                                                wrench.wrench.torque.z,
////                                                0.0, (char)id) < DHD_NO_ERROR){
////            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
////        }


//        dhdGetOrientationRad(&locked_orient[0], &locked_orient[1],&locked_orient[2]);
    }
    else if (lock_orient){
        drdRegulatePos  (false);
        drdRegulateRot  (true);
        drdRegulateGrip (false);
        drdStart();
        drdMoveToRot (locked_orient[0], locked_orient[1],locked_orient[2]);
        drdStop(true);
    }
    else{
        if (dhdSetForceAndTorqueAndGripperForce(.0, .0, .0, .0, .0, .0, 0.,
                                                (char)id) < DHD_NO_ERROR)
            printf("error: cannot set force (%s)\n", dhdErrorGetLastStr());
        }

}

