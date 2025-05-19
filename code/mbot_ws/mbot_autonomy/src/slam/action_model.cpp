#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)     // True: robot moved; False: robot not moved
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if (!initialized_){
        previousPose_ = odometry;
        initialized_ = true;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);

    trans_ = std::sqrt(dx_ * dx_ + dy_ * dy_);

    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);
    if(std::fabs(rot1_) > M_PI/2.0){
        rot1_ = angle_diff(M_PI, rot1_);
        trans_ *= -1.0;
    }
    
    rot2_ = angle_diff(dtheta_, rot1_);


    // moved_ = (dx_ != 0.0) || (dy_ != 0.0) || (dtheta_ != 0.0);
    moved_ = (trans_ >= min_dist_) || (std::fabs(dtheta_) >= min_theta_);

    if (moved_){
        rot1Std_ = std::sqrt(k1_ * std::fabs(rot1_));
        transStd_ = std::sqrt(k2_ * std::fabs(trans_));
        rot2Std_ = std::sqrt(k1_ * std::fabs(rot2_));
    }

    resetPrevious(odometry);        // previousPose_ = odometry;
    utime_ = odometry.utime;

    return moved_;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    
    float sampledRot1 = std::normal_distribution<>(rot1_, rot1Std_)(numberGenerator_);
    float sampledTrans = std::normal_distribution<>(trans_, transStd_)(numberGenerator_);
    float sampledRot2 = std::normal_distribution<>(rot2_, rot2Std_)(numberGenerator_);

    newSample.pose.x += sampledTrans * std::cos(sampledRot1 + sample.pose.theta);
    newSample.pose.y += sampledTrans * std::sin(sampledRot1 + sample.pose.theta);
    newSample.pose.theta = wrap_to_pi(sample.pose.theta + sampledRot1 + sampledRot2);
    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}
