#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "flyappy_autonomy_code/flyappy.hpp"

geometry_msgs::Vector3 getPoint(geometry_msgs::Vector3 pos, double angle, double range);
geometry_msgs::Vector3 getIntersectPoint(geometry_msgs::Vector3 pos, double angle, float x);

class Obstacle 
{
  public:
    Obstacle();
    void clear();
    void add(float y, int state);
    std::array<int, 32> getObstacleArray();

  private:
    std::array<int, 32> obstacleArray_;
};

class ObstaclePair
{
  public:
    ObstaclePair();
    void clear();
    void add(geometry_msgs::Vector3 pos, double angle, double range);
    std::array<int, 32> getObstacleArray(int i);

  private:
    Obstacle obs1_;
    Obstacle obs2_;
};

class FlyappyRos
{
  public:
    FlyappyRos(ros::NodeHandle& nh);

  private:
    void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void gameEndedCallback(const std_msgs::Bool::ConstPtr& msg);

    ros::Publisher pub_acc_cmd_;      ///< Publisher for acceleration command
    ros::Subscriber sub_vel_;         ///< Subscriber for velocity
    ros::Subscriber sub_laser_scan_;  ///< Subscriber for laser scan
    ros::Subscriber sub_game_ended_;  ///< Subscriber for crash detection

    Flyappy flyappy_;  ///< ROS-free main code

    geometry_msgs::Vector3 pos_;    ///< Position 
    bool started_ = false;          ///< Whether the game has started, for position normalization

    ObstaclePair obs_pair_;         ///< Obstacle pair
};