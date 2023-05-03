#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "flyappy_autonomy_code/flyappy.hpp"

const int y_max_ = 4.1f;
const int obs_array_size_ = int(y_max_ * 20.0f) + 1;

geometry_msgs::Vector3 getPoint(geometry_msgs::Vector3 pos, double angle, double range);
geometry_msgs::Vector3 getIntersectPoint(geometry_msgs::Vector3 pos, double angle, float x);
int getGapQuality(int unknown_count, int free_count);

struct gap
{
  float y;
  int quality;
};

class Obstacle 
{
  public:
    Obstacle();
    void clear();
    void add(float y, int state);
    std::array<int, obs_array_size_> getObstacleArray();
    gap findGap();

  private:
    std::array<int, obs_array_size_> obstacleArray_;
};

class ObstaclePair
{
  public:
    ObstaclePair();
    void clear();
    void moveObs();
    void add(geometry_msgs::Vector3 pos, double angle, double range);
    std::array<int, obs_array_size_> getObstacleArray(int i);
    gap findGap();

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

    gap current_gap;
};