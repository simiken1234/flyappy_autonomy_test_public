#pragma once

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include "flyappy_autonomy_code/flyappy.hpp"

const int y_max_ = 4.1f;
const int obs_array_size_ = int(y_max_ * 20.0f) + 1;
const float dt_ = 1.0f/30.0f;  // Time step at 30Hz

geometry_msgs::Vector3 getPoint(geometry_msgs::Vector3 pos, double angle, double range);
geometry_msgs::Vector3 getIntersectPoint(geometry_msgs::Vector3 pos, double angle, float x);
int getGapQuality(int unknown_count, int free_count);

struct gap
{
  float y;
  int quality;
};

struct gap_quality_limits
{
  float q_1_min;
  float q_1_max;
  float q_3_min;
  float q_3_max;
};

class Obstacle 
{
  public:
    Obstacle();
    void clear();
    void add(float y, int state);
    void setObstacleArray(std::array<int, obs_array_size_> new_obstacle_array);
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
    void add(geometry_msgs::Vector3 pos, double angle, double range, bool print);
    std::array<int, obs_array_size_> getObstacleArray(int i);
    gap findGap(int i);

  private:
    Obstacle obs1_;
    Obstacle obs2_;
};

class FlyappyRos
{
  public:
    FlyappyRos(ros::NodeHandle& nh);
    FlyappyRos(); // For testing

    std::vector<double> getMaxYDecelSequence(double y_vel, double dist_left);
    std::vector<double> getYVelSequence(geometry_msgs::Vector3 pos, double y_vel_init, double y_target);
    double getXAccelCommand(geometry_msgs::Vector3 pos, double x_vel_init);
    void setPos(geometry_msgs::Vector3 pos);
    void setGap(int i, gap new_gap);
    double getMaxAccXDt();
    double getMaxAccYDt();
    double getMaxAccY();
    gap_quality_limits getGapQualityLimits();

  private:
    void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void gameEndedCallback(const std_msgs::Bool::ConstPtr& msg);

    void accelCommand(double x_vel, double y_vel);

    ros::Publisher pub_acc_cmd_;      ///< Publisher for acceleration command
    ros::Subscriber sub_vel_;         ///< Subscriber for velocity
    ros::Subscriber sub_laser_scan_;  ///< Subscriber for laser scan
    ros::Subscriber sub_game_ended_;  ///< Subscriber for crash detection

    Flyappy flyappy_;  ///< ROS-free main code
    geometry_msgs::Vector3 pos_;    ///< Position 
    double max_acc_y_ = 35.0d;      ///< Maximum acceleration in y direction
    double max_acc_y_dt_ = max_acc_y_ * dt_;    ///< Maximum acceleration in y direction, in a time-step
    double max_acc_x_ = 3.0d;       ///< Maximum acceleration in x direction
    double max_acc_x_dt_ = max_acc_x_ * dt_;    ///< Maximum acceleration in x direction, in a time-step
    bool started_ = false;          ///< Whether the game has started, for position normalization
    ObstaclePair obs_pair_;         ///< Obstacle pair
    gap current_gap_;
    gap next_gap_;
    std::vector <double> y_vel_seq_;
    std::vector <double> y_vel_seq_next_;
    gap_quality_limits gap_quality_limits_;
};