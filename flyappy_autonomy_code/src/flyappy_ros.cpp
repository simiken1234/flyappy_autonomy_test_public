#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;
const float dt = 1.0f/30.0f;

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;

    acc_cmd.x = 0;
    acc_cmd.y = 0;
    pub_acc_cmd_.publish(acc_cmd);

    ROS_INFO("Velocity: %f, %f", msg->x, msg->y);

    // Calculate position of Flyappy
    pos_.x += msg->x * dt;
    pos_.y += msg->y * dt;

    // Normalize to last pipe (new pipe every 1.92m)
    // screenwidth 432, pipespacing 192
    // first pipe = screenwidth + 200 + pipespacing = 824
    // our initial spawn = int(screenwidth * 0.13) = 56
    // first pipe relative to spawn = 824 - 56 = 768
    // 768 - 192 = 576
    // 768 - 200 = 568
    // Account for the starting procedure
    if (pos_.x > 3.68f)
    {
        pos_.x -= 3.68f;
        started_ = true;
    }

    // Normalize to last pipe (New pipe every 1.92m)
    if (started_)
    {
        pos_.x = std::fmod(pos_.x, 1.92f);
    }

    ROS_INFO("Position: %f, %f", pos_.x, pos_.y); 
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // Example of printing laser angle and range
    //ROS_INFO("Laser range: %f, angle: %f", msg->ranges[0], msg->angle_min);
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data)
    {
        ROS_INFO("Crash detected.");
    }
    else
    {
        ROS_INFO("End of countdown.");
    }

    // Reset parameters
    pos_.x = 0;
    pos_.y = 0;
    started_ = false;

    flyappy_ = {};
}