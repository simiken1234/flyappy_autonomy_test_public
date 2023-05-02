#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;
const float dt = 1.0f/30.0f;  // Define time step 30Hz
const float y_init = 1.36f;   // Approx initial height
const float y_max = 3.8f;     // Approx max height

//------------------------------------------------------------------------------
// GENERAL FUNCTIONS
//------------------------------------------------------------------------------

geometry_msgs::Vector3 getPoint(geometry_msgs::Vector3 pos, double angle, double range)
{
    // Calculate coordinate of laser impact from current position and laser angle and range
    geometry_msgs::Vector3 point;

    point.x = pos.x + range * std::cos(angle);
    point.y = pos.y + range * std::sin(angle);

    return point;
}

//------------------------------------------------------------------------------
// OBSTACLE CLASS
//------------------------------------------------------------------------------

Obstacle::Obstacle() 
{
    // Obstacle represents the pipe by discretizing it into a 32 element array
    // Array elements represent know state at each y-position
    // 0 = no obstacle, 1 = obstacle, 2 = unknown
    clear();
}

void Obstacle::clear() 
{
    obstacleArray_.fill(2);  // 2 = unknown
}

void Obstacle::add(float y) 
{
    // Convert y to index
    int y_index = (int)std::round((y / y_max) * 31.0f);
    //obstacleArray_[x] |= 1 << y;
}

//------------------------------------------------------------------------------
// OBSTACLEPAIR CLASS
//------------------------------------------------------------------------------

ObstaclePair::ObstaclePair() 
{
    // ObstaclePair represents the next two pipes as Obstacle objects
    clear();
}

void ObstaclePair::clear() 
{
    obs1_.clear();
    obs2_.clear();
}

void ObstaclePair::add(float x, float y) 
{
    // Convert x to index
    int x_index = (int)std::round((x / 1.92f) * 31.0f);

    // Add obstacle to correct pipe
    if (x_index < 16)
    {
        obs1_.add(y);
    }
    else
    {
        obs2_.add(y);
    }
}

//------------------------------------------------------------------------------
// FLYAPPYROS CLASS
//------------------------------------------------------------------------------

FlyappyRos::FlyappyRos(ros::NodeHandle& nh)
    : pub_acc_cmd_(nh.advertise<geometry_msgs::Vector3>("/flyappy_acc", QUEUE_SIZE)),
      sub_vel_(nh.subscribe("/flyappy_vel", QUEUE_SIZE, &FlyappyRos::velocityCallback,
                            this)),
      sub_laser_scan_(nh.subscribe("/flyappy_laser_scan", QUEUE_SIZE,
                                   &FlyappyRos::laserScanCallback, this)),
      sub_game_ended_(nh.subscribe("/flyappy_game_ended", QUEUE_SIZE,
                                   &FlyappyRos::gameEndedCallback, this))
{
    pos_.x = 0;
    pos_.y = y_init;
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Example of publishing acceleration command to Flyappy
    geometry_msgs::Vector3 acc_cmd;

    acc_cmd.x = 0;
    acc_cmd.y = 0;
    pub_acc_cmd_.publish(acc_cmd);

    //ROS_INFO("Velocity: %f, %f", msg->x, msg->y);

    // Calculate position of Flyappy
    pos_.x += msg->x * dt;
    pos_.y += msg->y * dt;

    // Account for the starting procedure
    if (pos_.x > 4.38f)
    {
        pos_.x -= 4.38f;
        started_ = true;
    }

    // Normalize to last pipe (New pipe every 1.92m)
    if (started_ && pos_.x > 1.92f)
    {
        pos_.x = std::fmod(pos_.x, 1.92f);
    }

    ROS_INFO("Position: %f, %f", pos_.x, pos_.y); 
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    geometry_msgs::Vector3 impact_point;

    // Loop through the angles in the laser scan message
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];
        
        // Get point of laser impact
        impact_point = getPoint(pos_, angle, range);

        // Add point to ObstaclePair
        obs_pair_.add(impact_point.x, impact_point.y);
    }
    
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
    pos_.y = y_init;
    started_ = false;

    flyappy_ = {};
}