#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;
const float dt = 1.0f/30.0f;  // Time step at 30Hz
const double max_laser_range = 3.550000; // Known max laser range

const float y_init = 1.36f;   // Approx initial height
const float y_max = 3.8f;     // Approx max height

const float obs_width = 0.5f; // Approx obstacle width
const float obs_spacing = 1.92f; // Approx obstacle spacing

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

geometry_msgs::Vector3 getIntersectPoint(geometry_msgs::Vector3 pos, double angle, float x)
{
    // Calculate coordinate of intersection point between laser and vertical line at x
    geometry_msgs::Vector3 point;

    point.x = x;
    point.y = pos.y + ((x - pos.x) * std::tan(angle));

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

void Obstacle::add(float y, int state) 
{
    // Convert y to array index
    int i = (int)std::round((y / y_max) * 31.0f);

    // Only change state if it isn't already known to be obstacle
    if (obstacleArray_[i] != 1)
    {
        obstacleArray_[i] = state;
    }
}

std::array<int, 32> Obstacle::getObstacleArray() 
{
    return obstacleArray_;
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

void ObstaclePair::moveObs()
{
    // Move obs2 to obs1 and clear obs2 for next obstacle
    obs1_ = std::move(obs2_);
    obs2_.clear();
}

void ObstaclePair::add(geometry_msgs::Vector3 pos, double angle, double range) 
{
    // Calculate coordinate of laser impact
    geometry_msgs::Vector3 impact_point = getPoint(pos, angle, range);

    // Find which obstacle the point belongs to
    if ((obs_spacing - obs_width) <= impact_point.x && impact_point.x <= obs_spacing)
    {
        obs1_.add(impact_point.y, 1);
    }
    else if (impact_point.x > obs_spacing)
    {   
        geometry_msgs::Vector3 intersect_point = getIntersectPoint(pos, angle, (obs_spacing - (0.5*obs_width)));
        obs1_.add(intersect_point.y, 0);
        if (((2*obs_spacing) - obs_width) <= impact_point.x && impact_point.x <= (2*obs_spacing) && range < max_laser_range)
        {
            obs2_.add(impact_point.y, 1);
        }
        else if (impact_point.x > (2*obs_spacing))
        {
            intersect_point = getIntersectPoint(pos, angle, ((2*obs_spacing) - (0.5*obs_width)));
            obs2_.add(intersect_point.y, 0);
        }
    }
}

std::array<int, 32> ObstaclePair::getObstacleArray(int i) 
{
    if (i == 1)
    {
        return obs1_.getObstacleArray();
    }
    else if (i == 2)
    {
        return obs2_.getObstacleArray();
    }
    else
    {
        return {};
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
    if (started_ && pos_.x > obs_spacing)
    {
        obs_pair_.moveObs();
        pos_.x = std::fmod(pos_.x, obs_spacing);
    }

    //ROS_INFO("Position: %f, %f", pos_.x, pos_.y); 
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    geometry_msgs::Vector3 impact_point;

    // Loop through the angles in the laser scan message
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double angle = msg->angle_min + i * msg->angle_increment;
        double range = msg->ranges[i];

        // Add point to ObstaclePair
        obs_pair_.add(pos_, angle, range);
    }
    
    ROS_INFO("Laser range: %f, angle: %f", msg->ranges[4], (msg->angle_min + (msg->angle_increment * 4)));
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