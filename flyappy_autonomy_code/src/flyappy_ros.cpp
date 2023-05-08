#include "flyappy_autonomy_code/flyappy_ros.hpp"

constexpr uint32_t QUEUE_SIZE = 5u;
const double max_laser_range = 3.50000; // Known max laser range - 0.05 for margin

const float y_init = 1.555f;   // Approx initial height

const float obs_width = 0.7f; // Approx obstacle width
const float obs_width_x_accel = 0.7f; // Obstacle width for x-accel
const float obs_spacing = 1.92f; // Approx obstacle spacing
const float obs_gap = 0.5f;  // The gap between upper and lower obstacle
const int obs_gap_i = (int)std::ceil((obs_gap / y_max_) * float(obs_array_size_)) - 2; // The gap between upper and lower obstacle in array index

const float early_accel_offset = 0.1f; // Offset to start accelerating early

//------------------------------------------------------------------------------
// GENERAL FUNCTIONS
//------------------------------------------------------------------------------

geometry_msgs::Vector3 getPoint(geometry_msgs::Vector3 pos, double angle, double range)
{
    /*
    Calculate coordinate of laser impact from current position and laser angle and range
    - pos: current position [m]
    - angle: angle from pos, 0 = straight ahead, positive up [rad]
    - range: distance to impact along angle [m]
    returns: coordinate of impact in 2D space [m]
    */
    geometry_msgs::Vector3 point;

    point.x = pos.x + range * std::cos(angle);
    point.y = pos.y + range * std::sin(angle);

    return point;
}

geometry_msgs::Vector3 getIntersectPoint(geometry_msgs::Vector3 pos, double angle, float x)
{
    /*
    Calculate coordinate of intersection point between laser and vertical line at x
    - pos: current position [m]
    - angle: angle from pos, 0 = straight ahead, positive up [rad]
    - x: x-coordinate of vertical line to intersect with angle [m]
    returns: coordinate of intersection point in 2D space [m]
    */
    geometry_msgs::Vector3 point;

    point.x = x;
    point.y = pos.y + ((x - pos.x) * std::tan(angle));

    return point;
}

int getGapQuality(int unknown_count, int free_count)
{
    /*
    Calculate quality of gap based on number of unknown and free elements
    - unknown_count: number of unknown elements in gap
    - free_count: number of free elements in gap
    returns: quality of gap
    */
    if ((unknown_count + free_count) >= obs_gap_i)
    {
        // If gap is big enough, evaluate quality
        int gap_quality = unknown_count + (obs_array_size_ * free_count); // Rationale for formula in docs
        return gap_quality;
    }
    else {return 0;}
}

//------------------------------------------------------------------------------
// OBSTACLE CLASS
//------------------------------------------------------------------------------

Obstacle::Obstacle() 
{
    /*
    Obstacle represents the pipe by discretizing it into a obs_array_size_ element array
    Array elements represent know state at each y-position
    0 = no obstacle, 1 = obstacle, 2 = unknown
    */
    clear();
}

void Obstacle::clear() 
{
    /*
    Clears the obstacle array by filling it with 2's = unknown state
    */
    obstacleArray_.fill(2);
}

void Obstacle::add(float y, int state) 
{
    /*
    Adds a new obstacle reading to the obstacle array
    - y: y-coordinate of reading [m]
    - state: state of reading, 0 = no obstacle, 1 = obstacle
    */

    // Convert y to array index
    int i = (int)std::round((y / y_max_) * float(obs_array_size_ - 1));

    // Only change state if it isn't already known to be obstacle
    if (obstacleArray_[i] != 1)
    {
        obstacleArray_[i] = state;
    }
}

void Obstacle::setObstacleArray(std::array<int, obs_array_size_> new_obstacle_array)
{
    /*
    Sets the obstacle array to a new array, used for testing
    - new_obstacle_array: new obstacle array
    */
    obstacleArray_ = new_obstacle_array;
}

gap Obstacle::findGap()
{
    /*
    Find best gap available and return its quality and middle y coordinate
    returns: gap struct with quality and y-coordinate of the best gap found
    */

    int free_count = 0;
    int unknown_count = 0;

    int current_gap_quality;

    int best_gap_quality = 0;
    float best_gap_y = 0.0f;

    for (int i = 0; i < obs_array_size_; i++)
    {
        switch (obstacleArray_[i])
        {
            case 0:
                free_count++;
                break;
            case 2:
                unknown_count++;
                break;
            case 1:
                // Once obstacle found, check current gap quality
                current_gap_quality = getGapQuality(unknown_count, free_count);
                if (current_gap_quality > best_gap_quality)
                {
                    // If gap is the best found yet, update best
                    best_gap_quality = current_gap_quality;
                    best_gap_y = (float(i - 1) - (float(free_count + unknown_count) / 2)) * (y_max_ / float(obs_array_size_)); 
                    // y is for middle of the gap
                }

                // Reset unknown and free counters, new gap beginning
                unknown_count = 0;
                free_count = 0;
                break;
        }
    }

    // Check gap once more, in case last element was not an obstacle
    current_gap_quality = getGapQuality(unknown_count, free_count);
    if (current_gap_quality > best_gap_quality)
    {
        // If gap is the best found yet, update best
        best_gap_quality = current_gap_quality;
        best_gap_y = (float(obs_array_size_ - 1) - (float(free_count + unknown_count) / 2)) * (y_max_ / float(obs_array_size_));
    }

    // If no suitable gap was found, something must have gone wrong. Failsafe by clearing obs to start over
    if (best_gap_quality == 0)
    {
        clear();
    }
    return {best_gap_y, best_gap_quality};
}

std::array<int, obs_array_size_> Obstacle::getObstacleArray() 
{
    /*
    Returns the obstacle array, used for testing
    returns: obstacle array
    */
    return obstacleArray_;
}

//------------------------------------------------------------------------------
// OBSTACLEPAIR CLASS
//------------------------------------------------------------------------------

ObstaclePair::ObstaclePair() 
{
    /*
    ObstaclePair represents the next two pipes as Obstacle objects
    */
    clear();
}

void ObstaclePair::clear() 
{
    /*
    Clears the obstacle pair by clearing both obstacles, setting all their states to unknown
    */
    obs1_.clear();
    obs2_.clear();
}

void ObstaclePair::moveObs()
{
    /*
    Move obs2 to obs1 and clear obs2 for next obstacle
    Used once Flyappy moves past obs1, which then no longer needs to be tracked
    */
    obs1_ = std::move(obs2_);
    obs2_.clear();
}

void ObstaclePair::add(geometry_msgs::Vector3 pos, double angle, double range, bool print) 
{
    /*
    Adds a new obstacle reading from laser to the obstacle pair, if it belongs to either obstacle
    - pos: current position [m]
    - angle: angle of laser [rad]
    - range: range of laser measurement [m]
    - print: whether to print debug info
    */

    // Calculate coordinate of laser impact
    geometry_msgs::Vector3 impact_point = getPoint(pos, angle, range);

    // Find which obstacle the point belongs to
    if ((obs_spacing - obs_width) <= impact_point.x && impact_point.x <= obs_spacing)
    {
        if (print){ROS_INFO("o1-s1");}
        obs1_.add(impact_point.y, 1);
    }
    else if (impact_point.x > obs_spacing)
    {   
        geometry_msgs::Vector3 intersect_point = getIntersectPoint(pos, angle, (obs_spacing - (0.5*obs_width)));
        if (print){ROS_INFO("o1-s0");}
        obs1_.add(intersect_point.y, 0);
        if (((2*obs_spacing) - obs_width) <= impact_point.x && impact_point.x <= (2*obs_spacing) && range < max_laser_range)
        {
            if (print){ROS_INFO("o2-s1");}
            obs2_.add(impact_point.y, 1);
        }
        else if (impact_point.x > (2*obs_spacing))
        {
            if (print){ROS_INFO("o2-s0");}
            intersect_point = getIntersectPoint(pos, angle, ((2*obs_spacing) - (0.5*obs_width)));
            obs2_.add(intersect_point.y, 0);
        }
    }
}

gap ObstaclePair::findGap(int i)
{
    /*
    Find best gap available and return its quality and middle y coordinate
    - i: which obstacle to find gap for
    returns: gap struct with quality and y-coordinate of the best gap found
    */

    if (i == 1)
    {
        return obs1_.findGap();
    }
    else if (i == 2)
    {
        return obs2_.findGap();
    }
    else
    {
        return {};
    }
}

std::array<int, obs_array_size_> ObstaclePair::getObstacleArray(int i) 
{
    /*
    Returns the obstacle array of the specified obstacle, used for testing
    - i: which obstacle to return array for
    returns: obstacle array
    */
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
    /*
    The main class of the flyappy, handles all the logic, stores all the data and publishes the acceleration commands
    */
    pos_.x = 0;
    pos_.y = y_init;
    current_gap_ = {y_max_ / 2, 0};
    obs_pair_.clear();
    // Gap quality margins used for x-accel
    gap_quality_limits_.q_1_max = 0.95;
    gap_quality_limits_.q_1_min = 0.65;
    gap_quality_limits_.q_3_max = 0.8;
    gap_quality_limits_.q_3_min = 0.5;
}

FlyappyRos::FlyappyRos()
{
    // The same constructor but without the ros requirements, for testing
    pos_.x = 0;
    pos_.y = y_init;
    current_gap_ = {y_max_ / 2, 0};
    obs_pair_.clear();
    gap_quality_limits_.q_1_max = 0.95;
    gap_quality_limits_.q_1_min = 0.65;
    gap_quality_limits_.q_3_max = 0.8;
    gap_quality_limits_.q_3_min = 0.5;
}

void FlyappyRos::velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    /*
    Callback function for velocity, updates the position of Flyappy
    This also acts as the "main" loop for control, updating the planned y-velocity sequence and publishing the acceleration commands
    - msg: velocity message
    */

    // Calculate position of Flyappy
    pos_.x += msg->x * dt_;
    pos_.y += msg->y * dt_;

    // Account for the starting procedure
    if (pos_.x > 4.38f)   // 4.38 means there is no chance for flappy to hit the back of his head
    {
        pos_.x -= 4.38f;
        started_ = true;
        obs_pair_.clear();
    }

    // Normalize to last pipe (New pipe every 1.92m)
    if (started_ && pos_.x > obs_spacing)
    {
        obs_pair_.moveObs();
        pos_.x = std::fmod(pos_.x, obs_spacing);
    }

    // Update y-velocity sequence
    if (started_){
        y_vel_seq_ = getYVelSequence(pos_, msg->y, current_gap_.y);
        geometry_msgs::Vector3 pos_next_gate;
        pos_next_gate.x = 0;
        pos_next_gate.y = current_gap_.y;
        y_vel_seq_next_ = getYVelSequence(pos_next_gate, 0.0d, next_gap_.y);
    }

    // Calculate and publish accel command
    accelCommand(msg->x, msg->y);
}

void FlyappyRos::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{   
    /*
    Callback function for laser scan, updates the obstacle pair using the laser scan data
    - msg: laser scan message
    */

    bool print = false;
    geometry_msgs::Vector3 impact_point;
    if (started_)
    {
        // Loop through the angles in the laser scan message
        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double angle = msg->angle_min + i * msg->angle_increment;
            double range = msg->ranges[i];

            // Add point to ObstaclePair
            obs_pair_.add(pos_, angle, range, print);
            print = false;
        }

        // Update gaps with new info
        current_gap_ = obs_pair_.findGap(1);
        next_gap_ = obs_pair_.findGap(2);
    }
}

void FlyappyRos::gameEndedCallback(const std_msgs::Bool::ConstPtr& msg)
{
    /*
    Callback function for game ended, resets the parameters
    - msg: game ended message
    */

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

    obs_pair_.clear();
}

void FlyappyRos::accelCommand(double x_vel, double y_vel)
{
    /*
    Calculates the acceleration command and publishes it
    Sources the x-accel command from getXAccelCommand function
    Sources the y-accel command from the predicted y-velocity sequence
    Adds a pipe wiggle if when inside the pipe to help exploration
    - x_vel: x-velocity of Flyappy
    - y_vel: y-velocity of Flyappy
    */

    geometry_msgs::Vector3 acc_cmd;

    if (started_)
    {
        acc_cmd.x = getXAccelCommand(pos_, x_vel);
        
        if (pos_.x >= (obs_spacing - early_accel_offset))
        {
            // Start accelerating for the next gate before fully out of the pipe
            acc_cmd.y += (y_vel_seq_next_[1] - y_vel_seq_next_[0]) * max_acc_y_;
        }
        else
        {
            acc_cmd.y += (y_vel_seq_[1] - y_vel_seq_[0]) * max_acc_y_;
        }

        // Add a pipe wiggle
        if (pos_.x > obs_spacing - obs_width && pos_.x < obs_spacing - early_accel_offset)
        {
            if (std::abs(acc_cmd.y) < 0.1 && std::abs(y_vel) < 0.1)
            {
                acc_cmd.y += max_acc_y_;
            }
        }
    }
    else
    {
        acc_cmd.x = 0;
        acc_cmd.y = 0;
    }

    // Publish command
    pub_acc_cmd_.publish(acc_cmd);
}

std::vector<double> FlyappyRos::getMaxYDecelSequence(double y_vel, double dist_left)
{
    /*
    Calculates the maximum y-deceleration sequence that can be applied
    Used by getYVelSequence function
    - y_vel: y-velocity at start of decel. [normalized]
    - dist_left: distance left to travel [normalized]
    returns: a (potentially unfinished) y-velocity sequence [normalized]
    */

    std::vector<double> vel_seq;
    vel_seq.push_back(y_vel);
    int vel_sign_init = (y_vel > 0) ? 1 : -1;

    // We can decelerate at max. rate until the sign of the velocity changes
    while (((y_vel - (vel_sign_init * 1)) >= 0) == (vel_sign_init > 0)){
        y_vel -= vel_sign_init;
        vel_seq.push_back(y_vel);
        dist_left -= y_vel * dt_;
    }

    // Check how much past zero we can accelerate and add the last velocity
    if (std::abs(dist_left) >= std::abs((y_vel - (vel_sign_init * 1)) * dt_))
    {
        // If we are not reaching the target, apply max. acceleration
        y_vel -= vel_sign_init;
        vel_seq.push_back(y_vel);
        dist_left -= y_vel * dt_;
    }
    else
    {
        // Else, apply acceleration to exactly meet target
        vel_seq.push_back(dist_left / dt_);
        dist_left = 0;
    }

    return vel_seq;
}

std::vector<double> FlyappyRos::getYVelSequence(geometry_msgs::Vector3 pos, double y_vel_init, double y_target)
{
    /*
    Calculates the y-velocity sequence that can be applied to reach the target
    This y-velocity sequence is almost optimal given the acceleration limits
    Getting as close to y_target as possible as soon as possible is prioritized, to avoid hitting the bottom of the pipe
    - pos: current position [m]
    - y_vel_init: current y-velocity [m/s]
    - y_target: target y-position [m]
    returns: y-velocity sequence [normalized to max. acceleration units]
    */

    // Normalize distance and velocity to units of 1 maximum acceleration
    double dist_target = (y_target - pos.y) / max_acc_y_dt_;
    double dist_left = dist_target; // For tracking distance travelled
    y_vel_init = y_vel_init / max_acc_y_dt_;

    int vel_sign_init = (y_vel_init >= 0) ? 1 : -1;
    int dist_sign_init = (dist_target >= 0) ? 1 : -1;

    std::vector<double> vel_seq;

    // Check that velocity and distance left have the same sign
    if (vel_sign_init != dist_sign_init)
    {
        // If not, special case
        // Decrease vel to zero, update pos and call function again
        vel_seq = getMaxYDecelSequence(y_vel_init, dist_left);

        // Update pos and y_vel_init
        for (int i = 1; i < vel_seq.size(); i++)
        {
            pos.y += vel_seq[i] * max_acc_y_dt_ * dt_;
        }
        y_vel_init = vel_seq.back() * max_acc_y_dt_;
        vel_seq.pop_back(); // This value will be added back by the recursive call

        // Recursive call to finish the rest of the sequence
        std::vector <double> vel_seq_rest = getYVelSequence(pos, y_vel_init, y_target);
        for (int i = 0; i < vel_seq_rest.size(); i++)
        {
            vel_seq.push_back(vel_seq_rest[i]);
        }

        return vel_seq;
    }

    // Create optimal sequence of velocities
    vel_seq.push_back(y_vel_init);

    int y_vel_int;
    if (vel_sign_init == 1){
        y_vel_int = std::floor(y_vel_init);
    }
    else
    {
        y_vel_int = std::ceil(y_vel_init);
    }
    
    // First fill with deceleration from current vel to zero
    vel_seq.push_back(y_vel_int); // Highest velocity first
    dist_left -= (y_vel_int * dt_);

    while(y_vel_int != 0)
    {
        y_vel_int -= vel_sign_init;
        vel_seq.push_back(y_vel_int);
        dist_left -= y_vel_int * dt_;
    }

    // Special case for no distance left after decel
    if (dist_left == 0)
    {
        return vel_seq;
    }

    // Special case for overshoot
    if ((dist_left * dist_sign_init) < 0)
    {
        // Throw out the current vel_seq and get one from max_decel, then see whats next recursively. Unless dist_left after decel is zero
        vel_seq = getMaxYDecelSequence(y_vel_init, dist_target);
        // Update pos
        for (int i = 1; i < vel_seq.size(); i++) // Start at 1 to skip the first value which is just the speed at start
        {
            pos.y += vel_seq[i] * max_acc_y_dt_ * dt_;
        }

        // Check if we are done
        if (pos.y == y_target)
        {
            vel_seq.push_back(0);
            return vel_seq;
        }

        // If not done, update y_vel_init and call function again
        y_vel_init = vel_seq.back() * max_acc_y_dt_;
        vel_seq.pop_back(); // This value will be added back by the recursive call

        // Recursive call to finish the rest of the sequence if not done
        std::vector <double> vel_seq_rest = getYVelSequence(pos, y_vel_init, y_target);
        for (int i = 0; i < vel_seq_rest.size(); i++)
        {
            vel_seq.push_back(vel_seq_rest[i]);
        }

        return vel_seq;
    }

    // If positive distance left (default case) try to accelerate

    // Get the max int currently in the vector
    int vel_int_max = vel_seq[1];
    int i_max;

    // If we are going to accelerate by more than 1 full unit, we need to decelerate back from it as well
    // Therefore, while dist left fulfils the condition below, we can keep going faster
    while (std::abs(dist_left) >= std::abs(2 * (vel_int_max + vel_sign_init) * dt_))
    {   
        // Find the first instance of the biggest number in the vel_seq, insert (vel_int_max + 1) after it twice
        i_max = 0;
        while (std::abs(vel_seq[i_max]) < std::abs(vel_int_max))
        {
            i_max++;
        }
        vel_int_max += (vel_sign_init * 1);
        vel_seq.insert(vel_seq.begin() + i_max + 1, vel_int_max);
        vel_seq.insert(vel_seq.begin() + i_max + 2, vel_int_max);
        dist_left -= 2 * vel_int_max * dt_;
    }

    // If dist_left is equal to between (vel_int_max + 1) and 2*(vel_int_max + 1), we need to add one more acceleration
    if (std::abs(dist_left) >= std::abs((vel_int_max + vel_sign_init)) * dt_)
    {
        // Find the first instance of vel_int_max in the vector, insert vel_int_max + 1 after it
        i_max = 0;
        while (std::abs(vel_seq[i_max]) < std::abs(vel_int_max))
        {
            i_max++;
        }
        vel_int_max += (vel_sign_init * 1);
        vel_seq.insert(vel_seq.begin() + i_max + 1, vel_int_max);
        dist_left -= vel_int_max * dt_;
    }

    // Once again, special case, if there is no distance left, we do not need to continue
    if (dist_left == 0)
    {
        return vel_seq;
    }

    // After the previous sequence, we will have reached maximum velocity.
    // If there is still an integer left in dist_left, we can put it into the sequence next to the last matching integer
    // Last because this will be a slower than max velocity, so we want to delay it for as long as possible
    int dist_left_int;
    if (dist_left >= 0)
    {
        dist_left_int = std::floor(dist_left / dt_);
    }
    else
    {
        dist_left_int = std::ceil(dist_left / dt_);
    }

    // Find the last matching integer in the sequence
    int i_match = vel_seq.size() - 1;
    while (vel_seq[i_match] != dist_left_int)
    {
        i_match--;
    }
    // Fit the rest of the distance before that integer
    vel_seq.insert(vel_seq.begin() + i_match, dist_left / dt_);

    // TODO: if there is a pair of integers right after the first float, you can put part of the decimal part of the rest of the dist
    //      on the first of that pair of integers, resulting in faster speed. This is a special case, so it is not implemented yet.

    return vel_seq;
}

double FlyappyRos::getXAccelCommand(geometry_msgs::Vector3 pos, double x_vel_init)
{
    /*
    Calculates the x acceleration command based on the current position and velocity,
    and the y-vel sequences to reach the current pipe and next pipe, along with the
    confidence of the y-positions of those gaps for velocity scaling.
    - pos: current position [m]
    - x_vel_init: current velocity [m/s]
    returns: x acceleration command [m/s^2]
    */

    //----|   ->                 |--------|                   |--------|       ...
    //        0 = current        1 = start of pipe            3 = start of next pipe
    //                                    2 = end of pipe

    // Creating quality parameters for scaling velocities

    double q_1 = current_gap_.quality / double(obs_array_size_ * (obs_gap_i));  // Normalized to maximum quality
    double q_3 = next_gap_.quality / double(obs_array_size_ * (obs_gap_i));     // But sometimes higher than 1 due to additional free detections

    // Make sure that both q_1 and q_3 are within tuneable limits
    if (q_1 > gap_quality_limits_.q_1_max){q_1 = gap_quality_limits_.q_1_max;}
    else if (q_1 < gap_quality_limits_.q_1_min){q_1 = gap_quality_limits_.q_1_min;}
    if (q_3 > gap_quality_limits_.q_3_max){q_3 = gap_quality_limits_.q_3_max;}
    else if (q_3 < gap_quality_limits_.q_3_min){q_3 = gap_quality_limits_.q_3_min;}

    // Q1: How fast can I go at x_2 to reach x_3 at t_3 at constant velocity?

    double a;
    double t_3 = (y_vel_seq_next_.size() - 1) * dt_;  // -1 for start vel
    double x_3 = obs_spacing - obs_width_x_accel + early_accel_offset;  // x_3 defined relative to x_2
    double v_2 = (x_3 / t_3) * q_3;

    // Q2: How fast can I go now, at x_0, to reach x_1 at t_1 at constant velocity?

    // Special case if we are already in pipe (between x_1 and x_2)
    if (pos.x >= obs_spacing - obs_width_x_accel)
    {
        // Solve for accel command to reach v_2
        a = (v_2 - x_vel_init) / dt_;

        // But are we in the first half of the pipe? If yes, maybe we can accelerate
        if (pos.x < obs_spacing - ((obs_width_x_accel - early_accel_offset) / 2.0))
        {
            // What speed can we decelerate to v_2 from, from the middle of the pipe?
            double d_decel = ((obs_width_x_accel - early_accel_offset) / 2.0);
            double t_decel = ((-v_2) + std::sqrt((v_2 * v_2) + (6 * max_acc_x_ * d_decel))) / (3 * max_acc_x_); // quadratic formula
            double v_decel = v_2 + (max_acc_x_ * t_decel);

            a = (v_decel - x_vel_init) / dt_;
        }
       
        if (std::abs(a) > max_acc_x_)
        {
            a = (a/std::abs(a)) * max_acc_x_; //maintaining the sign
        }
        return a;
    }

    double t_1 = (y_vel_seq_.size() - 1) * dt_;  // -1 for start vel
    double x_1 = obs_spacing - obs_width_x_accel - pos.x + early_accel_offset;  // x_1 defined relative to x_0
    double v_1 = (x_1 / t_1) * q_1; // So far assuming v_1 = v_0

    // Q3: Is v_2 limiting v_1?

    if (v_2 < v_1)
    {
        // Can we slow down in time to reach v_2 at x_2?
        double x_temp = 0.0d;
        double v_temp = v_1;
        while (x_temp < (obs_spacing - early_accel_offset) && v_temp > v_2)
        {
            x_temp += v_temp * dt_;
            v_temp -= max_acc_x_dt_ * dt_;
        }
        if (v_temp > v_2)
        {
            // We cant slow down in time. Limit v_1
            v_1 -= (v_temp - v_2);
        }
        // Otherwise, v_2 is not limiting v_1
    }

    // Q4: Now that we have solved for v_1, let's solve for the acceleration command
    a = (v_1 - x_vel_init) / dt_;
    if (std::abs(a) > max_acc_x_)
    {
        a = (a/std::abs(a)) * max_acc_x_; //maintaining the sign
    }
    return a;
}

void FlyappyRos::setPos(geometry_msgs::Vector3 pos)
{
    /*
    Sets position to new position, used for testing
    - pos: new position [m]
    */
    pos_ = pos;
}

void FlyappyRos::setGap(int i, gap new_gap)
{
    /*
    Sets gap i to new_gap, used for testing
    - i: index of gap to set
    - new_gap: new gap to set
    */
    if (i == 0)
    {
        current_gap_ = new_gap;
    }
    else if (i == 1)
    {
        next_gap_ = new_gap;
    }
}

double FlyappyRos::getMaxAccYDt()
{
    return max_acc_y_dt_;
}

double FlyappyRos::getMaxAccY()
{
    return max_acc_y_;
}

double FlyappyRos::getMaxAccXDt()
{
    return max_acc_x_dt_;
}

gap_quality_limits FlyappyRos::getGapQualityLimits()
{
    return gap_quality_limits_;
}