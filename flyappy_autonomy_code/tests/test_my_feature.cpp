#include <gtest/gtest.h>

#include "flyappy_autonomy_code/flyappy.hpp"
#include "flyappy_autonomy_code/flyappy_ros.hpp"

const float gap_y_tolerance = 0.1f;

TEST(GeneralFunctions, getPoint)
{
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double angle = 0.0;
    double range = 3.0;

    geometry_msgs::Vector3 point = getPoint(pos, angle, range);

    ASSERT_EQ(point.x, 3.0);
    ASSERT_EQ(point.y, 0.0);

    angle = 1.5708;
    point = getPoint(pos, angle, range);

    ASSERT_NEAR(point.x, 0.0, 0.0001);
    ASSERT_NEAR(point.y, 3.0, 0.0001);
}

TEST(GeneralFunctions, getIntersectPoint)
{
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double angle = 0.0;
    float x = 3.0;

    geometry_msgs::Vector3 point = getIntersectPoint(pos, angle, x);

    ASSERT_EQ(point.x, 3.0);
    ASSERT_EQ(point.y, 0.0);

    angle = 0.785398;
    point = getIntersectPoint(pos, angle, x);

    ASSERT_NEAR(point.x, 3.0, 0.0001);
    ASSERT_NEAR(point.y, 3.0, 0.0001);
}

TEST(GeneralFunctions, getGapQuality)
{
    // All counts zero case
    int q = getGapQuality(0, 0);
    ASSERT_EQ(0, q);

    // Gap too small case
    q = getGapQuality(1, 1);
    ASSERT_EQ(0, q);

    // Gap big enough
    q = getGapQuality(20, 20);
    ASSERT_TRUE(q > 0);

    // Compare unknown gap and known free gap
    int q_unk = getGapQuality(40, 0);
    int q_kno = getGapQuality(0, 40);

    ASSERT_TRUE(q_kno > q_unk);
    ASSERT_EQ(q_unk, 40);
    ASSERT_EQ(q_kno, obs_array_size_ * 40);

    // Combined
    q = getGapQuality(30, 20);
    ASSERT_EQ(q, 30 + (20 * obs_array_size_));
}

TEST(Obstacle, findGap_AllUnknown)
{
    Obstacle obs;
    obs.clear();
    gap g = obs.findGap();
    ASSERT_NEAR(g.y, y_max_/2.0f, gap_y_tolerance);
    ASSERT_EQ(g.quality, obs_array_size_);
}

TEST(Obstacle, findGap_PartUnknown)
{
    Obstacle obs;
    std::array<int, obs_array_size_> new_obs_array;
    new_obs_array.fill(1);
    for (int i = 0; i < 40; i++)
    {
        new_obs_array[i] = 2;
    }
    obs.setObstacleArray(new_obs_array);
    gap g = obs.findGap();

    float y_exp = (y_max_/float(obs_array_size_)) * 20.0f;
    ASSERT_NEAR(g.y, y_exp, gap_y_tolerance);
    ASSERT_EQ(g.quality, 40);
}

TEST(Obstacle, findGap_Complex)
{
    Obstacle obs;
    std::array<int, obs_array_size_> new_obs_array;
    new_obs_array.fill(1);

    // Caluclate minimum gap size (copied from flyappy_ros)
    const float obs_gap = 0.5f;
    const int obs_gap_i = (int)std::ceil((obs_gap / y_max_) * float(obs_array_size_)) - 2;

    // Known free gap but too small
    for (int i = 0; i < (obs_gap_i - 1); i++)
    {
        new_obs_array[i+5] = 0;
    }
    obs.setObstacleArray(new_obs_array);
    gap g = obs.findGap();

    ASSERT_EQ(g.quality, 0);
    // Obstacle will have reset measurements

    // Unknown gap but big enough
    for (int i = 0; i < (obs_gap_i + 1); i++)
    {
        new_obs_array[i+10+obs_gap_i] = 2;
    }
    obs.setObstacleArray(new_obs_array);
    g = obs.findGap();

    float y_exp = (y_max_/float(obs_array_size_)) * (10.0f + float(obs_gap_i) + float(obs_gap_i)/2.0f);
    ASSERT_NEAR(g.y, y_exp, gap_y_tolerance);
    ASSERT_EQ(g.quality, obs_gap_i+1);

    // Add a couple new free measuremnts to change the quality but not position
    new_obs_array[10+obs_gap_i+1] = 0;
    new_obs_array[10+obs_gap_i+2] = 0;
    obs.setObstacleArray(new_obs_array);
    g = obs.findGap();

    ASSERT_EQ(g.quality, obs_gap_i-1 + (obs_array_size_ * 2));
    ASSERT_NEAR(g.y, y_exp, gap_y_tolerance);
}

TEST(ObstaclePair, Add_FreeObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range, false);

    // Check that the free reading has been added to obs1_ correctly
    std::array<int, obs_array_size_> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[0] = 0;

    std::array <int,obs_array_size_> obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, Add_ObstacleAfterFreeObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range, false);

    // Check that the free reading has been added to obs1_ correctly
    std::array<int, obs_array_size_> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[obs_array_size_ - 1] = 0;

    std::array <int,obs_array_size_> obs1Array = obs_pair.getObstacleArray(1);
    for (int i = 0; i < obs_array_size_; i++)
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add an obstacle in the same place
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 1.8;

    obs_pair.add(pos, angle, range, false);

    // Check that the obstacle has been added to obs1_ correctly
    expected_obs[obs_array_size_ - 1] = 1;

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, Add_FreeAfterObstacleObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range, false);

    // Check that the free reading has been added to obs1_ correctly
    std::array<int, obs_array_size_> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[obs_array_size_ - 1] = 0;

    std::array <int,obs_array_size_> obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add an obstacle in the same place
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 1.8;

    obs_pair.add(pos, angle, range, false);

    // Check that the obstacle has been added to obs1_ correctly
    expected_obs[obs_array_size_ - 1] = 1;

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add a free reading in the same spot again, which should not change anything
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 3.0;

    obs_pair.add(pos, angle, range, false);

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, Add_FreeObstacleFreeObs2)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    std::array<int, obs_array_size_> expected_obs;
    expected_obs.fill(2);

    // First, obs2_ should be all unknown
    expected_obs[obs_array_size_ - 1] = 2;
    std::array <int, obs_array_size_> obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    // Perform same routine as on obs1_ - free reading, obstacle, free reading
    geometry_msgs::Vector3 pos;
    pos.x = 1.9;
    pos.y = y_max_ - 0.001;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range, false);

    expected_obs[obs_array_size_ - 1] = 0;
    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    pos.x = 1.9;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 1.82;

    obs_pair.add(pos, angle, range, false);

    expected_obs[obs_array_size_ - 1] = 1;
    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    pos.x = 1.9;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 3.0;

    obs_pair.add(pos, angle, range, false);

    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);
}

TEST(ObstaclePair, moveObs)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add random detections
    geometry_msgs::Vector3 pos;
    pos.x = 1.5;
    pos.y = 1.0;

    double angle = 0.0;
    double range = 2.4;
    obs_pair.add(pos, angle, range, false);

    range = 3.0;
    angle = 0.485;
    obs_pair.add(pos, angle, range, false);

    // Save contents of obs2
    std::array<int, obs_array_size_> obs2Array = obs_pair.getObstacleArray(2);

    obs_pair.moveObs();

    std::array<int, obs_array_size_> obs1Array = obs_pair.getObstacleArray(1);
    
    // Check that old obs2 array matches new obs1 array
    ASSERT_TRUE(obs1Array == obs2Array);

    // Update obs2 array
    obs2Array = obs_pair.getObstacleArray(2);
    std::array<int, obs_array_size_> clearArray;
    clearArray.fill(2);

    // Check that obs2 array is all set to unknown
    ASSERT_TRUE(obs2Array == clearArray);
}

TEST(FlyappyRos, getMaxYDecelSequence_FullMaxDecel)
{
    FlyappyRos f;

    double dist_left = 0.0d;
    double y_vel = -1.0d;

    std::vector<double> exp_seq = {-1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    ASSERT_EQ(exp_seq, y_vel_seq);

    dist_left = 1.0d * dt_;
    y_vel = -1.5d;

    exp_seq = {-1.5d, -0.5d, 0.5d};
    y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    ASSERT_EQ(exp_seq, y_vel_seq);

    dist_left = 1.0d * dt_;
    y_vel = 5.0d;

    exp_seq = {5.0d, 4.0d, 3.0d, 2.0d, 1.0d, 0.0d, -1.0d};
    y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    ASSERT_EQ(exp_seq, y_vel_seq);

    dist_left = 0.1d * dt_;
    y_vel = 2.3d;

    exp_seq = {2.3d, 1.3d, 0.3d, -0.7d};
    y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getMaxYDecelSequence_StopInTime)
{
    FlyappyRos f;

    double dist_left = 0.0d;
    double y_vel = -1.5d;

    std::vector<double> exp_seq = {-1.5d, -0.5, 0.5d};
    std::vector<double> y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    ASSERT_EQ(exp_seq, y_vel_seq);

    dist_left = -0.3d * dt_;
    y_vel = -1.5d;

    exp_seq = {-1.5d, -0.5d, 0.2d};
    y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    dist_left = 9.5d * dt_;
    y_vel = 5.0d;

    exp_seq = {5.0d, 4.0d, 3.0d, 2.0d, 1.0d, 0.0d, -0.5d};
    y_vel_seq = f.getMaxYDecelSequence(y_vel, dist_left);

    ASSERT_EQ(exp_seq, y_vel_seq);
}

TEST(FlyappyRos, getYVelSequence_StartWithoutVel)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_target = max_acc_y_dt * dt_;
    double y_vel = 0.0d;

    std::vector<double> exp_seq = {0.0d, 1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = 1.5d * (max_acc_y_dt) * dt_;

    exp_seq = {0.0d, 1.0d, 0.5d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    y_target = (3.5d * max_acc_y_dt) * dt_;

    exp_seq = {0.0d, 1.0d, 1.5d, 1.0d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = (6.5d * max_acc_y_dt) * dt_;

    exp_seq = {0.0d, 1.0d, 2.0d, 2.0d, 1.0d, 0.5d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(exp_seq, y_vel_seq);

    y_target = (19.3d * max_acc_y_dt) * dt_;

    exp_seq = {0.0d, 1.0d, 2.0d, 3.0d, 4.0d, 3.3d, 3.0d, 2.0d, 1.0d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getYVelSequence_StartWithVel_DecelOnly)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_target = max_acc_y_dt * dt_;
    double y_vel = max_acc_y_dt;

    std::vector<double> exp_seq = {1.0d, 1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = 1.5d * max_acc_y_dt * dt_;
    y_vel = max_acc_y_dt;

    exp_seq = {1.0d, 1.0d, 0.5d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getYVelSequence_StartWithNegVel_DecelOnly)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_target = -max_acc_y_dt * dt_;
    double y_vel = -max_acc_y_dt;

    std::vector<double> exp_seq = {-1.0d, -1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = -1.5 * max_acc_y_dt * dt_;
    y_vel = -max_acc_y_dt;

    exp_seq = {-1.0d, -1.0d, -0.5, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getYVelSequence_StartWithVel_WithAccel)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_target = 8.2 * max_acc_y_dt * dt_;
    double y_vel = 2.3 * max_acc_y_dt;

    std::vector<double> exp_seq = {2.3d, 3.0d, 2.2d, 2.0d, 1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    y_target = 0.6 * max_acc_y_dt * dt_;
    y_vel = 0.1 * max_acc_y_dt;

    exp_seq = {0.1d, 0.6d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    pos.y = 1.5 * max_acc_y_dt * dt_;
    pos.x = 73.0d;
    y_target = 9.7 * max_acc_y_dt * dt_;
    y_vel = 2.3 * max_acc_y_dt;
    
    exp_seq = {2.3d, 3.0d, 2.2d, 2.0d, 1.0d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getYVelSequence_StartWithNegVel_WithAccel)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_target = -8.2 * max_acc_y_dt * dt_;
    double y_vel = -2.3 * max_acc_y_dt;

    std::vector<double> exp_seq = {-2.3d, -3.0d, -2.2d, -2.0d, -1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    y_vel = -0.1 * max_acc_y_dt;
    y_target = -0.6 * max_acc_y_dt * dt_;

    exp_seq = {-0.1d, -0.6d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getYVelSequence_DiffSigns)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_target = 1.0 * max_acc_y_dt * dt_;
    double y_vel = -1.0 * max_acc_y_dt;

    std::vector<double> exp_seq = {-1.0d, 0.0d, 1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = 2.0 * max_acc_y_dt * dt_;
    y_vel = -1.0 * max_acc_y_dt;

    exp_seq = {-1.0d, 0.0d, 1.0d, 1.0d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = -2.0 * max_acc_y_dt * dt_;
    y_vel = 1.0 * max_acc_y_dt;

    exp_seq = {1.0d, 0.0d, -1.0d, -1.0d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_target = -2.3 * max_acc_y_dt * dt_;
    y_vel = 1.0 * max_acc_y_dt;

    exp_seq = {1.0d, 0.0d, -1.0d, -1.0d, -0.3d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    y_target = -2.3 * max_acc_y_dt * dt_;
    y_vel = 2.1 * max_acc_y_dt;

    exp_seq = {2.1d, 1.1d, 0.1d, -0.9d, -1.0d, -1.0d, -0.6d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    pos.y = 1 * max_acc_y_dt * dt_;
    pos.x = 1 * max_acc_y_dt;
    y_target = -1.3 * max_acc_y_dt * dt_;
    y_vel = 2.1 * max_acc_y_dt;

    exp_seq = {2.1d, 1.1d, 0.1d, -0.9d, -1.0d, -1.0d, -0.6d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

TEST(FlyappyRos, getYVelSequence_Overshoot)
{
    FlyappyRos f;
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double max_acc_y_dt = f.getMaxAccYDt();

    double y_vel = 3.0 * max_acc_y_dt;
    double y_target = 2.0 * max_acc_y_dt * dt_;

    std::vector<double> exp_seq = {3.0d, 2.0d, 1.0d, 0.0d, -1.0d, 0.0d};
    std::vector<double> y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    ASSERT_EQ(y_vel_seq, exp_seq);

    y_vel = 3.2 * max_acc_y_dt;
    y_target = 2.5 * max_acc_y_dt * dt_;

    exp_seq = {3.2d, 2.2d, 1.2d, 0.2d, -0.8d, -0.3d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    y_vel = -3.2 * max_acc_y_dt;
    y_target = -2.5 * max_acc_y_dt * dt_;

    exp_seq = {-3.2d, -2.2d, -1.2d, -0.2d, 0.8d, 0.3d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    y_vel = 1.4 * max_acc_y_dt;
    y_target = 0.0 * max_acc_y_dt * dt_;

    exp_seq = {1.4d, 0.4d, -0.4d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }

    pos.y = 1.0 * max_acc_y_dt * dt_;
    y_vel = -3.2 * max_acc_y_dt;
    y_target = -1.5 * max_acc_y_dt * dt_;

    exp_seq = {-3.2d, -2.2d, -1.2d, -0.2d, 0.8d, 0.3d, 0.0d};
    y_vel_seq = f.getYVelSequence(pos, y_vel, y_target);

    for (int i = 0; i < exp_seq.size(); i++)
    {
        ASSERT_NEAR(y_vel_seq[i], exp_seq[i], 0.0001d);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
