#include <gtest/gtest.h>

#include "flyappy_autonomy_code/flyappy.hpp"
#include "flyappy_autonomy_code/flyappy_ros.hpp"

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

TEST(ObstaclePair, AddFreeObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 0.0;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range);

    // Check that the free reading has been added to obs1_ correctly
    std::array<int, 32> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[0] = 0;

    std::array <int,32> obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, AddObstacleAfterFreeObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 3.75;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range);

    // Check that the free reading has been added to obs1_ correctly
    std::array<int, 32> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[31] = 0;

    std::array <int,32> obs1Array = obs_pair.getObstacleArray(1);
    for (int i = 0; i < 32; i++)
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add an obstacle in the same place
    pos.x = 0.0;
    pos.y = 3.75;
    angle = 0.0;
    range = 1.8;

    obs_pair.add(pos, angle, range);

    // Check that the obstacle has been added to obs1_ correctly
    expected_obs[31] = 1;

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, AddFreeAfterObstacleObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = 3.75;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range);

    // Check that the free reading has been added to obs1_ correctly
    std::array<int, 32> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[31] = 0;

    std::array <int,32> obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add an obstacle in the same place
    pos.x = 0.0;
    pos.y = 3.75;
    angle = 0.0;
    range = 1.8;

    obs_pair.add(pos, angle, range);

    // Check that the obstacle has been added to obs1_ correctly
    expected_obs[31] = 1;

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add a free reading in the same spot again, which should not change anything
    pos.x = 0.0;
    pos.y = 3.75;
    angle = 0.0;
    range = 3.0;

    obs_pair.add(pos, angle, range);

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, AddFreeObstacleFreeObs2)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    std::array<int, 32> expected_obs;
    expected_obs.fill(2);

    // First, obs2_ should be all unknown
    expected_obs[31] = 2;
    std::array <int, 32> obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    // Perform same routine as on obs1_ - free reading, obstacle, free reading
    geometry_msgs::Vector3 pos;
    pos.x = 1.9;
    pos.y = 3.75;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range);

    expected_obs[31] = 0;
    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    pos.x = 1.9;
    pos.y = 3.75;
    angle = 0.0;
    range = 1.82;

    obs_pair.add(pos, angle, range);

    expected_obs[31] = 1;
    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    pos.x = 1.9;
    pos.y = 3.75;
    angle = 0.0;
    range = 3.0;

    obs_pair.add(pos, angle, range);

    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
