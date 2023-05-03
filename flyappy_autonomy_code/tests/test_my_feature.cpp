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

TEST(Obstacle, findGapAllUnknown)
{
    Obstacle obs;
    obs.clear();
    gap g = obs.findGap();
    ASSERT_NEAR(g.y, y_max_/2.0f, gap_y_tolerance);
    ASSERT_EQ(g.quality, obs_array_size_);
}

TEST(Obstacle, findGapPartUnknown)
{
    Obstacle obs;
    std::array<int, obs_array_size_> new_obs_array;
    new_obs_array.fill(1);
    for (int i = 0; i < 50; i++)
    {
        new_obs_array[i] = 2;
    }
    obs.setObstacleArray(new_obs_array);
    gap g = obs.findGap();

    float y_exp = (y_max_/float(obs_array_size_)) * 25.0f;
    ASSERT_NEAR(g.y, y_exp, gap_y_tolerance);
    ASSERT_EQ(g.quality, 50);
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
    std::array<int, obs_array_size_> expected_obs;
    // Fill expected_obs with 2
    expected_obs.fill(2);
    expected_obs[0] = 0;

    std::array <int,obs_array_size_> obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);
}

TEST(ObstaclePair, AddObstacleAfterFreeObs1)
{
    ObstaclePair obs_pair;
    obs_pair.clear();

    // Add free reading in the first obstacle array
    geometry_msgs::Vector3 pos;
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range);

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

    obs_pair.add(pos, angle, range);

    // Check that the obstacle has been added to obs1_ correctly
    expected_obs[obs_array_size_ - 1] = 1;

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
    pos.y = y_max_ - 0.001;
    double angle = 0.0;
    double range = 3.0;

    obs_pair.add(pos, angle, range);

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

    obs_pair.add(pos, angle, range);

    // Check that the obstacle has been added to obs1_ correctly
    expected_obs[obs_array_size_ - 1] = 1;

    obs1Array = obs_pair.getObstacleArray(1);
    ASSERT_TRUE(obs1Array == expected_obs);

    // Add a free reading in the same spot again, which should not change anything
    pos.x = 0.0;
    pos.y = y_max_ - 0.001;
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

    obs_pair.add(pos, angle, range);

    expected_obs[obs_array_size_ - 1] = 0;
    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    pos.x = 1.9;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 1.82;

    obs_pair.add(pos, angle, range);

    expected_obs[obs_array_size_ - 1] = 1;
    obs2Array = obs_pair.getObstacleArray(2);
    ASSERT_TRUE(obs2Array == expected_obs);

    pos.x = 1.9;
    pos.y = y_max_ - 0.001;
    angle = 0.0;
    range = 3.0;

    obs_pair.add(pos, angle, range);

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
        obs_pair.add(pos, angle, range);

        range = 3.0;
        angle = 0.485;
        obs_pair.add(pos, angle, range);

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

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
