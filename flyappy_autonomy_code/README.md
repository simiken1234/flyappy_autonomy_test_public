# Key functions explained

There are a couple functions in the code that could use some more explanation than just comments. Specifically, this doc will cover the functions for finding the best gaps, planning the y-velocity sequence, getting an x-acceleration command and the pipe wiggle maneuver.

## Finding the best gap

Flyappy evaluates all laser range readings and decides what they mean for each of the two obstacles being tracked (the pipe right in front, and the next pipe behind it). If a laser hits something in the x-position range where one of the pipes should be, this counts as an obstacle reading at the calculated impact's y-position for that pipe. If a laser goes through and past the x-position range where it could have hit an obstacle, that counts as a free reading for that pipe (at the intersection of a vertical line in the middle of the pipe and the laser). That is unless there is already an obstacle reading detected at that y-position for that obstacle. By default, all the y-positions for an obstacle are marked as unknown, until they are known to be either free or occupied by an obstacle.

Now that the readings are sorted, they need to be post-processed to find one y-target destination and a confidence rating for that destination. To do this, the program will go through all the elements of the array that stores the aforementioned unknown/obstacle/free information per y-height, sequentially. If it finds a gap big enough to be ***the*** gap we are looking for, it will evaluate the quality of that gap. The quality of that gap is determined by the number and type (known free or just unknown) of readings that gap consists of. This is done in a fashion where known free readings far outweigh unknown readings. Basically, if there is a sufficiently big gap that has at least one free reading, it will have a higher quality rating than any size gap which only has unknown readings. Unknown readings still help a gaps' quality rating though. Once the program goes through all the gaps, the quality rating the the y-coordinate of the middle of the best gap is returned.

## Planning the y-velocity sequence

The y-velocity sequence planner is the most convoluded part of the program and it could use a refactor. That being said, it works, and it produces near optimal y-velocity sequences for any combination of current speed, position and target position.

In continuous time, the problem of getting from one position to the next with 0 velocity at the start and the end as fast as possible is very easy. Just accelerate for half the time and decelerate for the other half. If there is some initial velocity, it becomes a bit more complex, but still nothing too problematic. The difficulty of the problem here is the significant discrete time effects, which are accounted for in the program. These apply an additional time constraint on the goal position, since you have to be at the target position, at 0 velocity, at an integer multiple of the time step. The algorithm to solve this can quite easily be solved by hand, so I will go through an example:

$a_{max} = 1$ \
$dt = 1$

$v_0 = 2.3$ \
$y_0 = 0$ \
$y_{target} = 14.2$

|Step|Velocity sequence | Distance left |
|----|------------------|---------------|
|1|**2.3**, **0.0**                            |14.2|
|2|2.3, **2.0**, **1.0**, 0.0                 |11.2|
|3|2.3, **3.0**, **3.0**, 2.0, 1.0, 0.0       |5.2|
|4|2.3, 3.0, **4.0**, 3.0, 2.0, 1.0, 0.0      |1.2|
|5|2.3, 3.0, 4.0, 3.0, 2.0, **1.2**, 1.0, 0.0 |0.0|

**Step by step:**
1. We have to start with $v_0$ and end at 0 velocity so we can add that to the sequence.\
The first velocity in the sequence is just the current velocity, which doesn't affect the distance left.

2. We will also have to slow down from $v_0$ to 0 at some point, so we can add the steps to do that, the steps being steps of max. acceleration. \
Notice that we first substract the decimal, and only then start decelerating at steps of max. accel. This is because we want to get as far as we can as soon as we can, and only start braking with max. power when we need to. This is a design choice, to decrease the chance of hitting the bottom of the pipe.

3. With that sorted, we still have some distance left, and maybe we can accelerate.
If we want to accelerate by more than one unit, we will need to brake back from it as well. Therefore, we need the distance to travel at the (current max velocity + 1) for at least 2 time steps if we want to accelerate by +2. (current max velocity + 1) * 2 = 6 and 11.2 > 6, so we can do it.

4. Now we can try to do that again, but we will not have enough distance left. That means we won't be able to accelerate by another +2, but we can still accelerate by a +1, if we have the space for it. (3 + 1) < 5.2 so we have the space for it.

5. The last step is to add the remaining distance to the left of its rounded-down integer relative, leaving us at y-target.


Note that this is where the algorithm is actually not quite optimal. In the case we just completed, there isn't an opportunity to cut down the number of time steps to get to the final position, but there is to "go faster sooner", by distributing as much of the 1.2 to earlier numbers as possible. This would be a pain to implement though, and it would not make much of a difference. In some cases, where the distance left is less than 1 at the end, all of the remaining distance can be off-loaded onto one of the numbers in the sequence, cutting the time to get to the target by one time step. An example of this is shown below. However, even though we get to the target one time step sooner, the fact that the distance left was less than 1 means we are already almost at the target the at the same time step regardless. Therefore, this exception has not been implemented either.

**Example of distributing distance left:**

|Step|Velocity sequence|Distance left|Note|
|----|-----------------|-------------|----|
|0 |2.3, 3.0, 3.0, 2.0, 1.0, 0.0 |0.2 ||
|1 |2.3, 3.0, 3.0, 2.0, 1.0, **0.2**, 0.0 |0.0 |The way it's done now|
|1 |2.3, **3.2**, 3.0, 2.0, 1.0, 0.0 |0.0 |The optimal way|

More example solutions can be seen in the test_my_feature file, in tests for the getYVelSequence function.

The y-velocity sequence, once calculated, is used for x-acceleration control as described below, and also for y-acceleration control, simply by subtracting the first number in the sequence (current speed) from the second (the next planned speed).

## Getting an x-acceleration command

The last function to explain is the x-acceleration command generation. This function is covered in code comment quite thoroughly, so feel free to just go through that, but I will still give a short explanation here.

The x-accel code uses the knowledge stored in the y-velocity sequences to get to the next pipe and the pipe after that. Let's define a convention that subscript 0 means current, 1 means start of the closest pipe, 2 means end of the closes pipe and 3 means start of the pipe after that. This convention is also used in the code. The length of the velocity sequences defines the amount of time steps required to get from the current position (0) to the start of the next pipe (1) and from the end of the next pipe (2) to the start of the pipe after (3). Knowing this, we can go from the back and determine at what constant velocity do we need to be going to reach 3 from 2 in the defined amount of time steps - that is $v_2$. Then we can look at what constant velocity we can reach 1 from 0 in the defined amount of time steps - that is $v_1$. Now we are interested whether we can slow down to $v_2$ from $v_1$ in time, that is in the time it takes to traverse the pipe. If not, we need to limit $v_1$ to a velocity from which it is possible to slow down to $v_2$ in the limited time. That fully defines $v_1$, and now we can just subtract $v_0$ from $v_1$ and get our acceleration command. There are a couple special cases, e.g. when already in the pipe, but this is the overarching principle of the x-acceleration control.

## The pipe wiggle

The pipe wiggle is a little added "special effect" for y-acceleration control. Without it, Flyappy was going through the pipes with 0 y-velocity, which limited the exploration being done with the lasers. The pipe wiggle is not an optimized solution for this problem, but it at least introduces some y-velocity into the mix. It does this by applying one unit of maximum acceleration upwards whenever Flyappy is in the pipe and has 0 velocity in the y_direction. The y-velocity sequence based control then brings Flyappy back to the centerline of the gap after the disturbance.
