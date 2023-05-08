# Key functions explained

There are a couple of functions in the code that could use some more explanation than just comments. Specifically, this doc will cover the functions for finding the best gaps, planning the y-velocity sequence, getting an x-acceleration command and the pipe wiggle manoeuvre.

## Finding the best gap

Flyappy tracks two obstacles - the pipe right in front, and the next pipe behind it, both of which have a known width, and spacing and are known to have a gap in them of a given size. The information Flyappy has about these pipes is stored in arrays representing the information about discrete y-coordinate ranges of the obstacle. At first, these arrays are filled with the "unknown" state. Then, for every laser range reading received, Flyappy decides what it means for each of the obstacles being tracked and stores this information in the arrays. If a laser hits somewhere in the space where one of the pipes is expected to be, it will save this information as a known obstacle reading for that pipe at the y-coordinate of the impact. If a laser goes through and past the space where it could have hit an obstacle, but it does not hit anything in that range, it will save this information as a known free space reading for that pipe at the right y-coordinate (at the intersection of a vertical line in the middle of the pipe and the line aligned with the laser beam). That is unless there is already an obstacle reading detected at that y-position for that obstacle, in which case the free reading is ignored.

Now that some information about the obstacles has been collected, this can help us make informed decisions. The readings will be post-processed to determine the most likely location of the gap for an obstacle and a confidence rating for that location. This location is then later used for control. To achieve this, the program will go through all the elements of the obstacle reading array that stores the aforementioned unknown/obstacle/free information per y-height. If it finds a gap between known-obstacle readings which is big enough to be ***the*** gap we are looking for, it will evaluate the quality of that gap. The quality of a gap is determined by the number and type (known to be free or just unknown) of readings that the gap consists of. This is done in a fashion where known free readings far outweigh unknown readings. If there is a sufficiently big gap that has at least one free reading, it will have a higher quality rating than any size gap which only consists of unknown readings. Unknown readings still help a gap's quality rating though. Once the program goes through all the gaps, the best gap's quality rating and the y-coordinate of its middle are returned.

## Planning the y-velocity sequence

The y-velocity sequence in question here is the calculated optimal sequence of velocities in the y-direction needed to reach the destination in the y-axis (that is the gap found by the method described above). The y-velocity sequence planner is the most convoluted part of the program and it could use a refactor. However, it works, and it produces near-optimal y-velocity sequences for any combination of current speed, current position and target position.

The problem of getting from one position to the next as fast as possible, with a 0 velocity constraint at the start and the end, is very easy in continuous time. Accelerate for half the time or distance and decelerate for the other half. If there is some initial velocity, it becomes a bit more complex, but still nothing too problematic. The difficulty of the problem here lies in the significant effects caused by time discretization, which are accounted for in the program. These apply an additional time constraint on the goal position since you have to be at the target position, at 0 velocity, and at a time that is an integer multiple of the time step. The algorithm to solve this can quite easily be solved by hand, so I will go through an example:

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
The first velocity in the sequence is just the current velocity, which doesn't affect the distance left, per design.

2. We will also have to slow down from $v_0$ to 0 at some point, so we can add the steps to do that. \
Notice that we first subtract the decimal, and only then start decelerating at steps of max. acceleration, which is 1 in this case. This is because we want to get as far as we can, as soon as we can, and only start braking with max. power (that is decelerating by 1 per time step) when we need to. This is a design choice, to decrease the chance of hitting a corner of the pipe.

3. With that sorted, we still have some distance left, and maybe we can accelerate.
If we want to accelerate by more than one unit, we will need to brake back from that velocity as well. Therefore, we need the distance to travel at the (current max velocity + 1) velocity for at least 2 time steps if we want to accelerate to (current max velocity + 2) in the future. (current max velocity + 1) * 2 = 6 and 11.2 > 6, so we shall do it.

4. Now we can try to do that again, but we will not have enough distance left. That means we won't be able to accelerate by another +2, but we can still accelerate by a +1 if we have the space for it (we created an affordance for that in the last step). (3 + 1) < 5.2 so we have the space for it, and we shall do it.

5. The last step is to add the remaining distance to the left of its rounded-down integer relative. That is in this case 1.2 being added left of 1.0, again in the spirit of going as far as possible, as soon as we can. This will leave us at y-target, with zero distance to go.

Note that the last step is where the algorithm is not quite optimal. In the case we just completed, there isn't an opportunity to cut down the number of time steps to get to the final position, but there is a possibility to "go faster sooner", by distributing as much as possible of the 1.2 to earlier numbers. This would be a pain to implement though, and it would not make much of a difference. In some cases, where the distance left is less than 1 at the end, all of the remaining distance can be off-loaded onto one of the numbers in the sequence, cutting the time to get to the target by one time step. An example of this is shown below. However, even though we get to the target one time step sooner, the fact that the distance left was less than 1 means we are already almost at the target at the same time step regardless. Therefore, this exception has not been implemented either.

**Example of distributing distance left:**

|Step|Velocity sequence|Distance left|Note|
|----|-----------------|-------------|----|
|0 |2.3, 3.0, 3.0, 2.0, 1.0, 0.0 |0.2 ||
|1 |2.3, 3.0, 3.0, 2.0, 1.0, **0.2**, 0.0 |0.0 |The way it's done now|
|1 |2.3, **3.2**, 3.0, 2.0, 1.0, 0.0 |0.0 |The optimal way|

More example solutions can be seen in the test_my_feature file, in tests for the getYVelSequence function.

The y-velocity sequence, once calculated, is used for x-acceleration control as described below, and also for y-acceleration control, simply by subtracting the first number in the sequence (current speed) from the second (the next planned speed).

## Getting an x-acceleration command

The next function to explain is the x-acceleration command generation. This function is covered in code comments quite thoroughly, so feel free to just go through that, but I will still give a short explanation here.

The x-accel code uses the knowledge stored in the y-velocity sequences to get to the next pipe and the pipe after that in the x-direction at a "good" speed. Let's define a convention that subscript 0 means current, 1 means start of the closest pipe, 2 means end of the closest pipe and 3 means start of the pipe after that. This convention is also used in the code. The length of the y-velocity sequences defines the number of time steps required to get from the current position (0) to the start of the next pipe (1) and from the end of the next pipe (2) to the start of the pipe after (3). Knowing this, we can go from the back and determine at what constant velocity we need to be going at 2 to reach 3 in the defined amount of time steps - that velocity is $v_2$. Then we can look at what constant velocity we can reach 1 from 0 in the defined amount of time steps - that is $v_1$. Now we are interested in whether we can slow down to $v_2$ from $v_1$ in time, that is in the time it takes to traverse the gap in the pipe. If not, we need to limit $v_1$ to a velocity from which it is possible to slow down to $v_2$ in the limited time. That fully defines $v_1$, and now we can just subtract $v_0$ from $v_1$ and get our acceleration command. In some cases, this will saturate the acceleration command, but in these cases, this is the best thing we can do anyways. There are a couple of special cases, e.g. when already in the pipe, but this covers the overarching principle of the x-acceleration control.

## The pipe wiggle

The pipe wiggle is a little added "special effect" for y-acceleration control. Without it, Flyappy was going through the pipes with 0 y-velocity, which limited the exploration being done with the lasers. The pipe wiggle is not an optimized solution for this problem, but it at least introduces some y-velocity into the mix. It does this by applying one unit of maximum acceleration upwards whenever Flyappy is in the pipe and has 0 velocity in the y-direction. The regular y-accel control then takes care of bringing Flyappy back to the centerline of the gap after the disturbance.
