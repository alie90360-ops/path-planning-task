1. Approach 
##============##

The main idea is to generate a path between the blue and yellow cones. First, I separate the cones into two groups: 
        1. Blue cones = left side         ,            2. Yellow cones = right side
When I have cones on both sides, I use the middle point between the cones to generate the path. If one side is missing, I estimate the missing side using a virtual boundary. This helps the car generate a path even when some cones are missing. I also use the car position and yaw to know the driving direction.
After generation the path points, I connect them together and add more points between them to make the path smoother.

=========================================================================================================

2. Handling Different Cone Cases 
##===============================##

The code handles different cases depending on the available cones. If both sides have cones, the path is generated between the two sides. If only one side has cones, the other side is estimated using the available cones and the car position. If there is only one cone, I generate a path point beside the cone instead of passing through it. The path is then extended when needed so that the car has enough path to follow.

=========================================================================================================

3. Part2 - Three Cones on one side
##================================##

For Part2, I changed the approach to also handle cases where there are three cones on the same side. The cones are ordered in the driving direction and used to generate several path points. I also added three new scenarios to test this part: 
    1. Scenario 21: three bluw cones and one yellow cone. 
    2. Scenario 22: three yellow cones and one blue cone.
    3. Scenario 23: three blue cones only.
These scenarios were added to check how the path behaves when there are three cones on one side.

=========================================================================================================

4. Assumptions
##==============##

When one side of the track is missing, the exact track boundaries are not known. In this case, I assume an approximate track width and create a virtual boundary using the available cones. I also assume that the car pose and yaw give the correct current position and direction of the car.

=========================================================================================================

5. Limitations
##==============##

The solution works well for the simple cone layouts used in the provided scenarios. However, if many cones are missing or the track has a very complex shape, the estimated boundary may not match the real track. The solution also does not use advanced path planning algorithms or optimizaion methods. I kept the approach simple because the task does not require a complex algorithm.

=========================================================================================================

6. Testing
##==============##

I tested the provided scenarios and checked the generated paths visually. I also added three extra scenarios for part2: .Scenario 21           .Scenario 22            .Scenario 23
The paths were checked to make sure that they follow the cone boundaries and do not pass directly through the cones.

