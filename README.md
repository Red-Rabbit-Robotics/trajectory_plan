# trajectory_plan
ROS1 trajectory planner for the RX1 dual arm humanoid. This planner uses OMPL library and is able to take obstacle into consideration when planning trajectories. It is similar to MoveIt while it is way more light-weighted.

[![Humanoid Robot Demo](https://img.youtube.com/vi/vnZ0dgju0Sk/maxresdefault.jpg)](https://www.youtube.com/watch?v=vnZ0dgju0Sk)

## Dependencies
* Install [RX1 ROS package](https://github.com/Red-Rabbit-Robotics/rx1)   
* Install OMPL:  
`sudo apt-get install ros-noetic-ompl`

## Demo 
1. Launch everything:  
`roslaunch trajectory_planner trajectory_planner_with_gui.launch`

2. Load the RQT GUI: click on "Plugins" in the rqt window and select "Trajectory planner rqt plugin". Then you should be able to see something like this:  
![image](https://github.com/Red-Rabbit-Robotics/trajectory_plan/blob/master/media/traj_plan_plugin.png)

3. Click and drag the Interactive Marker to determine the target pose of the left or right arm.  
![image](https://github.com/Red-Rabbit-Robotics/trajectory_plan/blob/master/media/interactive_marker.png)  
**note: Don't skip this step, try at least click on the marker. Also make sure the target is not too far and the orientation is achievable.**

4. Click either "Plan Left Trajectory" or "Plan Right Trajectory" and wait until the planning succeeds.  
![image](https://github.com/Red-Rabbit-Robotics/trajectory_plan/blob/master/media/traj_plan_succeed.png)

5. Click either "Visualize Left Trajectory" or "Visualize Right Trajectory" to preview the trajectory.

6. Click either "Publish Left Trajectory" or "Publish Right Trajectory" to execute the trajectory.

7. Add obstacle by publishing a virtual cube.  
`roslaunch cube_publisher cube_publisher.launch`  
Plan trajectories again and you will see the planned trajectory avoiding the obstacle.  
![image](https://github.com/Red-Rabbit-Robotics/trajectory_plan/blob/master/media/cube.png)