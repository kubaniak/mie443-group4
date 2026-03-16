To preview this file: Ctrl+Shift+V

### TODO ON THURSDAY
- Currently: 
    - We navigate and face the objects with an offset of 0.5. This worked quite fine, but it can be tweaked. 
    - If we don't detect an object, we just continue. This is bad. We should keep trying to see an object. LINE 234, object detected, else (LINE 284) we just log a warning. We should do something with looking left/right, running yolo multiple times?
- What we want:
    - Detect the object, and stay in the area until we do (turning side to side, etc). 
    - Still have a fallback (if it takes more than 30 sec? continue. We missed that object)
    - fix the outfile issues, file not updating properly. I think it's overwriting everything. (see working_outputs.png, should have saved viewed objects)


    - Optional: Retry if time remaining, only visiting locations where no object was yolo'd

### REMEMBER

Run colcon build only after in the ros workspace folder (cd ros2_ws)

### HOW TO START NAV (every number is in a new terminal)
1. ssh into robot: run `ssh ubuntu@100.69.127.157`
    - run `ros2 run mie443_contest2 image_capture_server`
2. on laptop terminal: run `source /contest2/bin/activate`
    - run `ros2 run mie443_contest2 yolo_detector.py`
3. Put the robot in the contest space. Run `ros2 launch turtlebot4_navigation localization.launch.py map:=/home/turtlebot/ros2_ws/src/contest2/mie443_contest2/maps/Contest2MapPractice.yaml`. WAIT FOR STARTUP
4. Run `ros2 launch turtlebot4_viz view_navigation.launch.py`, wait a little, then set the initial pose in rviz. 
5. Run `ros2 launch turtlebot4_navigation nav2.launch.py`, wait for it to finish launching
6. Run `ros2 run mie443_contest2 contest2`


### Other TODOs if time:

- Drop Position
- Time Management (navigate home when 20s remaining, e.g.)

- We are currently not really using the apriltags to our advantage. Multi-angle detection? Localization, what to do with the robot once we have the apriltag. 
