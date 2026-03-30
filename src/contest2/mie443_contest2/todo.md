To preview this file: Ctrl+Shift+V

### TODO ON THURSDAY
- Currently: 
    - (DONE) We implemented a panning scan when an object is not immediately detected (with second scan closer by 0.3m).
    - We navigate and face the objects with an offset of 0.7. This worked quite fine, but it can be tweaked.
- What we want:
    - (DONE) Added panning behavior to turn side to side and retry YOLO detection.
    - (DONE) Added fallback behavior where if the initial scan misses, the robot moves 0.3m closer and tries panning again.
    - Still have a fallback (if it takes more than 30 sec? continue. We missed that object)
    - (DONE) outfile is opened once and remains open instead of being overwritten (or use append mode: std::ios_base::app if reopening is needed, though currently it is open the whole time).
    - (DONE) Implemented single-use detections for scene classes (cup, motorcycle, clock, plant, water bottle) so duplicate boxes aren't re-logged.


    - Optional: Retry if time remaining, only visiting locations where no object was yolo'd

### REMEMBER

Run colcon build only after in the ros workspace folder (cd ros2_ws)

### HOW TO START NAV (every number is in a new terminal)
1. ssh into robot: run `ssh ubuntu@100.69.127.157`
    - run `ros2 run mie443_contest2 image_capture_server`
2. on laptop terminal: run `source ~/contest2/bin/activate`
    - run `ros2 run mie443_contest2 yolo_detector.py`
3. Put the robot in the contest space. Run `ros2 launch turtlebot4_navigation localization.launch.py map:=/home/turtlebot/ros2_ws/src/contest2/mie443_contest2/maps/Contest2MapPractice.yaml`. WAIT FOR STARTUP
4. Run `ros2 launch turtlebot4_viz view_navigation.launch.py`, wait a little, then set the initial pose in rviz. 
5. Run `ros2 launch turtlebot4_navigation nav2.launch.py`, wait for it to finish launching
6. Run `ros2 run mie443_contest2 contest2`


### Other TODOs if time:

- (DONE) Added `std::ios_base::app` to `contest2_output.txt` so that it doesn't overwrite between runs/restarts.
- Fine-tune AprilTag-based alignment (already merged in #8) and verify stability on site.
- Implement exact bin drop position logic (uncomment/tune logic in `contest2.cpp` for exact `drop_pose`).

- Time Management (navigate home when 20s remaining, e.g.)

- We are currently not really using the apriltags to our advantage. Multi-angle detection? Localization, what to do with the robot once we have the apriltag.

### TO EXPERIMENT ON SITE
- Test the new 0.7m box offset for navigation and facing objects.
- Evaluate the two new AprilTag alignment variations for bin placement: Nav2 vs. Control Loop (`use_nav2_for_tag_align`).
- Tune/uncomment exact bin drop position logic (`drop_pose` values in `contest2.cpp`).
- Evaluate checking-off scene classes to make sure we don't skip unvisited boxes after one was detected.
- Tune the closer offset panning (0.3m) on failure, and double check collision with boxes.
