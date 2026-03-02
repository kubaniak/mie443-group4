# Contest 2 Testing and TODOs

This document outlines the remaining TODOs in the Contest 2 implementation and provides step-by-step instructions for real-life testing with a Turtlebot4 and the SO-ARM 101.

## Remaining TODOs in `contest2.cpp`
The following coordinate values must be updated with the actual values provided by Instructors/TAs on the day of the contest.

1. **Arm Base Frame Pickup Coordinates:**
   - Update `pickup_x`, `pickup_y`, `pickup_z`
   - Update `pickup_roll`, `pickup_pitch`, `pickup_yaw`
   - *Location:* `contest2.cpp` around line 122

2. **AprilTag Candidate IDs:**
   - Update `candidate_tags` vector with actual bin AprilTag IDs
   - *Location:* `contest2.cpp` around line 180

3. **Arm Base Frame Drop Coordinates:**
   - Update `drop_x`, `drop_y`, `drop_z`
   - Update `drop_roll`, `drop_pitch`, `drop_yaw`
   - *Location:* `contest2.cpp` around line 187

## Step-by-Step Real-Life Testing Instructions (Turtlebot4)

The contest involves Nav2 (navigation), MoveIt2 (arm manipulation), YOLO (object detection), and AprilTags (localization). Testing should be performed progressively to ensure each subsystem functions properly before running the full contest node.

### 1. Prerequisites
- Ensure the Turtlebot4 is powered on, connected to the local network, and successfully communicating with your laptop/PC (e.g., via ROS 2 Discovery).
- Verify the SO-ARM 101, OAK-D Lite camera, and wrist camera are properly connected and powered.
- Make sure `boxes.xml` or equivalent coordinate database is populated with valid goal coordinates.

### 2. Subsystem Verification

#### A. Navigation (Nav2) & Localization (AMCL)
1. **Launch standard Turtlebot4 navigation stack.**
2. Set initial pose in RViz.
3. Drive the robot manually using teleop to ensure AMCL converges on the actual pose.
4. Send a simple Nav2 goal via RViz to ensure the robot can autonomously plan and reach a target without colliding with obstacles.

#### B. YOLO Detection (OAK-D and Wrist Cameras)
1. Launch the image capture servers/nodes for both cameras.
2. Start the `yolo_detector` node.
3. Place a target object (e.g., "cup", "bottle", "motorcycle", "clock", "potted plant") in front of the wrist camera and trigger the `/detect_object` service manually via ROS CLI:
   ```bash
   ros2 service call /detect_object mie443_contest2/srv/DetectObject "{save_detected_image: true}"
   ```
4. Check the service response to ensure confidence is `> 0.5` and the class name matches. Look for `detected_manipulable_object.jpg` in the workspace to visually verify the bounding box.
5. Repeat for the OAK-D camera.

#### C. Arm Manipulation (MoveIt2)
1. Launch the MoveIt2 configuration for the SO-ARM 101.
2. In a controlled, safe environment, run a simple arm test script or interact via RViz to command the arm to the intended **Pickup** and **Drop** poses (using the updated TODO values).
3. Ensure the arm moves smoothly without self-collision or colliding with the environment/Turtlebot.
4. Verify gripper open/close operations.

#### D. AprilTag Detection
1. Launch the AprilTag detection node.
2. Place a known AprilTag in the camera's field of view.
3. Echo the AprilTag detection topic (e.g., `/tf` or specific tag array topics) to verify the tag ID is correctly identified and its relative pose is estimated accurately.

### 3. Full System Integration Test
1. **Initial Setup:** Place the robot at the designated starting location. Set its initial pose in RViz.
2. **Launch Everything:** Start Nav2, MoveIt2, camera nodes, YOLO detector, and AprilTag detector.
3. **Start Contest Node:** Run the `contest2` node.
4. **Observation:**
   - Watch the robot move its arm to pick up the designated manipulable object.
   - Verify it securely grasps the object and moves to a safe carry position.
   - Follow the robot as it navigates to the first box coordinate.
   - Check if the OAK-D/Wrist camera correctly identifies the scene object.
   - Observe if the robot matches the scene object to the manipulable object.
   - If a match is found, verify it locates the correct AprilTag, moves the arm to the drop position, and successfully releases the object.
   - If no match is found, verify it continues navigating through the list of box coordinates.
   - Ensure the robot returns to the starting location once the task is complete or the 300-second timer expires.
5. **Logs:** Check `contest2_output.txt` for correct logging of detected objects and confidences at each coordinate.
