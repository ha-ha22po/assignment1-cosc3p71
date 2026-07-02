# COSC 3P71 Assignment 1

## GitHub Repository

Repository link:

```text
https://github.com/ha-ha22po/assignment1-cosc3p71
```
This repository contains:

jetauto_teleop.py — the main code file used for the robot demo.
jetauto_final_program.json — the saved calibration/movement file containing the recorded paths and saved arm poses.
README.md — instructions on how to launch the program, load the JSON file, set up the project, record paths, and run the full autonomous pick/drop sequence.
Reflection/Report.md — the reflection/report explaining the approach, problems faced, fixes, and known limitations.

## Group members who worked on this assignment

```text
Ayaan Alam Biabani Feroz - 7811375
Hassan Altaf - 7631914
Nima Abadinezhad - 7744329
Sazid Saad - 7562804
Ikenna
```

## Code repository

The main file used for the demo was:

```text
jetauto_teleop.py
```

The program is a Python GUI for controlling and running the JetRover pick-and-drop task. It connects to the robot through rosbridge, shows the live camera feed, records driving paths, saves arm/gripper positions, and runs the autonomous pickup and drop sequence.

The saved calibration/movement file for the robot is:

```text
jetauto_final_program.json
```

This file stores the recorded pickup/drop paths, saved arm poses, selected target colour, and the saved camera target for pickup. This was useful because we did not need to redo the whole setup every time. If one part was wrong, we could just re-save that one part.

Important: `jetauto_final_program.json` must be in the same folder as `jetauto_teleop.py`. After opening the GUI, click:

```text
Load Program / Paths
```

This loads the saved JSON file into the program.

```python
PROGRAM_FILE = "jetauto_final_program.json"
```

## Dependencies

The program uses:

```text
websocket-client
Pillow
tkinter
```

Install the needed packages with:

```bash
py -m pip install websocket-client Pillow
```

There were no trained models or trained weights used. The camera part was done with simple colour detection.

## How to launch

From the folder where the file is saved, run:

```bash
py jetauto_teleop.py
```

For the lab demo we used this path:

```bash
cd "C:\Brock University\2025 - 2026\Spring\COSC 3P71\Assignment 1\new"
py jetauto_teleop.py
```

Make sure these two files are in the same folder:

```text
jetauto_teleop.py
jetauto_final_program.json
```

After the GUI opens, click:

```text
Load Program / Paths
```

The GUI should open with the live camera feed and the driving/servo controls.

The program uses `/cmd_vel` for driving the base and `/ros_robot_controller/bus_servo/set_position` for controlling the arm and gripper. The camera stream is read from the RGB camera topic.

```python
ROBOT_IP = "100.97.124.22"

CMD_VEL_TOPIC = "/cmd_vel"
SERVO_TOPIC = "/ros_robot_controller/bus_servo/set_position"
CAMERA_TOPIC = "/depth_cam/rgb/image_raw"
```

## How our system works and approach taken

Our final method was a hybrid method. We did not rely on one fully fixed path, because the robot was not perfectly repeatable. Even if we started it from the same taped starting position, it could still stop a few centimetres off because of wheel slip, floor friction, the starting angle, or the battery level. A few centimetres was enough for the gripper to miss the block or push it away.

Because of this, we used recorded driving for the rough movement, and the camera for the final pickup correction.

After the `RUN FULL AUTO PICK/DROP` button is pressed, the robot runs the following sequence automatically:

1. Move the arm to a safe view/camera pose.
2. Replay a rough recorded drive path so the robot stops close to the block.
3. Use the camera to line up with the block.
4. Move the arm through the saved pickup poses.
5. Close the gripper and lift the block.
6. Replay the recorded drive path to the drop-off area.
7. Move the arm to the saved drop pose.
8. Open the gripper and release the block.

## Setup and calibration steps

These are the setup steps used before running the full autonomous demo.

### 1. Check camera, driving, and servos

Before saving anything, check that:

```text
live camera feed is visible
robot can drive using the GUI
servo sliders move the arm/gripper
```

Do not start recording paths until camera, driving, and arm movement all work.

### 2. Choose target block colour

In the GUI, choose the target colour:

```text
blue / red / green
```

If the target colour is changed later, a new camera target should be saved.

### 3. Save VIEW/CAMERA pose

Move the arm into a safe position where:

```text
the gripper is not blocking the camera
the camera can see the floor/block area
the arm is safe while the robot drives
```

Then click:

```text
Save VIEW/CAMERA Pose
```

Test it by clicking:

```text
Move to VIEW/CAMERA Pose
```

### 4. Save pickup camera target

Manually drive the robot to the correct pickup position. The robot should be close enough to the block, the arm should be able to reach it, and the block should be visible in the camera.

Then click:

```text
Save TARGET from camera
```



This saves where the block appears in the camera when the robot is correctly lined up.

### 5. Save pickup arm poses

Stay in the correct pickup position and save the pickup poses:

```text
Save PRE-PICKUP Pose
Save Pickup OPEN Pose
Save Grip CLOSED Pose
Save LIFTED Pose
```

The `PRE-PICKUP` pose is used so the arm moves above the block first, instead of moving directly into the block and pushing it away.

Test the poses:

```text
Test PRE
Test OPEN
Test CLOSE
Test LIFT
```

### 6. Record rough pickup path

Put the robot at the marked home/start position.

Click:

```text
Move to VIEW/CAMERA Pose
Start Drive to Pickup
```

Drive the robot near the target block, but do not drive all the way into the block. Stop when the block is visible in the camera and the robot is close enough for camera correction.

Then click:

```text
Stop / Save Pickup Drive
```

### 7. Test pickup camera docking

Place the robot near the block, roughly where the pickup path stops. The block must be visible in the camera.

Click:

```text
VISION DOCK TO TARGET NOW
```

The robot should slowly adjust forward/backward and sideways until the block matches the saved camera target.



### 8. Run pickup-only test

Put the robot back at the marked start position and click:

```text
RUN PICKUP ONLY TEST
```

The robot should:

```text
move to VIEW/CAMERA pose
drive rough pickup path
vision dock to target block
move to PRE-PICKUP pose
move to Pickup OPEN pose
close gripper
lift block
```



### 9. Record rough drop path

After pickup-only works, the robot should be holding the block in the lifted pose.

Click:

```text
Start Drive to Drop
```

Drive the robot to the drop-off zone.

Then click:

```text
Stop / Save Drop Drive
```

This saves the drop path. During the full autonomous run, the robot replays this saved path automatically after it picks up the block.

### 10. Save drop poses

At the drop-off zone, save the arm positions:

```text
Save DROP Pose
Save DROP OPEN Pose
```

The `DROP` pose moves the arm to the release position. The `DROP OPEN` pose opens the gripper to release the block.

### 11. Save everything

Click:

```text
Save Program / Paths
```

This saves the paths and poses into:

```text
jetauto_final_program.json
```

### 12. Test loading

Close and reopen the GUI, then click:

```text
Load Program / Paths
```

Test:

```text
Move to VIEW/CAMERA Pose
Test PRE
Test OPEN
Test CLOSE
Test LIFT
```



### 13. Run full autonomous test

Put the robot exactly on the marked start position.


Then click:

```text
RUN FULL AUTO PICK/DROP
```

The robot should run the full pickup and drop sequence automatically.

## Camera setup and pickup alignment

Before running the robot, we first had to set the camera/arm position correctly. We moved the arm into a `VIEW/CAMERA` pose so the gripper was not blocking the camera and the camera could see the floor area in front of the robot. This was important because if the block was not visible after the rough driving path, the camera correction would not work.

After that, we manually placed the robot in the correct pickup position. This means the robot was close enough to the block, the arm could reach it, and the block was visible in the camera. Then we clicked `Save TARGET from camera`.

When we saved the camera target, the code looked at the current camera image and found the selected block colour. For example, if blue was selected, it searched the image for pixels that looked blue. It then calculated the average position of those pixels in the camera image and saved that as the target position.

During the actual run, the recorded pickup path only had to bring the robot close to the block. Once the robot was close, the camera checked where the block appeared now and compared it to the saved position from the image we took. If the block appeared too far left or right, the robot strafed slightly. If it appeared too high or low in the image, the robot moved forward or backward. It kept making small movements until the block was close enough to the saved camera position. After it was stable for a few frames, the robot stopped and started the arm pickup sequence.

This made the pickup more reliable than using only a fixed recorded path.

## Arm and gripper movement

The arm movement was controlled using saved servo poses. We saved these poses from the GUI:

```text
VIEW/CAMERA pose
PRE-PICKUP pose
Pickup OPEN pose
Grip CLOSED pose
LIFTED pose
DROP pose
DROP OPEN pose
```

The `PRE-PICKUP` pose was added because we noticed that moving straight to the pickup pose could sometimes hit or push the block. With the pre-pickup pose, the arm first moves above the block, then lowers more slowly into the open pickup position.

Another problem we ran into was that the robot sometimes lifted the arm before the gripper had fully closed. When that happened, the arm position was correct, but the block was missed because the gripper had not finished closing yet. We fixed this by increasing the duration/wait time for the gripper closing step before the lift command. After adding more time, the robot had a better chance to fully close around the block before lifting.

```python
GRIP_CLOSE_DURATION = 4
LIFT_DURATION = 3

self._arm_to("grip_closed", duration=GRIP_CLOSE_DURATION)
self._arm_to("lifted", duration=LIFT_DURATION)

time.sleep(duration + 0.3)
```

## Drop-off method

For the drop-off, we first recorded the drive path from the pickup area to the drop-off zone during setup. After this path was saved, it was replayed automatically by the program during the full run. the robot followed the saved drive path after it picked up and lifted the block.

During full run, after the robot lifted the block, the dropoff was done in this sequence:

1. Replay the saved drive path to the drop-off area.
2. Move the arm to the saved drop pose.
3. Open the gripper using the saved drop-open pose.
4. Stop after releasing the block.

## Known limitations

The system still depends on calibration. It works best when the robot starts from the same taped home position and the same starting angle. Some issues were reduced during testing, but they could still happen if the setup changes.

Known limitations:

- Wheel slip and floor friction can still change where the robot stops. We reduced this by using the recorded path only for rough movement and using the camera to correct the final pickup position, but the robot can still drift if the floor or starting angle changes drastically.
- The camera needs to see the target block after the rough pickup path. We saved a proper `VIEW/CAMERA` pose and recorded the pickup path so the robot stops with the block visible, but the camera correction can still fail if the block is blocked, too far away, or outside the camera view.
- Lighting can affect the camera detection. The colour detection worked in our lab setup, but low lighting, shadows, glare, or a major lighting change could make it harder for the camera to detect the target block correctly.
- The arm poses had to be saved carefully because small changes in robot position could affect gripper alignment. We fixed this by saving separate poses for `PRE-PICKUP`, `Pickup OPEN`, `Grip CLOSED`, and `LIFTED` instead of using one arm movement.
- The gripper timing was not long enough at first, so the lift sometimes started before the block was properly held. We fixed this by increasing the gripper close duration before moving to the lifted pose.
- Low battery can make the robot shut off or behave less consistently.
