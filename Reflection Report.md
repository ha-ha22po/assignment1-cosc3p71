# COSC 3P71 Assignment 1 Reflection / Report

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

## How our system works

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



---

## Camera setup and pickup alignment

Before running the robot, we first had to set the camera/arm position correctly. We moved the arm into a `VIEW/CAMERA` pose so the gripper was not blocking the camera and the camera could see the floor area in front of the robot. This was important because if the block was not visible after the rough driving path, the camera correction would not work.

After that, we manually placed the robot in the correct pickup position. This means the robot was close enough to the block, the arm could reach it, and the block was visible in the camera. Then we clicked `Save TARGET from camera`.

When we saved the camera target, the code looked at the current camera image and found the selected block colour. For example, if blue was selected, it searched the image for pixels that looked blue. It then calculated the average position of those pixels in the camera image and saved that as the target position.

During the actual run, the recorded pickup path only had to bring the robot close to the block. Once the robot was close, the camera checked where the block appeared now and compared it to the saved position from image we took. If the block appeared too far left or right, the robot strafed slightly. If it appeared too high or low in the image, the robot moved forward or backward. It kept making small movements until the block was close enough to the saved camera position. After it was stable for a few frames, the robot stopped and started the arm pickup sequence.

This made the pickup more reliable than using only a fixed recorded path.

---

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


another problem we ran into was that the robot sometimes lifted the arm before the gripper had fully closed. When that happened, the arm position was correct, but the block was missed because the gripper had not finished closing yet. We fixed this by increasing the duration/wait time for the gripper closing step before the lift command. After adding more time, the robot had a better chance to fully close around the block before lifting.
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


---

## Why we chose this approach

We chose this approach because it was realistic for the time and equipment we had. A fully open-loop solution was simple, but it was too sensitive to small errors, such as wheel slip, floor friction, slight changes in the starting angle or the robot stopping a few centimetres differently each run. The hybrid method gave us a practical middle ground because the recorded path handled the rough movement, while the camera helped correct the final pickup position.

The recorded paths handled the larger movements, and the camera correction handled the most important part, which was lining up with the block before pickup. This was the part where a small error mattered the most.

---

## Problems faced and how we fixed those problems

One issue was the robot not stopping in exactly the same place every time. This affected both pickup and drop-off. We reduced the effect at pickup by using the camera to correct the final position instead of expecting the recorded path to be perfect.

Another issue was the arm pushing the block away. We fixed this by adding the pre-pickup pose and slowing down the movement into the pickup-open pose. This made the arm approach the block more carefully.

We also had an issue where the robot lifted before the gripper had fully closed. We fixed this by increasing the gripper close duration so the gripper had more time before the lift command happened.

Battery life was also a problem during testing. The robot battery drained quickly and the robot turned off multiple times. This slowed down testing because we had to reconnect, check the GUI again, and sometimes repeat calibration/testing steps. The battery level also affected consistency because the robot did not always drive or respond exactly the same when the battery was low.

---

## Known limitations

The system still depends on calibration. It works best when the robot starts from the same taped home position and the same starting angle. Some issues were reduced during testing, but they could still happen if the setup changes.

Known limitations:

- Wheel slip and floor friction can still change where the robot stops. We reduced this by using the recorded path only for rough movement and using the camera to correct the final pickup position, but the robot can still drift if the floor or starting angle changes drastically.
- The camera needs to see the target block after the rough pickup path. We saved a proper `VIEW/CAMERA` pose and recorded the pickup path so the robot stops with the block visible, but the camera correction can still fail if the block is blocked, too far away, or outside the camera view.
- Lighting can affect the camera detection. The colour detection worked in our lab setup, but low lighting, shadows, glare, or a major lighting change could make it harder for the camera to detect the target block correctly.
- The arm poses had to be saved carefully because small changes in robot position could affect gripper alignment. We fixed this by saving separate poses for `PRE-PICKUP`, `Pickup OPEN`, `Grip CLOSED`, and `LIFTED` instead of using one arm movement.
- The gripper timing was not long enough at first, so the lift sometimes started before the block was properly held. We fixed this by increasing the gripper close duration before moving to the lifted pose.
- Low battery can make the robot shut off or behave less consistently.

## What we would do with more time

For drop-off we would also like to test camera alignment in the future. In our demo, we used the recorded drop path and saved drop poses but with more time we would try using the camera for the drop-off part as well.

## Contribution statement

All group members worked together on the assignment. We were all involved in different parts of the project, including the code, testing the robot in the lab, recording and re-testing the pickup/drop paths, fixing issues during the demo setup, and preparing the README and reflection/report. The work was completed as a group effort rather than being split into completely separate individual parts.
