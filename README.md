# Arduino Node for ROS Control of a CSJBOT Alice's Neck and Arm Motors

An Arduino application for an Arduino Uno with CNC shield that can control the
neck and arm stepper motors in a customised [CSJBOT Alice robot](https://en.csjbot.com/content/12/1289.html)
. The application
uses [rosserial](https://wiki.ros.org/rosserial) to connect to the ROS master.

## Subscribers

### /csjbot_alice_joints_reset

Accepts a `std_msgs.Empty` message. Causes the joints to move back to the zero
position.

### /csjbot_alice_joints_move_to

Accepts a `csjbot_alice.msg.JointMovement` message. Moves the joints to the
specified positions in the message:
* bool neck - true if neck should move
* int16 neck_to - move the neck to step count
* int16 neck_speed - move neck at speed (steps per second)
* bool left_arm - true if left arm should move
* int16 left_arm_to - move the left arm to step count
* int16 left_arm_speed - move the left arm at speed (steps per second)
* bool right_arm - true if the right arm should move
* int16 right_arm_to - move the right arm to step count
* int16 right_arm_speed - move the right arm at speed (steps per second)

The neck has limits of -380 steps back and 960 steps forward. A speed of 5000
steps per second seems to be a reasonable.

The left and right arms can rotate continuously, with 10,000 steps for a
complete rotation. 3000 steps per second seems to be reasonable.

Subsequent messages published on this topic will override prior messages.

## TODO
* Implement speed limits
* Implement a publisher to publish current joint positions and speeds
