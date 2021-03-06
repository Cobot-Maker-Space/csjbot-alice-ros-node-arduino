# Arduino Node for ROS Control of a CSJBOT Alice's Neck and Arm Motors

An Arduino application for an Arduino Uno with CNC shield that can control the
neck and arm stepper motors in a customised
[CSJBOT Alice](https://en.csjbot.com/content/12/1289.html) robot.
The application uses [rosserial](https://wiki.ros.org/rosserial) to connect
to the ROS master.

## Subscribers

### /reset_joints

Accepts a `std_msgs.Empty` message. Causes the joints to move back to the zero
position.

### /move_joints

Accepts a `csjbot_alice/JointMovement` message. Moves the joints to the
positions specified in the message:

- bool neck - true if neck should move
- int16 neck_to - move the neck to step count
- int16 neck_speed - move neck at speed (steps per second)
- bool left_arm - true if left arm should move
- int16 left_arm_to - move the left arm to step count
- int16 left_arm_speed - move the left arm at speed (steps per second)
- bool right_arm - true if the right arm should move
- int16 right_arm_to - move the right arm to step count
- int16 right_arm_speed - move the right arm at speed (steps per second)

The neck has limits of -380 steps back and 960 steps forward. A speed of 5000
steps per second seems to be a reasonable speed.

The left and right arms can rotate continuously, with 10,000 steps for a
complete rotation. 3000 steps per second seems to be a reasonable speed.

Subsequent messages published on this topic will override prior messages.

## Acknowledgements
This work was developed as part of the the [Robots Mediating Interaction project][1],
 supported by [UK Engineering and Physical Sciences Research Council][2]
(EPSRC) through [Horizon: Trusted Data-Driven Products][3] ([EP/T022493/1][4]).

[1]: https://www.horizon.ac.uk/project/robots-mediating-interaction/
[2]: https://www.ukri.org/councils/epsrc/
[3]: https://www.horizon.ac.uk/
[4]: https://gtr.ukri.org/projects?ref=EP%2FT022493%2F1
