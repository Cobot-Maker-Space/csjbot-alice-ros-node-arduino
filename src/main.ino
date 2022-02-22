#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include "JointMovement.h"

#define DIR_PIN_LEFT 5
#define STEP_PIN_LEFT 2
#define STEPS_PER_REV_LEFT 10000
#define END_STOP_PIN_LEFT 12

#define DIR_PIN_RIGHT 6
#define STEP_PIN_RIGHT 3
#define STEPS_PER_REV_RIGHT 10000
#define END_STOP_PIN_RIGHT 13

#define DIR_PIN_NECK 7
#define STEP_PIN_NECK 4
#define STEPS_PER_REV_NECK 300
#define END_STOP_PIN_NECK 11
#define NECK_STEPS_FORWARD_END_STOP 960
#define NECK_STEPS_BEHIND_END_STOP 380

#define ENABLE_PIN 8

#define PUBLISH_RATE 1000

class Stepper {
private:
  uint8_t stop_pin;
  void reset_run_a();
  void reset_run_b();

protected:
  AccelStepper stepper;
  void (Stepper::*run_func)() = NULL;
  void moveTo_run();

public:
  Stepper(
      uint8_t step_pin,
      uint8_t dir_pin,
      uint8_t stop_pin,
      uint8_t enable_pin,
      bool reverse_orientation);
  long getCurrentPosition();
  void reset();
  void run();
  virtual void moveTo(long target, long speed) = 0;
};

Stepper::Stepper(
    uint8_t step_pin,
    uint8_t dir_pin,
    uint8_t stop_pin,
    uint8_t enable_pin,
    bool reverse_orientation)
    : stepper(AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin)),
      stop_pin(stop_pin) {
  pinMode(stop_pin, INPUT);
  stepper.setEnablePin(enable_pin);
  stepper.setPinsInverted(reverse_orientation, false, true);
  stepper.enableOutputs();
}

void Stepper::reset_run_a() {
  if (digitalRead(stop_pin) != 1) {
    stepper.move(1);
    stepper.run();
  } else {
    run_func = &reset_run_b;
  }
}

void Stepper::reset_run_b() {
  if (digitalRead(stop_pin) == 1) {
    stepper.move(-1);
    stepper.run();
  } else {
    stepper.setCurrentPosition(0);
    run_func = NULL;
  }
}

void Stepper::moveTo_run() {
  if (!stepper.run()) {
    run_func = NULL;
  }
}

void Stepper::reset() {
  stepper.setMaxSpeed(1000000);
  stepper.setAcceleration(1000000);
  stepper.setSpeed(1000000);
  if (digitalRead(stop_pin) == 0) {
    run_func = &reset_run_a;
  } else {
    run_func = &reset_run_b;
  }
}

void Stepper::run() {
  if (run_func != NULL) {
    (this->*run_func)();
  }
}

long Stepper::getCurrentPosition() {
  return stepper.currentPosition();
}

class BoundedStepper : public Stepper {
private:
  uint8_t clockwise_bound;
  uint8_t anticlockwise_bound;

public:
  BoundedStepper(
      uint8_t step_pin,
      uint8_t dir_pin,
      uint8_t stop_pin,
      uint8_t enable_pin,
      bool reverse_orientation,
      uint8_t clockwise_bound,
      uint8_t anticlockwise_bound);
  void moveTo(long target, long speed);
};

BoundedStepper::BoundedStepper(
    uint8_t step_pin,
    uint8_t dir_pin,
    uint8_t stop_pin,
    uint8_t enable_pin,
    bool reverse_orientation,
    uint8_t clockwise_bound,
    uint8_t anticlockwise_bound)
    : Stepper(step_pin, dir_pin, stop_pin, enable_pin, reverse_orientation),
      clockwise_bound(clockwise_bound),
      anticlockwise_bound(anticlockwise_bound) {
}

void BoundedStepper::moveTo(long target, long speed) {
  if (target > clockwise_bound) {
    target = clockwise_bound;
  } else if (target < (-anticlockwise_bound)) {
    target = -anticlockwise_bound;
  }
  stepper.setMaxSpeed(speed);
  stepper.setSpeed(speed);
  stepper.setAcceleration(speed);
  stepper.moveTo(target);
  run_func = &moveTo_run;
}

class UnboundedStepper : public Stepper {
private:
  uint8_t steps_per_revolution;

public:
  UnboundedStepper(
      uint8_t step_pin,
      uint8_t dir_pin,
      uint8_t stop_pin,
      uint8_t enable_pin,
      bool reverse_orientation,
      uint8_t steps_per_revolution);
  void moveTo(long target, long speed);
};

UnboundedStepper::UnboundedStepper(
    uint8_t step_pin,
    uint8_t dir_pin,
    uint8_t stop_pin,
    uint8_t enable_pin,
    bool reverse_orientation,
    uint8_t steps_per_revolution)
    : Stepper(step_pin, dir_pin, stop_pin, enable_pin, reverse_orientation),
      steps_per_revolution(steps_per_revolution) {
}

void UnboundedStepper::moveTo(long target, long speed) {
  stepper.setMaxSpeed(speed);
  stepper.setSpeed(speed);
  stepper.setAcceleration(speed);
  stepper.moveTo(target);
  run_func = &moveTo_run;
}

BoundedStepper *neck = new BoundedStepper(
    STEP_PIN_NECK,
    DIR_PIN_NECK,
    END_STOP_PIN_NECK,
    ENABLE_PIN,
    true,
    NECK_STEPS_FORWARD_END_STOP,
    NECK_STEPS_BEHIND_END_STOP);

UnboundedStepper *left_arm = new UnboundedStepper(
    STEP_PIN_LEFT,
    DIR_PIN_LEFT,
    END_STOP_PIN_LEFT,
    ENABLE_PIN,
    true,
    STEPS_PER_REV_LEFT);

UnboundedStepper *right_arm = new UnboundedStepper(
    STEP_PIN_RIGHT,
    DIR_PIN_RIGHT,
    END_STOP_PIN_RIGHT,
    ENABLE_PIN,
    false,
    STEPS_PER_REV_RIGHT);

ros::NodeHandle *ros_handle = new ros::NodeHandle();

void reset_cb(const std_msgs::Empty &msg) {
  neck->reset();
  left_arm->reset();
  right_arm->reset();
}

ros::Subscriber<std_msgs::Empty> *reset_subscriber =
    new ros::Subscriber<std_msgs::Empty>("csjbot_alice_joints_reset", &reset_cb);

void moveTo_cb(const csjbot_alice::JointMovement &msg) {
  char buffer[100];
  sprintf(buffer, "%d|%d|%d|%d|%d|%d|%d|%d|%d",
    msg.neck,
    msg.neck_to,
    msg.neck_speed,
    msg.left_arm,
    msg.left_arm_to,
    msg.left_arm_speed,
    msg.right_arm,
    msg.right_arm_to,
    msg.right_arm_speed);
  if (msg.neck) {
    neck->moveTo(msg.neck_to, msg.neck_speed);
  }
  if (msg.left_arm) {
    left_arm->moveTo(msg.left_arm_to, msg.left_arm_speed);
  }
  if (msg.right_arm) {
    right_arm->moveTo(msg.right_arm_to, msg.right_arm_speed);
  }
}

/*std_msgs::Int64MultiArray joint_states;
ros::Publisher *joint_state_publisher =
    new ros::Publisher("csjbot_alice_joint_states", &joint_states);*/

ros::Subscriber<csjbot_alice::JointMovement> *move_to_subscriber =
    new ros::Subscriber<csjbot_alice::JointMovement>("csjbot_alice_joints_move_to", &moveTo_cb);

void setup() {
  ros_handle->initNode();
  ros_handle->subscribe(*reset_subscriber);
  ros_handle->subscribe(*move_to_subscriber);
  //ros_handle->advertise(*joint_state_publisher);

  /*left_arm.moveTo(left_pos);
  left_arm.setMaxSpeed(3000);
  left_arm.setSpeed(3000);
  left_arm.setAcceleration(3000);

  right_arm.moveTo(right_pos);
  right_arm.setMaxSpeed(3000);
  right_arm.setSpeed(3000);
  right_arm.setAcceleration(3000);

  neck.moveTo(neck_pos);
  neck.setMaxSpeed(5000);
  neck.setSpeed(5000);
  neck.setAcceleration(5000);*/
}

unsigned long next_publish_time = 0;
int64_t joint_positions[3];

void loop() {
  ros_handle->spinOnce();
  neck->run();
  left_arm->run();
  right_arm->run();
  /*unsigned long ctime = millis();
  if (ctime >= next_publish_time) {
    next_publish_time = ctime + PUBLISH_RATE;
    joint_states.data_length = 3;
    joint_positions[0] = neck->getCurrentPosition();
    joint_positions[1] = left_arm->getCurrentPosition();
    joint_positions[2] = right_arm->getCurrentPosition();
    joint_states.data = joint_positions;
    joint_state_publisher->publish(&joint_states);
  }*/
}
