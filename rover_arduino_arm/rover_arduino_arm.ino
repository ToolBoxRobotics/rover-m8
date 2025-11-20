#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

const int NUM_JOINTS = 5;

// Stepper pins (example)
AccelStepper steppers[NUM_JOINTS] = {
  AccelStepper(AccelStepper::DRIVER, 2, 3),   // STEP, DIR
  AccelStepper(AccelStepper::DRIVER, 4, 5),
  AccelStepper(AccelStepper::DRIVER, 6, 7),
  AccelStepper(AccelStepper::DRIVER, 8, 9),
  AccelStepper(AccelStepper::DRIVER, 10, 11),
};

// Enable pins (optional, all tied)
const uint8_t EN_PIN = 12;

// Limit switches
const uint8_t limitPins[NUM_JOINTS] = {22, 23, 24, 25, 26};

long steps_per_rad[NUM_JOINTS] = {
  200*16 / (2*3.14159), // assume 200steps/rev *16µsteps per rad, gear ratio 1:1
  200*16 / (2*3.14159),
  200*16 / (2*3.14159),
  200*16 / (2*3.14159),
  200*16 / (2*3.14159)
};

sensor_msgs::JointState joint_state_msg;
ros::Publisher pub_js("arm/joint_states", &joint_state_msg);

sensor_msgs::JointState cmd_msg;
bool cmd_received = false;

void cmdCb(const sensor_msgs::JointState& msg) {
  cmd_msg = msg;
  cmd_received = true;
}
ros::Subscriber<sensor_msgs::JointState> sub_cmd("arm/joint_position_cmd", &cmdCb);

void homeJoint(int i) {
  steppers[i].setMaxSpeed(200.0);
  steppers[i].setAcceleration(100.0);
  // Move negative until switch
  while (digitalRead(limitPins[i]) == HIGH) {
    steppers[i].setSpeed(-100.0);
    steppers[i].runSpeed();
  }
  steppers[i].setCurrentPosition(0);
}

void setup() {
  nh.initNode();
  nh.subscribe(sub_cmd);
  nh.advertise(pub_js);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // enable drivers

  for (int i=0;i<NUM_JOINTS;i++) {
    pinMode(limitPins[i], INPUT_PULLUP);
    steppers[i].setMaxSpeed(1000.0);
    steppers[i].setAcceleration(500.0);
  }

  // Homing
  for (int i=0;i<NUM_JOINTS;i++) {
    homeJoint(i);
  }

  joint_state_msg.name.resize(NUM_JOINTS);
  joint_state_msg.position.resize(NUM_JOINTS);
  joint_state_msg.name[0] = "arm_shoulder_joint";
  joint_state_msg.name[1] = "arm_elbow_joint";
  joint_state_msg.name[2] = "arm_wrist_pitch_joint";
  joint_state_msg.name[3] = "arm_wrist_roll_joint";
  joint_state_msg.name[4] = "arm_gripper_joint";
}

unsigned long last_pub = 0;

void loop() {
  nh.spinOnce();

  // Handle new command
  if (cmd_received) {
    cmd_received = false;
    for (int i=0;i<NUM_JOINTS && i < cmd_msg.position_length;i++) {
      long target_steps = (long)(cmd_msg.position[i] * steps_per_rad[i]);
      steppers[i].moveTo(target_steps);
    }
  }

  // Run steppers
  for (int i=0;i<NUM_JOINTS;i++) {
    steppers[i].run();
  }

  // Publish joint states
  unsigned long now = millis();
  if (now - last_pub > 20) {
    last_pub = now;
    joint_state_msg.header.stamp = nh.now();
    for (int i=0;i<NUM_JOINTS;i++) {
      joint_state_msg.position[i] =
        (float)steppers[i].currentPosition() / (float)steps_per_rad[i];
    }
    pub_js.publish(&joint_state_msg);
  }
}
