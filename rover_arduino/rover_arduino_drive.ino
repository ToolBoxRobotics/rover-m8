#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>   // PCA9685
#include <Adafruit_INA219.h>
#include <Adafruit_AHTX0.h>
#include "MPU6050.h"                   // Use a lightweight MPU6050 lib
#include <ros.h>
#include <rover_arduino_drive/DriveCommand.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>

// ==== ROS ====
ros::NodeHandle nh;

// ==== Motor driver + PWM ====
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// DRI0002 DIR pins
const uint8_t DIR_FL = 22;
const uint8_t DIR_ML = 23;
const uint8_t DIR_RL = 24;
const uint8_t DIR_FR = 25;
const uint8_t DIR_MR = 26;
const uint8_t DIR_RR = 27;

// Map wheel to PCA9685 channels
const uint8_t PWM_FL = 0;
const uint8_t PWM_ML = 1;
const uint8_t PWM_RL = 2;
const uint8_t PWM_FR = 3;
const uint8_t PWM_MR = 4;
const uint8_t PWM_RR = 5;

// Steering servos on PCA9685 channels
const uint8_t SERVO_FL = 8;
const uint8_t SERVO_FR = 9;
const uint8_t SERVO_RL = 10;
const uint8_t SERVO_RR = 11;

// Servo calibration
const uint16_t SERVO_MIN = 150;  // ~0 deg
const uint16_t SERVO_MAX = 600;  // ~180 deg

// ==== Encoders (example pins) ====
volatile long enc_fl = 0, enc_ml = 0, enc_rl = 0, enc_fr = 0, enc_mr = 0, enc_rr = 0;
// attachInterrupt() etc. in setup() - omitted detail

// ==== I2C Mux ====
#define TCA_ADDR 0x70

void tca_select(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// ==== Sensors ====
Adafruit_AHTX0 aht;
Adafruit_INA219 ina219;
MPU6050 mpu;

sensor_msgs::Imu imu_msg;
sensor_msgs::Temperature temp_msg;
sensor_msgs::RelativeHumidity hum_msg;
sensor_msgs::BatteryState batt_msg;
sensor_msgs::JointState joint_state_msg;

// ==== Drive command callback ====
rover_arduino_drive::DriveCommand drive_cmd;

void driveCmdCb(const rover_arduino_drive::DriveCommand& msg) {
  drive_cmd = msg;

  // Convert velocities to PWM: simple proportional mapping
  auto setMotor = [](float vel, uint8_t dir_pin, uint8_t pwm_chan) {
    const float MAX_VEL = 10.0; // rad/s or similar
    int pwm_val = (int)(fabs(vel) / MAX_VEL * 4095);
    if (pwm_val > 4095) pwm_val = 4095;
    digitalWrite(dir_pin, vel >= 0 ? HIGH : LOW);
    pwm.setPWM(pwm_chan, 0, pwm_val);
  };

  setMotor(msg.fl_velocity, DIR_FL, PWM_FL);
  setMotor(msg.ml_velocity, DIR_ML, PWM_ML);
  setMotor(msg.rl_velocity, DIR_RL, PWM_RL);
  setMotor(msg.fr_velocity, DIR_FR, PWM_FR);
  setMotor(msg.mr_velocity, DIR_MR, PWM_MR);
  setMotor(msg.rr_velocity, DIR_RR, PWM_RR);

  // Steering: radians -> servo pulse
  auto setServoAngle = [](float angle_rad, uint8_t chan) {
    float angle_deg = angle_rad * 180.0 / 3.14159;
    if (angle_deg < -45) angle_deg = -45;
    if (angle_deg > 45)  angle_deg = 45;
    float norm = (angle_deg + 45.0) / 90.0; // 0..1
    uint16_t pulse = SERVO_MIN + norm * (SERVO_MAX - SERVO_MIN);
    pwm.setPWM(chan, 0, pulse);
  };

  setServoAngle(msg.fl_steer_angle, SERVO_FL);
  setServoAngle(msg.fr_steer_angle, SERVO_FR);
  setServoAngle(msg.rl_steer_angle, SERVO_RL);
  setServoAngle(msg.rr_steer_angle, SERVO_RR);
}

ros::Subscriber<rover_arduino_drive::DriveCommand> sub_drive("rover/drive_cmd", &driveCmdCb);

// Publishers
ros::Publisher pub_imu("imu/data", &imu_msg);
ros::Publisher pub_temp("aht20/temperature", &temp_msg);
ros::Publisher pub_hum("aht20/humidity", &hum_msg);
ros::Publisher pub_batt("ina219/battery", &batt_msg);
ros::Publisher pub_joint("joint_states_raw", &joint_state_msg);

unsigned long last_pub = 0;

void setup() {
  // ROS
  nh.initNode();
  nh.subscribe(sub_drive);
  nh.advertise(pub_imu);
  nh.advertise(pub_temp);
  nh.advertise(pub_hum);
  nh.advertise(pub_batt);
  nh.advertise(pub_joint);

  // Pins
  pinMode(DIR_FL, OUTPUT); pinMode(DIR_ML, OUTPUT); pinMode(DIR_RL, OUTPUT);
  pinMode(DIR_FR, OUTPUT); pinMode(DIR_MR, OUTPUT); pinMode(DIR_RR, OUTPUT);

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // servos & motors

  // Init sensors via mux
  // MPU6050
  tca_select(0);
  mpu.initialize();

  // AHT20
  tca_select(1);
  aht.begin();

  // INA219
  tca_select(2);
  ina219.begin();

  // Init encoder interrupts here (omitted for brevity)
}

void loop() {
  nh.spinOnce();

  unsigned long now = millis();
  if (now - last_pub > 20) { // 50 Hz
    last_pub = now;

    // Read IMU
    tca_select(0);
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    imu_msg.header.stamp = nh.now();
    imu_msg.linear_acceleration.x = ax * 9.81 / 16384.0;
    imu_msg.linear_acceleration.y = ay * 9.81 / 16384.0;
    imu_msg.linear_acceleration.z = az * 9.81 / 16384.0;
    imu_msg.angular_velocity.x    = gx * 3.14159 / (180.0 * 131.0);
    imu_msg.angular_velocity.y    = gy * 3.14159 / (180.0 * 131.0);
    imu_msg.angular_velocity.z    = gz * 3.14159 / (180.0 * 131.0);
    // Orientation left zero; fuse on PC
    pub_imu.publish(&imu_msg);

    // AHT20
    tca_select(1);
    sensors_event_t hum, temp;
    aht.getEvent(&hum, &temp);

    temp_msg.header.stamp = imu_msg.header.stamp;
    temp_msg.temperature  = temp.temperature;
    hum_msg.header.stamp  = imu_msg.header.stamp;
    hum_msg.relative_humidity = hum.relative_humidity;
    pub_temp.publish(&temp_msg);
    pub_hum.publish(&hum_msg);

    // INA219
    tca_select(2);
    float bus_v = ina219.getBusVoltage_V();
    float current_mA = ina219.getCurrent_mA();

    batt_msg.header.stamp = imu_msg.header.stamp;
    batt_msg.voltage = bus_v;
    batt_msg.current = current_mA / 1000.0;
    batt_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    pub_batt.publish(&batt_msg);

    // Joint states from encoders
    joint_state_msg.header.stamp = imu_msg.header.stamp;
    joint_state_msg.name.clear();
    joint_state_msg.position.clear();
    // Add wheel joint names + positions (convert counts to radians)
    const float TICKS_PER_REV = 2048.0;
    auto add_joint = [&](const char* name, long counts) {
      joint_state_msg.name.push_back(name);
      joint_state_msg.position.push_back(2.0 * 3.14159 * (float)counts / TICKS_PER_REV);
    };
    add_joint("front_left_wheel_joint",  enc_fl);
    add_joint("mid_left_wheel_joint",    enc_ml);
    add_joint("rear_left_wheel_joint",   enc_rl);
    add_joint("front_right_wheel_joint", enc_fr);
    add_joint("mid_right_wheel_joint",   enc_mr);
    add_joint("rear_right_wheel_joint",  enc_rr);
    pub_joint.publish(&joint_state_msg);
  }
}
