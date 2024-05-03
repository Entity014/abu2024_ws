#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <PWMServo.h>
#include <WinsonLib.h>
#include <SimpleKalmanFilter.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < Define > -------------------------------------//

rcl_publisher_t amp_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t volt_publisher;
rcl_publisher_t start_publisher;
rcl_publisher_t debug_motor_publisher;
rcl_publisher_t debug_pwm_publisher;
rcl_publisher_t debug_encoder_publisher;
rcl_publisher_t debug_heading_publisher;
// rcl_publisher_t imu_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t state_subscriber;

std_msgs__msg__Int8 start_msg;
std_msgs__msg__Float32 amp_msg;
std_msgs__msg__Float32 volt_msg;
std_msgs__msg__String state_msg;
nav_msgs__msg__Odometry odom_msg;
// sensor_msgs__msg__Imu imu_msg;
geometry_msgs__msg__Twist twist_msg;
geometry_msgs__msg__Twist debug_motor_msg;
geometry_msgs__msg__Twist debug_pwm_msg;
geometry_msgs__msg__Twist debug_encoder_msg;
geometry_msgs__msg__Twist debug_heading_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t rgb_timer;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PWMServo servo1_controller;
PWMServo servo2_controller;
PWMServo servo3_controller;
PWMServo servo4_controller;

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::SWERVE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    RPM_RATIO,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

Odometry odometry;
// IMU imu;

WCS WCS1 = WCS(0, _WCS1700);

SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

float now_heading1 = 90;
float now_heading2 = 90;
float now_heading3 = 90;
float now_heading4 = 90;

//------------------------------ < Fuction Prototype > ------------------------------//

void flashLED(int n_times);
void rclErrorLoop();
void syncTime();
void moveBase();
void readSensor();
void publishData();
bool createEntities();
bool destroyEntities();
struct timespec getTime();

//------------------------------ < Main > -------------------------------------//

void setup()
{
    WCS1.Reset();
    pinMode(LED_PIN, OUTPUT);
    pinMode(VOLT_METER, INPUT);
    pinMode(EMERGENCY, OUTPUT);
    pinMode(START, INPUT_PULLUP);

    // bool imu_ok = imu.init();
    // if (!imu_ok)
    // {
    //     while (1)
    //     {
    //         flashLED(3);
    //     }
    // }

    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    servo1_controller.attach(SERVO1);
    servo2_controller.attach(SERVO2);
    servo3_controller.attach(SERVO3);
    servo4_controller.attach(SERVO4);
    servo1_controller.write(90);
    servo2_controller.write(90);
    servo3_controller.write(90);
    servo4_controller.write(90);
}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            digitalWrite(EMERGENCY, HIGH);
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        servo1_controller.write(90);
        servo2_controller.write(90);
        servo3_controller.write(90);
        servo4_controller.write(90);
        digitalWrite(EMERGENCY, LOW);
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        readSensor();
        moveBase();
        publishData();
    }
}

void rgbCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
    }
}

void twistCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

void stateCallback(const void *msgin)
{
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom/unfiltered"));
    RCCHECK(rclc_publisher_init_default(
        &volt_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "voltage"));
    RCCHECK(rclc_publisher_init_default(
        &amp_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "ampere"));
    RCCHECK(rclc_publisher_init_default(
        &start_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "button/start"));

    RCCHECK(rclc_publisher_init_default(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/motor"));
    RCCHECK(rclc_publisher_init_default(
        &debug_pwm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/pwm"));

    RCCHECK(rclc_publisher_init_default(
        &debug_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/encoder"));

    RCCHECK(rclc_publisher_init_default(
        &debug_heading_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/heading"));
    // create IMU publisher
    // RCCHECK(rclc_publisher_init_default(
    //     &imu_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    //     "imu/data"));

    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default(
        &twist_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));
    RCCHECK(rclc_subscription_init_default(
        &state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robot/state"));

    state_msg.data.capacity = 10;
    state_msg.data.size = 10;
    state_msg.data.data = (char *)malloc(state_msg.data.capacity * sizeof(char));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    RCCHECK(rclc_timer_init_default(
        &rgb_timer,
        &support,
        RCL_MS_TO_NS(20),
        rgbCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &twist_subscriber,
        &twist_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &state_subscriber,
        &state_msg,
        &stateCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &rgb_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_publisher_fini(&debug_pwm_publisher, &node);
    rcl_publisher_fini(&debug_encoder_publisher, &node);
    rcl_publisher_fini(&debug_heading_publisher, &node);
    rcl_publisher_fini(&start_publisher, &node);
    rcl_publisher_fini(&odom_publisher, &node);
    rcl_publisher_fini(&volt_publisher, &node);
    rcl_publisher_fini(&amp_publisher, &node);
    // rcl_publisher_fini(&imu_publisher, &node);
    rcl_subscription_fini(&state_subscriber, &node);
    rcl_subscription_fini(&twist_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void readSensor()
{
    start_msg.data = !digitalRead(START);
    amp_msg.data = WCS1.A_DC();
    volt_msg.data = map((double)analogRead(VOLT_METER), 0, 1023, 0, 25);
}

void moveBase()
{
    if (((millis() - prev_cmd_time) >= 200))
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }

    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z);
    Kinematics::heading req_heading = kinematics.getHeading(
        twist_msg.linear.x,
        twist_msg.linear.y,
        twist_msg.angular.z);

    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();
    float current_heading1 = 90 + constrain(map(req_heading.motor1 * RAD_TO_DEG, -180, 180, -140, 140), -70, 70);
    float current_heading2 = 90 + constrain(map(req_heading.motor2 * RAD_TO_DEG, -180, 180, -140, 140), -70, 70);
    float current_heading3 = 90 + constrain(map(req_heading.motor3 * RAD_TO_DEG, -180, 180, -140, 140), -70, 70);
    float current_heading4 = 90 + constrain(map(req_heading.motor4 * RAD_TO_DEG, -180, 180, -140, 140), -70, 70);
    current_rpm1 = round(simpleKalmanFilter.updateEstimate(current_rpm1));
    current_rpm2 = round(simpleKalmanFilter.updateEstimate(current_rpm2));
    current_rpm3 = round(simpleKalmanFilter.updateEstimate(current_rpm3));
    current_rpm4 = round(simpleKalmanFilter.updateEstimate(current_rpm4));

    debug_motor_msg.linear.x = req_rpm.motor1;
    debug_motor_msg.linear.y = req_rpm.motor2;
    debug_motor_msg.linear.z = req_rpm.motor3;
    debug_motor_msg.angular.x = req_rpm.motor4;
    debug_encoder_msg.linear.x = current_rpm1;
    debug_encoder_msg.linear.y = current_rpm2;
    debug_encoder_msg.linear.z = current_rpm3;
    debug_encoder_msg.angular.x = current_rpm4;

    if (now_heading1 < round(current_heading1))
    {
        now_heading1 += map(abs(now_heading1 - current_heading1), 0, 140, 0, 20);
    }
    else if (now_heading1 > round(current_heading1))
    {
        now_heading1 -= map(abs(now_heading1 - current_heading1), 0, 140, 0, 20);
    }

    if (now_heading2 < round(current_heading2))
    {
        now_heading2 += map(abs(now_heading2 - current_heading2), 0, 140, 0, 20);
    }
    else if (now_heading2 > round(current_heading2))
    {
        now_heading2 -= map(abs(now_heading2 - current_heading2), 0, 140, 0, 20);
    }

    if (now_heading3 < round(current_heading3))
    {
        now_heading3 += map(abs(now_heading3 - current_heading3), 0, 140, 0, 20);
    }
    else if (now_heading3 > round(current_heading3))
    {
        now_heading3 -= map(abs(now_heading3 - current_heading3), 0, 140, 0, 20);
    }

    if (now_heading4 < round(current_heading4))
    {
        now_heading4 += map(abs(now_heading4 - current_heading4), 0, 140, 0, 20);
    }
    else if (now_heading4 > round(current_heading4))
    {
        now_heading4 -= map(abs(now_heading4 - current_heading4), 0, 140, 0, 20);
    }

    servo1_controller.write(current_heading1);
    servo2_controller.write(current_heading2);
    servo3_controller.write(current_heading3);
    servo4_controller.write(current_heading4);

    debug_heading_msg.linear.x = round(now_heading1);
    debug_heading_msg.linear.y = round(now_heading2);
    debug_heading_msg.linear.z = round(now_heading3);
    debug_heading_msg.angular.x = round(now_heading4);
    // debug_heading_msg.linear.x = req_heading.motor1 * RAD_TO_DEG;
    // debug_heading_msg.linear.y = req_heading.motor2 * RAD_TO_DEG;
    // debug_heading_msg.linear.z = req_heading.motor3 * RAD_TO_DEG;
    // debug_heading_msg.angular.x = req_heading.motor4 * RAD_TO_DEG;

    if (round(now_heading1) == round(current_heading1))
    {
        debug_pwm_msg.linear.x = motor1_pid.compute(req_rpm.motor1, current_rpm1);
        motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
    }
    if (round(now_heading2) == round(current_heading2))
    {
        debug_pwm_msg.linear.y = motor2_pid.compute(req_rpm.motor2, current_rpm2);
        motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
    }
    if (round(now_heading3) == round(current_heading3))
    {
        debug_pwm_msg.linear.z = motor3_pid.compute(req_rpm.motor3, current_rpm3);
        motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
    }
    if (round(now_heading4) == round(current_heading4))
    {
        debug_pwm_msg.angular.x = motor4_pid.compute(req_rpm.motor4, current_rpm4);
        motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));
    }

    Kinematics::velocities current_vel = kinematics.getVelocities(
        req_heading,
        current_rpm1,
        current_rpm2,
        current_rpm3,
        current_rpm4);

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt,
        current_vel.linear_x,
        current_vel.linear_y,
        current_vel.angular_z);
}

void publishData()
{
    odom_msg = odometry.getData();
    // imu_msg = imu.getData();

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // imu_msg.header.stamp.sec = time_stamp.tv_sec;
    // imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    // RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
    RCSOFTCHECK(rcl_publish(&amp_publisher, &amp_msg, NULL));
    RCSOFTCHECK(rcl_publish(&volt_publisher, &volt_msg, NULL));
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
    RCSOFTCHECK(rcl_publish(&start_publisher, &start_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_pwm_publisher, &debug_pwm_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_encoder_publisher, &debug_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&debug_heading_publisher, &debug_heading_msg, NULL));
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
        flashLED(2);
    }
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}