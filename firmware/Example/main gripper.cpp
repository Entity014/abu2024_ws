#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>
#include <PWMServo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include "config.h"
#include "motor.h"

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

rcl_publisher_t debug_publisher;
rcl_publisher_t limit_publisher;
// rcl_publisher_t color_publisher;
rcl_subscription_t arm_subscriber;
rcl_subscription_t hand_subscriber;
rcl_subscription_t motor_subscriber;
rcl_subscription_t state_subscriber;
rcl_subscription_t main_subscriber;

geometry_msgs__msg__Twist debug_msg;
geometry_msgs__msg__Twist limit_msg;
std_msgs__msg__Bool motor_msg;
std_msgs__msg__String arm_msg;
std_msgs__msg__String state_msg;
std_msgs__msg__Int8 main_msg;
std_msgs__msg__Int16MultiArray hand_msg;
// std_msgs__msg__Int16MultiArray color_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_timer_t rgb_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

struct rgb_colors
{
    int r, g, b;
};

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PWMServo servo1_controller;
PWMServo servo2_controller;
PWMServo servo3_controller;

bool motor_bool;
int theta[2] = {15, 110};

//------------------------------ < Fuction Prototype > ------------------------------//

void flashLED(int n_times);
void rclErrorLoop();
void syncTime();
void commandGripper();
void publishData();
bool createEntities();
bool destroyEntities();
rgb_colors GetColors();
struct timespec getTime();

//------------------------------ < Main > -------------------------------------//

void setup()
{
    pinMode(LED_PIN, OUTPUT);

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    pinMode(TOP_LIM_SWITCH, INPUT_PULLUP);
    pinMode(BOTTOM_LIM_SWITCH, INPUT_PULLUP);

    servo1_controller.attach(SERVO1);
    servo2_controller.attach(SERVO2);
    servo3_controller.attach(SERVO3);
    servo1_controller.write(15);
    servo2_controller.write(110);
    servo3_controller.write(90);
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
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        servo1_controller.write(15);
        servo2_controller.write(110);
        servo3_controller.write(90);
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
        commandGripper();
        publishData();
    }
}

void rgbCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        if (strcmp(state_msg.data.data, "IDLE") == 0)
        {
        }
        else if (strcmp(state_msg.data.data, "START") == 0)
        {
        }
        else if (strcmp(state_msg.data.data, "RESET") == 0)
        {
        }
        else
        {
        }
    }
}

void armCallback(const void *msgin)
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void handCallback(const void *msgin)
{
    theta[0] = hand_msg.data.data[0];
    theta[1] = hand_msg.data.data[1];
}

void motorCallback(const void *msgin)
{
    motor_bool = motor_msg.data;
}

void stateCallback(const void *msgin)
{
    if (strcmp(state_msg.data.data, "START") == 0)
    {
        servo3_controller.write(0);
    }
    else
    {
        servo3_controller.write(90);
    }
}

void mainCallback(const void *msgin)
{
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
    RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &debug_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/gripper"));
    RCCHECK(rclc_publisher_init_default(
        &limit_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "gripper/limit"));

    // RCCHECK(rclc_publisher_init_default(
    //     &color_publisher,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    //     "gripper/color"));

    // color_msg.data.capacity = 3;
    // color_msg.data.size = 3;
    // color_msg.data.data = (int16_t *)malloc(color_msg.data.capacity * sizeof(int16_t));

    RCCHECK(rclc_subscription_init_default(
        &arm_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "gripper/arm"));

    arm_msg.data.capacity = 10;
    arm_msg.data.size = 10;
    arm_msg.data.data = (char *)malloc(arm_msg.data.capacity * sizeof(char));

    RCCHECK(rclc_subscription_init_default(
        &hand_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        "gripper/hand"));

    hand_msg.data.capacity = 2;
    hand_msg.data.size = 2;
    hand_msg.data.data = (int16_t *)malloc(hand_msg.data.capacity * sizeof(int16_t));

    RCCHECK(rclc_subscription_init_default(
        &motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "gripper/motor"));

    RCCHECK(rclc_subscription_init_default(
        &state_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "robot/state"));
    state_msg.data.capacity = 10;
    state_msg.data.size = 10;
    state_msg.data.data = (char *)malloc(state_msg.data.capacity * sizeof(char));

    RCCHECK(rclc_subscription_init_default(
        &main_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
        "robot/main"));

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
        RCL_MS_TO_NS(control_timeout),
        rgbCallback));

    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 7, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &arm_subscriber,
        &arm_msg,
        &armCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &hand_subscriber,
        &hand_msg,
        &handCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &motor_subscriber,
        &motor_msg,
        &motorCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &state_subscriber,
        &state_msg,
        &stateCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &main_subscriber,
        &main_msg,
        &mainCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &rgb_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_publisher, &node);
    rcl_publisher_fini(&limit_publisher, &node);
    // rcl_publisher_fini(&color_publisher, &node);
    rcl_subscription_fini(&motor_subscriber, &node);
    rcl_subscription_fini(&hand_subscriber, &node);
    rcl_subscription_fini(&arm_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rcl_timer_fini(&rgb_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    digitalWrite(LED_PIN, HIGH);

    return true;
}

void commandGripper()
{
    if ((!digitalRead(TOP_LIM_SWITCH)) == HIGH && strcmp(arm_msg.data.data, "TOP") == 0)
    {
        motor1_controller.brake();
    }
    else if ((!digitalRead(TOP_LIM_SWITCH)) == LOW && strcmp(arm_msg.data.data, "TOP") == 0)
    {
        motor1_controller.spin(250);
    }
    else if ((!digitalRead(BOTTOM_LIM_SWITCH)) == HIGH && strcmp(arm_msg.data.data, "BOTTOM") == 0)
    {
        motor1_controller.brake();
    }
    else if ((!digitalRead(BOTTOM_LIM_SWITCH)) == LOW && strcmp(arm_msg.data.data, "BOTTOM") == 0)
    {
        motor1_controller.spin(-100);
    }

    servo1_controller.write(theta[0]);
    servo2_controller.write(theta[1]);

    if (motor_bool)
    {
        motor2_controller.spin(512);
    }
    else
    {
        motor2_controller.spin(0);
    }

    // rgb_colors rgb;
    // rgb = GetColors();
    // color_msg.data.data[0] = rgb.r; // Example value
    // color_msg.data.data[1] = rgb.g; // Example value
    // color_msg.data.data[2] = rgb.b; // Example value

    debug_msg.linear.x = 0.0;
    debug_msg.linear.y = !digitalRead(TOP_LIM_SWITCH);
    debug_msg.linear.z = !digitalRead(BOTTOM_LIM_SWITCH);
    limit_msg.linear.x = !digitalRead(TOP_LIM_SWITCH);
    limit_msg.linear.y = !digitalRead(BOTTOM_LIM_SWITCH);
}

void publishData()
{
    struct timespec time_stamp = getTime();

    RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
    RCSOFTCHECK(rcl_publish(&limit_publisher, &limit_msg, NULL));
    // RCSOFTCHECK(rcl_publish(&color_publisher, &color_msg, NULL));
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