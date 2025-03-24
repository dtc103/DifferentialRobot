#include "credentials.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <Arduino.h>
#include <Robot.h>

DifferentialRobot robot;
bool direction_message_arrived;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t yaw_sub;
rcl_subscription_t direction_sub;
rcl_timer_t direction_heartbeat_timer;

std_msgs__msg__Int32 yaw_msg;
std_msgs__msg__Int32 direction_msg;

void yaw_callback(const void* in_msg){
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*) in_msg;

    if(msg->data == 1){
        robot.set_yaw(robot.get_yaw() + 1);
    }else if(msg->data == -1){
        robot.set_yaw(robot.get_yaw() - 1);
    }
}

void direction_callback(const void* in_msg){
    const std_msgs__msg__Int32* msg = (const std_msgs__msg__Int32*) in_msg;

    direction_message_arrived = true;

    if(msg->data == 0){
        robot.stop();
    }
    if(msg->data == 1){
        robot.forward();
    }
    if(msg->data == -1){
        robot.backward();
    }
}

void direction_heartbeat_callback(rcl_timer_t *timer, int64_t last_call_time){
    RCLC_UNUSED(last_call_time);
    if(timer != NULL){
        if(!direction_message_arrived){
            robot.stop();
        }else{
            direction_message_arrived = false;
        }
    }
}

void setup() {
    Serial.begin(115200);

    robot.init();
    robot.set_status_lights(false);
    robot.set_yaw(0);

    IPAddress ipaddr;
    ipaddr.fromString(AGENT_IP);
    set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, ipaddr, AGENT_PORT);

    allocator = rcl_get_default_allocator();
    if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK){
        robot.error();
    }

    if(rclc_node_init_default(&node, "robot_node", "", &support) != RCL_RET_OK){
        robot.error();
    }

    if(rclc_subscription_init_default(&yaw_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "robot_yaw") != RCL_RET_OK){
        robot.error();
    }

    if(rclc_subscription_init_default(&direction_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "robot_direction") != RCL_RET_OK){
        robot.error();
    }

    if(rclc_timer_init_default(&direction_heartbeat_timer, &support, RCL_MS_TO_NS(100), direction_heartbeat_callback) != RCL_RET_OK){
        robot.error();
    }

    if(rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK){
        robot.error();
    }

    if(rclc_executor_add_subscription(&executor, &yaw_sub, &yaw_msg, &yaw_callback, ON_NEW_DATA) != RCL_RET_OK){
        robot.error();
    }

    if(rclc_executor_add_subscription(&executor, &direction_sub, &direction_msg, &direction_callback, ON_NEW_DATA) != RCL_RET_OK){
        robot.error();
    }

    if(rclc_executor_add_timer(&executor, &direction_heartbeat_timer) != RCL_RET_OK){
        robot.error();
    }

    robot.set_status_lights(true);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200));
    robot.update_actuator();
}