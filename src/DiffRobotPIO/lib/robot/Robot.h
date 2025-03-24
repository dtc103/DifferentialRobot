#ifndef ROBOT_H
#define ROBOT_H

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps612.h>
#include <Wire.h>
#include <PID.h>
#include <SparkFun_TB6612.h>

class DifferentialRobot{
    public:
        static const int LEFT_MOTOR_PIN_PWM = GPIO_NUM_5;
        static const int LEFT_MOTOR_PIN_IN1 = GPIO_NUM_16;
        static const int LEFT_MOTOR_PIN_IN2 = GPIO_NUM_17;
        static const int RIGHT_MOTOR_PIN_PWM = GPIO_NUM_4;
        static const int RIGHT_MOTOR_PIN_IN1 = GPIO_NUM_2;
        static const int RIGHT_MOTOR_PIN_IN2 = GPIO_NUM_0;
        static const int STANDBY_PIN = GPIO_NUM_15;

        static const int STATUS_LED_RED_PIN = GPIO_NUM_18;
        static const int STATUS_LED_GREEN_PIN = GPIO_NUM_19;        

        //if new data in the mpu arrives
        static const int INTERRUPT_PIN = GPIO_NUM_1;
        static volatile bool data_ready;
        static void interrupt_handler(){
            DifferentialRobot::data_ready = true;
        }

        DifferentialRobot();

        void init();

        void update_actuator();
        void forward();
        void backward();
        void stop();

        void set_status_lights(bool);

        float read_yaw();
        void set_yaw(int);
        int get_yaw();
        
        void error();
        void emergency_stop();

    private:
        int left_motor_vel = 0;
        int right_motor_vel = 0;
        int cruising_speed = 230; //to leave some space for turning (speed between 0 and 255)
        int direction = 0; //0=stop, 1=forward, -1=backward

        Motor* m1;
        Motor* m2;

        uint16_t packet_size;
        uint8_t fifo_buffer[64];
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];
        float euler[3];
        int yaw = 0;

        MPU6050 mpu;

        PID pid;
};


#endif