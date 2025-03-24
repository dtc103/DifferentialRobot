#include "Robot.h"

volatile bool DifferentialRobot::data_ready = false;

DifferentialRobot::DifferentialRobot(){
    pinMode(this->STATUS_LED_RED_PIN, OUTPUT);
    pinMode(this->STATUS_LED_GREEN_PIN, OUTPUT);
    this->set_status_lights(false);
}

void DifferentialRobot::init(){
    DifferentialRobot::data_ready = false;

    Wire.begin();
    Wire.setClock(400000);

    this->mpu.initialize();

    uint8_t dev_status = this->mpu.dmpInitialize();

    if(dev_status == 0){
        this->mpu.CalibrateAccel(6);
        this->mpu.CalibrateGyro(6);
        this->mpu.setDMPEnabled(true);

        attachInterrupt(digitalPinToInterrupt(DifferentialRobot::INTERRUPT_PIN), DifferentialRobot::interrupt_handler, RISING);

        this->packet_size = this->mpu.dmpGetFIFOPacketSize();

        //let the mpu warm up a bit 
        delay(10000);
    }else{
        this->error();
    }

    this->yaw = (int)read_yaw();

    this->pid = PID(1.0, 0.0, 0.1, (float)this->yaw);

    this->m1 = new Motor(DifferentialRobot::LEFT_MOTOR_PIN_IN1, DifferentialRobot::LEFT_MOTOR_PIN_IN2, DifferentialRobot::LEFT_MOTOR_PIN_PWM, 1, DifferentialRobot::STANDBY_PIN);
    this->m2 = new Motor(DifferentialRobot::RIGHT_MOTOR_PIN_IN1, DifferentialRobot::RIGHT_MOTOR_PIN_IN2, DifferentialRobot::RIGHT_MOTOR_PIN_PWM, 1, DifferentialRobot::STANDBY_PIN);

    this->set_status_lights(true);
}

void DifferentialRobot::update_actuator(){
    float curr_yaw = this->yaw;
    if(curr_yaw == (int)1000){
        return;
    }
    float error = this->pid.update(curr_yaw);

    if(this->direction == 0){
        m1->drive(error / 2);
        m2->drive(-error / 2);
    }
    if(this->direction == 1){
        m1->drive(this->cruising_speed + (error / 2));
        m2->drive(this->cruising_speed - (error / 2));
    }
    if(this->direction == -1){
        m1->drive(-(this->cruising_speed) + (error / 2));
        m2->drive(-(this->cruising_speed) - (error / 2));
    }
}

void DifferentialRobot::set_status_lights(bool is_ok){
    if(is_ok){
        digitalWrite(DifferentialRobot::STATUS_LED_GREEN_PIN, HIGH);
        digitalWrite(DifferentialRobot::STATUS_LED_RED_PIN, LOW);
    }else{
        digitalWrite(DifferentialRobot::STATUS_LED_GREEN_PIN, LOW);
        digitalWrite(DifferentialRobot::STATUS_LED_RED_PIN, HIGH);
    }
}

void DifferentialRobot::forward(){
    this->direction = 1;
}

void DifferentialRobot::backward(){
    this->direction = -1;
}

void DifferentialRobot::stop(){
    this->direction = 0;
}

float DifferentialRobot::read_yaw(){
    if(data_ready){
        DifferentialRobot::data_ready = false;
        if(mpu.dmpGetCurrentFIFOPacket(fifo_buffer)){
            mpu.dmpGetQuaternion(&(this->q), fifo_buffer);
            mpu.dmpGetGravity(&(this->gravity), &(this->q));
            mpu.dmpGetYawPitchRoll(ypr, &(this->q), &(this->gravity));

            float yaw_degrees = ypr[0] * 180 / M_PI;
            float normalized_yaw = fmod(yaw_degrees + 360.0, 360.0);
            
            return normalized_yaw;
        }
    }
    return 1000.0;
}

void DifferentialRobot::set_yaw(int yaw){
    float normalized = fmod(yaw + 360.0, 360.0);
    this->yaw = normalized;
    Serial.println(this->yaw);
}

int DifferentialRobot::get_yaw(){
    return this->yaw;
}

void DifferentialRobot::error(){
    bool is_on = true;
    digitalWrite(DifferentialRobot::STATUS_LED_GREEN_PIN, LOW);
    
    while(1){
        if(is_on){
            digitalWrite(DifferentialRobot::STATUS_LED_RED_PIN, LOW);
            is_on = false;
        }else{
            digitalWrite(DifferentialRobot::STATUS_LED_RED_PIN, HIGH);
            is_on = true;
        }
        delay(250);
    }
}

void DifferentialRobot::emergency_stop(){
    brake(*m1, *m2);
    this->error();
}




