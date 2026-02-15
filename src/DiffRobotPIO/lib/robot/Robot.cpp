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

    // Initialize all controllers
    // TODO: Tune PID gains for robot
    this->pid_controller = PID(4.0f, 1.5f, 3.0f, (float)this->yaw);

    // TODO: Tune Stanley gains for robot (Keep parameter set here to remember)
    // k_heading=1.0: heading error gain
    // k_crosstrack=0.5: cross-track gain (no effect until encoders added)
    // k_soft=1.0: softening at low speed
    // k_damp=0.1: yaw rate damping (increase if oscillating)
    this->stanley_controller = StanleyController(1.0f, 0.5f, 1.0f, 0.1f);

    // TODO: Tune MPC parameters for robot (Keep parameter set here to remember)
    // horizon=5: prediction horizon
    // model_dt=0.05: time step (matches ~20Hz control loop)
    // model_a=0.8: yaw rte decay (0.8 = moderate friction)
    // model_b=0.5: input-to-yaw-rate gain
    // cost_q=10.0: heading error weight
    // cost_r=0.1: control effort weight
    // cost_qf=20.0: terminal heading error weight
    // u_min=-255: motor min
    // u_max=255: motor max
    this->mpc_controller = MPC(5, 0.05f, 0.8f, 0.5f, 10.0f, 0.1f, 20.0f, -255.0f, 255.0f);

    // Start with PID controller (proven to work on the robot)
    this->control_mode = PID_MODE;
    this->active_controller = &this->pid_controller;

    this->last_update_time = millis();

    this->m1 = new Motor(DifferentialRobot::LEFT_MOTOR_PIN_IN1, DifferentialRobot::LEFT_MOTOR_PIN_IN2, DifferentialRobot::LEFT_MOTOR_PIN_PWM, 1, DifferentialRobot::STANDBY_PIN);
    this->m2 = new Motor(DifferentialRobot::RIGHT_MOTOR_PIN_IN1, DifferentialRobot::RIGHT_MOTOR_PIN_IN2, DifferentialRobot::RIGHT_MOTOR_PIN_PWM, 1, DifferentialRobot::STANDBY_PIN);

    this->set_status_lights(true);
}

void DifferentialRobot::update_actuator(){
    float measured_yaw = this->read_yaw();
    if(measured_yaw == 1000.0f){
        return;
    }

    // Compute dt in seconds
    unsigned long now = millis();
    float dt_s = (float)(now - this->last_update_time) / 1000.0f;
    this->last_update_time = now;

    // Guard against unreasonable dt
    if(dt_s <= 0.0f || dt_s > 1.0f){
        dt_s = 0.05f;  // fallback to 50ms
    }

    // Read yaw rate from gyroscope
    float yaw_rate = this->read_yaw_rate();

    // Build controller input
    ControllerInput input;
    input.current_yaw = measured_yaw;
    input.current_yaw_rate = yaw_rate;
    input.target_yaw = (float)this->yaw;
    input.velocity = 0.0f;  // No velocity sensor yet
    input.dt = dt_s;

    float correction = this->active_controller->compute(input);

    if(this->direction == 0){
        m1->drive(correction / 2);
        m2->drive(-correction / 2);
    }
    if(this->direction == 1){
        m1->drive(this->cruising_speed + (correction / 2));
        m2->drive(this->cruising_speed - (correction / 2));
    }
    if(this->direction == -1){
        m1->drive(-(this->cruising_speed) + (correction / 2));
        m2->drive(-(this->cruising_speed) - (correction / 2));
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

float DifferentialRobot::read_yaw_rate(){
    // Read raw gyroscope Z-axis data.
    int16_t gz_raw = mpu.getRotationZ();
    // TODO: Adjust this sensitivity if changing the gyro full-scale range in MPU6050 config.
    // Default (FS_SEL=0, +/-250 deg/s): sensitivity = 131.0 LSB/(deg/s)
    // FS_SEL=1 (+/-500 deg/s):  sensitivity = 65.5
    // FS_SEL=2 (+/-1000 deg/s): sensitivity = 32.8
    // FS_SEL=3 (+/-2000 deg/s): sensitivity = 16.4
    float yaw_rate_dps = (float)gz_raw / 131.0f;
    return yaw_rate_dps;
}

void DifferentialRobot::set_yaw(int yaw){
    float normalized = fmod(yaw + 360.0, 360.0);
    this->yaw = normalized;
    // Keep PID setpoint in sync (for legacy update() method compatibility)
    this->pid_controller.new_set_point(normalized);
    Serial.println(this->yaw);
}

int DifferentialRobot::get_yaw(){
    return this->yaw;
}

void DifferentialRobot::set_control_mode(int mode){
    if(mode < 0 || mode > 2) return;

    ControlMode new_mode = (ControlMode)mode;
    if(new_mode == this->control_mode) return;

    this->control_mode = new_mode;

    switch(new_mode){
        case PID_MODE:
            this->active_controller = &this->pid_controller;
            Serial.println("Controller: PID");
            break;
        case STANLEY_MODE:
            this->active_controller = &this->stanley_controller;
            Serial.println("Controller: Stanley");
            break;
        case MPC_MODE:
            this->active_controller = &this->mpc_controller;
            Serial.println("Controller: MPC");
            break;
    }

    // Reset new controller state to avoid transients from stale internal data
    this->active_controller->reset();
}

int DifferentialRobot::get_control_mode(){
    return (int)this->control_mode;
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
