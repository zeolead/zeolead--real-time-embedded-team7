/*
Motor Control layer
You can control motor with speed and threads now
1. Initial default setting of a motor class
2. Stop this motor thread
3. Initial motor in real world, send direction to outside, and open a thread for the chosen one
4. Close this motor thread, and cut its power.
5. Change the speed of this running motor, and direcion
6. The program runs in thread. The chosen motor runs forever with changing speed.
7. Send direction to outside
8. Send rpm to outside
*/

#include "MotorControl.hpp"
#include "DRV8825.hpp"
#include <iostream>
#include <functional>
#include <unistd.h> // usleep

//1. Initial default setting of a motor class
MotorControl::MotorControl(UBYTE motor_id)
	: motor_id_(motor_id), running_(false), rpm_(0), direction_(DRV8825::FORWARD) {}

// Stop thread
MotorControl::~MotorControl() {
	this->stop();
}

// 3. Initial motor in real world, send direction to outside, and open a thread for the chosen one
void MotorControl::start() {
	std::lock_guard<std::mutex> lock(mutex_);
	if (running_) return;
	running_ = true;

	DRV8825::SelectMotor(motor_, motor_id_);
	DRV8825::Enable(motor_);
	//Debug::Log("motor %d, start\n",motor_id_);
	if (statusCallback_) statusCallback_(direction_, static_cast<int>(rpm_));
	control_thread_ = std::thread(&MotorControl::Run, this);
}

// 4. Close this motor thread, and cut its power.
void MotorControl::stop() {
	{
		std::lock_guard<std::mutex> lock(mutex_);
		running_ = false;
	}
	if (control_thread_.joinable()) {
		control_thread_.join();
	}
	DRV8825::Stop(motor_);
}

/*change spead and direction
change direction with negative rpm*/
void MotorControl::setRPM(float rpm) {
	std::lock_guard<std::mutex> lock(mutex_);
	if (rpm >= 0) {
		direction_ = DRV8825::FORWARD;
	}
	else {
		direction_ = DRV8825::BACKWARD;
		rpm = -rpm;  //
	}
	rpm_ = rpm;
	Debug::Log("set rpm %f  (dir %d) for motor %d\n", rpm, direction_.load(), motor_id_);
	if (statusCallback_) statusCallback_(direction_, static_cast<int>(rpm_));
}

// not used
//void MotorControl::SetDirection(UBYTE dir) {
//	direction_ = dir;
//}

// 6. The program runs in thread. The chosen motor runs forever with changing speed.
void MotorControl::Run() {
	constexpr int steps_per_rev = 800;
	auto next_time = std::chrono::steady_clock::now();
	while (running_) {
		float current_rpm;
		UBYTE dir;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			current_rpm = rpm_;
			dir = direction_;
		}
        if (current_rpm > 0) {
            int delay_us = static_cast<int>(60 * 1e5 / (current_rpm * steps_per_rev));
            DRV8825::TurnStep(motor_, dir, 1, delay_us);
            if (stepCallback_) stepCallback_();
            next_time += std::chrono::microseconds(delay_us);
            std::this_thread::sleep_until(next_time);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
	}
}

void MotorControl::setStatusCallback(const std::function<void(uint8_t,int)>& cb) {
    statusCallback_ = cb;
}

void MotorControl::setStepCallback(const std::function<void()>& cb) {
    stepCallback_ = cb;
}
