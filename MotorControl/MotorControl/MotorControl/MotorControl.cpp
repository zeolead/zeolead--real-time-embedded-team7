#include "MotorControl.hpp"
#include "DRV8825.hpp"
#include <iostream>
#include <unistd.h> // usleep

MotorControl::MotorControl(UBYTE motor_id)
	: motor_id_(motor_id), running_(false), rpm_(0), direction_(DRV8825::FORWARD) {}

MotorControl::~MotorControl() {
	this->stop();
}

void MotorControl::start() {
	std::lock_guard<std::mutex> lock(mutex_);
	if (running_) return;
	running_ = true;

	DRV8825::SelectMotor(motor_, motor_id_);
	DRV8825::Enable();
	
	control_thread_ = std::thread(&MotorControl::Run, this);
}

void MotorControl::stop() {
	std::lock_guard<std::mutex> lock(mutex_);
	running_ = false;
	if (control_thread_.joinable()) {
		control_thread_.join();
	}
	DRV8825::Stop();
}

void MotorControl::setRPM(float rpm) {
	std::lock_guard<std::mutex> lock(mutex_);
	rpm_ = rpm;
}

void MotorControl::SetDirection(UBYTE dir) {
	direction_ = dir;
}

void MotorControl::Run() {
	constexpr int steps_per_rev = 800;    //每圈步数
	while (running_) {
		std::lock_guard<std::mutex> lock(mutex_);
		float current_rpm = rpm_;
		if (current_rpm <= 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		// 计算步进间隔时间（us）
		int delay_us = static_cast<int>(60 * 1e6 / (current_rpm * steps_per_rev));
		DRV8825::TurnStep(motor_, direction_, 1, delay_us ); // 转一步
	}
}