#include "MotorControl.hpp"
#include <iostream>
#include <unistd.h> // usleep

MotorControl::MotorControl(UBYTE motor_id)
	: motor_id_(motor_id), running_(false), rpm_(0), direction_(DRV8825::FORWARD) {}

MotorControl::~MotorControl() {
	stop;
}

void MotorControl::start() {
	if (running_) return;
	running_ = true;

	DRV8825::SelectMotor(motor_id_);
	DRV8825::Enable;
	
	control_thread_ = std::thread(&MotorControl::Run, this);
}

void MotorControl::stop() {
	running_ = false;
	if (control_thread_.joinable()) {
		control_thread_.join();
	}
	DRV8825::Stop();
}

void MotorControl::setRPM(float rpm) {
	rpm_ = rpm;
}

void MotorControl::SetDirection(UBYTE dir) {
	direction_ = dir;
}

void MotorControl::Run() {
	constexpr int steps_per_rev = 800;    //每圈步数
	while (running_) {
		float current_rpm = rpm_;
		if (current_rpm <= 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
			continue;
		}

		// 计算步进间隔时间（us）
		int delay_us = static_cast<int>(60 * 1e6 / (current_rpm * steps_per_rev));
		DRV8825::TurnStep(direction_, 1, delay_us ); // 转一步
	}
}