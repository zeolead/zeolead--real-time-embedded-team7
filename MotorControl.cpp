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
	DRV8825::Enable(motor_);
	//Debug::Log("motor %d, start\n",motor_id_);
	control_thread_ = std::thread(&MotorControl::Run, this);
}

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

void MotorControl::setRPM(float rpm) {
	std::lock_guard<std::mutex> lock(mutex_);
	if (rpm >= 0) {
		direction_ = DRV8825::FORWARD;
	}
	else {
		direction_ = DRV8825::BACKWARD;
		rpm = -rpm;  // �� rpm �����ֵ�����⸺������ delay_us ����
	}
	rpm_ = rpm;
	Debug::Log("set rpm %f  (dir %d) for motor %d\n", rpm, direction_.load(), motor_id_);
}

//void MotorControl::SetDirection(UBYTE dir) {
//	direction_ = dir;
//}

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
		if (current_rpm <= 0) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		int delay_us = static_cast<int>(60 * 1e5 / (current_rpm * steps_per_rev));
		// ����һ������
		DRV8825::TurnStep(motor_, dir, 1, delay_us); 
		// ������һ��Ӧ�÷�����ʱ���
		next_time += std::chrono::microseconds(delay_us);
		// ˯���Ǹ�ʱ�䣨��֤׼ȷ���ࣩ
		std::this_thread::sleep_until(next_time);
	}
}


//void MotorControl::Run() {
//	constexpr int steps_per_rev = 800;    //ÿȦ����
//	while (running_) {
//		std::lock_guard<std::mutex> lock(mutex_);
//		float current_rpm = rpm_;
//		if (current_rpm <= 0) {
//			std::this_thread::sleep_for(std::chrono::milliseconds(1));
//			continue;
//		}
//
//		// ���㲽�����ʱ�䣨us��
//		int delay_us = static_cast<int>(60 * 1e5 / (current_rpm * steps_per_rev));
//		DRV8825::TurnStep(motor_, direction_, 1, delay_us); // תһ��
//	}
//}