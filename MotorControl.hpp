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

#ifndef MotorControl_HPP_
#define MotorControl_HPP_

#include <thread>
#include <atomic>
#include<mutex>

#include "Debug.hpp"
#include "DEV_Config.hpp"
#include "DRV8825.hpp"
#include <functional>

/*------------------------------------------------------------------------------------------------------*/
class MotorControl {
public:
	MotorControl(UBYTE motor_id);    // Initial class
	~MotorControl();                            // close thread

	void start();                                      // open thread
	void stop();                                      // close thread and cut power
	void setRPM(float rpm);                   // change running rpm and direction
	//void SetDirection(UBYTE dir);       // not used now, change direction

    void setStatusCallback(const std::function<void(uint8_t,int)>& cb);  // Status callback: direction (0/1) and target RPM
    void setStepCallback(const std::function<void()>& cb);                   // Step callback: called after each step pulse

private:
	void Run();                      // run motor forever in thread

	DRV8825::Motor motor_;                  // Identify motor
	UBYTE motor_id_;
	std::mutex mutex_;
	std::thread control_thread_;
	std::atomic<bool> running_;
	std::atomic<float> rpm_;
	std::atomic<UBYTE> direction_;

	std::function<void(uint8_t,int)> statusCallback_;
    std::function<void()> stepCallback_;
};

#endif // MotorControl_HPP_
