#ifndef MotorControl_HPP_
#define MotorControl_HPP_

#include <thread>
#include <atomic>
#include "Debug.hpp"
#include "DEV_Config.hpp"
#include "DRV8825.hpp"

/*------------------------------------------------------------------------------------------------------*/
class MotorControl {
public:
	MotorControl(UBYTE motor_id);
	~MotorControl();    

	void start();                                  //启动线程
	void stop();                                  //停止线程
	void setRPM(float rpm);               //实时设置转速
	void SetDirection(UBYTE dir);     //设置方向
private:
	void Run();                      //线程函数，持续发出脉冲
	
	UBYTE motor_id_;
	std::thread control_thread_;
	std::atomic<bool> running_;
	std::atomic<float> rpm_;
	std::atomic<UBYTE> direction_;
};

#endif // MotorControl_HPP_