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
	MotorControl(UBYTE motor_id);
	~MotorControl();    

	void start();                                  //�����߳�
	void stop();                                  //ֹͣ�߳�
	void setRPM(float rpm);               //ʵʱ����ת��
	//void SetDirection(UBYTE dir);     //���÷���

    // Status callback: direction (0/1) and target RPM
    void setStatusCallback(const std::function<void(uint8_t,int)>& cb);
    // Step callback: called after each step pulse
    void setStepCallback(const std::function<void()>& cb);

private:
	void Run();                      //�̺߳�����������������

	DRV8825::Motor motor_;                  // ÿ��������ά���Լ��ĵ����������
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
