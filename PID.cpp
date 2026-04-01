#include "PID.h"

// 构造函数
PID::PID(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    
    _integral = 0.0f;
    _prev_error = 0.0f;
    _max_integral = 100.0f; // 默认积分限幅
    _max_output = 10000.0f;   // 默认输出限幅
    _min_output = 0.0f;   // 默认输出下限
}

// 设置参数
void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// 重置
void PID::reset() {
    _integral = 0.0f;
    _prev_error = 0.0f;
}

// 设置积分限幅
void PID::setIntegralLimit(float limit) {
    _max_integral = limit;
}

// 设置输出限幅
void PID::setOutputLimit(float limit) {
    _max_output = limit;
}

// 核心算法：计算控制量
float PID::compute(float input) {
    // 1. 比例项 (P)
    // 当前误差就是 input
    //if (input < 10) return 0; // 输入限幅，防止过大输入导致系统失控
    float p_term = _kp * input;

    // 2. 积分项 (I)
    _integral += input;
    // 积分限幅：防止积分过大导致系统失控（积分饱和）
    if (_integral > _max_integral) _integral = _max_integral;
    if (_integral < -_max_integral) _integral = -_max_integral;
    float i_term = _ki * _integral;

    // 3. 微分项 (D)
    // 微分 = 当前误差 - 上次误差
    float d_term = _kd * (input - _prev_error);

    // 保存当前误差供下次使用
    _prev_error = input;

    // 4. 计算总输出
    float output = p_term + i_term + d_term;

    // 输出限幅：确保输出值在合理范围内
    if (output > _max_output) output = _max_output;
    if (output < _min_output) output = _min_output;

    return output;
}