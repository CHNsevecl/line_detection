#ifndef PID_H
#define PID_H

class PID {
private:
    // PID 参数
    float _kp; // 比例系数
    float _ki; // 积分系数
    float _kd; // 微分系数

    // 内部状态变量
    float _integral;      // 积分累积值
    float _prev_error;    // 上一次的误差
    float _max_integral;  // 积分限幅（防止积分饱和）
    float _max_output;    // 输出限幅
    float _min_output;    // 输出下限

public:
    // 构造函数：初始化参数
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

    // 设置 PID 参数
    void setTunings(float kp, float ki, float kd);

    // 重置控制器（清除积分和历史误差）
    void reset();

    // 核心计算函数
    // input: 当前的误差 (例如：目标位置 - 当前位置)
    // return: 计算出的控制量 (例如：舵机角度修正值)
    float compute(float input);

    // 设置积分限幅 (可选)
    void setIntegralLimit(float limit);
    
    // 设置输出限幅 (可选)
    void setOutputLimit(float limit);
};

#endif