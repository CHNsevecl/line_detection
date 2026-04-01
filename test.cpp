#include <iostream>
#include "uart.h"

using namespace std;

UART uart;

int main()
{
    uart.init("/dev/ttyAMA0", 115200); // 初始化 UART
    // 确保在有图形界面的环境下运行
    setenv("DISPLAY", ":0", 1);

    vector<string> info = {"Hello, MCU!", "PID Output: 123.45"};
    // 模拟发送数据到单片机，这里我们只是打印出来
    for (const auto& msg : info) {
        uart.send(msg + "\n"); // 发送数据到单片机，每条信息后加换行符
        cout << "发送到单片机: " << msg <<  endl;
        usleep(100000); // 模拟发送间隔，100ms
    }
    

    return 0;
}