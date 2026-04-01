#include <string>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

using namespace std;

class UART {
private:
    int fd;

public:
    // 构造函数初始化 fd
    UART() : fd(-1) {}

    // 1. 初始化串口
    bool init(const char* port, int baudrate) {
        // 打开设备
        // 注意：去掉了 O_NDELAY，保持和参考程序一致
        fd = open(port, O_RDWR | O_NOCTTY);
        if (fd == -1) {
            cout << "无法打开串口: " << port << endl;
            return false;
        }

        // 配置参数
        struct termios options;
        memset(&options, 0, sizeof(options));
        
        // 获取当前配置
        if (tcgetattr(fd, &options) != 0) {
            cout << "获取串口配置失败" << endl;
            return false;
        }

        // 设置波特率
        cfsetispeed(&options, baudrate);
        cfsetospeed(&options, baudrate);

        // 控制选项
        options.c_cflag |= (CLOCAL | CREAD); // 开启接收
        options.c_cflag &= ~PARENB;          // 无校验位
        options.c_cflag &= ~CSTOPB;          // 1位停止位
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;              // 8位数据位
        
        // 本地选项
        options.c_lflag &= ~(ICANON | ECHO); // 关闭规范模式和回显

        // 🔑 关键：设置超时 (复制参考程序的配置)
        options.c_cc[VMIN] = 0;    // 不等待固定字节数
        options.c_cc[VTIME] = 10;  // 1秒超时（单位0.1秒）

        // 应用配置
        tcsetattr(fd, TCSANOW, &options);
        
        cout << "串口 " << port << " 初始化成功！" << endl;
        return true;
    }

    // 2. 发送数据
    void send(const string& data) {
        if (fd != -1) {
            write(fd, data.c_str(), data.length());
        }
    }
    
    // 3. 关闭串口
    void close_port() {
        if (fd != -1) {
            ::close(fd);
            fd = -1;
        }
    }
};