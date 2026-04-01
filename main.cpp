#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>
#include <thread>
#include "uart.h"
#include "PID.h"
#include <mutex>

using namespace std;
using namespace cv;
UART uart; // 假设单片机连接在 /dev/ttyAMA0，波特率为 115200
mutex uart_mutex;
bool uart_flag = true; // UART 初始化标志

void info_to_MCU(vector<string> info) {
    // 模拟发送数据到单片机，这里我们只是打印出来
    lock_guard<mutex> lock(uart_mutex);
    string messages = "";
    for(int i=0; i<info.size(); i++) {
        messages = messages + " " + info[i]; // 发送数据到单片机，每条信息后加换行符
    }
    messages += "\n";
    uart.send(messages);
    usleep(50000); // 模拟发送间隔，100ms
    uart_flag = true; // 发送完成后重置标志
}

int main()
{   
    uart.init("/dev/ttyAMA0", 115200); // 初始化 UART
    setenv("DISPLAY",":0",1);
    cout << "黑线检测程序启动" << endl;
    
    // 1. 打开摄像头
    string  pipeline={
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
        "video/x-raw, format=NV12, width=640, height=480, framerate=60/1 ! "
        "appsink drop=true max-buffers=0 sync=false"
    };

    VideoCapture cap(pipeline,CAP_GSTREAMER);
    if (!cap.isOpened()){
        cerr << "错误：无法打开摄像头" << endl;
        return -1;
    }

    cout << "摄像头打开成功" << endl;

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat frame;
    Mat gray;
    int start_row = 240;
    int end_row = 480;
    int img_width = 640;

    PID pid(40.0f, 0.01f, 0.02f); // 初始化 PID 控制器

    while (true){
        cap >> frame;
        vector<Point> center_points(240);// 存储每一行重心x的坐标  
        vector<string> info(2);
        if(frame.empty()){
            std::cout << "抓捕失败" << endl;
            break;
        }
        cvtColor(frame,frame,COLOR_YUV2BGR_NV12);
        cvtColor(frame,gray,COLOR_BGR2GRAY);
        
        for (int y = start_row; y < end_row; y++) {
            // 获取当前行的指针
            // ptr<uchar>(y) 直接指向第 y 行的内存地址，速度最快
            uchar* row_ptr = gray.ptr<uchar>(y);

            double sum_weight = 0;  // 分母：总权重
            double sum_x_weight = 0; // 分子：位置 * 权重

            for (int x = 0; x < img_width; x++) {
                uchar pixel_value = row_ptr[x]; // 获取当前像素值
                double weight = 0; 
                if (pixel_value < 120) { // 权重：越暗的像素权重越大
                    weight = 255 - pixel_value;
                }
                sum_weight += weight;
                sum_x_weight += x * weight; // 位置 * 权重
            }

            if (sum_weight > 0) {
                int center_x = static_cast<int>((sum_x_weight / sum_weight)+0.5); // 计算重心x坐标,z暂时取整数
                center_points[y - start_row] = Point(center_x, y); // 存储重心坐标
                circle(frame, Point(center_x, y), 5, Scalar(0, 0, 255), -1); // 在原图上标记重心位置
            }
        }


    

        // ========== 3. 显示 ==========
        circle(frame, Point(320, 260), 5, Scalar(255, 0, 0), -1);
        imshow("Camera", frame);

        int error_code = center_points[200].x - 320; // 以中心点偏移为输入
        int error_output = pid.compute(abs(error_code)); // 计算 PID 输出
        
        info[0] = to_string(error_code);
        info[1] = to_string(error_output);

        if(uart_flag) { 
            uart_flag = false; // 只初始化一次 UART
            thread send_thread(info_to_MCU, info);
            send_thread.detach(); // 分离线程，让它独立运行
        }
            

        //cout << "误差: " << info[0] << endl;
        //cout << "d PID输出: " << info[1] << endl; // 计算 PID 输出（以中心点偏移为输入）

            if (waitKey(1) == 27) {
                break;
            }
        }

    
    cap.release();
    destroyAllWindows();
    
    cout << "程序结束" << endl;
    return 0;
}