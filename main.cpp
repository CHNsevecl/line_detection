#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>
#include <sstream> 
#include <thread>
#include <iomanip>
#include "uart.h"
#include "PID.h"
#include <mutex>

using namespace std;
using namespace cv;
UART uart; // 假设单片机连接在 /dev/ttyAMA0，波特率为 115200
mutex uart_mutex;
bool uart_flag = true; // UART 初始化标志
bool About_to_turn_flag = false; // 即将转弯标志
bool direction = false; // false 右转，true 左转


void info_to_MCU(vector<vector<string>> info) {
    // 模拟发送数据到单片机，这里我们只是打印出来
    lock_guard<mutex> lock(uart_mutex);
    for(int i=0; i<info.size(); i++) {
        string messages = "";
        for (int j=0; j<info[i].size(); j++) {
            messages = messages + " " + info[i][j]; // 发送数据到单片机，每条信息后加换行符
        }
        messages += "\n";
        
        uart.send(messages);
        
        usleep(2500);//
    }
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
        "video/x-raw, format=NV12, width=640, height=480, framerate=90/1 ! "
        "appsink drop=true max-buffers=0 sync=false"
    };

    VideoCapture cap(pipeline,CAP_GSTREAMER);
    if (!cap.isOpened()){
        cerr << "错误：无法打开摄像头" << endl;
        return -1;
    }

    cout << "摄像头打开成功" << endl;

    Mat frame;
    Mat gray;
    //int sum_of_marked_points = 0; 
    int start_row = 0;
    int end_row = 480;
    int img_width = 640;
    float kp_now = 0.9;
    float kp_future = 0.1;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    vector<vector<string>> info(end_row/10/2, vector<string>(2)); // 存储 PID 输出和误差的字符串信息
    

    PID pid(60.0f, 0.01f, 1.0f); // 初始化 PID 控制器

    while (true){
        cap >> frame;
        vector<Point> center_points(end_row - start_row);// 存储每一行重心x的坐标  
        
     
        if(frame.empty()){
            std::cout << "抓捕失败" << endl;
            break;
        }
        cvtColor(frame,frame,COLOR_YUV2BGR_NV12);
        cvtColor(frame,gray,COLOR_BGR2GRAY);
        
        //sum_of_marked_points = 0; // 重置计数器
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
                //sum_of_marked_points++; // 计数器加1
                circle(frame, Point(center_x, y), 5, Scalar(0, 0, 255), -1); // 在原图上标记重心位置
            }
        }


    

        // ========== 3. 显示 ==========
        circle(frame, Point(320, 240), 5, Scalar(255, 0, 0), -1);
        imshow("Camera", frame);


        // =================拟合线计算=======================
        if (true) {
            int dx_now =0;
            for(int y = end_row-1;y >= end_row*3/4 + 10 ;y -=10){
                int dx = center_points[y].x - 320;
                dx_now += dx;
            }
            dx_now /= (end_row/10/4); // 取平均值

            int dx_future =0;
            for(int y = end_row*3/4;y >= end_row/2 ;y -=10){
                int dx = center_points[y].x - 320;
                dx_future += dx;
            }
            dx_future /= (end_row/10/4); // 取平均值

            float dx = static_cast<float>(kp_now * dx_now + kp_future * dx_future); // 综合当前和未来的偏移，得到最终的 PID 输入
            info[0][0] = to_string(static_cast<int>(dx)); // 存储误差
            info[0][1] = to_string(static_cast<int>(pid.compute(abs(dx)))); // 存储 PID 输出 

            cout << "当前偏移: " << dx_now << " 未来偏移: " << dx_future << " 综合偏移: " << dx << " PID 输出: " << pid.compute(abs(dx)) << endl;
        }
        //===================转弯判断=======================
        
        if(!About_to_turn_flag) { // 只有在不转弯状态下才进行转弯判断
            int count =0;
            int turn_x =0;
            for (int y = 0; y < end_row/2 ; y += 1) {
                if (center_points[y].y < 240 && (center_points[y].x > 480 || center_points[y].x < 160)) { // 右转
                    turn_x += center_points[y].x -320;
                    count++;
                }
            }
            turn_x /= count; // 取平均值
            cout<<"1."<<turn_x <<". "<<count<<endl;

            if ((turn_x < -200 || turn_x > 200) && count >5 && !About_to_turn_flag) { // 转弯条件：重心偏移大于200且满足数量条件，且当前不在转弯状态
                About_to_turn_flag = true;
                if (turn_x > 0) {
                    direction = false; // 右转
                } else {
                    direction = true; // 左转
                }
            } 
        }

        
        //===================转弯执行===================   
        if (About_to_turn_flag ) {//&& !turning_flag
            //turning_flag = true; // 设置转弯执行标志
            int count =0;
            int turn_x =0;
            for (int y = 240; y < end_row ; y += 1) {
                if ((center_points[y].x < 480 || center_points[y].x > 160)) { // 右转
                    turn_x += center_points[y].x -320;
                    //cout << "y: " << y << " x: " << center_points[y].x << endl;
                    count++;
                }
            }
            turn_x /= count; // 取平均值
            cout<<"2."<<turn_x <<". "<<count<<endl;
            
            if (turn_x == -320){
                if (direction){
                    info[0][0] = "TURN_LEFT"; // 存储转弯信息
                    cout << "左转" << endl;
                }
                else{
                    info[0][0] = "TURN_RIGHT"; // 存储转弯信息
                    cout << "右转" << endl;
                }
                
                
                
            }
        } 
        //==================转弯结束检测==================
        if ((info[0][0] == "TURN_LEFT" || info[0][0] == "TURN_RIGHT")) {
            int delta_x = abs(center_points[240].x - 320);
            if (delta_x < 10){
                info[0][1] = to_string(static_cast<int>(0)); // 存储转弯方向
                //turning_flag = false;
                About_to_turn_flag = false;
            }
            else{
                int delta_speed = delta_x *70;
                info[0][1] = to_string(static_cast<int>(delta_speed)); // 存储转弯方向
            }
            
        }

        //===================UART发送==================
        if(uart_flag) { 
            uart_flag = false; // 只初始化一次 UART
            thread send_thread(info_to_MCU, info);
            send_thread.detach(); // 分离线程，让它独立运行
        }
            

            if (waitKey(1) == 27) {
                break;
            }
    }

    
    cap.release();
    destroyAllWindows();
    
    cout << "程序结束" << endl;
    return 0;
}
