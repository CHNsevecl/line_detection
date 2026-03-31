#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    setenv("DISPLAY",":0",1);
    cout << "黑线检测程序启动" << endl;
    
    // 1. 打开摄像头
    string  pipeline={
        "libcamerasrc camera-name=/base/axi/pcie@1000120000/rp1/i2c@88000/imx708@1a ! "
        "video/x-raw, format=NV12, width=640, height=480, framerate=30/1 ! "
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
    vector<Point> center_points;// 存储每一行重心x的坐标   

    while (true){
        cap >> frame;

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
                center_points.push_back(Point(center_x, y)); // 存储重心坐标
                circle(frame, Point(center_x, y), 5, Scalar(0, 0, 255), -1); // 在原图上标记重心位置
            }
        }


    

    // ========== 3. 显示 ==========
    imshow("Camera", frame);

        if (waitKey(1) == 27) {
            break;
        }
    }

    
    cap.release();
    destroyAllWindows();
    
    cout << "程序结束" << endl;
    return 0;
}