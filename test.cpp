#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    // 确保在有图形界面的环境下运行
    setenv("DISPLAY", ":0", 1);

    cout << "黑线检测程序启动" << endl;

    // 1. 修正后的 GStreamer Pipeline
    // 注意：这里使用了 BGRx 格式，更兼容 OpenCV
    string pipeline =
        "libcamerasrc ! "
        "video/x-raw, width=1920, height=1080, framerate=30/1, format=BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=BGR ! "
        "appsink drop=true max-buffers=0 sync=false";

    VideoCapture cap(pipeline, CAP_GSTREAMER);
    if (!cap.isOpened()) {
        cerr << "错误：无法打开摄像头" << endl;
        return -1;
    }

    cout << "摄像头打开成功" << endl;

    // 定义形态学操作的核 (用于消除噪点)
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));

    while (true) {
        // 将 frame 定义在循环内部，确保每次都能正确读取
        UMat frame, gray, binary, morphed;

        // 读取一帧
        cap >> frame;
        if (frame.empty()) {
            cerr << "错误：无法获取帧" << endl;
            break;
        }

        // 1. 转换为灰度图
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // 2. 二值化 (处理反光噪点)
        // 使用自适应阈值，能更好地应对光线不均匀
        // 如果光线均匀，可以继续使用 threshold
        threshold(gray, binary, 120, 255, THRESH_BINARY_INV);

        // 3. 形态学开运算 (关键步骤：去除噪点)
        // 先腐蚀后膨胀，能有效去除小的噪点
        morphologyEx(binary, morphed, MORPH_OPEN, kernel);

        // 4. 找轮廓
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(morphed, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 5. 筛选并绘制黑线
        int best_idx = -1;
        double max_area = 0;
        Rect best_rect;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            Rect rect = boundingRect(contours[i]);
            double aspect = (double)rect.width / rect.height;

            // 筛选条件：面积、宽高比
            if (area > 300 && rect.width > 30 && rect.height > 10 && aspect > 2.0) {
                if (area > max_area) {
                    max_area = area;
                    best_idx = i;
                    best_rect = rect;
                }
            }
        }

        if (best_idx != -1) {
            drawContours(frame, contours, best_idx, Scalar(0, 0, 255), 3);
            rectangle(frame, best_rect, Scalar(255, 0, 0), 2);
            Point center(best_rect.x + best_rect.width / 2, best_rect.y + best_rect.height / 2);
            circle(frame, center, 5, Scalar(0, 255, 255), -1);
            int offset = center.x - 320; // 假设分辨率为 640x480，中心为 320
            cout << "黑线偏移: " << offset << endl;
        }

        // 6. 显示结果
        imshow("Camera", frame);
        imshow("Binary", morphed); // 显示形态学处理后的图像，方便调试

        if (waitKey(1) == 27) { // 按 ESC 键退出
            break;
        }
    }

    cap.release();
    destroyAllWindows();
    cout << "程序结束" << endl;

    return 0;
}