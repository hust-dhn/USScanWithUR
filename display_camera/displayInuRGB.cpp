/**
 * Inuitive NU4000 相机实时RGB图像显示程序
 * 编译命令: g++ -std=c++11 -o nu4000_display nu4000_display.cpp `pkg-config --cflags --libs opencv4` -I/path/to/inuitive/sdk -L/path/to/inuitive/lib -linuitive_core
 * 请将/path/to/inuitive/sdk和/path/to/inuitive/lib替换为实际的SDK路径
 */

#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

// OpenCV头文件
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// Inuitive SDK头文件（请根据实际路径调整）
#include "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/sample/example/example/CPP/include/inuCommon.h"
#include "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/sample/example/example/CPP/include/inuSensor.h"
#include "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/sample/example/example/CPP/include/inuParams.h"
#include "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/sample/example/example/CPP/include/inuDataPath.h"
#include "/home/zzz/ros2_ws/src/my_ur10e_control/SDK_CAMERA/sample/example/example/CPP/include/inuRGB.h"

using namespace Inuchip;
using namespace cv;

// 全局变量
std::atomic<bool> g_running{true};
std::mutex g_display_mutex;

/**
 * RGB图像显示线程函数
 * 负责从数据路径获取图像并实时显示
 */
void displayRGBThread(InuDataPath* data_path) {
    std::cout << "RGB显示线程启动..." << std::endl;
    
    while (g_running) {
        // 使用互斥锁保护对共享数据的访问
        std::unique_lock<std::mutex> lock(data_path->m_rgbFrameMutex);
        
        // 等待新的RGB帧到达（使用条件变量）
        // 注意：这里使用了超时等待，避免程序无法退出
        if (data_path->m_rgbCV.wait_for(lock, std::chrono::milliseconds(100)) == std::cv_status::timeout) {
            // 超时，继续循环
            continue;
        }
        
        // 获取最新的RGB帧
        RGBFrame rgb_frame = data_path->getLatestRGBFrame();
        lock.unlock();
        
        // 检查是否获取到有效帧数据
        if (!rgb_frame.frame_data || rgb_frame.datasize == 0) {
            continue;
        }
        
        // 计算数据偏移量（某些格式可能有头部信息）
        int offset = rgb_frame.totalsize - rgb_frame.datasize;
        int height = rgb_frame.height;
        int width = rgb_frame.width;
        
        cv::Mat img, display_img;
        
        try {
            // 根据图像格式进行相应的处理
            switch (rgb_frame.compressed) {
                case eRGB:
                    // RGB格式：需要转换为BGR用于OpenCV显示
                    img = cv::Mat(height, width, CV_8UC3, rgb_frame.frame_data + offset);
                    cv::cvtColor(img, display_img, cv::COLOR_RGB2BGR);
                    break;
                    
                case eBGR:
                    // BGR格式：OpenCV原生格式，直接使用
                    img = cv::Mat(height, width, CV_8UC3, rgb_frame.frame_data + offset);
                    display_img = img;
                    break;
                    
                case eBGRA:
                    // BGRA格式：需要移除Alpha通道
                    img = cv::Mat(height, width, CV_8UC4, rgb_frame.frame_data + offset);
                    cv::cvtColor(img, display_img, cv::COLOR_BGRA2BGR);
                    break;
                    
                case eRGBA:
                    // RGBA格式：需要转换为BGR
                    img = cv::Mat(height, width, CV_8UC4, rgb_frame.frame_data + offset);
                    cv::cvtColor(img, display_img, cv::COLOR_RGBA2BGR);
                    break;
                    
                case eYVU422:  // YUYV格式
                case eYUYV:    // YUYV格式别名
                    // YUYV格式：需要解码为BGR
                    img = cv::Mat(height, width, CV_8UC2, rgb_frame.frame_data + offset);
                    cv::cvtColor(img, display_img, cv::COLOR_YUV2BGR_YUYV);
                    break;
                    
                default:
                    std::cerr << "不支持的图像格式: " << rgb_frame.compressed << std::endl;
                    delete[] rgb_frame.frame_data;
                    continue;
            }
            
            // 显示图像
            cv::imshow("Inuitive NU4000 - RGB实时图像", display_img);
            
            // 等待1ms并检查按键
            // 按ESC键(27)或'q'键退出
            int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q') {
                std::cout << "检测到退出按键，程序将退出..." << std::endl;
                g_running = false;
            }
            
            // 释放帧数据内存
            delete[] rgb_frame.frame_data;
            
        } catch (const cv::Exception& e) {
            std::cerr << "OpenCV错误: " << e.what() << std::endl;
            if (rgb_frame.frame_data) {
                delete[] rgb_frame.frame_data;
            }
        } catch (const std::exception& e) {
            std::cerr << "处理图像时发生错误: " << e.what() << std::endl;
            if (rgb_frame.frame_data) {
                delete[] rgb_frame.frame_data;
            }
        }
    }
    
    std::cout << "RGB显示线程结束" << std::endl;
}

/**
 * 简化的主程序
 * 初始化相机并启动显示线程
 */
int main() {
    std::cout << "=== Inuitive NU4000 相机实时显示程序 ===" << std::endl;
    std::cout << "按 ESC 或 Q 键退出程序" << std::endl;
    
    try {
        // 1. 初始化参数管理器
        std::string config_file = "./Params.yaml"; // 配置文件路径
        if (config_file.empty()) {
            std::cout << "使用默认参数配置..." << std::endl;
        }
        
        InuParams inu_params(config_file);
        std::cout << "参数管理器初始化完成" << std::endl;
        
        // 2. 初始化数据路径
        InuDataPath inu_data_path;
        std::cout << "数据路径初始化完成" << std::endl;
        
        // 3. 初始化传感器
        InuSensor inu_sensor(&inu_params);
        std::cout << "传感器初始化完成" << std::endl;
        
        // 4. 检查设备状态
        std::cout << "检查设备连接状态..." << std::endl;
        int retry_count = 0;
        const int max_retries = 10;
        
        while (!inu_sensor.checkDeviceState() && retry_count < max_retries) {
            std::cout << "等待设备连接 (" << (retry_count + 1) << "/" << max_retries << ")..." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            retry_count++;
        }
        
        if (retry_count >= max_retries) {
            std::cerr << "错误: 无法检测到设备，请检查:" << std::endl;
            std::cerr << "  1. 相机是否正确连接" << std::endl;
            std::cerr << "  2. USB线是否正常" << std::endl;
            std::cerr << "  3. 是否有其他程序占用相机" << std::endl;
            return -1;
        }
        
        std::cout << "设备检测成功!" << std::endl;
        
        // 5. 设备初始化
        std::cout << "正在初始化设备..." << std::endl;
        inu_sensor.DeviceInit();
        std::cout << "设备初始化完成" << std::endl;
        
        // 6. 创建并启动RGB流
        std::cout << "启动RGB图像流..." << std::endl;
        InuRGB* inu_rgb = new InuRGB(&inu_sensor, &inu_data_path);
        inu_rgb->StartStream();
        std::cout << "RGB流已启动" << std::endl;
        
        // 7. 创建显示窗口
        cv::namedWindow("Inuitive NU4000 - RGB实时图像", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("Inuitive NU4000 - RGB实时图像", 100, 100);
        
        // 8. 启动显示线程
        std::thread display_thread(displayRGBThread, &inu_data_path);
        
        // 9. 主循环：等待退出信号
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // 10. 清理资源
        std::cout << "正在停止RGB流..." << std::endl;
        inu_rgb->StopStream();
        delete inu_rgb;
        
        std::cout << "正在终止设备连接..." << std::endl;
        inu_sensor.DeviceTerminate();
        
        // 等待显示线程结束
        if (display_thread.joinable()) {
            display_thread.join();
        }
        
        // 关闭OpenCV窗口
        cv::destroyAllWindows();
        
        std::cout << "程序正常退出" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "程序发生异常: " << e.what() << std::endl;
        return -1;
    }
    
    return 0;
}