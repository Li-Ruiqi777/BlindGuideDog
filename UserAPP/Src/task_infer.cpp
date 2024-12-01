#include "predict.h"
#include "visual.h"
#include "yaml_config.h"
#include "opencv.hpp"
#include <librealsense2/rs.hpp>
#include "task.h"

void Task_infer(RobotController &robot_a1, Custom &custom)
{
  mtx.lock();
  /* 定义相关变量 */
  Mat pred, added;
  double start, end, time;
  const int width = 640;
  const int height = 480;
  const int fps = 30;
  Mat H = calibration();

  /* 加载模型 */
  string yaml_path = "../Model/deploy.yaml";
  YamlConfig yaml_config = load_yaml(yaml_path);
  auto predictor = create_predictor(yaml_config);

  /* 配置Realsense */
  rs2::frameset frames;
  rs2::pipeline pipe;
  rs2::config cfg;
  cfg.disable_all_streams();
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);  // 深度流
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30); // RGB流
  pipe.start(cfg);                                                    // 指示管道使用所请求的配置启动流
  mtx.unlock();

  /* 配置VideoWriter */
  VideoWriter Color_Video;
  VideoWriter Depth_Video;

  while (1)
  {
    start = static_cast<double>(getTickCount());

    /* 1.预测 */
    frames = pipe.wait_for_frames();                         // 等待所有配置的流生成框架
    rs2::depth_frame depth_frame = frames.get_depth_frame(); // 获得深度图
    rs2::frame color_frame = frames.get_color_frame();
    Mat src(Size(width, height), CV_8UC3, (void *)color_frame.get_data(), Mat::AUTO_STEP);              // RGB图
    Mat depth_image(Size(width, height), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP); // 深度图

    rs2::colorizer color_map(0); // 声明深度着色器以实现深度数据的可视化
    rs2::frame presudo_frame = depth_frame.apply_filter(color_map);
    // rs2::frame presudo_frame = color_map.colorize(depth_frame);
    Mat depth_image_show(Size(width, height), CV_8UC3, (void*)presudo_frame.get_data(), cv::Mat::AUTO_STEP);
    cv::cvtColor(depth_image_show, depth_image_show, COLOR_RGB2BGR);

    // Mat depth_image_show;
    // depth_image.convertTo(depth_image_show, CV_8UC1, 255.0 / 1000);
    // cvtColor(depth_image_show, depth_image_show, COLOR_GRAY2RGB);
    Run_Predict(predictor, yaml_config, src, pred);

    /* 2.可视化 */
    added = addedImage(src, pred);

    /* 3.盲道图像后处理 */
    uint16_t thresh = 1000;
    if (robot_a1.mode != RobotController::Avoid)
    {
      Rect ROI(220, 80, 150, 150);
      // 只有stage_flag = 0时才会进入Avoid模式,通过将robota1构造函数初始值改成-1,即可失能避障功能
      robot_a1.Judge_Avoid(thresh, RobotController::Dir_Right, depth_image, ROI, custom);
      if (robot_a1.mode != RobotController::Avoid)
      {
        robot_a1.mode_choose1(pred, H);
      }
    }

    else if (robot_a1.mode == RobotController::Avoid)
    {
      Rect ROI(220, 80, 220, 150);
      robot_a1.mode_choose2(thresh, depth_image, ROI, custom);
    }

    /* 4.计算速度 */
    mtx.lock();
    robot_a1.calculate_speed();
    mtx.unlock();

    /* 5.录制视频相关 */
    if (!Color_Video.isOpened())
    {
      if (robot_a1.isVideo == true)
      {
        char buf[80];
        auto now = chrono::system_clock::now();
        time_t time = chrono::system_clock::to_time_t(now);
        strftime(buf, sizeof(buf), "%H_%M_%S", localtime(&time));
        string color_name = "../Video/color_" + string(buf) + ".mp4";
        string depth_name = "../Video/depth_" + string(buf) + ".mp4";
        cout << color_name << endl;
        cout << depth_name << endl;
        Color_Video.open(color_name, VideoWriter::fourcc('X', '2', '6', '4'), 8, Size(width, height));
        Depth_Video.open(depth_name, VideoWriter::fourcc('X', '2', '6', '4'), 8, Size(width, height));
        if (!Color_Video.isOpened())
        {
          cout << "fail to open!" << endl;
          return;
        }
      }
    }

    else if (Color_Video.isOpened())
    {
      if (robot_a1.isVideo == true)
      {
        Color_Video << src; // 保存当前帧
        Depth_Video << depth_image_show;
      }
      else
      {
        Color_Video.release();
        Depth_Video.release();
        cout << "Video Writer Released!" << endl;
      }
    }

    end = static_cast<double>(getTickCount());
    time = (end - start) / getTickFrequency();
    cout << "FPS " << (int)(1 / time) << endl;

    // imshow("pred", added);
    // waitKey(1);

    this_thread::sleep_for(chrono::milliseconds(10));
  }
}
