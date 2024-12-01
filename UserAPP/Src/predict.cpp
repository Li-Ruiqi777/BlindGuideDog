#include "predict.h"
#include "visual.h"
#include <numeric>
#include "iostream"
#include "stdint.h"
std::shared_ptr<paddle_infer::Predictor> create_predictor(const YamlConfig& yaml_config)
{
  paddle_infer::Config infer_config;
  infer_config.SetModel(yaml_config.model_file, yaml_config.params_file);
  infer_config.EnableMemoryOptim();

  /*CPU*/
  //infer_config.EnableMKLDNN();
  //infer_config.SetCpuMathLibraryNumThreads(5);

  /*GPU*/
  infer_config.EnableUseGpu(100, 0);

  auto predictor = paddle_infer::CreatePredictor(infer_config);
  return predictor;
}

void Run_Predict(std::shared_ptr<paddle_infer::Predictor> predictor, const YamlConfig& yaml_config,
                 cv::Mat& src, cv::Mat& dst)
{
  // Prepare data
  cv::Mat img = Image_Preprocess(src,yaml_config);
  int rows = img.rows;
  int cols = img.cols;
  int chs = img.channels();
  std::vector<float> input_data(1 * chs * rows * cols, 0.0f);
  hwc_img_2_chw_data(img, input_data.data());

  // Set input
  auto input_names = predictor->GetInputNames();
  auto input_t = predictor->GetInputHandle(input_names[0]);
  std::vector<int> input_shape = { 1, chs, rows, cols };
  input_t->Reshape(input_shape); //送入神经网络的数据必须是[N,C,H,W]
  input_t->CopyFromCpu(input_data.data());

  //std::cout << "Input Type is :" << input_t->type() << std::endl;
  // Run
  predictor->Run();

  // Get output
  auto output_names = predictor->GetOutputNames();
  auto output_t = predictor->GetOutputHandle(output_names[0]);
  std::vector<int> output_shape = output_t->shape();
  //int out_num = std::accumulate(output_shape.begin(), output_shape.end(), 1,
  //              std::multiplies<int>());//统计输出张量中元素的个数,与deploy的resize大小有关(后期可以删掉这句话)
  //std::cout << out_num << std::endl;
  int out_num = 307200; //640*480时的个数
  std::vector<int64_t> out_data(out_num);//输出的数据类型和模型有关
  output_t->CopyToCpu(out_data.data());

  // Get pseudo image
  std::vector<uint8_t> out_data_u8(out_num);
  for (int i = 0; i < out_num; i++)
  {
    out_data_u8[i] = static_cast<uint8_t>(out_data[i]);//将数据类型转换成uint8_t
  }
  //好好学习Mat的这种构造方法,这就实现了由Tensor->vector->Mat的转换
  cv::Mat pred(output_shape[1], output_shape[2], CV_8UC1, out_data_u8.data());//语义分割的单通道图
  pred.copyTo(dst);
  //cv::equalizeHist(pred, dst);//直方图均衡化(仍是单通道),使分割结果能直观的看见

}