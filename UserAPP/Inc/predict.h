#ifndef __PREDICT__H
#define __PREDICT__H

#include "opencv.hpp"
#include "yaml_config.h"
#include "paddle/include/paddle_inference_api.h"

/**
 * @brief 由.yaml文件创建predictor
 * @param yaml_config 
 * @return 
*/
std::shared_ptr<paddle_infer::Predictor> create_predictor(const YamlConfig& yaml_config);

/**
 * @brief 
 * @param predictor 
 * @param yaml_config 
 * @param src 
 * @param dst 
*/
void Run_Predict(std::shared_ptr<paddle_infer::Predictor> predictor, const YamlConfig& yaml_config,
                 cv::Mat& src, cv::Mat& dst);
#endif