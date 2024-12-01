#ifndef __YAML_CONFIG_H
#define __YAML_CONFIG_H

#include <iostream>
#include "yaml-cpp/yaml.h"

typedef struct 
{
  std::string model_file;
  std::string params_file;
  bool is_normalize;
  bool is_resize;
  int resize_width;
  int resize_height;
}YamlConfig;

/**
 * @brief 由描述模型结构的.yaml文件初始化YamlConfig结构体
 * @param yaml_path .yaml文件路径
 * @return YamlConfig结构体
*/
YamlConfig load_yaml(const std::string& yaml_path);

#endif
