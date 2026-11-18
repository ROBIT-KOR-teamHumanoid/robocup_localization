#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "robocup_localization/inference/inference_types.hpp"

namespace robocup_localization::inference
{

namespace
{

bool readText(const std::string& path, std::string& text)
{
  std::ifstream file(path);
  if (!file)
  {
    return false;
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  text = buffer.str();
  return true;
}

// "key": <number> 의 첫 번째 값.
bool readNumber(const std::string& text, const std::string& key, double& value)
{
  std::size_t pos = text.find("\"" + key + "\"");
  if (pos == std::string::npos)
  {
    return false;
  }
  pos = text.find(':', pos + key.size() + 2);
  if (pos == std::string::npos)
  {
    return false;
  }
  const char* start = text.c_str() + pos + 1;
  char* end = nullptr;
  value = std::strtod(start, &end);
  return end != start;
}

}

bool readModelField(const std::string& model_path, double& field_a, double& field_b)
{
  std::string text;
  return readText(model_path + ".json", text)
      && readNumber(text, "A", field_a)
      && readNumber(text, "B", field_b);
}

// Only one key is needed, so no JSON library.
bool readFeatureMinGate(const std::string& model_path, int& feature_min)
{
  std::ifstream file(model_path + ".json");
  if (!file)
  {
    return false;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();

  const std::string key = "\"feature_min_gate\"";
  std::size_t pos = text.find(key);
  if (pos == std::string::npos)
  {
    return false;
  }
  pos = text.find(':', pos + key.size());
  if (pos == std::string::npos)
  {
    return false;
  }

  const char* start = text.c_str() + pos + 1;
  char* end = nullptr;
  const long value = std::strtol(start, &end, 10);
  if (end == start || value < 0)
  {
    return false;
  }

  feature_min = static_cast<int>(value);
  return true;
}

}
