#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include "robocup_localization/inference/inference_types.hpp"

namespace robocup_localization::inference
{

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
