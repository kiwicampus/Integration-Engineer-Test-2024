/*! @package utils
    File: console.hpp
*/

#include "utils/console.hpp"

std::string getEnv(const char* var_name, const char* default_value)
{
    const char* str_value = getenv(var_name);
    return std::string(str_value ? str_value : default_value);
}

std::vector<std::string> getEnv(const char* var, const std::initializer_list<const char*>& default_var)
{
    std::vector<std::string> string_vec;
    string_vec.assign(default_var.begin(), default_var.end());
    return getEnv(var, string_vec);
}

void splitString(const std::string& str, const char delimiter, std::vector<std::string>& substrings)
{
    std::string substring;
    for (const char c : str)
    {
        if (c == delimiter)
        {
            substrings.push_back(substring);
            substring.clear();
        }
        else
            substring += c;
    }
    substrings.push_back(substring);
}


extern std::chrono::nanoseconds headers2Dt(const std_msgs::msg::Header& header1, const std_msgs::msg::Header& header2)
{
    auto dt_ns = (rclcpp::Time(header1.stamp) - rclcpp::Time(header2.stamp)).nanoseconds();
    return std::chrono::nanoseconds(dt_ns);
}

extern float variance(const std::vector<float>& vec, int offset)
{
    float mean = 0, M2 = 0, variance = 0;

    size_t n = vec.size();
    for (size_t i = offset; i < n; ++i)
    {
        double delta = vec[i] - mean;
        mean += delta / (i + 1);
        M2 += delta * (vec[i] - mean);
        variance = M2 / (i + 1);
    }

    return variance;
}

extern float variance(const std::deque<float>& deque, int offset)
{
    float mean = 0, M2 = 0, variance = 0;

    auto iterator = deque.begin() + offset;
    for (size_t i = offset; iterator != deque.end(); ++i, iterator++)
    {
        double delta = *iterator - mean;
        mean += delta / (i + 1);
        M2 += delta * delta;
        variance = M2 / (i + 1);
    }

    return variance;
}


bool directoryExists(const char* path)
{
    struct stat info;
    return (stat(path, &info) == 0 && S_ISDIR(info.st_mode));
}

bool createDirectory(const char* path)
{
    if (directoryExists(path))
    {
        return true;
    }
    else
    {
        return mkdir(path, 0777) == 0;
    }
}

bool deleteFile(const char* path) { return std::remove(path) == 0; }
