/*! @package utils
    File: console.hpp
*/

#ifndef UTILS_CONSOLE_H
#define UTILS_CONSOLE_H
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <exception>
#include <fstream>
#include <memory>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>
#include "rclcpp/time.hpp"
#include "std_msgs/msg/header.hpp"

#include "utils/string_utils.hpp"



/* Env. Variables Functions */
/*
    Function to cast a string to any numeric value
*/
template <typename T>
T cast_char_str(const char* str_value)
{
    std::stringstream convert(str_value);
    T value;
    convert >> value;
    return value;
}

/*!
    Wrapper around getenv function that accepts default values.
    @param var_name Environment variable
    @param default_value Default value if env variable not defined
    @return environment value.
*/
template <typename T>
T getEnv(const char* var_name, const T default_value)
{
    const char* str_value = getenv(var_name);
    return str_value ? cast_char_str<T>(str_value) : default_value;
}

std::string getEnv(const char* var_name, const char* default_value);

/*!
    @brief Get a vector of strings from an environment variable
    @param var Environment variable
    @param default_var Default value if env variable not defined
    @return vector of strings with the environment value.
 */
template <typename T>
std::vector<T> getEnv(const char* var, const std::vector<T>& default_var)
{
    const char* str_value = getenv(var);
    if (!str_value) return default_var;

    std::vector<std::string> vec = str_split(str_value, ',');
    if (typeid(T) == typeid(std::string)) return reinterpret_cast<std::vector<T>&>(vec);
    std::vector<T> value;
    std::transform(vec.begin(), vec.end(), std::back_inserter(value),
                   [](std::string item) { return cast_char_str<T>(item.c_str()); });
    return value;
}

template <typename T>
std::vector<T> getEnv(const char* var, const std::initializer_list<T>& default_var)
{
    return getEnv(var, std::vector<T>(default_var));
}

std::vector<std::string> getEnv(const char* var, const std::initializer_list<const char*>& default_var);

/* Header operations Functions */
extern std::chrono::nanoseconds headers2Dt(const std_msgs::msg::Header& header1, const std_msgs::msg::Header& header2);

/* Statistical functions */
extern float variance(const std::vector<float>& vec, int offset);
extern float variance(const std::deque<float>& deque, int offset);

/*  directory management*/
bool directoryExists(const char* path);
bool createDirectory(const char* path);
bool deleteFile(const char* path);

#endif /* End of UTILS_CONSOLE_H */