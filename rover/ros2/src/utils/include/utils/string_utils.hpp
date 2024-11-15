/*! @package utils
    Code Information:
        Maintainer: Wilmer David Caceres Garzon
        Mail: wilmer.garzon@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/

#ifndef UTILS_STRING_H
#define UTILS_STRING_H

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <vector>

/*!
    Join a vector string in a single string
    @param str_vector, vector to join.
    @param c, character to put in the middle of each string.
 */
std::string str_join(std::vector<std::string> const& str_vector, char c = ' ');

/*!
    split a string in a vector of strings
    @param s string to split.
    @param c character to identify where to split.
*/
std::vector<std::string> str_split(std::string const& s, char c = ' ');

/*!
    Convert to upper case an string
*/

std::string str_toupper(std::string s);

/*!
    Replace a part of a string with a new fragment
    @param base I/O string to be changed
    @param find_str fragment to be replaced
    @param new_str fragment that will replace
*/
void replace(std::string& base, const std::string find_str, const std::string new_str);


#endif  // End if UTILS_STRING_H