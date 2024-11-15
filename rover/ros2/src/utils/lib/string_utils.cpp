/*! @package utils
    Code Information:
        Maintainer: Wilmer David Caceres Garzon
        Mail: wilmer.garzon@kiwibot.com
        Kiwi Campus / AI & Robotics Team
*/
#include "utils/string_utils.hpp"

std::string str_join(std::vector<std::string> const& str_vector, char c)
{
    std::string joined_str = "";
    for (auto const& str : str_vector)
    {
        joined_str += str;
        joined_str += c;
    }
    joined_str.pop_back();
    return joined_str;
}

std::vector<std::string> str_split(std::string const& s, char c)
{
    std::vector<std::string> splitted;
    size_t last_pos = 0;
    auto pos = s.find(c);

    std::string substr;

    while (pos != std::string::npos)
    {
        substr = s.substr(last_pos, pos - last_pos);
        splitted.push_back(substr);
        last_pos = pos + 1;
        pos = s.find(c, last_pos);
    }
    substr = s.substr(last_pos, pos - last_pos);
    splitted.push_back(substr);

    return splitted;
}

std::string str_toupper(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::toupper(c); }  // correct
    );
    return s;
}


void replace(std::string& base, const std::string find_str, const std::string new_str)
{
    size_t found = base.find(find_str);
    if (found == std::string::npos) return;

    base.replace(found, find_str.length(), new_str);
}
