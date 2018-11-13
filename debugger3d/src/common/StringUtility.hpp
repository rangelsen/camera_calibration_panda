#ifndef STRING_UTILITY_HPP
#define STRING_UTILITY_HPP

#include <string>
#include <vector>

namespace StringUtility{
    
    std::vector<std::string> split(std::string input, char delim);
	char* get_directory(const char* src);
}

#endif // STRING_UTILITY_HPP

/// @file
