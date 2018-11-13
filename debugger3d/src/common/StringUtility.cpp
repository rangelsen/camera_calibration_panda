#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <stdlib.h>
#include <string.h>

#include "StringUtility.hpp"

std::vector<std::string> StringUtility::split(std::string input, char delim) {
    std::vector<std::string> output;
    
    std::stringstream ss(input);
    std::string element;

    while(std::getline(ss, element, delim)) {
        
        output.push_back(element);
    }

    return output;
}

char* StringUtility::get_directory(const char* src) {
    
    int i = strlen(src) - 1;

    while (src[i--] != '/') ;

    char* dest = (char*) malloc(sizeof(char) * i + 1);
    memcpy(dest, src, i + 2);
    return dest;
}

/// @file
