#include <stdio.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/time.h>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstring>
#include "utils.h"

using namespace std; 

//@returns the current system time in milliseconds
long Utils::currentTimeMs() {
    struct timeval start;
    gettimeofday(&start, NULL);

    return (long)(start.tv_usec / 1000 + start.tv_sec * 1000);
}

//suspend execution for millisecond intervals
void Utils::delayMs(int x) {
    if (x > 0)
        usleep(x * 1000);
}

void Utils::trim(char *str) {
    int start = 0;
    while (isspace(str[start])) {
        start++;
    }
    
    int end = strlen(str) - 1;
    while (end > start && isspace(str[end])) {
        end--;
    }

    int j = 0;
    for (int i = start; i <= end; i++) {
        str[j++] = str[i];
    }
    str[j] = '\0';    
}

//for string delimiter
std::vector<std::string> Utils::split(std::string s, std::string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}
