#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>
#include <vector>

class Utils {
    public:
    static long currentTimeMs();
    static void delayMs(int x);
    static void trim(char *str);
    static std::vector<std::string> split(std::string s, std::string delimiter);
};

#endif
