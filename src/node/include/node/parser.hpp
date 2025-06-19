#ifndef _PARSER_
#define _PARSER_

#include <vector>
#include <string>

#include "std_msgs/msg/string.hpp"

#include "include.hpp"

class Parser
{
public:
    int encode(const message& input, message& output);
    int decode(const message& input, message& output);
};

#endif