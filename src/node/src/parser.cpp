#include "node/parser.hpp"

#include <cassert>
#include <string>

using std::string;

int Parser::encode(const message &input, message &output)
{
    // 帧头加帧尾
    string raw = input.data;

    string value;
    value.push_back(FH_LO);
    value.push_back(FH_HI);
    value.insert(value.end(), raw.begin(), raw.end());
    value.push_back(FT_LO);
    value.push_back(FT_HI);
    output.data = value;

    return 0;
}

int Parser::decode(const message &input, message &output)
{
    /*int recstatus = 0;
    for (int i = 0; i < input.size(); i++)
    {
        switch (recstatus)
        {
        case 0:
            if (input[i] == FH_LO)
                recstatus++;
            break;
        
        case 1:
            if (input[i] == FH_HI)
                recstatus++;
            else
                recstatus--;
            break;
        
        case 2:
            if (input[i] == FT_LO)
                recstatus++;
            

            break;

        case 3:

        default:
            break;
        }
    }*/

    string raw = input.data;
    assert(raw[0] == FH_LO);
    assert(raw[1] == FH_HI);
    assert(raw[raw.size() - 2] == FT_LO);
    assert(raw[raw.size() - 1] == FT_HI);

    string value;
    value.insert(value.end(), raw.begin() + 2, raw.end() - 2);
    output.data = value;
    
    return 0;
}
