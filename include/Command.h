#ifndef COMMAND_H
#define COMMAND_H

#include <string>
#include <vector>

class Command {

public:
    Command(std::string name, std::vector<std::string> params) : name(name), params(params) {};

    static bool isValid(std::string input);
    static Command parse(std::string input);

    std::string name;
    std::vector<std::string> params;

};

#endif // COMMAND_H
