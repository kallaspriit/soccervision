#include "Command.h"

#include "Util.h"

#include <iostream>

bool Command::isValid(std::string input) {
    return input.substr(0, 1) == "<" && input.substr(input.length() - 1, 1) == ">";
}

Command Command::parse(std::string input) {
    std::string name;
    std::vector<std::string> params;
    std::string body = input.substr(1).substr(0, input.length() - 2);

    size_t colonPos = Util::strpos(body, ":");

    if (colonPos == std::string::npos) {
        name = body;
    } else {
        name = body.substr(0, colonPos);

        std::string paramsBody = body.substr(colonPos + 1);

        size_t commaPos;
        std::string param;

        while (true) {
            commaPos = Util::strpos(paramsBody, ",");

            if (commaPos == std::string::npos) {
                break;
            }

            param = paramsBody.substr(0, commaPos);
            paramsBody = paramsBody.substr(commaPos + 1);

            params.push_back(param);
        }

        params.push_back(paramsBody);
    }

    return Command(name, params);
}
