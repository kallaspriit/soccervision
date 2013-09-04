#include "LookupTable.h"

#include <cstdio>
#include <iostream>

float LookupTable::getValue(float search) {
    float key;
    float lastKey = -1;
    float lastValue = -1;

    LookupMapIt finalIt = (map.end())--;

    for (LookupMapIt it = map.begin(); it != map.end(); it++) {
        key = it->first;

        if (key == search) {
            return it->second;
        }

        if (key >= search) {
            if (it == map.begin()) {
                if (it != finalIt) {
                    float v1 = it->second;
                    it++;
                    float v2 = it->second;
                    float diff = key - lastKey;
                    float share = (search - lastKey) / diff;
                    float result = share * v1 + v2 * (1.0f - share);

					/*std::cout << "------ 1 ---------" << std::endl;
					std::cout << "search: " << search << std::endl;
					std::cout << "v1: " << v1 << std::endl;
					std::cout << "v2: " << v2 << std::endl;
					std::cout << "key: " << key << std::endl;
					std::cout << "lastKey: " << lastKey << std::endl;
					std::cout << "diff: " << diff << std::endl;
					std::cout << "share: " << share << std::endl;
					std::cout << "result: " << result << std::endl;*/

					return result;
                } else {
                    return it->second;
                }
            }

            float v1 = it->second;
            float v2 = lastValue;
            float diff = key - lastKey;
            float share = (search - lastKey) / diff;
            float result = share * v1 + v2 * (1.0f - share);

            /*std::cout << "------ 2 ---------" << std::endl;
            std::cout << "search: " << search << std::endl;
            std::cout << "v1: " << v1 << std::endl;
            std::cout << "v2: " << v2 << std::endl;
            std::cout << "key: " << key << std::endl;
            std::cout << "lastKey: " << lastKey << std::endl;
            std::cout << "diff: " << diff << std::endl;
            std::cout << "share: " << share << std::endl;
            std::cout << "result: " << result << std::endl;*/

            return result;
        }

        lastKey = it->first;
        lastValue = it->second;
    }

    return lastValue;
}

bool LookupTable::load(std::string filename, float valueDiff) {
    FILE* in;

    in = fopen(filename.c_str(), "rt");

    if (!in) {
        return(false);
    }

    const int lineBufferSize = 256;
    char buf[lineBufferSize];
    float key;
    float value;

    while (fgets(buf, lineBufferSize, in)) {
        if (sscanf(buf, "%f %f", &key, &value)) {
            addValue(key, value + valueDiff);
        }
    };

    return true;
}
