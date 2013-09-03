#include "Util.h"
//#include "Config.h"
//#include "jpge.h"

//#include <math.h>
#include <iostream>
//#include <sys/time.h>
#include <ctime>
//#include <stdlib.h>
#include <stdio.h>
#include <Windows.h>

double Util::queryPerformanceFrequency = 0;
__int64 Util::timerStartCount = 0;

void Util::yuyvToRgb(int width, int height, unsigned char *data, unsigned char *out) {
    int w2 = width / 2;

    for(int x=0; x<w2; x++) {
        for(int y=0; y<height; y++) {
            int i = (y*w2+x)*4;
            int y0 = data[i];
            int u = data[i+1];
            int y1 = data[i+2];
            int v = data[i+3];

            int r = (int)(y0 + (1.370705 * (v-128)));
            int g = (int)(y0 - (0.698001 * (v-128)) - (0.337633 * (u-128)));
            int b = (int)(y0 + (1.732446 * (u-128)));

            if(r > 255) r = 255;
            if(g > 255) g = 255;
            if(b > 255) b = 255;
            if(r < 0) r = 0;
            if(g < 0) g = 0;
            if(b < 0) b = 0;

            i = (y*width+2*x)*3;

            out[i] = (unsigned char)(r);
            out[i+1] = (unsigned char)(g);
            out[i+2] = (unsigned char)(b);

            r = (int)(y1 + (1.370705 * (v-128)));
            g = (int)(y1 - (0.698001 * (v-128)) - (0.337633 * (u-128)));
            b = (int)(y1 + (1.732446 * (u-128)));

            if(r > 255) r = 255;
            if(g > 255) g = 255;
            if(b > 255) b = 255;
            if(r < 0) r = 0;
            if(g < 0) g = 0;
            if(b < 0) b = 0;

            out[i+3] = (unsigned char)(r);
            out[i+4] = (unsigned char)(g);
            out[i+5] = (unsigned char)(b);
        }
    }
}

__int64 Util::timerStart() {
	LARGE_INTEGER li;
    QueryPerformanceFrequency(&li);

    queryPerformanceFrequency = double(li.QuadPart)/1000.0;

    QueryPerformanceCounter(&li);
    timerStartCount = li.QuadPart;

	return li.QuadPart;
}

double Util::timerEnd(__int64 startTime) {
	LARGE_INTEGER li;
    QueryPerformanceCounter(&li);

    return double(li.QuadPart - (startTime != -1 ? startTime : timerStartCount)) / queryPerformanceFrequency;
}

const std::string Util::base64Chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

std::string Util::base64Encode(const unsigned char* data, unsigned int length) {
    std::string ret;
    int i = 0;
    int j = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];

    while (length--) {
        char_array_3[i++] = *(data++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;

            for(i = 0; (i <4) ; i++)
                ret += base64Chars[char_array_4[i]];
            i = 0;
        }
    }

    if (i) {
        for(j = i; j < 3; j++)
            char_array_3[j] = '\0';

        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;

        for (j = 0; (j < i + 1); j++)
            ret += base64Chars[char_array_4[j]];

        while((i++ < 3))
            ret += '=';

    }

    return ret;
}

/*void Util::jpegEncode(const unsigned char* input, void* output, int &bufferSize, int width, int height, int channelCount) {
    jpge::compress_image_to_jpeg_file_in_memory(output, bufferSize, width, height, channelCount, input);
}*/

double Util::millitime() {
	//return (double)GetTickCount() / 1000.0;
	//return ((double)clock() / (double)CLOCKS_PER_SEC);
	return timeGetTime() / 1000.0;

	//return time(0) * 1000.0;

    /*timeval timeOfDay;

    gettimeofday(&timeOfDay, 0);

    long seconds  = timeOfDay.tv_sec;
    long useconds = timeOfDay.tv_usec;

    return (double)seconds + (double)useconds / 1000000.0d;*/

    //long mtime = (seconds * 1000 + useconds / 1000.0) + 0.5;

    //std::cout << "mtime:" << mtime << "\n";

    //return mtime / 1000;
}

double Util::duration(double start) {
    double diff = Util::millitime() - start;

    //std::cout << "diff:" << diff << "\n";

    return diff;
}

float Util::signum(float value) {
    if (value > 0) return 1;
    if (value < 0) return -1;

    return 0;
}

float Util::limit(float num, float min, float max) {
    if (num < min) {
        num = min;
    } else if (num > max) {
        num = max;
    }

    return num;
}

size_t Util::strpos(const std::string &haystack, const std::string &needle) {
    int inputLength = haystack.length();
    int needleLength = needle.length();

    if (inputLength == 0 || needleLength == 0) {
        return std::string::npos;
    }

    for (int i = 0, j = 0; i < inputLength; j = 0, i++) {
        while (i + j < inputLength && j < needleLength && haystack[i + j] == needle[j]) {
            j++;
        }

        if (j == needleLength) {
            return i;
        }
    }

    return std::string::npos;
}

bool Util::replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);

    if (start_pos == std::string::npos) {
        return false;
    }

    str.replace(start_pos, from.length(), to);

    return true;
}

/*std::string Util::exec(const std::string& cmd) {
    FILE* pipe = popen(cmd.c_str(), "r");

    if (!pipe) {
        return "ERROR";
    }

    char buffer[128];
    std::string result = "";

    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL) {
            result += buffer;
        }
    }

    pclose(pipe);

    return result;
}*/

/*std::string Util::getWorkingDirectory() {
    char stackBuffer[255];

    if (getcwd(stackBuffer, sizeof(stackBuffer)) != NULL) {
        return stackBuffer;
    } else {
        return "ERROR";
    }
}
*/

/*void Util::correctCameraPoint(int& x, int& y) {
	float k = Config::cameraCorrectionK;
	float zoom = Config::cameraCorrectionZoom;
	float centerX = Config::cameraWidth / 2.0f - 0.5f;
	float centerY = Config::cameraHeight / 2.0f - 0.5f;
	float centerOffsetX = x - centerX;
	float centerOffsetY = y - centerY;
	float krd2 = k * (centerOffsetX * centerOffsetX + centerOffsetY * centerOffsetY);
	float magnifacationFactor = 1.0f / (1.0f + krd2);

	x = zoom * centerOffsetX / magnifacationFactor + centerX,
	y = zoom * centerOffsetY / magnifacationFactor + centerY;
}

void Util::confineField(float& x, float& y) {
	if (x < -Config::confineMargin) {
		x = -Config::confineMargin;
	} else if (x > Config::fieldWidth + Config::confineMargin) {
		x = Config::fieldWidth + Config::confineMargin;
	}

	if (y < -Config::confineMargin) {
		y = -Config::confineMargin;
	} else if (y > Config::fieldHeight + Config::confineMargin) {
		y = Config::fieldHeight + Config::confineMargin;
	}
}*/