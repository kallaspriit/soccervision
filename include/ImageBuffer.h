#ifndef IMAGEBUFFER_H
#define IMAGEBUFFER_H

#include <cstddef>
#include <string>

class ImageBuffer {
    public:
        ImageBuffer() : data(NULL), width(-1), height(-1), swapRB(false) { maxIndex = width * height * 3 - 2; }
        ImageBuffer(unsigned char* data, int width, int height) : data(data), width(width), height(height), swapRB(false) {}

        void setPixelAt(int x, int y, int red = 255, int green = 0, int blue = 0);
        void drawBox(int x, int y, int width, int height, int red = 255, int green = 0, int blue = 0);
        void drawBoxCentered(int x, int y, int width, int height, int red = 255, int green = 0, int blue = 0) {
            drawBox(x - width / 2, y - height / 2, width, height, red, green, blue);
        }
		void drawCircle(int centerX, int centerY, int radius, int red = 255, int green = 0, int blue = 0);
		void fillCircle(int centerX, int centerY, int radius, int red = 255, int green = 0, int blue = 0);
        void drawChar(int imageX, int imageY, int index);
        void drawText(int imageX, int imageY, std::string text);
        void drawLine(int x1, int y1, int x2, int y2, int red = 255, int green = 0, int blue = 0);
        void drawMarker(int x, int y, int red = 255, int green = 0, int blue = 0, bool tiny = false);

        unsigned char* data;
        int width;
        int height;
		bool swapRB;

    private:
        static const unsigned char font[256][8];
		int maxIndex;
};

#endif // IMAGEBUFFER_H
