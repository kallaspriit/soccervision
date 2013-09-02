#include "DisplayWindow.h"
#include "Gui.h"

#include <iostream>

DisplayWindow::DisplayWindow(HINSTANCE instance, int width, int height, std::string name, Gui* gui) : instance(instance), gui(gui), width(width), height(height), name(name), firstDraw(true) {
    hWnd = CreateWindowEx(
		NULL,
		"Window Class",
		name.c_str(),
		WS_OVERLAPPEDWINDOW,
		200,
		200,
		width + 16,
		height + 38,
		NULL,
		NULL,
		instance,
		(void*)this
	);

	if (!hWnd) {
		int nResult = GetLastError();

		MessageBox(NULL,
			"Window creation failed",
			"Window Creation Failed",
			MB_ICONERROR);
	}

	info.bmiHeader.biSize = sizeof(info.bmiHeader);
	info.bmiHeader.biWidth = width;
	info.bmiHeader.biHeight = height;
	info.bmiHeader.biPlanes = 1;
	info.bmiHeader.biBitCount = 24;
	info.bmiHeader.biCompression = BI_RGB;
	info.bmiHeader.biSizeImage = 0;
	info.bmiHeader.biClrUsed = 0;
	info.bmiHeader.biClrImportant = 0;

	hdc = GetDC(hWnd);
	cDC = CreateCompatibleDC(hdc);
	hBitmap = CreateCompatibleBitmap(hdc, width, height);
}

DisplayWindow::~DisplayWindow() {
	DeleteObject(SelectObject(cDC, hBitmap));
	DeleteDC(cDC);
	DeleteObject(hBitmap);
}

void DisplayWindow::setImage(unsigned char* image, bool rgb2bgr) {
	if (firstDraw) {
		ShowWindow(hWnd, SW_SHOWNORMAL);

		firstDraw = false;
	}

	/*if (!IsWindowVisible(hWnd)) {
		std::cout << "! Window not visible" << std::endl;

		return;
	}*/

	if (rgb2bgr) {
		// BGR to RGB..
		unsigned char blue;

		for (int i = 0; i < width * height * 3 - 3; i += 3) {
			blue = image[i];
			image[i] = image[i + 2];
			image[i + 2] = blue;
		}
	}

    SetDIBits (hdc, hBitmap, 0, height, image, &info, DIB_RGB_COLORS);
	hBitmap = (HBITMAP) SelectObject (cDC, hBitmap);
	//BitBlt (hdc, 0, 0, width, height, cDC, 0, 0, SRCCOPY);
	StretchBlt(hdc, 0, height, width, -height, cDC, 0, 0, width, height, SRCCOPY);

	//RedrawWindow(hWnd, NULL, NULL, RDW_INVALIDATE);
}

LRESULT DisplayWindow::handleMessage(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
	int x, y;

	switch(msg) {
		case WM_LBUTTONDOWN:
			if (gui != NULL) {
				x = (short)LOWORD(lParam);
				y = (short)HIWORD(lParam);

				//gui->onMouseClick(x, y);
			}
		break;

		default:
			return DefWindowProc(hWnd, msg, wParam, lParam);
		break;
	}

	return 0;
}