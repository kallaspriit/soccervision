#ifndef CONFIG_H
#define CONFIG_H

namespace Config {

	const int cameraWidth = 1280;
	const int cameraHeight = 1024;
	const float cameraCorrectionK = 0.00000049f;
	const float cameraCorrectionZoom = 0.969f;

	const float fieldWidth = 4.5f;
	const float fieldHeight = 3.0f;
	const float confineMargin = 0.0f;

} // namespace Config

enum Side {
    YELLOW = 1,
    BLUE = 2,
	UNKNOWN = 3
};

enum Dir {
    FRONT = 1,
    REAR = 2
};

#endif // CONFIG_H
