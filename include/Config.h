#ifndef CONFIG_H
#define CONFIG_H

#include <string>

namespace Config {

	// camera resolution
	const int cameraWidth = 1280;
	const int cameraHeight = 1024;

	// constants for camera correction
	const float cameraCorrectionK = 0.00000049f;
	const float cameraCorrectionZoom = 0.969f;

	// field dimensions
	const float fieldWidth = 4.5f;
	const float fieldHeight = 3.0f;

	// confinement factor in meters for confining objects on the field
	const float confineMargin = 0.0f;

	// how big is the ball/goal blobs distance to still be considered overlapping
	const int goalOverlapMargin = 20;
	const int ballOverlapMargin = 10;

	// minimum areas of blobs for objects
	const int ballBlobMinArea = 4;
	const int goalBlobMinArea = 16;

	// minimum area for objects to be considered valid
	const int ballMinArea = 4;
	const int goalMinArea = 64;

	// maximum width/height ratio for objects to be considered valid
	const float maxBallSizeRatio = 5.0f;

	// goals with area over this value are definately considered to be valid
	const int goalCertainArea = 10000;

	// if a goal starts lower than this value then it's not considered valid
	const int goalTopMaxY = 15;

	// surround metric is taken into account if ball bottom is below this threshold
	const int surroundSenseThresholdY = cameraHeight - 50;

	// minimum object metric thresholds to be considered valid
	const float minValidBallSurroundThreshold = 0.5f;
	const float minValidBallPathThreshold = 0.65f;

	// the ball's bottom needs to be below this line to consider path metric
	const int ballPathSenseThresholdY = cameraHeight - 80;

	// color sense start Y
	const int colorDistanceStartY = cameraHeight;

	// maximum ball surround metric sense radius
	const int maxBallSenseRadius = 100;

	// ball is considered to be in the goal if it's surrounded by goal colors by more than this
	const float ballInGoalSurroundThreshold = 0.5f;

	// whether ball is in the goal is considered only if it's closer than this
	const float ballInGoalConsiderMaxDistance = 1.0f;

	// additional linear distance correction to apply to object distances lookup
	const float distanceCorrection = 0.0f;

	// base number of steps used for underside metric
	const int undersideMetricBaseSteps = 20;

	// if underside metric is calculated below this point and no white nor black is seen, the metric returns zero
	const int undersideMetricBlackWhiteMinY = 100;

	// at least how many goal colors needed to be matched in underside metric
	const int undersideMetricValidGoalMinMatches = 10;

	// obstruction calculation parameters
	const int obstructionsStartY = 60;
	const int obstructionsSenseWidth = 120;
	const int obstructionsSenseHeight = 150;
	const float obstructedThreshold = 0.5f;

	// configuration filenames
	const std::string blobberConfigFilename = "config/blobber.cfg";
	const std::string frontDistanceLookupFilename = "config/distance-front.cfg";
	const std::string rearDistanceLookupFilename = "config/distance-rear.cfg";
	const std::string frontAngleLookupFilename = "config/angle-front.cfg";
	const std::string rearAngleLookupFilename = "config/angle-rear.cfg";

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
