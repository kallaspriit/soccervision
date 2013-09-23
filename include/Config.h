#ifndef CONFIG_H
#define CONFIG_H

#include <string>

namespace Config {

	// camera serials
	const int frontCameraSerial = 857735761;
	const int rearCameraSerial = 857769553;

	// indexes of motors according to the communication messages
	const int wheelFLId = 3;
	const int wheelFRId = 1;
	const int wheelRLId = 4;
	const int wheelRRId = 0;
	const int dribblerId = 2;

	// ethernet communication host and port
	const std::string communicationHost = "192.168.4.1";
	const int communicationPort = 8042;

	// camera resolution
	const int cameraWidth = 1280;
	const int cameraHeight = 1024;

	// default startup controller name
	const std::string defaultController = "manual";

	// how big of a buffer to allocate for generating jpeg images
	const int jpegBufferSize = 5000 * 1024;

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
	const int colorDistanceStartY = cameraHeight - 160;

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

	// robot wheen angles
	const float robotWheelAngle1 = -135.0f;
	const float robotWheelAngle2 = -45.0f;
	const float robotWheelAngle3 = 45.0f;
	const float robotWheelAngle4 = 135.0f;

	// distance between two robot wheels diagonally
	const float robotWheelOffset = 0.1170f;

	// radius of a wheel
	const float robotWheelRadius = 0.034f;

	// in how many seconds to spin around the dribbler
	const float robotSpinAroundDribblerPeriod = 2.0f;
	
	// in how big of a radius to spin around the dribbler
	const float robotSpinAroundDribblerRadius = 0.1f;

	// how fast to drive forward while spinning around the dribbler
	const float robotSpinAroundDribblerForwardSpeed = 0.2f;

	// fluid movement steps
	const float robotfluidSpeedStep = 1.5f;
	const float robotfluidOmegaStep = 6.28f;

	// how fast to spin the dribbler
	const float robotDribblerNormalOmega = 3.14f;

	// for how many frames must the real wheel speed vary considerably from target speed to be considered stalled
	const int robotWheelStalledThreshold = 60;

	// how long the ball needs to be in the dribbler to be considered stable (seconds)
	const float ballInDribblerThreshold = 0.2f;

	// how long must the ball have not been detected to be considered lost (seconds)
	const float dribblerBallLostThreshold = 0.0f;

	// omega threshold for the robot to be considered not spinning any more
	const float rotationStoppedOmegaThreshold = 0.25f;

	// multiplier to act against robot spinning
	const float rotationCancelMultiplier = 0.3f;

	// multipler to jump robot by certain angle
	const float jumpAngleStopMultiplier = 1.0f;

	// particle filter robot localizer parameters
	const int robotLocalizerParticleCount = 1000;
	const float robotLocalizerForwardNoise = 0.25f;
	const float robotLocalizerTurnNoise = 0.3f; // 45deg
	const float robotLocalizerDistanceNoise = 0.35f;
	const float robotLocalizerAngleNoise = 0.2f; // ~~11deg

	// configuration filenames
	const std::string blobberConfigFilename = "config/blobber.cfg";
	const std::string frontDistanceLookupFilename = "config/distance-front.cfg";
	const std::string rearDistanceLookupFilename = "config/distance-rear.cfg";
	const std::string frontAngleLookupFilename = "config/angle-front.cfg";
	const std::string rearAngleLookupFilename = "config/angle-rear.cfg";

} // namespace Config

enum Side {
	BLUE = 0,
    YELLOW = 1,
	UNKNOWN = 2
};

enum Dir {
    FRONT = 1,
    REAR = 2
};

enum Obstruction {
	NONE = 0,
	LEFT = 1,
	RIGHT = 2,
	BOTH = 3,
};

#endif // CONFIG_H
