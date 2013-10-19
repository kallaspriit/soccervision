#ifndef CONFIG_H
#define CONFIG_H

#include <string>

namespace Config {

	// camera serials
	const int frontCameraSerial = 857769553;
	const int rearCameraSerial = 857735761;

	// indexes of motors according to the communication messages
	const int wheelFLId = 0;
	const int wheelFRId = 1;
	const int wheelRLId = 2;
	const int wheelRRId = 3;
	const int dribblerId = 4;

	// ethernet communication host and port
	const std::string communicationHost = "192.168.4.1";
	const int communicationPort = 8042;

	// camera resolution
	const int cameraWidth = 1280;
	const int cameraHeight = 1024;
	const float cameraFovDistance = 5.0f;
	//const float cameraFovAngle = 56.0f * 3.14f / 180.0f;
	const float cameraFovAngle = 40.0f * 3.14f / 180.0f;
	const float cameraFovWidth = tan(cameraFovAngle / 2.0f) * cameraFovDistance * 2.0f;

	// default startup controller name
	const std::string defaultController = "test";

	// how big of a buffer to allocate for generating jpeg images
	const int jpegBufferSize = 5000 * 1024;

	// constants for camera correction
	//const float cameraCorrectionK = 0.00000049f;
	//const float cameraCorrectionZoom = 0.969f;
	const float cameraCorrectionK = -0.00000013f;
	const float cameraCorrectionZoom = 1.100f;

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
	const int goalTopMaxY = 30;

	// surround metric is taken into account if ball bottom is below this threshold
	const int surroundSenseThresholdY = cameraHeight - 50;

	// minimum object metric thresholds to be considered valid
	const float minValidBallSurroundThreshold = 0.5f;
	const float minValidBallPathThreshold = 0.75f;
	const float minValidGoalPathThreshold = 0.65f;

	// the ball/goal bottom needs to be below this line to consider path metric
	const int ballPathSenseStartY = cameraHeight - 80;
	const int goalPathSenseStartY = cameraHeight - 200;

	// color sense start Y
	const int colorDistanceStartY = cameraHeight - 160;

	// maximum ball surround metric sense radius
	const int maxBallSenseRadius = 250;

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
	const float robotWheelOffset = 0.1167f;

	// how much to substract from observed distance to calculate distance from dribbler
	const float robotDribblerDistance = 0.17f;

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
	const int robotDribblerSpeed = 100;
	
	// proportional multiplier for looking at object, multiplied by object angle
	const float lookAtP = 5.0f;

	// maximum omega to apply to look at an object
	const float lookAtMaxOmega = 3.0f * 3.14f;

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

	// how much should the centerline be inside the goal to kick ball
	const float goalKickThreshold = 0.25f;

	// particle filter robot localizer parameters
	const int robotLocalizerParticleCount = 1000;
	const float robotLocalizerForwardNoise = 0.25f;
	const float robotLocalizerTurnNoise = 0.3f; // 45deg
	const float robotLocalizerDistanceNoise = 0.35f;
	const float robotLocalizerAngleNoise = 0.2f; // ~~11deg

	// default kick strength in microseconds
	const int robotDefaultKickStrength = 10000;

	// minimum kick interval
	const double minKickInterval = 1.0;

	// maximum time object can be lost and still considered for updating its velocity
	const double velocityUpdateMaxTime = 0.025;

	// drag to apply to a rolling object
	const float rollingDrag = 0.2f;

	// object closer then this are considered to be the same object as observed before
	const float objectIdentityDistanceThreshold = 0.25f;

	// a localized ball will me marked for deletion after this amount of time of not being visible
	const double objectMarkForRemovalThreshold = 5.0;

	// object is purged from localization map if not seen for this amount of time
	const double objectPurgeLifetime = 10.0f;

	// maximum velocity of an object to be considered valid
	const float objectMaxVelocity = 8.0f;

	// how close to the field-of-view must the object be to be considered in view
	const float objectFovCloseEnough = 0.5f;

	// configuration filenames
	const std::string blobberConfigFilename = "config/blobber.cfg";
	const std::string frontDistanceLookupFilename = "config/distance-front.cfg";
	const std::string rearDistanceLookupFilename = "config/distance-rear.cfg";
	const std::string frontAngleLookupFilename = "config/angle-front.cfg";
	const std::string rearAngleLookupFilename = "config/angle-rear.cfg";
	const std::string screenshotsDirectory = "screenshots";

} // namespace Config

enum Side {
	BLUE = 0,
    YELLOW = 1,
	UNKNOWN = 2
};

enum Dir {
    FRONT = 1,
    REAR = 2,
	ANY = 3
};

enum Obstruction {
	NONE = 0,
	LEFT = 1,
	RIGHT = 2,
	BOTH = 3,
};

#endif // CONFIG_H
