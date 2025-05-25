// Auto-generated MAVLink enums
// DO NOT EDIT MANUALLY

/// SERIAL_CONTROL flags (bitmask)
pub const SERIAL_CONTROL_FLAG = enum(u8) {
    /// Set if this is a reply
    SERIAL_CONTROL_FLAG_REPLY = 1,
    /// Set if the sender wants the receiver to send a response as another SERIAL_CONTROL message
    SERIAL_CONTROL_FLAG_RESPOND = 2,
    /// Set if access to the serial port should be removed from whatever driver is currently using it, giving exclusive access to the SERIAL_CONTROL protocol. The port can be handed back by sending a request without this flag set
    SERIAL_CONTROL_FLAG_EXCLUSIVE = 4,
    /// Block on writes to the serial port
    SERIAL_CONTROL_FLAG_BLOCKING = 8,
    /// Send multiple replies until port is drained
    SERIAL_CONTROL_FLAG_MULTI = 16,
};

/// List of possible units where failures can be injected.
pub const FAILURE_UNIT = enum(u32) {
    FAILURE_UNIT_SENSOR_GYRO = 0,
    FAILURE_UNIT_SENSOR_ACCEL = 1,
    FAILURE_UNIT_SENSOR_MAG = 2,
    FAILURE_UNIT_SENSOR_BARO = 3,
    FAILURE_UNIT_SENSOR_GPS = 4,
    FAILURE_UNIT_SENSOR_OPTICAL_FLOW = 5,
    FAILURE_UNIT_SENSOR_VIO = 6,
    FAILURE_UNIT_SENSOR_DISTANCE_SENSOR = 7,
    FAILURE_UNIT_SENSOR_AIRSPEED = 8,
    FAILURE_UNIT_SYSTEM_BATTERY = 100,
    FAILURE_UNIT_SYSTEM_MOTOR = 101,
    FAILURE_UNIT_SYSTEM_SERVO = 102,
    FAILURE_UNIT_SYSTEM_AVOIDANCE = 103,
    FAILURE_UNIT_SYSTEM_RC_SIGNAL = 104,
    FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL = 105,
};

/// These flags encode the MAV mode.
pub const MAV_MODE_FLAG = enum(u8) {
    /// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
    MAV_MODE_FLAG_SAFETY_ARMED = 128,
    /// 0b01000000 remote control input is enabled.
    MAV_MODE_FLAG_MANUAL_INPUT_ENABLED = 64,
    /// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
    MAV_MODE_FLAG_HIL_ENABLED = 32,
    /// 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
    MAV_MODE_FLAG_STABILIZE_ENABLED = 16,
    /// 0b00001000 guided mode enabled, system flies waypoints / mission items.
    MAV_MODE_FLAG_GUIDED_ENABLED = 8,
    /// 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
    MAV_MODE_FLAG_AUTO_ENABLED = 4,
    /// 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
    MAV_MODE_FLAG_TEST_ENABLED = 2,
    /// 0b00000001 Reserved for future use.
    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1,
};

/// Axes that will be autotuned by MAV_CMD_DO_AUTOTUNE_ENABLE.
///         Note that at least one flag must be set in MAV_CMD_DO_AUTOTUNE_ENABLE.param2: if none are set, the flight stack will tune its default set of axes.
pub const AUTOTUNE_AXIS = enum(u32) {
    /// Autotune roll axis.
    AUTOTUNE_AXIS_ROLL = 1,
    /// Autotune pitch axis.
    AUTOTUNE_AXIS_PITCH = 2,
    /// Autotune yaw axis.
    AUTOTUNE_AXIS_YAW = 4,
};

/// Flags for CURRENT_EVENT_SEQUENCE.
pub const MAV_EVENT_CURRENT_SEQUENCE_FLAGS = enum(u32) {
    /// A sequence reset has happened (e.g. vehicle reboot).
    MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET = 1,
};

/// Result from a MAVLink command (MAV_CMD)
pub const MAV_RESULT = enum(u8) {
    /// Command is valid (is supported and has valid parameters), and was executed.
    MAV_RESULT_ACCEPTED = 0,
    /// Command is valid, but cannot be executed at this time. This is used to indicate a problem that should be fixed just by waiting (e.g. a state machine is busy, can't arm because have not got GPS lock, etc.). Retrying later should work.
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    /// Command is invalid (is supported but has invalid parameters). Retrying same command and parameters will not work.
    MAV_RESULT_DENIED = 2,
    /// Command is not supported (unknown).
    MAV_RESULT_UNSUPPORTED = 3,
    /// Command is valid, but execution has failed. This is used to indicate any non-temporary or unexpected problem, i.e. any problem that must be fixed before the command can succeed/be retried. For example, attempting to write a file when out of memory, attempting to arm when sensors are not calibrated, etc.
    MAV_RESULT_FAILED = 4,
    /// Command is valid and is being executed. This will be followed by further progress updates, i.e. the component may send further COMMAND_ACK messages with result MAV_RESULT_IN_PROGRESS (at a rate decided by the implementation), and must terminate by sending a COMMAND_ACK message with final result of the operation. The COMMAND_ACK.progress field can be used to indicate the progress of the operation.
    MAV_RESULT_IN_PROGRESS = 5,
    /// Command has been cancelled (as a result of receiving a COMMAND_CANCEL message).
    MAV_RESULT_CANCELLED = 6,
    /// Command is only accepted when sent as a COMMAND_LONG.
    MAV_RESULT_COMMAND_LONG_ONLY = 7,
    /// Command is only accepted when sent as a COMMAND_INT.
    MAV_RESULT_COMMAND_INT_ONLY = 8,
    /// Command is invalid because a frame is required and the specified frame is not supported.
    MAV_RESULT_COMMAND_UNSUPPORTED_MAV_FRAME = 9,
};

/// Power supply status flags (bitmask)
pub const MAV_POWER_STATUS = enum(u16) {
    /// main brick power supply valid
    MAV_POWER_STATUS_BRICK_VALID = 1,
    /// main servo power supply valid for FMU
    MAV_POWER_STATUS_SERVO_VALID = 2,
    /// USB power is connected
    MAV_POWER_STATUS_USB_CONNECTED = 4,
    /// peripheral supply is in over-current state
    MAV_POWER_STATUS_PERIPH_OVERCURRENT = 8,
    /// hi-power peripheral supply is in over-current state
    MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT = 16,
    /// Power status has changed since boot
    MAV_POWER_STATUS_CHANGED = 32,
};

/// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
pub const MAV_MODE_FLAG_DECODE_POSITION = enum(u32) {
    /// First bit:  10000000
    MAV_MODE_FLAG_DECODE_POSITION_SAFETY = 128,
    /// Second bit: 01000000
    MAV_MODE_FLAG_DECODE_POSITION_MANUAL = 64,
    /// Third bit:  00100000
    MAV_MODE_FLAG_DECODE_POSITION_HIL = 32,
    /// Fourth bit: 00010000
    MAV_MODE_FLAG_DECODE_POSITION_STABILIZE = 16,
    /// Fifth bit:  00001000
    MAV_MODE_FLAG_DECODE_POSITION_GUIDED = 8,
    /// Sixth bit:   00000100
    MAV_MODE_FLAG_DECODE_POSITION_AUTO = 4,
    /// Seventh bit: 00000010
    MAV_MODE_FLAG_DECODE_POSITION_TEST = 2,
    /// Eighth bit: 00000001
    MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE = 1,
};

/// Enumeration of battery functions
pub const MAV_BATTERY_FUNCTION = enum(u8) {
    /// Battery function is unknown
    MAV_BATTERY_FUNCTION_UNKNOWN = 0,
    /// Battery supports all flight systems
    MAV_BATTERY_FUNCTION_ALL = 1,
    /// Battery for the propulsion system
    MAV_BATTERY_FUNCTION_PROPULSION = 2,
    /// Avionics battery
    MAV_BATTERY_FUNCTION_AVIONICS = 3,
    /// Payload battery
    MAV_BATTERY_FUNCTION_PAYLOAD = 4,
};

/// Type of GPS fix
pub const GPS_FIX_TYPE = enum(u8) {
    /// No GPS connected
    GPS_FIX_TYPE_NO_GPS = 0,
    /// No position information, GPS is connected
    GPS_FIX_TYPE_NO_FIX = 1,
    /// 2D position
    GPS_FIX_TYPE_2D_FIX = 2,
    /// 3D position
    GPS_FIX_TYPE_3D_FIX = 3,
    /// DGPS/SBAS aided 3D position
    GPS_FIX_TYPE_DGPS = 4,
    /// RTK float, 3D position
    GPS_FIX_TYPE_RTK_FLOAT = 5,
    /// RTK Fixed, 3D position
    GPS_FIX_TYPE_RTK_FIXED = 6,
    /// Static fixed, typically used for base stations
    GPS_FIX_TYPE_STATIC = 7,
    /// PPP, 3D position.
    GPS_FIX_TYPE_PPP = 8,
};

/// Gimbal device (low level) capability flags (bitmap).
pub const GIMBAL_DEVICE_CAP_FLAGS = enum(u16) {
    /// Gimbal device supports a retracted position.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT = 1,
    /// Gimbal device supports a horizontal, forward looking position, stabilized.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL = 2,
    /// Gimbal device supports rotating around roll axis.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS = 4,
    /// Gimbal device supports to follow a roll angle relative to the vehicle.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW = 8,
    /// Gimbal device supports locking to a roll angle (generally that's the default with roll stabilized).
    GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK = 16,
    /// Gimbal device supports rotating around pitch axis.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS = 32,
    /// Gimbal device supports to follow a pitch angle relative to the vehicle.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW = 64,
    /// Gimbal device supports locking to a pitch angle (generally that's the default with pitch stabilized).
    GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK = 128,
    /// Gimbal device supports rotating around yaw axis.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS = 256,
    /// Gimbal device supports to follow a yaw angle relative to the vehicle (generally that's the default).
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW = 512,
    /// Gimbal device supports locking to an absolute heading, i.e., yaw angle relative to North (earth frame, often this is an option available).
    GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK = 1024,
    /// Gimbal device supports yawing/panning infinitely (e.g. using slip disk).
    GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048,
    /// Gimbal device supports yaw angles and angular velocities relative to North (earth frame). This usually requires support by an autopilot via AUTOPILOT_STATE_FOR_GIMBAL_DEVICE. Support can go on and off during runtime, which is reported by the flag GIMBAL_DEVICE_FLAGS_CAN_ACCEPT_YAW_IN_EARTH_FRAME.
    GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME = 4096,
    /// Gimbal device supports radio control inputs as an alternative input for controlling the gimbal orientation.
    GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS = 8192,
};

/// Type of mission items being requested/sent in mission protocol.
pub const MAV_MISSION_TYPE = enum(u8) {
    /// Items are mission commands for main mission.
    MAV_MISSION_TYPE_MISSION = 0,
    /// Specifies GeoFence area(s). Items are MAV_CMD_NAV_FENCE_ GeoFence items.
    MAV_MISSION_TYPE_FENCE = 1,
    /// Specifies the rally points for the vehicle. Rally points are alternative RTL points. Items are MAV_CMD_NAV_RALLY_POINT rally point items.
    MAV_MISSION_TYPE_RALLY = 2,
    /// Only used in MISSION_CLEAR_ALL to clear all mission types.
    MAV_MISSION_TYPE_ALL = 255,
};

pub const MAV_ODID_ID_TYPE = enum(u8) {
    /// No type defined.
    MAV_ODID_ID_TYPE_NONE = 0,
    /// Manufacturer Serial Number (ANSI/CTA-2063 format).
    MAV_ODID_ID_TYPE_SERIAL_NUMBER = 1,
    /// CAA (Civil Aviation Authority) registered ID. Format: [ICAO Country Code].[CAA Assigned ID].
    MAV_ODID_ID_TYPE_CAA_REGISTRATION_ID = 2,
    /// UTM (Unmanned Traffic Management) assigned UUID (RFC4122).
    MAV_ODID_ID_TYPE_UTM_ASSIGNED_UUID = 3,
    /// A 20 byte ID for a specific flight/session. The exact ID type is indicated by the first byte of uas_id and these type values are managed by ICAO.
    MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID = 4,
};

/// Flags in ESTIMATOR_STATUS message
pub const ESTIMATOR_STATUS_FLAGS = enum(u16) {
    /// True if the attitude estimate is good
    ESTIMATOR_ATTITUDE = 1,
    /// True if the horizontal velocity estimate is good
    ESTIMATOR_VELOCITY_HORIZ = 2,
    /// True if the  vertical velocity estimate is good
    ESTIMATOR_VELOCITY_VERT = 4,
    /// True if the horizontal position (relative) estimate is good
    ESTIMATOR_POS_HORIZ_REL = 8,
    /// True if the horizontal position (absolute) estimate is good
    ESTIMATOR_POS_HORIZ_ABS = 16,
    /// True if the vertical position (absolute) estimate is good
    ESTIMATOR_POS_VERT_ABS = 32,
    /// True if the vertical position (above ground) estimate is good
    ESTIMATOR_POS_VERT_AGL = 64,
    /// True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
    ESTIMATOR_CONST_POS_MODE = 128,
    /// True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
    ESTIMATOR_PRED_POS_HORIZ_REL = 256,
    /// True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
    ESTIMATOR_PRED_POS_HORIZ_ABS = 512,
    /// True if the EKF has detected a GPS glitch
    ESTIMATOR_GPS_GLITCH = 1024,
    /// True if the EKF has detected bad accelerometer data
    ESTIMATOR_ACCEL_ERROR = 2048,
};

pub const MAV_ODID_OPERATOR_LOCATION_TYPE = enum(u8) {
    /// The location/altitude of the operator is the same as the take-off location.
    MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF = 0,
    /// The location/altitude of the operator is dynamic. E.g. based on live GNSS data.
    MAV_ODID_OPERATOR_LOCATION_TYPE_LIVE_GNSS = 1,
    /// The location/altitude of the operator are fixed values.
    MAV_ODID_OPERATOR_LOCATION_TYPE_FIXED = 2,
};

/// Enumeration of sensor orientation, according to its rotations
pub const MAV_SENSOR_ORIENTATION = enum(u8) {
    /// Roll: 0, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_NONE = 0,
    /// Roll: 0, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_YAW_45 = 1,
    /// Roll: 0, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_YAW_90 = 2,
    /// Roll: 0, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_YAW_135 = 3,
    /// Roll: 0, Pitch: 0, Yaw: 180
    MAV_SENSOR_ROTATION_YAW_180 = 4,
    /// Roll: 0, Pitch: 0, Yaw: 225
    MAV_SENSOR_ROTATION_YAW_225 = 5,
    /// Roll: 0, Pitch: 0, Yaw: 270
    MAV_SENSOR_ROTATION_YAW_270 = 6,
    /// Roll: 0, Pitch: 0, Yaw: 315
    MAV_SENSOR_ROTATION_YAW_315 = 7,
    /// Roll: 180, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_180 = 8,
    /// Roll: 180, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_ROLL_180_YAW_45 = 9,
    /// Roll: 180, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_180_YAW_90 = 10,
    /// Roll: 180, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_ROLL_180_YAW_135 = 11,
    /// Roll: 0, Pitch: 180, Yaw: 0
    MAV_SENSOR_ROTATION_PITCH_180 = 12,
    /// Roll: 180, Pitch: 0, Yaw: 225
    MAV_SENSOR_ROTATION_ROLL_180_YAW_225 = 13,
    /// Roll: 180, Pitch: 0, Yaw: 270
    MAV_SENSOR_ROTATION_ROLL_180_YAW_270 = 14,
    /// Roll: 180, Pitch: 0, Yaw: 315
    MAV_SENSOR_ROTATION_ROLL_180_YAW_315 = 15,
    /// Roll: 90, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90 = 16,
    /// Roll: 90, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_ROLL_90_YAW_45 = 17,
    /// Roll: 90, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_90_YAW_90 = 18,
    /// Roll: 90, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_ROLL_90_YAW_135 = 19,
    /// Roll: 270, Pitch: 0, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270 = 20,
    /// Roll: 270, Pitch: 0, Yaw: 45
    MAV_SENSOR_ROTATION_ROLL_270_YAW_45 = 21,
    /// Roll: 270, Pitch: 0, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_270_YAW_90 = 22,
    /// Roll: 270, Pitch: 0, Yaw: 135
    MAV_SENSOR_ROTATION_ROLL_270_YAW_135 = 23,
    /// Roll: 0, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_PITCH_90 = 24,
    /// Roll: 0, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_PITCH_270 = 25,
    /// Roll: 0, Pitch: 180, Yaw: 90
    MAV_SENSOR_ROTATION_PITCH_180_YAW_90 = 26,
    /// Roll: 0, Pitch: 180, Yaw: 270
    MAV_SENSOR_ROTATION_PITCH_180_YAW_270 = 27,
    /// Roll: 90, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 = 28,
    /// Roll: 180, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 = 29,
    /// Roll: 270, Pitch: 90, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 = 30,
    /// Roll: 90, Pitch: 180, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 = 31,
    /// Roll: 270, Pitch: 180, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 = 32,
    /// Roll: 90, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 = 33,
    /// Roll: 180, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 = 34,
    /// Roll: 270, Pitch: 270, Yaw: 0
    MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 = 35,
    /// Roll: 90, Pitch: 180, Yaw: 90
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 = 36,
    /// Roll: 90, Pitch: 0, Yaw: 270
    MAV_SENSOR_ROTATION_ROLL_90_YAW_270 = 37,
    /// Roll: 90, Pitch: 68, Yaw: 293
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 = 38,
    /// Pitch: 315
    MAV_SENSOR_ROTATION_PITCH_315 = 39,
    /// Roll: 90, Pitch: 315
    MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 = 40,
    /// Custom orientation
    MAV_SENSOR_ROTATION_CUSTOM = 100,
};

pub const MAV_ODID_HEIGHT_REF = enum(u8) {
    /// The height field is relative to the take-off location.
    MAV_ODID_HEIGHT_REF_OVER_TAKEOFF = 0,
    /// The height field is relative to ground.
    MAV_ODID_HEIGHT_REF_OVER_GROUND = 1,
};

pub const MAV_ODID_UA_TYPE = enum(u8) {
    /// No UA (Unmanned Aircraft) type defined.
    MAV_ODID_UA_TYPE_NONE = 0,
    /// Aeroplane/Airplane. Fixed wing.
    MAV_ODID_UA_TYPE_AEROPLANE = 1,
    /// Helicopter or multirotor.
    MAV_ODID_UA_TYPE_HELICOPTER_OR_MULTIROTOR = 2,
    /// Gyroplane.
    MAV_ODID_UA_TYPE_GYROPLANE = 3,
    /// VTOL (Vertical Take-Off and Landing). Fixed wing aircraft that can take off vertically.
    MAV_ODID_UA_TYPE_HYBRID_LIFT = 4,
    /// Ornithopter.
    MAV_ODID_UA_TYPE_ORNITHOPTER = 5,
    /// Glider.
    MAV_ODID_UA_TYPE_GLIDER = 6,
    /// Kite.
    MAV_ODID_UA_TYPE_KITE = 7,
    /// Free Balloon.
    MAV_ODID_UA_TYPE_FREE_BALLOON = 8,
    /// Captive Balloon.
    MAV_ODID_UA_TYPE_CAPTIVE_BALLOON = 9,
    /// Airship. E.g. a blimp.
    MAV_ODID_UA_TYPE_AIRSHIP = 10,
    /// Free Fall/Parachute (unpowered).
    MAV_ODID_UA_TYPE_FREE_FALL_PARACHUTE = 11,
    /// Rocket.
    MAV_ODID_UA_TYPE_ROCKET = 12,
    /// Tethered powered aircraft.
    MAV_ODID_UA_TYPE_TETHERED_POWERED_AIRCRAFT = 13,
    /// Ground Obstacle.
    MAV_ODID_UA_TYPE_GROUND_OBSTACLE = 14,
    /// Other type of aircraft not listed earlier.
    MAV_ODID_UA_TYPE_OTHER = 15,
};

/// Precision land modes (used in MAV_CMD_NAV_LAND).
pub const PRECISION_LAND_MODE = enum(u32) {
    /// Normal (non-precision) landing.
    PRECISION_LAND_MODE_DISABLED = 0,
    /// Use precision landing if beacon detected when land command accepted, otherwise land normally.
    PRECISION_LAND_MODE_OPPORTUNISTIC = 1,
    /// Use precision landing, searching for beacon if not found when land command accepted (land normally if beacon cannot be found).
    PRECISION_LAND_MODE_REQUIRED = 2,
};

pub const MAG_CAL_STATUS = enum(u8) {
    MAG_CAL_NOT_STARTED = 0,
    MAG_CAL_WAITING_TO_START = 1,
    MAG_CAL_RUNNING_STEP_ONE = 2,
    MAG_CAL_RUNNING_STEP_TWO = 3,
    MAG_CAL_SUCCESS = 4,
    MAG_CAL_FAILED = 5,
    MAG_CAL_BAD_ORIENTATION = 6,
    MAG_CAL_BAD_RADIUS = 7,
};

pub const MAV_ODID_CLASS_EU = enum(u8) {
    /// The class for the UA, according to the EU specification, is undeclared.
    MAV_ODID_CLASS_EU_UNDECLARED = 0,
    /// The class for the UA, according to the EU specification, is Class 0.
    MAV_ODID_CLASS_EU_CLASS_0 = 1,
    /// The class for the UA, according to the EU specification, is Class 1.
    MAV_ODID_CLASS_EU_CLASS_1 = 2,
    /// The class for the UA, according to the EU specification, is Class 2.
    MAV_ODID_CLASS_EU_CLASS_2 = 3,
    /// The class for the UA, according to the EU specification, is Class 3.
    MAV_ODID_CLASS_EU_CLASS_3 = 4,
    /// The class for the UA, according to the EU specification, is Class 4.
    MAV_ODID_CLASS_EU_CLASS_4 = 5,
    /// The class for the UA, according to the EU specification, is Class 5.
    MAV_ODID_CLASS_EU_CLASS_5 = 6,
    /// The class for the UA, according to the EU specification, is Class 6.
    MAV_ODID_CLASS_EU_CLASS_6 = 7,
};

/// Fence types to enable or disable when using MAV_CMD_DO_FENCE_ENABLE.
///         Note that at least one of these flags must be set in MAV_CMD_DO_FENCE_ENABLE.param2.
///         If none are set, the flight stack will ignore the field and enable/disable its default set of fences (usually all of them).
pub const FENCE_TYPE = enum(u32) {
    /// Maximum altitude fence
    FENCE_TYPE_ALT_MAX = 1,
    /// Circle fence
    FENCE_TYPE_CIRCLE = 2,
    /// Polygon fence
    FENCE_TYPE_POLYGON = 4,
    /// Minimum altitude fence
    FENCE_TYPE_ALT_MIN = 8,
};

/// Possible responses from a CELLULAR_CONFIG message.
pub const CELLULAR_CONFIG_RESPONSE = enum(u8) {
    /// Changes accepted.
    CELLULAR_CONFIG_RESPONSE_ACCEPTED = 0,
    /// Invalid APN.
    CELLULAR_CONFIG_RESPONSE_APN_ERROR = 1,
    /// Invalid PIN.
    CELLULAR_CONFIG_RESPONSE_PIN_ERROR = 2,
    /// Changes rejected.
    CELLULAR_CONFIG_RESPONSE_REJECTED = 3,
    /// PUK is required to unblock SIM card.
    CELLULAR_CONFIG_BLOCKED_PUK_REQUIRED = 4,
};

/// Type of landing target
pub const LANDING_TARGET_TYPE = enum(u8) {
    /// Landing target signaled by light beacon (ex: IR-LOCK)
    LANDING_TARGET_TYPE_LIGHT_BEACON = 0,
    /// Landing target signaled by radio beacon (ex: ILS, NDB)
    LANDING_TARGET_TYPE_RADIO_BEACON = 1,
    /// Landing target represented by a fiducial marker (ex: ARTag)
    LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2,
    /// Landing target represented by a pre-defined visual shape/feature (ex: X-marker, H-marker, square)
    LANDING_TARGET_TYPE_VISION_OTHER = 3,
};

pub const MAV_ARM_AUTH_DENIED_REASON = enum(u32) {
    /// Not a specific reason
    MAV_ARM_AUTH_DENIED_REASON_GENERIC = 0,
    /// Authorizer will send the error as string to GCS
    MAV_ARM_AUTH_DENIED_REASON_NONE = 1,
    /// At least one waypoint have a invalid value
    MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2,
    /// Timeout in the authorizer process(in case it depends on network)
    MAV_ARM_AUTH_DENIED_REASON_TIMEOUT = 3,
    /// Airspace of the mission in use by another vehicle, second result parameter can have the waypoint id that caused it to be denied.
    MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4,
    /// Weather is not good to fly
    MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5,
};

/// These flags are used in the AIS_VESSEL.fields bitmask to indicate validity of data in the other message fields. When set, the data is valid.
pub const AIS_FLAGS = enum(u16) {
    /// 1 = Position accuracy less than 10m, 0 = position accuracy greater than 10m.
    AIS_FLAGS_POSITION_ACCURACY = 1,
    AIS_FLAGS_VALID_COG = 2,
    AIS_FLAGS_VALID_VELOCITY = 4,
    /// 1 = Velocity over 52.5765m/s (102.2 knots)
    AIS_FLAGS_HIGH_VELOCITY = 8,
    AIS_FLAGS_VALID_TURN_RATE = 16,
    /// Only the sign of the returned turn rate value is valid, either greater than 5deg/30s or less than -5deg/30s
    AIS_FLAGS_TURN_RATE_SIGN_ONLY = 32,
    AIS_FLAGS_VALID_DIMENSIONS = 64,
    /// Distance to bow is larger than 511m
    AIS_FLAGS_LARGE_BOW_DIMENSION = 128,
    /// Distance to stern is larger than 511m
    AIS_FLAGS_LARGE_STERN_DIMENSION = 256,
    /// Distance to port side is larger than 63m
    AIS_FLAGS_LARGE_PORT_DIMENSION = 512,
    /// Distance to starboard side is larger than 63m
    AIS_FLAGS_LARGE_STARBOARD_DIMENSION = 1024,
    AIS_FLAGS_VALID_CALLSIGN = 2048,
    AIS_FLAGS_VALID_NAME = 4096,
};

/// Winch status flags used in WINCH_STATUS
pub const MAV_WINCH_STATUS_FLAG = enum(u32) {
    /// Winch is healthy
    MAV_WINCH_STATUS_HEALTHY = 1,
    /// Winch line is fully retracted
    MAV_WINCH_STATUS_FULLY_RETRACTED = 2,
    /// Winch motor is moving
    MAV_WINCH_STATUS_MOVING = 4,
    /// Winch clutch is engaged allowing motor to move freely.
    MAV_WINCH_STATUS_CLUTCH_ENGAGED = 8,
    /// Winch is locked by locking mechanism.
    MAV_WINCH_STATUS_LOCKED = 16,
    /// Winch is gravity dropping payload.
    MAV_WINCH_STATUS_DROPPING = 32,
    /// Winch is arresting payload descent.
    MAV_WINCH_STATUS_ARRESTING = 64,
    /// Winch is using torque measurements to sense the ground.
    MAV_WINCH_STATUS_GROUND_SENSE = 128,
    /// Winch is returning to the fully retracted position.
    MAV_WINCH_STATUS_RETRACTING = 256,
    /// Winch is redelivering the payload. This is a failover state if the line tension goes above a threshold during RETRACTING.
    MAV_WINCH_STATUS_REDELIVER = 512,
    /// Winch is abandoning the line and possibly payload. Winch unspools the entire calculated line length. This is a failover state from REDELIVER if the number of attempts exceeds a threshold.
    MAV_WINCH_STATUS_ABANDON_LINE = 1024,
    /// Winch is engaging the locking mechanism.
    MAV_WINCH_STATUS_LOCKING = 2048,
    /// Winch is spooling on line.
    MAV_WINCH_STATUS_LOAD_LINE = 4096,
    /// Winch is loading a payload.
    MAV_WINCH_STATUS_LOAD_PAYLOAD = 8192,
};

/// Actions that may be specified in MAV_CMD_OVERRIDE_GOTO to override mission execution.
pub const MAV_GOTO = enum(u32) {
    /// Hold at the current position.
    MAV_GOTO_DO_HOLD = 0,
    /// Continue with the next item in mission execution.
    MAV_GOTO_DO_CONTINUE = 1,
    /// Hold at the current position of the system
    MAV_GOTO_HOLD_AT_CURRENT_POSITION = 2,
    /// Hold at the position specified in the parameters of the DO_HOLD action
    MAV_GOTO_HOLD_AT_SPECIFIED_POSITION = 3,
};

pub const MAV_ODID_CLASSIFICATION_TYPE = enum(u8) {
    /// The classification type for the UA is undeclared.
    MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED = 0,
    /// The classification type for the UA follows EU (European Union) specifications.
    MAV_ODID_CLASSIFICATION_TYPE_EU = 1,
};

/// Supported component metadata types. These are used in the "general" metadata file returned by COMPONENT_METADATA to provide information about supported metadata types. The types are not used directly in MAVLink messages.
pub const COMP_METADATA_TYPE = enum(u32) {
    /// General information about the component. General metadata includes information about other metadata types supported by the component. Files of this type must be supported, and must be downloadable from vehicle using a MAVLink FTP URI.
    COMP_METADATA_TYPE_GENERAL = 0,
    /// Parameter meta data.
    COMP_METADATA_TYPE_PARAMETER = 1,
    /// Meta data that specifies which commands and command parameters the vehicle supports. (WIP)
    COMP_METADATA_TYPE_COMMANDS = 2,
    /// Meta data that specifies external non-MAVLink peripherals.
    COMP_METADATA_TYPE_PERIPHERALS = 3,
    /// Meta data for the events interface.
    COMP_METADATA_TYPE_EVENTS = 4,
    /// Meta data for actuator configuration (motors, servos and vehicle geometry) and testing.
    COMP_METADATA_TYPE_ACTUATORS = 5,
};

/// These defines are predefined OR-combined mode flags. There is no need to use values from this enum, but it
///                simplifies the use of the mode flags. Note that manual input is enabled in all modes as a safety override.
pub const MAV_MODE = enum(u8) {
    /// System is not ready to fly, booting, calibrating, etc. No flag is set.
    MAV_MODE_PREFLIGHT = 0,
    /// System is allowed to be active, under assisted RC control.
    MAV_MODE_STABILIZE_DISARMED = 80,
    /// System is allowed to be active, under assisted RC control.
    MAV_MODE_STABILIZE_ARMED = 208,
    /// System is allowed to be active, under manual (RC) control, no stabilization
    MAV_MODE_MANUAL_DISARMED = 64,
    /// System is allowed to be active, under manual (RC) control, no stabilization
    MAV_MODE_MANUAL_ARMED = 192,
    /// System is allowed to be active, under autonomous control, manual setpoint
    MAV_MODE_GUIDED_DISARMED = 88,
    /// System is allowed to be active, under autonomous control, manual setpoint
    MAV_MODE_GUIDED_ARMED = 216,
    /// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
    MAV_MODE_AUTO_DISARMED = 92,
    /// System is allowed to be active, under autonomous control and navigation (the trajectory is decided onboard and not pre-programmed by waypoints)
    MAV_MODE_AUTO_ARMED = 220,
    /// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
    MAV_MODE_TEST_DISARMED = 66,
    /// UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only.
    MAV_MODE_TEST_ARMED = 194,
};

/// Generalized UAVCAN node health
pub const UAVCAN_NODE_HEALTH = enum(u8) {
    /// The node is functioning properly.
    UAVCAN_NODE_HEALTH_OK = 0,
    /// A critical parameter went out of range or the node has encountered a minor failure.
    UAVCAN_NODE_HEALTH_WARNING = 1,
    /// The node has encountered a major failure.
    UAVCAN_NODE_HEALTH_ERROR = 2,
    /// The node has suffered a fatal malfunction.
    UAVCAN_NODE_HEALTH_CRITICAL = 3,
};

/// Enumeration of distance sensor types
pub const MAV_DISTANCE_SENSOR = enum(u8) {
    /// Laser rangefinder, e.g. LightWare SF02/F or PulsedLight units
    MAV_DISTANCE_SENSOR_LASER = 0,
    /// Ultrasound rangefinder, e.g. MaxBotix units
    MAV_DISTANCE_SENSOR_ULTRASOUND = 1,
    /// Infrared rangefinder, e.g. Sharp units
    MAV_DISTANCE_SENSOR_INFRARED = 2,
    /// Radar type, e.g. uLanding units
    MAV_DISTANCE_SENSOR_RADAR = 3,
    /// Broken or unknown type, e.g. analog units
    MAV_DISTANCE_SENSOR_UNKNOWN = 4,
};

/// Focus types for MAV_CMD_SET_CAMERA_FOCUS
pub const SET_FOCUS_TYPE = enum(u32) {
    /// Focus one step increment (-1 for focusing in, 1 for focusing out towards infinity).
    FOCUS_TYPE_STEP = 0,
    /// Continuous normalized focus in/out rate until stopped. Range -1..1, negative: in, positive: out towards infinity, 0 to stop focusing. Other values should be clipped to the range.
    FOCUS_TYPE_CONTINUOUS = 1,
    /// Focus value as proportion of full camera focus range (a value between 0.0 and 100.0)
    FOCUS_TYPE_RANGE = 2,
    /// Focus value in metres. Note that there is no message to get the valid focus range of the camera, so this can type can only be used for cameras where the range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera).
    FOCUS_TYPE_METERS = 3,
    /// Focus automatically.
    FOCUS_TYPE_AUTO = 4,
    /// Single auto focus. Mainly used for still pictures. Usually abbreviated as AF-S.
    FOCUS_TYPE_AUTO_SINGLE = 5,
    /// Continuous auto focus. Mainly used for dynamic scenes. Abbreviated as AF-C.
    FOCUS_TYPE_AUTO_CONTINUOUS = 6,
};

/// Reason for an event error response.
pub const MAV_EVENT_ERROR_REASON = enum(u32) {
    /// The requested event is not available (anymore).
    MAV_EVENT_ERROR_REASON_UNAVAILABLE = 0,
};

/// Camera tracking modes
pub const CAMERA_TRACKING_MODE = enum(u8) {
    /// Not tracking
    CAMERA_TRACKING_MODE_NONE = 0,
    /// Target is a point
    CAMERA_TRACKING_MODE_POINT = 1,
    /// Target is a rectangle
    CAMERA_TRACKING_MODE_RECTANGLE = 2,
};

/// Camera sources for MAV_CMD_SET_CAMERA_SOURCE
pub const CAMERA_SOURCE = enum(u32) {
    /// Default camera source.
    CAMERA_SOURCE_DEFAULT = 0,
    /// RGB camera source.
    CAMERA_SOURCE_RGB = 1,
    /// IR camera source.
    CAMERA_SOURCE_IR = 2,
    /// NDVI camera source.
    CAMERA_SOURCE_NDVI = 3,
};

/// Result of mission operation (in a MISSION_ACK message).
pub const MAV_MISSION_RESULT = enum(u8) {
    /// mission accepted OK
    MAV_MISSION_ACCEPTED = 0,
    /// Generic error / not accepting mission commands at all right now.
    MAV_MISSION_ERROR = 1,
    /// Coordinate frame is not supported.
    MAV_MISSION_UNSUPPORTED_FRAME = 2,
    /// Command is not supported.
    MAV_MISSION_UNSUPPORTED = 3,
    /// Mission items exceed storage space.
    MAV_MISSION_NO_SPACE = 4,
    /// One of the parameters has an invalid value.
    MAV_MISSION_INVALID = 5,
    /// param1 has an invalid value.
    MAV_MISSION_INVALID_PARAM1 = 6,
    /// param2 has an invalid value.
    MAV_MISSION_INVALID_PARAM2 = 7,
    /// param3 has an invalid value.
    MAV_MISSION_INVALID_PARAM3 = 8,
    /// param4 has an invalid value.
    MAV_MISSION_INVALID_PARAM4 = 9,
    /// x / param5 has an invalid value.
    MAV_MISSION_INVALID_PARAM5_X = 10,
    /// y / param6 has an invalid value.
    MAV_MISSION_INVALID_PARAM6_Y = 11,
    /// z / param7 has an invalid value.
    MAV_MISSION_INVALID_PARAM7 = 12,
    /// Mission item received out of sequence
    MAV_MISSION_INVALID_SEQUENCE = 13,
    /// Not accepting any mission commands from this communication partner.
    MAV_MISSION_DENIED = 14,
    /// Current mission operation cancelled (e.g. mission upload, mission download).
    MAV_MISSION_OPERATION_CANCELLED = 15,
};

/// Tune formats (used for vehicle buzzer/tone generation).
pub const TUNE_FORMAT = enum(u32) {
    /// Format is QBasic 1.1 Play: https://www.qbasic.net/en/reference/qb11/Statement/PLAY-006.htm.
    TUNE_FORMAT_QBASIC1_1 = 1,
    /// Format is Modern Music Markup Language (MML): https://en.wikipedia.org/wiki/Music_Macro_Language#Modern_MML.
    TUNE_FORMAT_MML_MODERN = 2,
};

pub const FENCE_BREACH = enum(u8) {
    /// No last fence breach
    FENCE_BREACH_NONE = 0,
    /// Breached minimum altitude
    FENCE_BREACH_MINALT = 1,
    /// Breached maximum altitude
    FENCE_BREACH_MAXALT = 2,
    /// Breached fence boundary
    FENCE_BREACH_BOUNDARY = 3,
};

pub const MAV_ODID_HOR_ACC = enum(u8) {
    /// The horizontal accuracy is unknown.
    MAV_ODID_HOR_ACC_UNKNOWN = 0,
    /// The horizontal accuracy is smaller than 10 Nautical Miles. 18.52 km.
    MAV_ODID_HOR_ACC_10NM = 1,
    /// The horizontal accuracy is smaller than 4 Nautical Miles. 7.408 km.
    MAV_ODID_HOR_ACC_4NM = 2,
    /// The horizontal accuracy is smaller than 2 Nautical Miles. 3.704 km.
    MAV_ODID_HOR_ACC_2NM = 3,
    /// The horizontal accuracy is smaller than 1 Nautical Miles. 1.852 km.
    MAV_ODID_HOR_ACC_1NM = 4,
    /// The horizontal accuracy is smaller than 0.5 Nautical Miles. 926 m.
    MAV_ODID_HOR_ACC_0_5NM = 5,
    /// The horizontal accuracy is smaller than 0.3 Nautical Miles. 555.6 m.
    MAV_ODID_HOR_ACC_0_3NM = 6,
    /// The horizontal accuracy is smaller than 0.1 Nautical Miles. 185.2 m.
    MAV_ODID_HOR_ACC_0_1NM = 7,
    /// The horizontal accuracy is smaller than 0.05 Nautical Miles. 92.6 m.
    MAV_ODID_HOR_ACC_0_05NM = 8,
    /// The horizontal accuracy is smaller than 30 meter.
    MAV_ODID_HOR_ACC_30_METER = 9,
    /// The horizontal accuracy is smaller than 10 meter.
    MAV_ODID_HOR_ACC_10_METER = 10,
    /// The horizontal accuracy is smaller than 3 meter.
    MAV_ODID_HOR_ACC_3_METER = 11,
    /// The horizontal accuracy is smaller than 1 meter.
    MAV_ODID_HOR_ACC_1_METER = 12,
};

/// Flags in the HIGHRES_IMU message indicate which fields have updated since the last message
pub const HIGHRES_IMU_UPDATED_FLAGS = enum(u16) {
    /// The value in the xacc field has been updated
    HIGHRES_IMU_UPDATED_XACC = 1,
    /// The value in the yacc field has been updated
    HIGHRES_IMU_UPDATED_YACC = 2,
    /// The value in the zacc field has been updated since
    HIGHRES_IMU_UPDATED_ZACC = 4,
    /// The value in the xgyro field has been updated
    HIGHRES_IMU_UPDATED_XGYRO = 8,
    /// The value in the ygyro field has been updated
    HIGHRES_IMU_UPDATED_YGYRO = 16,
    /// The value in the zgyro field has been updated
    HIGHRES_IMU_UPDATED_ZGYRO = 32,
    /// The value in the xmag field has been updated
    HIGHRES_IMU_UPDATED_XMAG = 64,
    /// The value in the ymag field has been updated
    HIGHRES_IMU_UPDATED_YMAG = 128,
    /// The value in the zmag field has been updated
    HIGHRES_IMU_UPDATED_ZMAG = 256,
    /// The value in the abs_pressure field has been updated
    HIGHRES_IMU_UPDATED_ABS_PRESSURE = 512,
    /// The value in the diff_pressure field has been updated
    HIGHRES_IMU_UPDATED_DIFF_PRESSURE = 1024,
    /// The value in the pressure_alt field has been updated
    HIGHRES_IMU_UPDATED_PRESSURE_ALT = 2048,
    /// The value in the temperature field has been updated
    HIGHRES_IMU_UPDATED_TEMPERATURE = 4096,
};

/// Smart battery supply status/fault flags (bitmask) for health indication. The battery must also report either MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY if any of these are set.
pub const MAV_BATTERY_FAULT = enum(u32) {
    /// Battery has deep discharged.
    MAV_BATTERY_FAULT_DEEP_DISCHARGE = 1,
    /// Voltage spikes.
    MAV_BATTERY_FAULT_SPIKES = 2,
    /// One or more cells have failed. Battery should also report MAV_BATTERY_CHARGE_STATE_FAILE (and should not be used).
    MAV_BATTERY_FAULT_CELL_FAIL = 4,
    /// Over-current fault.
    MAV_BATTERY_FAULT_OVER_CURRENT = 8,
    /// Over-temperature fault.
    MAV_BATTERY_FAULT_OVER_TEMPERATURE = 16,
    /// Under-temperature fault.
    MAV_BATTERY_FAULT_UNDER_TEMPERATURE = 32,
    /// Vehicle voltage is not compatible with this battery (batteries on same power rail should have similar voltage).
    MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE = 64,
    /// Battery firmware is not compatible with current autopilot firmware.
    MAV_BATTERY_FAULT_INCOMPATIBLE_FIRMWARE = 128,
    /// Battery is not compatible due to cell configuration (e.g. 5s1p when vehicle requires 6s).
    BATTERY_FAULT_INCOMPATIBLE_CELLS_CONFIGURATION = 256,
};

pub const MAV_ODID_OPERATOR_ID_TYPE = enum(u8) {
    /// CAA (Civil Aviation Authority) registered operator ID.
    MAV_ODID_OPERATOR_ID_TYPE_CAA = 0,
};

pub const MAV_ODID_ARM_STATUS = enum(u8) {
    /// Passing arming checks.
    MAV_ODID_ARM_STATUS_GOOD_TO_ARM = 0,
    /// Generic arming failure, see error string for details.
    MAV_ODID_ARM_STATUS_PRE_ARM_FAIL_GENERIC = 1,
};

/// RC sub-type of types defined in RC_TYPE. Used in MAV_CMD_START_RX_PAIR. Ignored if value does not correspond to the set RC_TYPE.
pub const RC_SUB_TYPE = enum(u32) {
    /// Spektrum DSM2
    RC_SUB_TYPE_SPEKTRUM_DSM2 = 0,
    /// Spektrum DSMX
    RC_SUB_TYPE_SPEKTRUM_DSMX = 1,
    /// Spektrum DSMX8
    RC_SUB_TYPE_SPEKTRUM_DSMX8 = 2,
};

/// Flags for the global position report.
pub const UTM_DATA_AVAIL_FLAGS = enum(u8) {
    /// The field time contains valid data.
    UTM_DATA_AVAIL_FLAGS_TIME_VALID = 1,
    /// The field uas_id contains valid data.
    UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE = 2,
    /// The fields lat, lon and h_acc contain valid data.
    UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE = 4,
    /// The fields alt and v_acc contain valid data.
    UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE = 8,
    /// The field relative_alt contains valid data.
    UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE = 16,
    /// The fields vx and vy contain valid data.
    UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE = 32,
    /// The field vz contains valid data.
    UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE = 64,
    /// The fields next_lat, next_lon and next_alt contain valid data.
    UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE = 128,
};

/// Generalized UAVCAN node mode
pub const UAVCAN_NODE_MODE = enum(u8) {
    /// The node is performing its primary functions.
    UAVCAN_NODE_MODE_OPERATIONAL = 0,
    /// The node is initializing; this mode is entered immediately after startup.
    UAVCAN_NODE_MODE_INITIALIZATION = 1,
    /// The node is under maintenance.
    UAVCAN_NODE_MODE_MAINTENANCE = 2,
    /// The node is in the process of updating its software.
    UAVCAN_NODE_MODE_SOFTWARE_UPDATE = 3,
    /// The node is no longer available online.
    UAVCAN_NODE_MODE_OFFLINE = 7,
};

/// States of the mission state machine.
///         Note that these states are independent of whether the mission is in a mode that can execute mission items or not (is suspended).
///         They may not all be relevant on all vehicles.
pub const MISSION_STATE = enum(u8) {
    /// The mission status reporting is not supported.
    MISSION_STATE_UNKNOWN = 0,
    /// No mission on the vehicle.
    MISSION_STATE_NO_MISSION = 1,
    /// Mission has not started. This is the case after a mission has uploaded but not yet started executing.
    MISSION_STATE_NOT_STARTED = 2,
    /// Mission is active, and will execute mission items when in auto mode.
    MISSION_STATE_ACTIVE = 3,
    /// Mission is paused when in auto mode.
    MISSION_STATE_PAUSED = 4,
    /// Mission has executed all mission items.
    MISSION_STATE_COMPLETE = 5,
};

pub const MAV_TUNNEL_PAYLOAD_TYPE = enum(u16) {
    /// Encoding of payload unknown.
    MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN = 0,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208,
    /// Registered for STorM32 gimbal controller.
    MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209,
    /// Registered for ModalAI remote OSD protocol.
    MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_REMOTE_OSD = 210,
    /// Registered for ModalAI ESC UART passthru protocol.
    MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_ESC_UART_PASSTHRU = 211,
    /// Registered for ModalAI vendor use.
    MAV_TUNNEL_PAYLOAD_TYPE_MODALAI_IO_UART_PASSTHRU = 212,
};

/// Actions for reading/writing parameters between persistent and volatile storage when using MAV_CMD_PREFLIGHT_STORAGE.
///         (Commonly parameters are loaded from persistent storage (flash/EEPROM) into volatile storage (RAM) on startup and written back when they are changed.)
pub const PREFLIGHT_STORAGE_PARAMETER_ACTION = enum(u32) {
    /// Read all parameters from persistent storage. Replaces values in volatile storage.
    PARAM_READ_PERSISTENT = 0,
    /// Write all parameter values to persistent storage (flash/EEPROM)
    PARAM_WRITE_PERSISTENT = 1,
    /// Reset all user configurable parameters to their default value (including airframe selection, sensor calibration data, safety settings, and so on). Does not reset values that contain operation counters and vehicle computed statistics.
    PARAM_RESET_CONFIG_DEFAULT = 2,
    /// Reset only sensor calibration parameters to factory defaults (or firmware default if not available)
    PARAM_RESET_SENSOR_DEFAULT = 3,
    /// Reset all parameters, including operation counters, to default values
    PARAM_RESET_ALL_DEFAULT = 4,
};

/// MAVLINK component type reported in HEARTBEAT message. Flight controllers must report the type of the vehicle on which they are mounted (e.g. MAV_TYPE_OCTOROTOR). All other components must report a value appropriate for their type (e.g. a camera must use MAV_TYPE_CAMERA).
pub const MAV_TYPE = enum(u8) {
    /// Generic micro air vehicle
    MAV_TYPE_GENERIC = 0,
    /// Fixed wing aircraft.
    MAV_TYPE_FIXED_WING = 1,
    /// Quadrotor
    MAV_TYPE_QUADROTOR = 2,
    /// Coaxial helicopter
    MAV_TYPE_COAXIAL = 3,
    /// Normal helicopter with tail rotor.
    MAV_TYPE_HELICOPTER = 4,
    /// Ground installation
    MAV_TYPE_ANTENNA_TRACKER = 5,
    /// Operator control unit / ground control station
    MAV_TYPE_GCS = 6,
    /// Airship, controlled
    MAV_TYPE_AIRSHIP = 7,
    /// Free balloon, uncontrolled
    MAV_TYPE_FREE_BALLOON = 8,
    /// Rocket
    MAV_TYPE_ROCKET = 9,
    /// Ground rover
    MAV_TYPE_GROUND_ROVER = 10,
    /// Surface vessel, boat, ship
    MAV_TYPE_SURFACE_BOAT = 11,
    /// Submarine
    MAV_TYPE_SUBMARINE = 12,
    /// Hexarotor
    MAV_TYPE_HEXAROTOR = 13,
    /// Octorotor
    MAV_TYPE_OCTOROTOR = 14,
    /// Tricopter
    MAV_TYPE_TRICOPTER = 15,
    /// Flapping wing
    MAV_TYPE_FLAPPING_WING = 16,
    /// Kite
    MAV_TYPE_KITE = 17,
    /// Onboard companion controller
    MAV_TYPE_ONBOARD_CONTROLLER = 18,
    /// Two-rotor Tailsitter VTOL that additionally uses control surfaces in vertical operation. Note, value previously named MAV_TYPE_VTOL_DUOROTOR.
    MAV_TYPE_VTOL_TAILSITTER_DUOROTOR = 19,
    /// Quad-rotor Tailsitter VTOL using a V-shaped quad config in vertical operation. Note: value previously named MAV_TYPE_VTOL_QUADROTOR.
    MAV_TYPE_VTOL_TAILSITTER_QUADROTOR = 20,
    /// Tiltrotor VTOL. Fuselage and wings stay (nominally) horizontal in all flight phases. It able to tilt (some) rotors to provide thrust in cruise flight.
    MAV_TYPE_VTOL_TILTROTOR = 21,
    /// VTOL with separate fixed rotors for hover and cruise flight. Fuselage and wings stay (nominally) horizontal in all flight phases.
    MAV_TYPE_VTOL_FIXEDROTOR = 22,
    /// Tailsitter VTOL. Fuselage and wings orientation changes depending on flight phase: vertical for hover, horizontal for cruise. Use more specific VTOL MAV_TYPE_VTOL_TAILSITTER_DUOROTOR or MAV_TYPE_VTOL_TAILSITTER_QUADROTOR if appropriate.
    MAV_TYPE_VTOL_TAILSITTER = 23,
    /// Tiltwing VTOL. Fuselage stays horizontal in all flight phases. The whole wing, along with any attached engine, can tilt between vertical and horizontal mode.
    MAV_TYPE_VTOL_TILTWING = 24,
    /// VTOL reserved 5
    MAV_TYPE_VTOL_RESERVED5 = 25,
    /// Gimbal
    MAV_TYPE_GIMBAL = 26,
    /// ADSB system
    MAV_TYPE_ADSB = 27,
    /// Steerable, nonrigid airfoil
    MAV_TYPE_PARAFOIL = 28,
    /// Dodecarotor
    MAV_TYPE_DODECAROTOR = 29,
    /// Camera
    MAV_TYPE_CAMERA = 30,
    /// Charging station
    MAV_TYPE_CHARGING_STATION = 31,
    /// FLARM collision avoidance system
    MAV_TYPE_FLARM = 32,
    /// Servo
    MAV_TYPE_SERVO = 33,
    /// Open Drone ID. See https://mavlink.io/en/services/opendroneid.html.
    MAV_TYPE_ODID = 34,
    /// Decarotor
    MAV_TYPE_DECAROTOR = 35,
    /// Battery
    MAV_TYPE_BATTERY = 36,
    /// Parachute
    MAV_TYPE_PARACHUTE = 37,
    /// Log
    MAV_TYPE_LOG = 38,
    /// OSD
    MAV_TYPE_OSD = 39,
    /// IMU
    MAV_TYPE_IMU = 40,
    /// GPS
    MAV_TYPE_GPS = 41,
    /// Winch
    MAV_TYPE_WINCH = 42,
    /// Generic multirotor that does not fit into a specific type or whose type is unknown
    MAV_TYPE_GENERIC_MULTIROTOR = 43,
    /// Illuminator. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light).
    MAV_TYPE_ILLUMINATOR = 44,
    /// Orbiter spacecraft. Includes satellites orbiting terrestrial and extra-terrestrial bodies. Follows NASA Spacecraft Classification.
    MAV_TYPE_SPACECRAFT_ORBITER = 45,
    /// A generic four-legged ground vehicle (e.g., a robot dog).
    MAV_TYPE_GROUND_QUADRUPED = 46,
};

/// Airborne status of UAS.
pub const UTM_FLIGHT_STATE = enum(u8) {
    /// The flight state can't be determined.
    UTM_FLIGHT_STATE_UNKNOWN = 1,
    /// UAS on ground.
    UTM_FLIGHT_STATE_GROUND = 2,
    /// UAS airborne.
    UTM_FLIGHT_STATE_AIRBORNE = 3,
    /// UAS is in an emergency flight state.
    UTM_FLIGHT_STATE_EMERGENCY = 16,
    /// UAS has no active controls.
    UTM_FLIGHT_STATE_NOCTRL = 32,
};

pub const MAV_STATE = enum(u8) {
    /// Uninitialized system, state is unknown.
    MAV_STATE_UNINIT = 0,
    /// System is booting up.
    MAV_STATE_BOOT = 1,
    /// System is calibrating and not flight-ready.
    MAV_STATE_CALIBRATING = 2,
    /// System is grounded and on standby. It can be launched any time.
    MAV_STATE_STANDBY = 3,
    /// System is active and might be already airborne. Motors are engaged.
    MAV_STATE_ACTIVE = 4,
    /// System is in a non-normal flight mode (failsafe). It can however still navigate.
    MAV_STATE_CRITICAL = 5,
    /// System is in a non-normal flight mode (failsafe). It lost control over parts or over the whole airframe. It is in mayday and going down.
    MAV_STATE_EMERGENCY = 6,
    /// System just initialized its power-down sequence, will shut down now.
    MAV_STATE_POWEROFF = 7,
    /// System is terminating itself (failsafe or commanded).
    MAV_STATE_FLIGHT_TERMINATION = 8,
};

/// Flags to report status/failure cases for a power generator (used in GENERATOR_STATUS). Note that FAULTS are conditions that cause the generator to fail. Warnings are conditions that require attention before the next use (they indicate the system is not operating properly).
pub const MAV_GENERATOR_STATUS_FLAG = enum(u64) {
    /// Generator is off.
    MAV_GENERATOR_STATUS_FLAG_OFF = 1,
    /// Generator is ready to start generating power.
    MAV_GENERATOR_STATUS_FLAG_READY = 2,
    /// Generator is generating power.
    MAV_GENERATOR_STATUS_FLAG_GENERATING = 4,
    /// Generator is charging the batteries (generating enough power to charge and provide the load).
    MAV_GENERATOR_STATUS_FLAG_CHARGING = 8,
    /// Generator is operating at a reduced maximum power.
    MAV_GENERATOR_STATUS_FLAG_REDUCED_POWER = 16,
    /// Generator is providing the maximum output.
    MAV_GENERATOR_STATUS_FLAG_MAXPOWER = 32,
    /// Generator is near the maximum operating temperature, cooling is insufficient.
    MAV_GENERATOR_STATUS_FLAG_OVERTEMP_WARNING = 64,
    /// Generator hit the maximum operating temperature and shutdown.
    MAV_GENERATOR_STATUS_FLAG_OVERTEMP_FAULT = 128,
    /// Power electronics are near the maximum operating temperature, cooling is insufficient.
    MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_WARNING = 256,
    /// Power electronics hit the maximum operating temperature and shutdown.
    MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_OVERTEMP_FAULT = 512,
    /// Power electronics experienced a fault and shutdown.
    MAV_GENERATOR_STATUS_FLAG_ELECTRONICS_FAULT = 1024,
    /// The power source supplying the generator failed e.g. mechanical generator stopped, tether is no longer providing power, solar cell is in shade, hydrogen reaction no longer happening.
    MAV_GENERATOR_STATUS_FLAG_POWERSOURCE_FAULT = 2048,
    /// Generator controller having communication problems.
    MAV_GENERATOR_STATUS_FLAG_COMMUNICATION_WARNING = 4096,
    /// Power electronic or generator cooling system error.
    MAV_GENERATOR_STATUS_FLAG_COOLING_WARNING = 8192,
    /// Generator controller power rail experienced a fault.
    MAV_GENERATOR_STATUS_FLAG_POWER_RAIL_FAULT = 16384,
    /// Generator controller exceeded the overcurrent threshold and shutdown to prevent damage.
    MAV_GENERATOR_STATUS_FLAG_OVERCURRENT_FAULT = 32768,
    /// Generator controller detected a high current going into the batteries and shutdown to prevent battery damage.
    MAV_GENERATOR_STATUS_FLAG_BATTERY_OVERCHARGE_CURRENT_FAULT = 65536,
    /// Generator controller exceeded it's overvoltage threshold and shutdown to prevent it exceeding the voltage rating.
    MAV_GENERATOR_STATUS_FLAG_OVERVOLTAGE_FAULT = 131072,
    /// Batteries are under voltage (generator will not start).
    MAV_GENERATOR_STATUS_FLAG_BATTERY_UNDERVOLT_FAULT = 262144,
    /// Generator start is inhibited by e.g. a safety switch.
    MAV_GENERATOR_STATUS_FLAG_START_INHIBITED = 524288,
    /// Generator requires maintenance.
    MAV_GENERATOR_STATUS_FLAG_MAINTENANCE_REQUIRED = 1048576,
    /// Generator is not ready to generate yet.
    MAV_GENERATOR_STATUS_FLAG_WARMING_UP = 2097152,
    /// Generator is idle.
    MAV_GENERATOR_STATUS_FLAG_IDLE = 4194304,
};

/// Specifies the datatype of a MAVLink extended parameter.
pub const MAV_PARAM_EXT_TYPE = enum(u8) {
    /// 8-bit unsigned integer
    MAV_PARAM_EXT_TYPE_UINT8 = 1,
    /// 8-bit signed integer
    MAV_PARAM_EXT_TYPE_INT8 = 2,
    /// 16-bit unsigned integer
    MAV_PARAM_EXT_TYPE_UINT16 = 3,
    /// 16-bit signed integer
    MAV_PARAM_EXT_TYPE_INT16 = 4,
    /// 32-bit unsigned integer
    MAV_PARAM_EXT_TYPE_UINT32 = 5,
    /// 32-bit signed integer
    MAV_PARAM_EXT_TYPE_INT32 = 6,
    /// 64-bit unsigned integer
    MAV_PARAM_EXT_TYPE_UINT64 = 7,
    /// 64-bit signed integer
    MAV_PARAM_EXT_TYPE_INT64 = 8,
    /// 32-bit floating-point
    MAV_PARAM_EXT_TYPE_REAL32 = 9,
    /// 64-bit floating-point
    MAV_PARAM_EXT_TYPE_REAL64 = 10,
    /// Custom Type
    MAV_PARAM_EXT_TYPE_CUSTOM = 11,
};

pub const GPS_INPUT_IGNORE_FLAGS = enum(u16) {
    /// ignore altitude field
    GPS_INPUT_IGNORE_FLAG_ALT = 1,
    /// ignore hdop field
    GPS_INPUT_IGNORE_FLAG_HDOP = 2,
    /// ignore vdop field
    GPS_INPUT_IGNORE_FLAG_VDOP = 4,
    /// ignore horizontal velocity field (vn and ve)
    GPS_INPUT_IGNORE_FLAG_VEL_HORIZ = 8,
    /// ignore vertical velocity field (vd)
    GPS_INPUT_IGNORE_FLAG_VEL_VERT = 16,
    /// ignore speed accuracy field
    GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY = 32,
    /// ignore horizontal accuracy field
    GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY = 64,
    /// ignore vertical accuracy field
    GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY = 128,
};

/// Stream status flags (Bitmap)
pub const VIDEO_STREAM_STATUS_FLAGS = enum(u16) {
    /// Stream is active (running)
    VIDEO_STREAM_STATUS_FLAGS_RUNNING = 1,
    /// Stream is thermal imaging
    VIDEO_STREAM_STATUS_FLAGS_THERMAL = 2,
    /// Stream can report absolute thermal range (see CAMERA_THERMAL_RANGE).
    VIDEO_STREAM_STATUS_FLAGS_THERMAL_RANGE_ENABLED = 4,
};

pub const MAV_ODID_CATEGORY_EU = enum(u8) {
    /// The category for the UA, according to the EU specification, is undeclared.
    MAV_ODID_CATEGORY_EU_UNDECLARED = 0,
    /// The category for the UA, according to the EU specification, is the Open category.
    MAV_ODID_CATEGORY_EU_OPEN = 1,
    /// The category for the UA, according to the EU specification, is the Specific category.
    MAV_ODID_CATEGORY_EU_SPECIFIC = 2,
    /// The category for the UA, according to the EU specification, is the Certified category.
    MAV_ODID_CATEGORY_EU_CERTIFIED = 3,
};

/// Possible safety switch states.
pub const SAFETY_SWITCH_STATE = enum(u32) {
    /// Safety switch is engaged and vehicle should be safe to approach.
    SAFETY_SWITCH_STATE_SAFE = 0,
    /// Safety switch is NOT engaged and motors, propellers and other actuators should be considered active.
    SAFETY_SWITCH_STATE_DANGEROUS = 1,
};

/// Standard modes with a well understood meaning across flight stacks and vehicle types.
///         For example, most flight stack have the concept of a "return" or "RTL" mode that takes a vehicle to safety, even though the precise mechanics of this mode may differ.
///         The modes supported by a flight stack can be queried using AVAILABLE_MODES and set using MAV_CMD_DO_SET_STANDARD_MODE.
///         The current mode is streamed in CURRENT_MODE.
///         See https://mavlink.io/en/services/standard_modes.html
pub const MAV_STANDARD_MODE = enum(u8) {
    /// Non standard mode.
    ///           This may be used when reporting the mode if the current flight mode is not a standard mode.
    MAV_STANDARD_MODE_NON_STANDARD = 0,
    /// Position mode (manual).
    ///           Position-controlled and stabilized manual mode.
    ///           When sticks are released vehicles return to their level-flight orientation and hold both position and altitude against wind and external forces.
    ///           This mode can only be set by vehicles that can hold a fixed position.
    ///           Multicopter (MC) vehicles actively brake and hold both position and altitude against wind and external forces.
    ///           Hybrid MC/FW ("VTOL") vehicles first transition to multicopter mode (if needed) but otherwise behave in the same way as MC vehicles.
    ///           Fixed-wing (FW) vehicles must not support this mode.
    ///           Other vehicle types must not support this mode (this may be revisited through the PR process).
    MAV_STANDARD_MODE_POSITION_HOLD = 1,
    /// Orbit (manual).
    ///           Position-controlled and stabilized manual mode.
    ///           The vehicle circles around a fixed setpoint in the horizontal plane at a particular radius, altitude, and direction.
    ///           Flight stacks may further allow manual control over the setpoint position, radius, direction, speed, and/or altitude of the circle, but this is not mandated.
    ///           Flight stacks may support the [MAV_CMD_DO_ORBIT](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_ORBIT) for changing the orbit parameters.
    ///           MC and FW vehicles may support this mode.
    ///           Hybrid MC/FW ("VTOL") vehicles may support this mode in MC/FW or both modes; if the mode is not supported by the current configuration the vehicle should transition to the supported configuration.
    ///           Other vehicle types must not support this mode (this may be revisited through the PR process).
    MAV_STANDARD_MODE_ORBIT = 2,
    /// Cruise mode (manual).
    ///           Position-controlled and stabilized manual mode.
    ///           When sticks are released vehicles return to their level-flight orientation and hold their original track against wind and external forces.
    ///           Fixed-wing (FW) vehicles level orientation and maintain current track and altitude against wind and external forces.
    ///           Hybrid MC/FW ("VTOL") vehicles first transition to FW mode (if needed) but otherwise behave in the same way as MC vehicles.
    ///           Multicopter (MC) vehicles must not support this mode.
    ///           Other vehicle types must not support this mode (this may be revisited through the PR process).
    MAV_STANDARD_MODE_CRUISE = 3,
    /// Altitude hold (manual).
    ///           Altitude-controlled and stabilized manual mode.
    ///           When sticks are released vehicles return to their level-flight orientation and hold their altitude.
    ///           MC vehicles continue with existing momentum and may move with wind (or other external forces).
    ///           FW vehicles continue with current heading, but may be moved off-track by wind.
    ///           Hybrid MC/FW ("VTOL") vehicles behave according to their current configuration/mode (FW or MC).
    ///           Other vehicle types must not support this mode (this may be revisited through the PR process).
    MAV_STANDARD_MODE_ALTITUDE_HOLD = 4,
    /// Safe recovery mode (auto).
    ///           Automatic mode that takes vehicle to a predefined safe location via a safe flight path, and may also automatically land the vehicle.
    ///           This mode is more commonly referred to as RTL and/or or Smart RTL.
    ///           The precise return location, flight path, and landing behaviour depend on vehicle configuration and type.
    ///           For example, the vehicle might return to the home/launch location, a rally point, or the start of a mission landing, it might follow a direct path, mission path, or breadcrumb path, and land using a mission landing pattern or some other kind of descent.
    MAV_STANDARD_MODE_SAFE_RECOVERY = 5,
    /// Mission mode (automatic).
    ///           Automatic mode that executes MAVLink missions.
    ///           Missions are executed from the current waypoint as soon as the mode is enabled.
    MAV_STANDARD_MODE_MISSION = 6,
    /// Land mode (auto).
    ///           Automatic mode that lands the vehicle at the current location.
    ///           The precise landing behaviour depends on vehicle configuration and type.
    MAV_STANDARD_MODE_LAND = 7,
    /// Takeoff mode (auto).
    ///           Automatic takeoff mode.
    ///           The precise takeoff behaviour depends on vehicle configuration and type.
    MAV_STANDARD_MODE_TAKEOFF = 8,
};

pub const MAV_ODID_VER_ACC = enum(u8) {
    /// The vertical accuracy is unknown.
    MAV_ODID_VER_ACC_UNKNOWN = 0,
    /// The vertical accuracy is smaller than 150 meter.
    MAV_ODID_VER_ACC_150_METER = 1,
    /// The vertical accuracy is smaller than 45 meter.
    MAV_ODID_VER_ACC_45_METER = 2,
    /// The vertical accuracy is smaller than 25 meter.
    MAV_ODID_VER_ACC_25_METER = 3,
    /// The vertical accuracy is smaller than 10 meter.
    MAV_ODID_VER_ACC_10_METER = 4,
    /// The vertical accuracy is smaller than 3 meter.
    MAV_ODID_VER_ACC_3_METER = 5,
    /// The vertical accuracy is smaller than 1 meter.
    MAV_ODID_VER_ACC_1_METER = 6,
};

/// MAV FTP opcodes: https://mavlink.io/en/services/ftp.html
pub const MAV_FTP_OPCODE = enum(u32) {
    /// None. Ignored, always ACKed
    MAV_FTP_OPCODE_NONE = 0,
    /// TerminateSession: Terminates open Read session
    MAV_FTP_OPCODE_TERMINATESESSION = 1,
    /// ResetSessions: Terminates all open read sessions
    MAV_FTP_OPCODE_RESETSESSION = 2,
    /// ListDirectory. List files and directories in path from offset
    MAV_FTP_OPCODE_LISTDIRECTORY = 3,
    /// OpenFileRO: Opens file at path for reading, returns session
    MAV_FTP_OPCODE_OPENFILERO = 4,
    /// ReadFile: Reads size bytes from offset in session
    MAV_FTP_OPCODE_READFILE = 5,
    /// CreateFile: Creates file at path for writing, returns session
    MAV_FTP_OPCODE_CREATEFILE = 6,
    /// WriteFile: Writes size bytes to offset in session
    MAV_FTP_OPCODE_WRITEFILE = 7,
    /// RemoveFile: Remove file at path
    MAV_FTP_OPCODE_REMOVEFILE = 8,
    /// CreateDirectory: Creates directory at path
    MAV_FTP_OPCODE_CREATEDIRECTORY = 9,
    /// RemoveDirectory: Removes directory at path. The directory must be empty.
    MAV_FTP_OPCODE_REMOVEDIRECTORY = 10,
    /// OpenFileWO: Opens file at path for writing, returns session
    MAV_FTP_OPCODE_OPENFILEWO = 11,
    /// TruncateFile: Truncate file at path to offset length
    MAV_FTP_OPCODE_TRUNCATEFILE = 12,
    /// Rename: Rename path1 to path2
    MAV_FTP_OPCODE_RENAME = 13,
    /// CalcFileCRC32: Calculate CRC32 for file at path
    MAV_FTP_OPCODE_CALCFILECRC = 14,
    /// BurstReadFile: Burst download session file
    MAV_FTP_OPCODE_BURSTREADFILE = 15,
    /// ACK: ACK response
    MAV_FTP_OPCODE_ACK = 128,
    /// NAK: NAK response
    MAV_FTP_OPCODE_NAK = 129,
};

/// Battery mode. Note, the normal operation mode (i.e. when flying) should be reported as MAV_BATTERY_MODE_UNKNOWN to allow message trimming in normal flight.
pub const MAV_BATTERY_MODE = enum(u8) {
    /// Battery mode not supported/unknown battery mode/normal operation.
    MAV_BATTERY_MODE_UNKNOWN = 0,
    /// Battery is auto discharging (towards storage level).
    MAV_BATTERY_MODE_AUTO_DISCHARGING = 1,
    /// Battery in hot-swap mode (current limited to prevent spikes that might damage sensitive electrical circuits).
    MAV_BATTERY_MODE_HOT_SWAP = 2,
};

/// These encode the sensors whose status is sent as part of the SYS_STATUS message.
pub const MAV_SYS_STATUS_SENSOR = enum(u32) {
    /// 0x01 3D gyro
    MAV_SYS_STATUS_SENSOR_3D_GYRO = 1,
    /// 0x02 3D accelerometer
    MAV_SYS_STATUS_SENSOR_3D_ACCEL = 2,
    /// 0x04 3D magnetometer
    MAV_SYS_STATUS_SENSOR_3D_MAG = 4,
    /// 0x08 absolute pressure
    MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE = 8,
    /// 0x10 differential pressure
    MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE = 16,
    /// 0x20 GPS
    MAV_SYS_STATUS_SENSOR_GPS = 32,
    /// 0x40 optical flow
    MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW = 64,
    /// 0x80 computer vision position
    MAV_SYS_STATUS_SENSOR_VISION_POSITION = 128,
    /// 0x100 laser based position
    MAV_SYS_STATUS_SENSOR_LASER_POSITION = 256,
    /// 0x200 external ground truth (Vicon or Leica)
    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH = 512,
    /// 0x400 3D angular rate control
    MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL = 1024,
    /// 0x800 attitude stabilization
    MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION = 2048,
    /// 0x1000 yaw position
    MAV_SYS_STATUS_SENSOR_YAW_POSITION = 4096,
    /// 0x2000 z/altitude control
    MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL = 8192,
    /// 0x4000 x/y position control
    MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL = 16384,
    /// 0x8000 motor outputs / control
    MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS = 32768,
    /// 0x10000 RC receiver
    MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 65536,
    /// 0x20000 2nd 3D gyro
    MAV_SYS_STATUS_SENSOR_3D_GYRO2 = 131072,
    /// 0x40000 2nd 3D accelerometer
    MAV_SYS_STATUS_SENSOR_3D_ACCEL2 = 262144,
    /// 0x80000 2nd 3D magnetometer
    MAV_SYS_STATUS_SENSOR_3D_MAG2 = 524288,
    /// 0x100000 geofence
    MAV_SYS_STATUS_GEOFENCE = 1048576,
    /// 0x200000 AHRS subsystem health
    MAV_SYS_STATUS_AHRS = 2097152,
    /// 0x400000 Terrain subsystem health
    MAV_SYS_STATUS_TERRAIN = 4194304,
    /// 0x800000 Motors are reversed
    MAV_SYS_STATUS_REVERSE_MOTOR = 8388608,
    /// 0x1000000 Logging
    MAV_SYS_STATUS_LOGGING = 16777216,
    /// 0x2000000 Battery
    MAV_SYS_STATUS_SENSOR_BATTERY = 33554432,
    /// 0x4000000 Proximity
    MAV_SYS_STATUS_SENSOR_PROXIMITY = 67108864,
    /// 0x8000000 Satellite Communication
    MAV_SYS_STATUS_SENSOR_SATCOM = 134217728,
    /// 0x10000000 pre-arm check status. Always healthy when armed
    MAV_SYS_STATUS_PREARM_CHECK = 268435456,
    /// 0x20000000 Avoidance/collision prevention
    MAV_SYS_STATUS_OBSTACLE_AVOIDANCE = 536870912,
    /// 0x40000000 propulsion (actuator, esc, motor or propellor)
    MAV_SYS_STATUS_SENSOR_PROPULSION = 1073741824,
    /// 0x80000000 Extended bit-field are used for further sensor status bits (needs to be set in onboard_control_sensors_present only)
    MAV_SYS_STATUS_EXTENSION_USED = 2147483648,
};

/// Flags to report failure cases over the high latency telemetry.
pub const HL_FAILURE_FLAG = enum(u16) {
    /// GPS failure.
    HL_FAILURE_FLAG_GPS = 1,
    /// Differential pressure sensor failure.
    HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE = 2,
    /// Absolute pressure sensor failure.
    HL_FAILURE_FLAG_ABSOLUTE_PRESSURE = 4,
    /// Accelerometer sensor failure.
    HL_FAILURE_FLAG_3D_ACCEL = 8,
    /// Gyroscope sensor failure.
    HL_FAILURE_FLAG_3D_GYRO = 16,
    /// Magnetometer sensor failure.
    HL_FAILURE_FLAG_3D_MAG = 32,
    /// Terrain subsystem failure.
    HL_FAILURE_FLAG_TERRAIN = 64,
    /// Battery failure/critical low battery.
    HL_FAILURE_FLAG_BATTERY = 128,
    /// RC receiver failure/no RC connection.
    HL_FAILURE_FLAG_RC_RECEIVER = 256,
    /// Offboard link failure.
    HL_FAILURE_FLAG_OFFBOARD_LINK = 512,
    /// Engine failure.
    HL_FAILURE_FLAG_ENGINE = 1024,
    /// Geofence violation.
    HL_FAILURE_FLAG_GEOFENCE = 2048,
    /// Estimator failure, for example measurement rejection or large variances.
    HL_FAILURE_FLAG_ESTIMATOR = 4096,
    /// Mission failure.
    HL_FAILURE_FLAG_MISSION = 8192,
};

/// Camera Modes.
pub const CAMERA_MODE = enum(u8) {
    /// Camera is in image/photo capture mode.
    CAMERA_MODE_IMAGE = 0,
    /// Camera is in video capture mode.
    CAMERA_MODE_VIDEO = 1,
    /// Camera is in image survey capture mode. It allows for camera controller to do specific settings for surveys.
    CAMERA_MODE_IMAGE_SURVEY = 2,
};

/// Type of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
pub const AIS_TYPE = enum(u8) {
    /// Not available (default).
    AIS_TYPE_UNKNOWN = 0,
    AIS_TYPE_RESERVED_1 = 1,
    AIS_TYPE_RESERVED_2 = 2,
    AIS_TYPE_RESERVED_3 = 3,
    AIS_TYPE_RESERVED_4 = 4,
    AIS_TYPE_RESERVED_5 = 5,
    AIS_TYPE_RESERVED_6 = 6,
    AIS_TYPE_RESERVED_7 = 7,
    AIS_TYPE_RESERVED_8 = 8,
    AIS_TYPE_RESERVED_9 = 9,
    AIS_TYPE_RESERVED_10 = 10,
    AIS_TYPE_RESERVED_11 = 11,
    AIS_TYPE_RESERVED_12 = 12,
    AIS_TYPE_RESERVED_13 = 13,
    AIS_TYPE_RESERVED_14 = 14,
    AIS_TYPE_RESERVED_15 = 15,
    AIS_TYPE_RESERVED_16 = 16,
    AIS_TYPE_RESERVED_17 = 17,
    AIS_TYPE_RESERVED_18 = 18,
    AIS_TYPE_RESERVED_19 = 19,
    /// Wing In Ground effect.
    AIS_TYPE_WIG = 20,
    AIS_TYPE_WIG_HAZARDOUS_A = 21,
    AIS_TYPE_WIG_HAZARDOUS_B = 22,
    AIS_TYPE_WIG_HAZARDOUS_C = 23,
    AIS_TYPE_WIG_HAZARDOUS_D = 24,
    AIS_TYPE_WIG_RESERVED_1 = 25,
    AIS_TYPE_WIG_RESERVED_2 = 26,
    AIS_TYPE_WIG_RESERVED_3 = 27,
    AIS_TYPE_WIG_RESERVED_4 = 28,
    AIS_TYPE_WIG_RESERVED_5 = 29,
    AIS_TYPE_FISHING = 30,
    AIS_TYPE_TOWING = 31,
    /// Towing: length exceeds 200m or breadth exceeds 25m.
    AIS_TYPE_TOWING_LARGE = 32,
    /// Dredging or other underwater ops.
    AIS_TYPE_DREDGING = 33,
    AIS_TYPE_DIVING = 34,
    AIS_TYPE_MILITARY = 35,
    AIS_TYPE_SAILING = 36,
    AIS_TYPE_PLEASURE = 37,
    AIS_TYPE_RESERVED_20 = 38,
    AIS_TYPE_RESERVED_21 = 39,
    /// High Speed Craft.
    AIS_TYPE_HSC = 40,
    AIS_TYPE_HSC_HAZARDOUS_A = 41,
    AIS_TYPE_HSC_HAZARDOUS_B = 42,
    AIS_TYPE_HSC_HAZARDOUS_C = 43,
    AIS_TYPE_HSC_HAZARDOUS_D = 44,
    AIS_TYPE_HSC_RESERVED_1 = 45,
    AIS_TYPE_HSC_RESERVED_2 = 46,
    AIS_TYPE_HSC_RESERVED_3 = 47,
    AIS_TYPE_HSC_RESERVED_4 = 48,
    AIS_TYPE_HSC_UNKNOWN = 49,
    AIS_TYPE_PILOT = 50,
    /// Search And Rescue vessel.
    AIS_TYPE_SAR = 51,
    AIS_TYPE_TUG = 52,
    AIS_TYPE_PORT_TENDER = 53,
    /// Anti-pollution equipment.
    AIS_TYPE_ANTI_POLLUTION = 54,
    AIS_TYPE_LAW_ENFORCEMENT = 55,
    AIS_TYPE_SPARE_LOCAL_1 = 56,
    AIS_TYPE_SPARE_LOCAL_2 = 57,
    AIS_TYPE_MEDICAL_TRANSPORT = 58,
    /// Noncombatant ship according to RR Resolution No. 18.
    AIS_TYPE_NONECOMBATANT = 59,
    AIS_TYPE_PASSENGER = 60,
    AIS_TYPE_PASSENGER_HAZARDOUS_A = 61,
    AIS_TYPE_PASSENGER_HAZARDOUS_B = 62,
    AIS_TYPE_PASSENGER_HAZARDOUS_C = 63,
    AIS_TYPE_PASSENGER_HAZARDOUS_D = 64,
    AIS_TYPE_PASSENGER_RESERVED_1 = 65,
    AIS_TYPE_PASSENGER_RESERVED_2 = 66,
    AIS_TYPE_PASSENGER_RESERVED_3 = 67,
    AIS_TYPE_PASSENGER_RESERVED_4 = 68,
    AIS_TYPE_PASSENGER_UNKNOWN = 69,
    AIS_TYPE_CARGO = 70,
    AIS_TYPE_CARGO_HAZARDOUS_A = 71,
    AIS_TYPE_CARGO_HAZARDOUS_B = 72,
    AIS_TYPE_CARGO_HAZARDOUS_C = 73,
    AIS_TYPE_CARGO_HAZARDOUS_D = 74,
    AIS_TYPE_CARGO_RESERVED_1 = 75,
    AIS_TYPE_CARGO_RESERVED_2 = 76,
    AIS_TYPE_CARGO_RESERVED_3 = 77,
    AIS_TYPE_CARGO_RESERVED_4 = 78,
    AIS_TYPE_CARGO_UNKNOWN = 79,
    AIS_TYPE_TANKER = 80,
    AIS_TYPE_TANKER_HAZARDOUS_A = 81,
    AIS_TYPE_TANKER_HAZARDOUS_B = 82,
    AIS_TYPE_TANKER_HAZARDOUS_C = 83,
    AIS_TYPE_TANKER_HAZARDOUS_D = 84,
    AIS_TYPE_TANKER_RESERVED_1 = 85,
    AIS_TYPE_TANKER_RESERVED_2 = 86,
    AIS_TYPE_TANKER_RESERVED_3 = 87,
    AIS_TYPE_TANKER_RESERVED_4 = 88,
    AIS_TYPE_TANKER_UNKNOWN = 89,
    AIS_TYPE_OTHER = 90,
    AIS_TYPE_OTHER_HAZARDOUS_A = 91,
    AIS_TYPE_OTHER_HAZARDOUS_B = 92,
    AIS_TYPE_OTHER_HAZARDOUS_C = 93,
    AIS_TYPE_OTHER_HAZARDOUS_D = 94,
    AIS_TYPE_OTHER_RESERVED_1 = 95,
    AIS_TYPE_OTHER_RESERVED_2 = 96,
    AIS_TYPE_OTHER_RESERVED_3 = 97,
    AIS_TYPE_OTHER_RESERVED_4 = 98,
    AIS_TYPE_OTHER_UNKNOWN = 99,
};

/// Flags to report ESC failures.
pub const ESC_FAILURE_FLAGS = enum(u32) {
    /// Over current failure.
    ESC_FAILURE_OVER_CURRENT = 1,
    /// Over voltage failure.
    ESC_FAILURE_OVER_VOLTAGE = 2,
    /// Over temperature failure.
    ESC_FAILURE_OVER_TEMPERATURE = 4,
    /// Over RPM failure.
    ESC_FAILURE_OVER_RPM = 8,
    /// Inconsistent command failure i.e. out of bounds.
    ESC_FAILURE_INCONSISTENT_CMD = 16,
    /// Motor stuck failure.
    ESC_FAILURE_MOTOR_STUCK = 32,
    /// Generic ESC failure.
    ESC_FAILURE_GENERIC = 64,
};

/// These flags are used to diagnose the failure state of CELLULAR_STATUS
pub const CELLULAR_NETWORK_FAILED_REASON = enum(u8) {
    /// No error
    CELLULAR_NETWORK_FAILED_REASON_NONE = 0,
    /// Error state is unknown
    CELLULAR_NETWORK_FAILED_REASON_UNKNOWN = 1,
    /// SIM is required for the modem but missing
    CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING = 2,
    /// SIM is available, but not usable for connection
    CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR = 3,
};

/// Parachute actions. Trigger release and enable/disable auto-release.
pub const PARACHUTE_ACTION = enum(u32) {
    /// Disable auto-release of parachute (i.e. release triggered by crash detectors).
    PARACHUTE_DISABLE = 0,
    /// Enable auto-release of parachute.
    PARACHUTE_ENABLE = 1,
    /// Release parachute and kill motors.
    PARACHUTE_RELEASE = 2,
};

/// Delay mission state machine until gate has been reached.
pub const MAV_CMD = enum(u16) {
    /// Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
    MAV_CMD_PREFLIGHT_STORAGE_ADVANCED = 0,
    /// Navigate to waypoint. This is intended for use in missions (for guided commands outside of missions use MAV_CMD_DO_REPOSITION).
    MAV_CMD_NAV_WAYPOINT = 16,
    /// Loiter around this waypoint an unlimited amount of time
    MAV_CMD_NAV_LOITER_UNLIM = 17,
    /// Loiter around this waypoint for X turns
    MAV_CMD_NAV_LOITER_TURNS = 18,
    /// Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint.
    MAV_CMD_NAV_LOITER_TIME = 19,
    /// Return to launch location
    MAV_CMD_NAV_RETURN_TO_LAUNCH = 20,
    /// Land at location.
    MAV_CMD_NAV_LAND = 21,
    /// Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode.
    MAV_CMD_NAV_TAKEOFF = 22,
    /// Land at local position (local frame only)
    MAV_CMD_NAV_LAND_LOCAL = 23,
    /// Takeoff from local position (local frame only)
    MAV_CMD_NAV_TAKEOFF_LOCAL = 24,
    /// Vehicle following, i.e. this waypoint represents the position of a moving vehicle
    MAV_CMD_NAV_FOLLOW = 25,
    /// Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached.
    MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT = 30,
    /// Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint.
    MAV_CMD_NAV_LOITER_TO_ALT = 31,
    /// Begin following a target
    MAV_CMD_DO_FOLLOW = 32,
    /// Reposition the MAV after a follow target command has been sent
    MAV_CMD_DO_FOLLOW_REPOSITION = 33,
    /// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
    MAV_CMD_NAV_ROI = 80,
    /// Control autonomous path planning on the MAV.
    MAV_CMD_NAV_PATHPLANNING = 81,
    /// Navigate to waypoint using a spline path.
    MAV_CMD_NAV_SPLINE_WAYPOINT = 82,
    /// Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.).
    MAV_CMD_NAV_VTOL_TAKEOFF = 84,
    /// Land using VTOL mode
    MAV_CMD_NAV_VTOL_LAND = 85,
    /// hand control over to an external controller
    MAV_CMD_NAV_GUIDED_ENABLE = 92,
    /// Delay the next navigation command a number of seconds or until a specified time
    MAV_CMD_NAV_DELAY = 93,
    /// Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload.
    MAV_CMD_NAV_PAYLOAD_PLACE = 94,
    /// NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration
    MAV_CMD_NAV_LAST = 95,
    /// Delay mission state machine.
    MAV_CMD_CONDITION_DELAY = 112,
    /// Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached.
    MAV_CMD_CONDITION_CHANGE_ALT = 113,
    /// Delay mission state machine until within desired distance of next NAV point.
    MAV_CMD_CONDITION_DISTANCE = 114,
    /// Reach a certain target angle.
    MAV_CMD_CONDITION_YAW = 115,
    /// NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration
    MAV_CMD_CONDITION_LAST = 159,
    /// Set system mode.
    MAV_CMD_DO_SET_MODE = 176,
    /// Jump to the desired command in the mission list.  Repeat this action only the specified number of times
    MAV_CMD_DO_JUMP = 177,
    /// Change speed and/or throttle set points. The value persists until it is overridden or there is a mode change
    MAV_CMD_DO_CHANGE_SPEED = 178,
    /// Sets the home position to either to the current position or a specified position.
    ///           The home position is the default position that the system will return to and land on.
    ///           The position is set automatically by the system during the takeoff (and may also be set using this command).
    ///           Note: the current home position may be emitted in a HOME_POSITION message on request (using MAV_CMD_REQUEST_MESSAGE with param1=242).
    MAV_CMD_DO_SET_HOME = 179,
    /// Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter.
    MAV_CMD_DO_SET_PARAMETER = 180,
    /// Set a relay to a condition.
    MAV_CMD_DO_SET_RELAY = 181,
    /// Cycle a relay on and off for a desired number of cycles with a desired period.
    MAV_CMD_DO_REPEAT_RELAY = 182,
    /// Set a servo to a desired PWM value.
    MAV_CMD_DO_SET_SERVO = 183,
    /// Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period.
    MAV_CMD_DO_REPEAT_SERVO = 184,
    /// Terminate flight immediately.
    ///           Flight termination immediately and irreversibly terminates the current flight, returning the vehicle to ground.
    ///           The vehicle will ignore RC or other input until it has been power-cycled.
    ///           Termination may trigger safety measures, including: disabling motors and deployment of parachute on multicopters, and setting flight surfaces to initiate a landing pattern on fixed-wing).
    ///           On multicopters without a parachute it may trigger a crash landing.
    ///           Support for this command can be tested using the protocol bit: MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION.
    ///           Support for this command can also be tested by sending the command with param1=0 (< 0.5); the ACK should be either MAV_RESULT_FAILED or MAV_RESULT_UNSUPPORTED.
    MAV_CMD_DO_FLIGHTTERMINATION = 185,
    /// Change altitude set point.
    MAV_CMD_DO_CHANGE_ALTITUDE = 186,
    /// Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter).
    MAV_CMD_DO_SET_ACTUATOR = 187,
    /// Mission item to mark the start of a mission landing pattern, or a command to land with a mission landing pattern.
    /// 
    ///         When used in a mission, this is a marker for the start of a sequence of mission items that represent a landing pattern.
    ///         It should be followed by a navigation item that defines the first waypoint of the landing sequence.
    ///         The start marker positional params are used only for selecting what landing pattern to use if several are defined in the mission (the selected pattern will be the one with the marker position that is closest to the vehicle when a landing is commanded).
    ///         If the marker item position has zero-values for latitude, longitude, and altitude, then landing pattern selection is instead based on the position of the first waypoint in the landing sequence.
    /// 
    ///       When sent as a command it triggers a landing using a mission landing pattern.
    ///       The location parameters are not used in this case, and should be set to 0.
    MAV_CMD_DO_LAND_START = 189,
    /// Mission command to perform a landing from a rally point.
    MAV_CMD_DO_RALLY_LAND = 190,
    /// Mission command to safely abort an autonomous landing.
    MAV_CMD_DO_GO_AROUND = 191,
    /// Reposition the vehicle to a specific WGS84 global position. This command is intended for guided commands (for missions use MAV_CMD_NAV_WAYPOINT instead).
    MAV_CMD_DO_REPOSITION = 192,
    /// If in a GPS controlled position mode, hold the current position or continue.
    MAV_CMD_DO_PAUSE_CONTINUE = 193,
    /// Set moving direction to forward or reverse.
    MAV_CMD_DO_SET_REVERSE = 194,
    /// Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message.
    MAV_CMD_DO_SET_ROI_LOCATION = 195,
    /// Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.
    MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET = 196,
    /// Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position.
    MAV_CMD_DO_SET_ROI_NONE = 197,
    /// Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message.
    MAV_CMD_DO_SET_ROI_SYSID = 198,
    /// Control onboard camera system.
    MAV_CMD_DO_CONTROL_VIDEO = 200,
    /// Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras.
    MAV_CMD_DO_SET_ROI = 201,
    /// Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
    MAV_CMD_DO_DIGICAM_CONFIGURE = 202,
    /// Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ).
    MAV_CMD_DO_DIGICAM_CONTROL = 203,
    /// Mission command to configure a camera or antenna mount
    MAV_CMD_DO_MOUNT_CONFIGURE = 204,
    /// Mission command to control a camera or antenna mount
    MAV_CMD_DO_MOUNT_CONTROL = 205,
    /// Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera.
    MAV_CMD_DO_SET_CAM_TRIGG_DIST = 206,
    /// Enable the geofence.
    ///           This can be used in a mission or via the command protocol.
    ///           The persistence/lifetime of the setting is undefined.
    ///           Depending on flight stack implementation it may persist until superseded, or it may revert to a system default at the end of a mission.
    ///           Flight stacks typically reset the setting to system defaults on reboot.
    MAV_CMD_DO_FENCE_ENABLE = 207,
    /// Mission item/command to release a parachute or enable/disable auto release.
    MAV_CMD_DO_PARACHUTE = 208,
    /// Command to perform motor test.
    MAV_CMD_DO_MOTOR_TEST = 209,
    /// Change to/from inverted flight.
    MAV_CMD_DO_INVERTED_FLIGHT = 210,
    /// Mission command to operate a gripper.
    MAV_CMD_DO_GRIPPER = 211,
    /// Enable/disable autotune.
    MAV_CMD_DO_AUTOTUNE_ENABLE = 212,
    /// Sets a desired vehicle turn angle and speed change.
    MAV_CMD_NAV_SET_YAW_SPEED = 213,
    /// Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera.
    MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL = 214,
    /// Mission command to control a camera or antenna mount, using a quaternion as reference.
    MAV_CMD_DO_MOUNT_CONTROL_QUAT = 220,
    /// set id of master controller
    MAV_CMD_DO_GUIDED_MASTER = 221,
    /// Set limits for external control
    MAV_CMD_DO_GUIDED_LIMITS = 222,
    /// Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines
    MAV_CMD_DO_ENGINE_CONTROL = 223,
    /// Set the mission item with sequence number seq as the current item and emit MISSION_CURRENT (whether or not the mission number changed).
    ///           If a mission is currently being executed, the system will continue to this new mission item on the shortest path, skipping any intermediate mission items.
    ///   Note that mission jump repeat counters are not reset unless param2 is set (see MAV_CMD_DO_JUMP param2).
    /// 
    ///           This command may trigger a mission state-machine change on some systems: for example from MISSION_STATE_NOT_STARTED or MISSION_STATE_PAUSED to MISSION_STATE_ACTIVE.
    ///           If the system is in mission mode, on those systems this command might therefore start, restart or resume the mission.
    ///           If the system is not in mission mode this command must not trigger a switch to mission mode.
    /// 
    ///           The mission may be "reset" using param2.
    ///           Resetting sets jump counters to initial values (to reset counters without changing the current mission item set the param1 to `-1`).
    ///           Resetting also explicitly changes a mission state of MISSION_STATE_COMPLETE to MISSION_STATE_PAUSED or MISSION_STATE_ACTIVE, potentially allowing it to resume when it is (next) in a mission mode.
    /// 
    ///   The command will ACK with MAV_RESULT_FAILED if the sequence number is out of range (including if there is no mission item).
    MAV_CMD_DO_SET_MISSION_CURRENT = 224,
    /// NOP - This command is only used to mark the upper limit of the DO commands in the enumeration
    MAV_CMD_DO_LAST = 240,
    /// Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero.
    MAV_CMD_PREFLIGHT_CALIBRATION = 241,
    /// Set sensor offsets. This command will be only accepted if in pre-flight mode.
    MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS = 242,
    /// Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named).
    MAV_CMD_PREFLIGHT_UAVCAN = 243,
    /// Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode.
    MAV_CMD_PREFLIGHT_STORAGE = 245,
    /// Request the reboot or shutdown of system components.
    MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246,
    /// Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position.
    MAV_CMD_OVERRIDE_GOTO = 252,
    /// Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera.
    MAV_CMD_OBLIQUE_SURVEY = 260,
    /// Enable the specified standard MAVLink mode.
    ///           If the specified mode is not supported, the vehicle should ACK with MAV_RESULT_FAILED.
    ///           See https://mavlink.io/en/services/standard_modes.html
    MAV_CMD_DO_SET_STANDARD_MODE = 262,
    /// start running a mission
    MAV_CMD_MISSION_START = 300,
    /// Actuator testing command. This is similar to MAV_CMD_DO_MOTOR_TEST but operates on the level of output functions, i.e. it is possible to test Motor1 independent from which output it is configured on. Autopilots must NACK this command with MAV_RESULT_TEMPORARILY_REJECTED while armed.
    MAV_CMD_ACTUATOR_TEST = 310,
    /// Actuator configuration command.
    MAV_CMD_CONFIGURE_ACTUATOR = 311,
    /// Arms / Disarms a component
    MAV_CMD_COMPONENT_ARM_DISARM = 400,
    /// Instructs a target system to run pre-arm checks.
    ///           This allows preflight checks to be run on demand, which may be useful on systems that normally run them at low rate, or which do not trigger checks when the armable state might have changed.
    ///           This command should return MAV_RESULT_ACCEPTED if it will run the checks.
    ///           The results of the checks are usually then reported in SYS_STATUS messages (this is system-specific).
    ///           The command should return MAV_RESULT_TEMPORARILY_REJECTED if the system is already armed.
    MAV_CMD_RUN_PREARM_CHECKS = 401,
    /// Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the system: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light).
    MAV_CMD_ILLUMINATOR_ON_OFF = 405,
    /// Configures illuminator settings. An illuminator is a light source that is used for lighting up dark areas external to the system: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light).
    MAV_CMD_DO_ILLUMINATOR_CONFIGURE = 406,
    /// Request the home position from the vehicle.
    ///   The vehicle will ACK the command and then emit the HOME_POSITION message.
    MAV_CMD_GET_HOME_POSITION = 410,
    /// Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting.
    MAV_CMD_INJECT_FAILURE = 420,
    /// Starts receiver pairing.
    MAV_CMD_START_RX_PAIR = 500,
    /// Request the interval between messages for a particular MAVLink message ID.
    ///           The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message.
    MAV_CMD_GET_MESSAGE_INTERVAL = 510,
    /// Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM.
    MAV_CMD_SET_MESSAGE_INTERVAL = 511,
    /// Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL).
    MAV_CMD_REQUEST_MESSAGE = 512,
    /// Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message
    MAV_CMD_REQUEST_PROTOCOL_VERSION = 519,
    /// Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message
    MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES = 520,
    /// Request camera information (CAMERA_INFORMATION).
    MAV_CMD_REQUEST_CAMERA_INFORMATION = 521,
    /// Request camera settings (CAMERA_SETTINGS).
    MAV_CMD_REQUEST_CAMERA_SETTINGS = 522,
    /// Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage.
    MAV_CMD_REQUEST_STORAGE_INFORMATION = 525,
    /// Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage.
    MAV_CMD_STORAGE_FORMAT = 526,
    /// Request camera capture status (CAMERA_CAPTURE_STATUS)
    MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS = 527,
    /// Request flight information (FLIGHT_INFORMATION)
    MAV_CMD_REQUEST_FLIGHT_INFORMATION = 528,
    /// Reset all camera settings to Factory Default
    MAV_CMD_RESET_CAMERA_SETTINGS = 529,
    /// Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming.
    MAV_CMD_SET_CAMERA_MODE = 530,
    /// Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success).
    MAV_CMD_SET_CAMERA_ZOOM = 531,
    /// Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success).
    MAV_CMD_SET_CAMERA_FOCUS = 532,
    /// Set that a particular storage is the preferred location for saving photos, videos, and/or other media (e.g. to set that an SD card is used for storing videos).
    ///           There can only be one preferred save location for each particular media type: setting a media usage flag will clear/reset that same flag if set on any other storage.
    ///           If no flag is set the system should use its default storage.
    ///           A target system can choose to always use default storage, in which case it should ACK the command with MAV_RESULT_UNSUPPORTED.
    ///           A target system can choose to not allow a particular storage to be set as preferred storage, in which case it should ACK the command with MAV_RESULT_DENIED.
    MAV_CMD_SET_STORAGE_USAGE = 533,
    /// Set camera source. Changes the camera's active sources on cameras with multiple image sensors.
    MAV_CMD_SET_CAMERA_SOURCE = 534,
    /// Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG.
    MAV_CMD_JUMP_TAG = 600,
    /// Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number.
    MAV_CMD_DO_JUMP_TAG = 601,
    /// Set gimbal manager pitch/yaw setpoints (low rate command). It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: only the gimbal manager will react to this command - it will be ignored by a gimbal device. Use GIMBAL_MANAGER_SET_PITCHYAW if you need to stream pitch/yaw setpoints at higher rate.
    MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000,
    /// Gimbal configuration to set which sysid/compid is in primary and secondary control.
    MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE = 1001,
    /// Start image capture sequence. CAMERA_IMAGE_CAPTURED must be emitted after each capture.
    /// 
    ///           Param1 (id) may be used to specify the target camera: 0: all cameras, 1 to 6: autopilot-connected cameras, 7-255: MAVLink camera component ID.
    ///           It is needed in order to target specific cameras connected to the autopilot, or specific sensors in a multi-sensor camera (neither of which have a distinct MAVLink component ID).
    ///           It is also needed to specify the target camera in missions.
    /// 
    ///           When used in a mission, an autopilot should execute the MAV_CMD for a specified local camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink camera (param1 = 7 - 255), setting the command's target_component as the param1 value (and setting param1 in the command to zero).
    ///           If the param1 is 0 the autopilot should do both.
    /// 
    ///           When sent in a command the target MAVLink address is set using target_component.
    ///           If addressed specifically to an autopilot: param1 should be used in the same way as it is for missions (though command should NACK with MAV_RESULT_DENIED if a specified local camera does not exist).
    ///           If addressed to a MAVLink camera, param 1 can be used to address all cameras (0), or to separately address 1 to 7 individual sensors. Other values should be NACKed with MAV_RESULT_DENIED.
    ///           If the command is broadcast (target_component is 0) then param 1 should be set to 0 (any other value should be NACKED with MAV_RESULT_DENIED). An autopilot would trigger any local cameras and forward the command to all channels.
    MAV_CMD_IMAGE_START_CAPTURE = 2000,
    /// Stop image capture sequence.
    /// 
    ///           Param1 (id) may be used to specify the target camera: 0: all cameras, 1 to 6: autopilot-connected cameras, 7-255: MAVLink camera component ID.
    ///           It is needed in order to target specific cameras connected to the autopilot, or specific sensors in a multi-sensor camera (neither of which have a distinct MAVLink component ID).
    ///           It is also needed to specify the target camera in missions.
    /// 
    ///           When used in a mission, an autopilot should execute the MAV_CMD for a specified local camera (param1 = 1-6), or resend it as a command if it is intended for a MAVLink camera (param1 = 7 - 255), setting the command's target_component as the param1 value (and setting param1 in the command to zero).
    ///           If the param1 is 0 the autopilot should do both.
    /// 
    ///           When sent in a command the target MAVLink address is set using target_component.
    ///           If addressed specifically to an autopilot: param1 should be used in the same way as it is for missions (though command should NACK with MAV_RESULT_DENIED if a specified local camera does not exist).
    ///           If addressed to a MAVLink camera, param1 can be used to address all cameras (0), or to separately address 1 to 7 individual sensors. Other values should be NACKed with MAV_RESULT_DENIED.
    ///           If the command is broadcast (target_component is 0) then param 1 should be set to 0 (any other value should be NACKED with MAV_RESULT_DENIED). An autopilot would trigger any local cameras and forward the command to all channels.
    MAV_CMD_IMAGE_STOP_CAPTURE = 2001,
    /// Re-request a CAMERA_IMAGE_CAPTURED message.
    MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE = 2002,
    /// Enable or disable on-board camera triggering system.
    MAV_CMD_DO_TRIGGER_CONTROL = 2003,
    /// If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking.
    MAV_CMD_CAMERA_TRACK_POINT = 2004,
    /// If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking.
    MAV_CMD_CAMERA_TRACK_RECTANGLE = 2005,
    /// Stops ongoing tracking.
    MAV_CMD_CAMERA_STOP_TRACKING = 2010,
    /// Starts video capture (recording).
    MAV_CMD_VIDEO_START_CAPTURE = 2500,
    /// Stop the current video capture (recording).
    MAV_CMD_VIDEO_STOP_CAPTURE = 2501,
    /// Start video streaming
    MAV_CMD_VIDEO_START_STREAMING = 2502,
    /// Stop the given video stream
    MAV_CMD_VIDEO_STOP_STREAMING = 2503,
    /// Request video stream information (VIDEO_STREAM_INFORMATION)
    MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION = 2504,
    /// Request video stream status (VIDEO_STREAM_STATUS)
    MAV_CMD_REQUEST_VIDEO_STREAM_STATUS = 2505,
    /// Request to start streaming logging data over MAVLink (see also LOGGING_DATA message)
    MAV_CMD_LOGGING_START = 2510,
    /// Request to stop streaming log data over MAVLink
    MAV_CMD_LOGGING_STOP = 2511,
    MAV_CMD_AIRFRAME_CONFIGURATION = 2520,
    /// Request to start/stop transmitting over the high latency telemetry
    MAV_CMD_CONTROL_HIGH_LATENCY = 2600,
    /// Create a panorama at the current position
    MAV_CMD_PANORAMA_CREATE = 2800,
    /// Request VTOL transition
    MAV_CMD_DO_VTOL_TRANSITION = 3000,
    /// Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request.
    /// If approved the COMMAND_ACK message progress field should be set with period of time that this authorization is valid in seconds.
    /// If the authorization is denied COMMAND_ACK.result_param2 should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
    MAV_CMD_ARM_AUTHORIZATION_REQUEST = 3001,
    /// This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
    MAV_CMD_SET_GUIDED_SUBMODE_STANDARD = 4000,
    /// This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
    MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE = 4001,
    /// Fence return point (there can only be one such point in a geofence definition). If rally points are supported they should be used instead.
    MAV_CMD_NAV_FENCE_RETURN_POINT = 5000,
    /// Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
    ///           The vertices for a polygon must be sent sequentially, each with param1 set to the total number of vertices in the polygon.
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION = 5001,
    /// Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
    ///           The vertices for a polygon must be sent sequentially, each with param1 set to the total number of vertices in the polygon.
    MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION = 5002,
    /// Circular fence area. The vehicle must stay inside this area.
    MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION = 5003,
    /// Circular fence area. The vehicle must stay outside this area.
    MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION = 5004,
    /// Rally point. You can have multiple rally points defined.
    MAV_CMD_NAV_RALLY_POINT = 5100,
    /// Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages.
    MAV_CMD_UAVCAN_GET_NODE_INFO = 5200,
    /// Change state of safety switch.
    MAV_CMD_DO_SET_SAFETY_SWITCH_STATE = 5300,
    /// Trigger the start of an ADSB-out IDENT. This should only be used when requested to do so by an Air Traffic Controller in controlled airspace. This starts the IDENT which is then typically held for 18 seconds by the hardware per the Mode A, C, and S transponder spec.
    MAV_CMD_DO_ADSB_OUT_IDENT = 10001,
    /// Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity.
    MAV_CMD_PAYLOAD_PREPARE_DEPLOY = 30001,
    /// Control the payload deployment.
    MAV_CMD_PAYLOAD_CONTROL_DEPLOY = 30002,
    /// Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location.
    MAV_CMD_FIXED_MAG_CAL_YAW = 42006,
    /// Command to operate winch.
    MAV_CMD_DO_WINCH = 42600,
    /// Provide an external position estimate for use when dead-reckoning. This is meant to be used for occasional position resets that may be provided by a external system such as a remote pilot using landmarks over a video link.
    MAV_CMD_EXTERNAL_POSITION_ESTIMATE = 43003,
    /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    MAV_CMD_WAYPOINT_USER_1 = 31000,
    /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    MAV_CMD_WAYPOINT_USER_2 = 31001,
    /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    MAV_CMD_WAYPOINT_USER_3 = 31002,
    /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    MAV_CMD_WAYPOINT_USER_4 = 31003,
    /// User defined waypoint item. Ground Station will show the Vehicle as flying through this item.
    MAV_CMD_WAYPOINT_USER_5 = 31004,
    /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_1 = 31005,
    /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_2 = 31006,
    /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_3 = 31007,
    /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_4 = 31008,
    /// User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item.
    MAV_CMD_SPATIAL_USER_5 = 31009,
    /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_1 = 31010,
    /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_2 = 31011,
    /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_3 = 31012,
    /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_4 = 31013,
    /// User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item.
    MAV_CMD_USER_5 = 31014,
    /// Request forwarding of CAN packets from the given CAN bus to this component. CAN Frames are sent using CAN_FRAME and CANFD_FRAME messages
    MAV_CMD_CAN_FORWARD = 32000,
};

/// Actuator output function. Values greater or equal to 1000 are autopilot-specific.
pub const ACTUATOR_OUTPUT_FUNCTION = enum(u32) {
    /// No function (disabled).
    ACTUATOR_OUTPUT_FUNCTION_NONE = 0,
    /// Motor 1
    ACTUATOR_OUTPUT_FUNCTION_MOTOR1 = 1,
    /// Motor 2
    ACTUATOR_OUTPUT_FUNCTION_MOTOR2 = 2,
    /// Motor 3
    ACTUATOR_OUTPUT_FUNCTION_MOTOR3 = 3,
    /// Motor 4
    ACTUATOR_OUTPUT_FUNCTION_MOTOR4 = 4,
    /// Motor 5
    ACTUATOR_OUTPUT_FUNCTION_MOTOR5 = 5,
    /// Motor 6
    ACTUATOR_OUTPUT_FUNCTION_MOTOR6 = 6,
    /// Motor 7
    ACTUATOR_OUTPUT_FUNCTION_MOTOR7 = 7,
    /// Motor 8
    ACTUATOR_OUTPUT_FUNCTION_MOTOR8 = 8,
    /// Motor 9
    ACTUATOR_OUTPUT_FUNCTION_MOTOR9 = 9,
    /// Motor 10
    ACTUATOR_OUTPUT_FUNCTION_MOTOR10 = 10,
    /// Motor 11
    ACTUATOR_OUTPUT_FUNCTION_MOTOR11 = 11,
    /// Motor 12
    ACTUATOR_OUTPUT_FUNCTION_MOTOR12 = 12,
    /// Motor 13
    ACTUATOR_OUTPUT_FUNCTION_MOTOR13 = 13,
    /// Motor 14
    ACTUATOR_OUTPUT_FUNCTION_MOTOR14 = 14,
    /// Motor 15
    ACTUATOR_OUTPUT_FUNCTION_MOTOR15 = 15,
    /// Motor 16
    ACTUATOR_OUTPUT_FUNCTION_MOTOR16 = 16,
    /// Servo 1
    ACTUATOR_OUTPUT_FUNCTION_SERVO1 = 33,
    /// Servo 2
    ACTUATOR_OUTPUT_FUNCTION_SERVO2 = 34,
    /// Servo 3
    ACTUATOR_OUTPUT_FUNCTION_SERVO3 = 35,
    /// Servo 4
    ACTUATOR_OUTPUT_FUNCTION_SERVO4 = 36,
    /// Servo 5
    ACTUATOR_OUTPUT_FUNCTION_SERVO5 = 37,
    /// Servo 6
    ACTUATOR_OUTPUT_FUNCTION_SERVO6 = 38,
    /// Servo 7
    ACTUATOR_OUTPUT_FUNCTION_SERVO7 = 39,
    /// Servo 8
    ACTUATOR_OUTPUT_FUNCTION_SERVO8 = 40,
    /// Servo 9
    ACTUATOR_OUTPUT_FUNCTION_SERVO9 = 41,
    /// Servo 10
    ACTUATOR_OUTPUT_FUNCTION_SERVO10 = 42,
    /// Servo 11
    ACTUATOR_OUTPUT_FUNCTION_SERVO11 = 43,
    /// Servo 12
    ACTUATOR_OUTPUT_FUNCTION_SERVO12 = 44,
    /// Servo 13
    ACTUATOR_OUTPUT_FUNCTION_SERVO13 = 45,
    /// Servo 14
    ACTUATOR_OUTPUT_FUNCTION_SERVO14 = 46,
    /// Servo 15
    ACTUATOR_OUTPUT_FUNCTION_SERVO15 = 47,
    /// Servo 16
    ACTUATOR_OUTPUT_FUNCTION_SERVO16 = 48,
};

pub const MAV_ODID_STATUS = enum(u8) {
    /// The status of the (UA) Unmanned Aircraft is undefined.
    MAV_ODID_STATUS_UNDECLARED = 0,
    /// The UA is on the ground.
    MAV_ODID_STATUS_GROUND = 1,
    /// The UA is in the air.
    MAV_ODID_STATUS_AIRBORNE = 2,
    /// The UA is having an emergency.
    MAV_ODID_STATUS_EMERGENCY = 3,
    /// The remote ID system is failing or unreliable in some way.
    MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE = 4,
};

/// These encode the sensors whose status is sent as part of the SYS_STATUS message in the extended fields.
pub const MAV_SYS_STATUS_SENSOR_EXTENDED = enum(u32) {
    /// 0x01 Recovery system (parachute, balloon, retracts etc)
    MAV_SYS_STATUS_RECOVERY_SYSTEM = 1,
};

/// These flags encode the cellular network status
pub const CELLULAR_STATUS_FLAG = enum(u8) {
    /// State unknown or not reportable.
    CELLULAR_STATUS_FLAG_UNKNOWN = 0,
    /// Modem is unusable
    CELLULAR_STATUS_FLAG_FAILED = 1,
    /// Modem is being initialized
    CELLULAR_STATUS_FLAG_INITIALIZING = 2,
    /// Modem is locked
    CELLULAR_STATUS_FLAG_LOCKED = 3,
    /// Modem is not enabled and is powered down
    CELLULAR_STATUS_FLAG_DISABLED = 4,
    /// Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state
    CELLULAR_STATUS_FLAG_DISABLING = 5,
    /// Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state
    CELLULAR_STATUS_FLAG_ENABLING = 6,
    /// Modem is enabled and powered on but not registered with a network provider and not available for data connections
    CELLULAR_STATUS_FLAG_ENABLED = 7,
    /// Modem is searching for a network provider to register
    CELLULAR_STATUS_FLAG_SEARCHING = 8,
    /// Modem is registered with a network provider, and data connections and messaging may be available for use
    CELLULAR_STATUS_FLAG_REGISTERED = 9,
    /// Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
    CELLULAR_STATUS_FLAG_DISCONNECTING = 10,
    /// Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
    CELLULAR_STATUS_FLAG_CONNECTING = 11,
    /// One or more packet data bearers is active and connected
    CELLULAR_STATUS_FLAG_CONNECTED = 12,
};

/// Gripper actions.
pub const GRIPPER_ACTIONS = enum(u32) {
    /// Gripper release cargo.
    GRIPPER_ACTION_RELEASE = 0,
    /// Gripper grab onto cargo.
    GRIPPER_ACTION_GRAB = 1,
};

/// Cellular network radio type
pub const CELLULAR_NETWORK_RADIO_TYPE = enum(u8) {
    CELLULAR_NETWORK_RADIO_TYPE_NONE = 0,
    CELLULAR_NETWORK_RADIO_TYPE_GSM = 1,
    CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2,
    CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3,
    CELLULAR_NETWORK_RADIO_TYPE_LTE = 4,
};

pub const CAN_FILTER_OP = enum(u8) {
    CAN_FILTER_REPLACE = 0,
    CAN_FILTER_ADD = 1,
    CAN_FILTER_REMOVE = 2,
};

/// Camera tracking target data (shows where tracked target is within image)
pub const CAMERA_TRACKING_TARGET_DATA = enum(u8) {
    /// Target data embedded in image data (proprietary)
    CAMERA_TRACKING_TARGET_DATA_EMBEDDED = 1,
    /// Target data rendered in image
    CAMERA_TRACKING_TARGET_DATA_RENDERED = 2,
    /// Target data within status message (Point or Rectangle)
    CAMERA_TRACKING_TARGET_DATA_IN_STATUS = 4,
};

/// Speed setpoint types used in MAV_CMD_DO_CHANGE_SPEED
pub const SPEED_TYPE = enum(u32) {
    /// Airspeed
    SPEED_TYPE_AIRSPEED = 0,
    /// Groundspeed
    SPEED_TYPE_GROUNDSPEED = 1,
    /// Climb speed
    SPEED_TYPE_CLIMB_SPEED = 2,
    /// Descent speed
    SPEED_TYPE_DESCENT_SPEED = 3,
};

/// Flags for high level gimbal manager operation The first 16 bits are identical to the GIMBAL_DEVICE_FLAGS.
pub const GIMBAL_MANAGER_FLAGS = enum(u32) {
    /// Based on GIMBAL_DEVICE_FLAGS_RETRACT.
    GIMBAL_MANAGER_FLAGS_RETRACT = 1,
    /// Based on GIMBAL_DEVICE_FLAGS_NEUTRAL.
    GIMBAL_MANAGER_FLAGS_NEUTRAL = 2,
    /// Based on GIMBAL_DEVICE_FLAGS_ROLL_LOCK.
    GIMBAL_MANAGER_FLAGS_ROLL_LOCK = 4,
    /// Based on GIMBAL_DEVICE_FLAGS_PITCH_LOCK.
    GIMBAL_MANAGER_FLAGS_PITCH_LOCK = 8,
    /// Based on GIMBAL_DEVICE_FLAGS_YAW_LOCK.
    GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16,
    /// Based on GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME.
    GIMBAL_MANAGER_FLAGS_YAW_IN_VEHICLE_FRAME = 32,
    /// Based on GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME.
    GIMBAL_MANAGER_FLAGS_YAW_IN_EARTH_FRAME = 64,
    /// Based on GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME.
    GIMBAL_MANAGER_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME = 128,
    /// Based on GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE.
    GIMBAL_MANAGER_FLAGS_RC_EXCLUSIVE = 256,
    /// Based on GIMBAL_DEVICE_FLAGS_RC_MIXED.
    GIMBAL_MANAGER_FLAGS_RC_MIXED = 512,
};

/// Defines how throttle value is represented in MAV_CMD_DO_MOTOR_TEST.
pub const MOTOR_TEST_THROTTLE_TYPE = enum(u32) {
    /// Throttle as a percentage (0 ~ 100)
    MOTOR_TEST_THROTTLE_PERCENT = 0,
    /// Throttle as an absolute PWM value (normally in range of 1000~2000).
    MOTOR_TEST_THROTTLE_PWM = 1,
    /// Throttle pass-through from pilot's transmitter.
    MOTOR_TEST_THROTTLE_PILOT = 2,
    /// Per-motor compass calibration test.
    MOTOR_TEST_COMPASS_CAL = 3,
};

/// Video stream encodings
pub const VIDEO_STREAM_ENCODING = enum(u8) {
    /// Stream encoding is unknown
    VIDEO_STREAM_ENCODING_UNKNOWN = 0,
    /// Stream encoding is H.264
    VIDEO_STREAM_ENCODING_H264 = 1,
    /// Stream encoding is H.265
    VIDEO_STREAM_ENCODING_H265 = 2,
};

/// Result from PARAM_EXT_SET message.
pub const PARAM_ACK = enum(u8) {
    /// Parameter value ACCEPTED and SET
    PARAM_ACK_ACCEPTED = 0,
    /// Parameter value UNKNOWN/UNSUPPORTED
    PARAM_ACK_VALUE_UNSUPPORTED = 1,
    /// Parameter failed to set
    PARAM_ACK_FAILED = 2,
    /// Parameter value received but not yet set/accepted. A subsequent PARAM_EXT_ACK with the final result will follow once operation is completed. This is returned immediately for parameters that take longer to set, indicating that the the parameter was received and does not need to be resent.
    PARAM_ACK_IN_PROGRESS = 3,
};

/// Gimbal device (low level) error flags (bitmap, 0 means no error)
pub const GIMBAL_DEVICE_ERROR_FLAGS = enum(u32) {
    /// Gimbal device is limited by hardware roll limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_ROLL_LIMIT = 1,
    /// Gimbal device is limited by hardware pitch limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_PITCH_LIMIT = 2,
    /// Gimbal device is limited by hardware yaw limit.
    GIMBAL_DEVICE_ERROR_FLAGS_AT_YAW_LIMIT = 4,
    /// There is an error with the gimbal encoders.
    GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR = 8,
    /// There is an error with the gimbal power source.
    GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR = 16,
    /// There is an error with the gimbal motors.
    GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR = 32,
    /// There is an error with the gimbal's software.
    GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR = 64,
    /// There is an error with the gimbal's communication.
    GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR = 128,
    /// Gimbal device is currently calibrating.
    GIMBAL_DEVICE_ERROR_FLAGS_CALIBRATION_RUNNING = 256,
    /// Gimbal device is not assigned to a gimbal manager.
    GIMBAL_DEVICE_ERROR_FLAGS_NO_MANAGER = 512,
};

/// Possible responses from a WIFI_CONFIG_AP message.
pub const WIFI_CONFIG_AP_RESPONSE = enum(i8) {
    /// Undefined response. Likely an indicative of a system that doesn't support this request.
    WIFI_CONFIG_AP_RESPONSE_UNDEFINED = 0,
    /// Changes accepted.
    WIFI_CONFIG_AP_RESPONSE_ACCEPTED = 1,
    /// Changes rejected.
    WIFI_CONFIG_AP_RESPONSE_REJECTED = 2,
    /// Invalid Mode.
    WIFI_CONFIG_AP_RESPONSE_MODE_ERROR = 3,
    /// Invalid SSID.
    WIFI_CONFIG_AP_RESPONSE_SSID_ERROR = 4,
    /// Invalid Password.
    WIFI_CONFIG_AP_RESPONSE_PASSWORD_ERROR = 5,
};

/// Possible actions an aircraft can take to avoid a collision.
pub const MAV_COLLISION_ACTION = enum(u8) {
    /// Ignore any potential collisions
    MAV_COLLISION_ACTION_NONE = 0,
    /// Report potential collision
    MAV_COLLISION_ACTION_REPORT = 1,
    /// Ascend or Descend to avoid threat
    MAV_COLLISION_ACTION_ASCEND_OR_DESCEND = 2,
    /// Move horizontally to avoid threat
    MAV_COLLISION_ACTION_MOVE_HORIZONTALLY = 3,
    /// Aircraft to move perpendicular to the collision's velocity vector
    MAV_COLLISION_ACTION_MOVE_PERPENDICULAR = 4,
    /// Aircraft to fly directly back to its launch point
    MAV_COLLISION_ACTION_RTL = 5,
    /// Aircraft to stop in place
    MAV_COLLISION_ACTION_HOVER = 6,
};

/// SERIAL_CONTROL device types
pub const SERIAL_CONTROL_DEV = enum(u8) {
    /// First telemetry port
    SERIAL_CONTROL_DEV_TELEM1 = 0,
    /// Second telemetry port
    SERIAL_CONTROL_DEV_TELEM2 = 1,
    /// First GPS port
    SERIAL_CONTROL_DEV_GPS1 = 2,
    /// Second GPS port
    SERIAL_CONTROL_DEV_GPS2 = 3,
    /// system shell
    SERIAL_CONTROL_DEV_SHELL = 10,
    /// SERIAL0
    SERIAL_CONTROL_SERIAL0 = 100,
    /// SERIAL1
    SERIAL_CONTROL_SERIAL1 = 101,
    /// SERIAL2
    SERIAL_CONTROL_SERIAL2 = 102,
    /// SERIAL3
    SERIAL_CONTROL_SERIAL3 = 103,
    /// SERIAL4
    SERIAL_CONTROL_SERIAL4 = 104,
    /// SERIAL5
    SERIAL_CONTROL_SERIAL5 = 105,
    /// SERIAL6
    SERIAL_CONTROL_SERIAL6 = 106,
    /// SERIAL7
    SERIAL_CONTROL_SERIAL7 = 107,
    /// SERIAL8
    SERIAL_CONTROL_SERIAL8 = 108,
    /// SERIAL9
    SERIAL_CONTROL_SERIAL9 = 109,
};

/// Flags to indicate the type of storage.
pub const STORAGE_TYPE = enum(u8) {
    /// Storage type is not known.
    STORAGE_TYPE_UNKNOWN = 0,
    /// Storage type is USB device.
    STORAGE_TYPE_USB_STICK = 1,
    /// Storage type is SD card.
    STORAGE_TYPE_SD = 2,
    /// Storage type is microSD card.
    STORAGE_TYPE_MICROSD = 3,
    /// Storage type is CFast.
    STORAGE_TYPE_CF = 4,
    /// Storage type is CFexpress.
    STORAGE_TYPE_CFE = 5,
    /// Storage type is XQD.
    STORAGE_TYPE_XQD = 6,
    /// Storage type is HD mass storage type.
    STORAGE_TYPE_HD = 7,
    /// Storage type is other, not listed type.
    STORAGE_TYPE_OTHER = 254,
};

/// Enumeration of landed detector states
pub const MAV_LANDED_STATE = enum(u8) {
    /// MAV landed state is unknown
    MAV_LANDED_STATE_UNDEFINED = 0,
    /// MAV is landed (on ground)
    MAV_LANDED_STATE_ON_GROUND = 1,
    /// MAV is in air
    MAV_LANDED_STATE_IN_AIR = 2,
    /// MAV currently taking off
    MAV_LANDED_STATE_TAKEOFF = 3,
    /// MAV currently landing
    MAV_LANDED_STATE_LANDING = 4,
};

/// Action required when performing CMD_PREFLIGHT_STORAGE
pub const MAV_PREFLIGHT_STORAGE_ACTION = enum(u32) {
    /// Read all parameters from storage
    MAV_PFS_CMD_READ_ALL = 0,
    /// Write all parameters to storage
    MAV_PFS_CMD_WRITE_ALL = 1,
    /// Clear all  parameters in storage
    MAV_PFS_CMD_CLEAR_ALL = 2,
    /// Read specific parameters from storage
    MAV_PFS_CMD_READ_SPECIFIC = 3,
    /// Write specific parameters to storage
    MAV_PFS_CMD_WRITE_SPECIFIC = 4,
    /// Clear specific parameters in storage
    MAV_PFS_CMD_CLEAR_SPECIFIC = 5,
    /// do nothing
    MAV_PFS_CMD_DO_NOTHING = 6,
};

/// Winch actions.
pub const WINCH_ACTIONS = enum(u32) {
    /// Allow motor to freewheel.
    WINCH_RELAXED = 0,
    /// Wind or unwind specified length of line, optionally using specified rate.
    WINCH_RELATIVE_LENGTH_CONTROL = 1,
    /// Wind or unwind line at specified rate.
    WINCH_RATE_CONTROL = 2,
    /// Perform the locking sequence to relieve motor while in the fully retracted position. Only action and instance command parameters are used, others are ignored.
    WINCH_LOCK = 3,
    /// Sequence of drop, slow down, touch down, reel up, lock. Only action and instance command parameters are used, others are ignored.
    WINCH_DELIVER = 4,
    /// Engage motor and hold current position. Only action and instance command parameters are used, others are ignored.
    WINCH_HOLD = 5,
    /// Return the reel to the fully retracted position. Only action and instance command parameters are used, others are ignored.
    WINCH_RETRACT = 6,
    /// Load the reel with line. The winch will calculate the total loaded length and stop when the tension exceeds a threshold. Only action and instance command parameters are used, others are ignored.
    WINCH_LOAD_LINE = 7,
    /// Spool out the entire length of the line. Only action and instance command parameters are used, others are ignored.
    WINCH_ABANDON_LINE = 8,
    /// Spools out just enough to present the hook to the user to load the payload. Only action and instance command parameters are used, others are ignored
    WINCH_LOAD_PAYLOAD = 9,
};

/// Fuel types for use in FUEL_TYPE. Fuel types specify the units for the maximum, available and consumed fuel, and for the flow rates.
pub const MAV_FUEL_TYPE = enum(u32) {
    /// Not specified. Fuel levels are normalized (i.e. maximum is 1, and other levels are relative to 1).
    MAV_FUEL_TYPE_UNKNOWN = 0,
    /// A generic liquid fuel. Fuel levels are in millilitres (ml). Fuel rates are in millilitres/second.
    MAV_FUEL_TYPE_LIQUID = 1,
    /// A gas tank. Fuel levels are in kilo-Pascal (kPa), and flow rates are in milliliters per second (ml/s).
    MAV_FUEL_TYPE_GAS = 2,
};

/// Modes of illuminator
pub const ILLUMINATOR_MODE = enum(u8) {
    /// Illuminator mode is not specified/unknown
    ILLUMINATOR_MODE_UNKNOWN = 0,
    /// Illuminator behavior is controlled by MAV_CMD_DO_ILLUMINATOR_CONFIGURE settings
    ILLUMINATOR_MODE_INTERNAL_CONTROL = 1,
    /// Illuminator behavior is controlled by external factors: e.g. an external hardware signal
    ILLUMINATOR_MODE_EXTERNAL_SYNC = 2,
};

/// Component ids (values) for the different types and instances of onboard hardware/software that might make up a MAVLink system (autopilot, cameras, servos, GPS systems, avoidance systems etc.).
///       Components must use the appropriate ID in their source address when sending messages. Components can also use IDs to determine if they are the intended recipient of an incoming message. The MAV_COMP_ID_ALL value is used to indicate messages that must be processed by all components.
///       When creating new entries, components that can have multiple instances (e.g. cameras, servos etc.) should be allocated sequential values. An appropriate number of values should be left free after these components to allow the number of instances to be expanded.
pub const MAV_COMPONENT = enum(u32) {
    /// Target id (target_component) used to broadcast messages to all components of the receiving system. Components should attempt to process messages with this component ID and forward to components on any other interfaces. Note: This is not a valid *source* component id for a message.
    MAV_COMP_ID_ALL = 0,
    /// System flight controller component ("autopilot"). Only one autopilot is expected in a particular system.
    MAV_COMP_ID_AUTOPILOT1 = 1,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER1 = 25,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER2 = 26,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER3 = 27,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER4 = 28,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER5 = 29,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER6 = 30,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER7 = 31,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER8 = 32,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER9 = 33,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER10 = 34,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER11 = 35,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER12 = 36,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER13 = 37,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER14 = 38,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER15 = 39,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER16 = 40,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER17 = 41,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER18 = 42,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER19 = 43,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER20 = 44,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER21 = 45,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER22 = 46,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER23 = 47,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER24 = 48,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER25 = 49,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER26 = 50,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER27 = 51,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER28 = 52,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER29 = 53,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER30 = 54,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER31 = 55,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER32 = 56,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER33 = 57,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER34 = 58,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER35 = 59,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER36 = 60,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER37 = 61,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER38 = 62,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER39 = 63,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER40 = 64,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER41 = 65,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER42 = 66,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER43 = 67,
    /// Telemetry radio (e.g. SiK radio, or other component that emits RADIO_STATUS messages).
    MAV_COMP_ID_TELEMETRY_RADIO = 68,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER45 = 69,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER46 = 70,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER47 = 71,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER48 = 72,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER49 = 73,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER50 = 74,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER51 = 75,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER52 = 76,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER53 = 77,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER54 = 78,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER55 = 79,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER56 = 80,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER57 = 81,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER58 = 82,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER59 = 83,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER60 = 84,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER61 = 85,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER62 = 86,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER63 = 87,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER64 = 88,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER65 = 89,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER66 = 90,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER67 = 91,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER68 = 92,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER69 = 93,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER70 = 94,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER71 = 95,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER72 = 96,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER73 = 97,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER74 = 98,
    /// Id for a component on privately managed MAVLink network. Can be used for any purpose but may not be published by components outside of the private network.
    MAV_COMP_ID_USER75 = 99,
    /// Camera #1.
    MAV_COMP_ID_CAMERA = 100,
    /// Camera #2.
    MAV_COMP_ID_CAMERA2 = 101,
    /// Camera #3.
    MAV_COMP_ID_CAMERA3 = 102,
    /// Camera #4.
    MAV_COMP_ID_CAMERA4 = 103,
    /// Camera #5.
    MAV_COMP_ID_CAMERA5 = 104,
    /// Camera #6.
    MAV_COMP_ID_CAMERA6 = 105,
    /// Servo #1.
    MAV_COMP_ID_SERVO1 = 140,
    /// Servo #2.
    MAV_COMP_ID_SERVO2 = 141,
    /// Servo #3.
    MAV_COMP_ID_SERVO3 = 142,
    /// Servo #4.
    MAV_COMP_ID_SERVO4 = 143,
    /// Servo #5.
    MAV_COMP_ID_SERVO5 = 144,
    /// Servo #6.
    MAV_COMP_ID_SERVO6 = 145,
    /// Servo #7.
    MAV_COMP_ID_SERVO7 = 146,
    /// Servo #8.
    MAV_COMP_ID_SERVO8 = 147,
    /// Servo #9.
    MAV_COMP_ID_SERVO9 = 148,
    /// Servo #10.
    MAV_COMP_ID_SERVO10 = 149,
    /// Servo #11.
    MAV_COMP_ID_SERVO11 = 150,
    /// Servo #12.
    MAV_COMP_ID_SERVO12 = 151,
    /// Servo #13.
    MAV_COMP_ID_SERVO13 = 152,
    /// Servo #14.
    MAV_COMP_ID_SERVO14 = 153,
    /// Gimbal #1.
    MAV_COMP_ID_GIMBAL = 154,
    /// Logging component.
    MAV_COMP_ID_LOG = 155,
    /// Automatic Dependent Surveillance-Broadcast (ADS-B) component.
    MAV_COMP_ID_ADSB = 156,
    /// On Screen Display (OSD) devices for video links.
    MAV_COMP_ID_OSD = 157,
    /// Generic autopilot peripheral component ID. Meant for devices that do not implement the parameter microservice.
    MAV_COMP_ID_PERIPHERAL = 158,
    /// Gimbal ID for QX1.
    MAV_COMP_ID_QX1_GIMBAL = 159,
    /// FLARM collision alert component.
    MAV_COMP_ID_FLARM = 160,
    /// Parachute component.
    MAV_COMP_ID_PARACHUTE = 161,
    /// Winch component.
    MAV_COMP_ID_WINCH = 169,
    /// Gimbal #2.
    MAV_COMP_ID_GIMBAL2 = 171,
    /// Gimbal #3.
    MAV_COMP_ID_GIMBAL3 = 172,
    /// Gimbal #4
    MAV_COMP_ID_GIMBAL4 = 173,
    /// Gimbal #5.
    MAV_COMP_ID_GIMBAL5 = 174,
    /// Gimbal #6.
    MAV_COMP_ID_GIMBAL6 = 175,
    /// Battery #1.
    MAV_COMP_ID_BATTERY = 180,
    /// Battery #2.
    MAV_COMP_ID_BATTERY2 = 181,
    /// CAN over MAVLink client.
    MAV_COMP_ID_MAVCAN = 189,
    /// Component that can generate/supply a mission flight plan (e.g. GCS or developer API).
    MAV_COMP_ID_MISSIONPLANNER = 190,
    /// Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
    MAV_COMP_ID_ONBOARD_COMPUTER = 191,
    /// Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
    MAV_COMP_ID_ONBOARD_COMPUTER2 = 192,
    /// Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
    MAV_COMP_ID_ONBOARD_COMPUTER3 = 193,
    /// Component that lives on the onboard computer (companion computer) and has some generic functionalities, such as settings system parameters and monitoring the status of some processes that don't directly speak mavlink and so on.
    MAV_COMP_ID_ONBOARD_COMPUTER4 = 194,
    /// Component that finds an optimal path between points based on a certain constraint (e.g. minimum snap, shortest path, cost, etc.).
    MAV_COMP_ID_PATHPLANNER = 195,
    /// Component that plans a collision free path between two points.
    MAV_COMP_ID_OBSTACLE_AVOIDANCE = 196,
    /// Component that provides position estimates using VIO techniques.
    MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197,
    /// Component that manages pairing of vehicle and GCS.
    MAV_COMP_ID_PAIRING_MANAGER = 198,
    /// Inertial Measurement Unit (IMU) #1.
    MAV_COMP_ID_IMU = 200,
    /// Inertial Measurement Unit (IMU) #2.
    MAV_COMP_ID_IMU_2 = 201,
    /// Inertial Measurement Unit (IMU) #3.
    MAV_COMP_ID_IMU_3 = 202,
    /// GPS #1.
    MAV_COMP_ID_GPS = 220,
    /// GPS #2.
    MAV_COMP_ID_GPS2 = 221,
    /// Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
    MAV_COMP_ID_ODID_TXRX_1 = 236,
    /// Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
    MAV_COMP_ID_ODID_TXRX_2 = 237,
    /// Open Drone ID transmitter/receiver (Bluetooth/WiFi/Internet).
    MAV_COMP_ID_ODID_TXRX_3 = 238,
    /// Component to bridge MAVLink to UDP (i.e. from a UART).
    MAV_COMP_ID_UDP_BRIDGE = 240,
    /// Component to bridge to UART (i.e. from UDP).
    MAV_COMP_ID_UART_BRIDGE = 241,
    /// Component handling TUNNEL messages (e.g. vendor specific GUI of a component).
    MAV_COMP_ID_TUNNEL_NODE = 242,
    /// Illuminator
    MAV_COMP_ID_ILLUMINATOR = 243,
    /// Deprecated, don't use. Component for handling system messages (e.g. to ARM, takeoff, etc.).
    MAV_COMP_ID_SYSTEM_CONTROL = 250,
};

/// Flags used in HIL_ACTUATOR_CONTROLS message.
pub const HIL_ACTUATOR_CONTROLS_FLAGS = enum(u64) {
    /// Simulation is using lockstep
    HIL_ACTUATOR_CONTROLS_FLAGS_LOCKSTEP = 1,
};

/// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b00000000 indicates that none of the setpoint dimensions should be ignored.
pub const ATTITUDE_TARGET_TYPEMASK = enum(u8) {
    /// Ignore body roll rate
    ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE = 1,
    /// Ignore body pitch rate
    ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE = 2,
    /// Ignore body yaw rate
    ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE = 4,
    /// Use 3D body thrust setpoint instead of throttle
    ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET = 32,
    /// Ignore throttle
    ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE = 64,
    /// Ignore attitude
    ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE = 128,
};

/// Enumeration of VTOL states
pub const MAV_VTOL_STATE = enum(u8) {
    /// MAV is not configured as VTOL
    MAV_VTOL_STATE_UNDEFINED = 0,
    /// VTOL is in transition from multicopter to fixed-wing
    MAV_VTOL_STATE_TRANSITION_TO_FW = 1,
    /// VTOL is in transition from fixed-wing to multicopter
    MAV_VTOL_STATE_TRANSITION_TO_MC = 2,
    /// VTOL is in multicopter state
    MAV_VTOL_STATE_MC = 3,
    /// VTOL is in fixed-wing state
    MAV_VTOL_STATE_FW = 4,
};

/// Zoom types for MAV_CMD_SET_CAMERA_ZOOM
pub const CAMERA_ZOOM_TYPE = enum(u32) {
    /// Zoom one step increment (-1 for wide, 1 for tele)
    ZOOM_TYPE_STEP = 0,
    /// Continuous normalized zoom in/out rate until stopped. Range -1..1, negative: wide, positive: narrow/tele, 0 to stop zooming. Other values should be clipped to the range.
    ZOOM_TYPE_CONTINUOUS = 1,
    /// Zoom value as proportion of full camera range (a percentage value between 0.0 and 100.0)
    ZOOM_TYPE_RANGE = 2,
    /// Zoom value/variable focal length in millimetres. Note that there is no message to get the valid zoom range of the camera, so this can type can only be used for cameras where the zoom range is known (implying that this cannot reliably be used in a GCS for an arbitrary camera)
    ZOOM_TYPE_FOCAL_LENGTH = 3,
    /// Zoom value as horizontal field of view in degrees.
    ZOOM_TYPE_HORIZONTAL_FOV = 4,
};

/// The ROI (region of interest) for the vehicle. This can be
///                 be used by the vehicle for camera/vehicle attitude alignment (see
///                 MAV_CMD_NAV_ROI).
pub const MAV_ROI = enum(u32) {
    /// No region of interest.
    MAV_ROI_NONE = 0,
    /// Point toward next waypoint, with optional pitch/roll/yaw offset.
    MAV_ROI_WPNEXT = 1,
    /// Point toward given waypoint.
    MAV_ROI_WPINDEX = 2,
    /// Point toward fixed location.
    MAV_ROI_LOCATION = 3,
    /// Point toward of given id.
    MAV_ROI_TARGET = 4,
};

/// RC type. Used in MAV_CMD_START_RX_PAIR.
pub const RC_TYPE = enum(u32) {
    /// Spektrum
    RC_TYPE_SPEKTRUM = 0,
    /// CRSF
    RC_TYPE_CRSF = 1,
};

/// Navigational status of AIS vessel, enum duplicated from AIS standard, https://gpsd.gitlab.io/gpsd/AIVDM.html
pub const AIS_NAV_STATUS = enum(u8) {
    /// Under way using engine.
    UNDER_WAY = 0,
    AIS_NAV_ANCHORED = 1,
    AIS_NAV_UN_COMMANDED = 2,
    AIS_NAV_RESTRICTED_MANOEUVERABILITY = 3,
    AIS_NAV_DRAUGHT_CONSTRAINED = 4,
    AIS_NAV_MOORED = 5,
    AIS_NAV_AGROUND = 6,
    AIS_NAV_FISHING = 7,
    AIS_NAV_SAILING = 8,
    AIS_NAV_RESERVED_HSC = 9,
    AIS_NAV_RESERVED_WIG = 10,
    AIS_NAV_RESERVED_1 = 11,
    AIS_NAV_RESERVED_2 = 12,
    AIS_NAV_RESERVED_3 = 13,
    /// Search And Rescue Transponder.
    AIS_NAV_AIS_SART = 14,
    /// Not available (default).
    AIS_NAV_UNKNOWN = 15,
};

pub const MAV_ODID_TIME_ACC = enum(u8) {
    /// The timestamp accuracy is unknown.
    MAV_ODID_TIME_ACC_UNKNOWN = 0,
    /// The timestamp accuracy is smaller than or equal to 0.1 second.
    MAV_ODID_TIME_ACC_0_1_SECOND = 1,
    /// The timestamp accuracy is smaller than or equal to 0.2 second.
    MAV_ODID_TIME_ACC_0_2_SECOND = 2,
    /// The timestamp accuracy is smaller than or equal to 0.3 second.
    MAV_ODID_TIME_ACC_0_3_SECOND = 3,
    /// The timestamp accuracy is smaller than or equal to 0.4 second.
    MAV_ODID_TIME_ACC_0_4_SECOND = 4,
    /// The timestamp accuracy is smaller than or equal to 0.5 second.
    MAV_ODID_TIME_ACC_0_5_SECOND = 5,
    /// The timestamp accuracy is smaller than or equal to 0.6 second.
    MAV_ODID_TIME_ACC_0_6_SECOND = 6,
    /// The timestamp accuracy is smaller than or equal to 0.7 second.
    MAV_ODID_TIME_ACC_0_7_SECOND = 7,
    /// The timestamp accuracy is smaller than or equal to 0.8 second.
    MAV_ODID_TIME_ACC_0_8_SECOND = 8,
    /// The timestamp accuracy is smaller than or equal to 0.9 second.
    MAV_ODID_TIME_ACC_0_9_SECOND = 9,
    /// The timestamp accuracy is smaller than or equal to 1.0 second.
    MAV_ODID_TIME_ACC_1_0_SECOND = 10,
    /// The timestamp accuracy is smaller than or equal to 1.1 second.
    MAV_ODID_TIME_ACC_1_1_SECOND = 11,
    /// The timestamp accuracy is smaller than or equal to 1.2 second.
    MAV_ODID_TIME_ACC_1_2_SECOND = 12,
    /// The timestamp accuracy is smaller than or equal to 1.3 second.
    MAV_ODID_TIME_ACC_1_3_SECOND = 13,
    /// The timestamp accuracy is smaller than or equal to 1.4 second.
    MAV_ODID_TIME_ACC_1_4_SECOND = 14,
    /// The timestamp accuracy is smaller than or equal to 1.5 second.
    MAV_ODID_TIME_ACC_1_5_SECOND = 15,
};

/// Indicates the ESC connection type.
pub const ESC_CONNECTION_TYPE = enum(u32) {
    /// Traditional PPM ESC.
    ESC_CONNECTION_TYPE_PPM = 0,
    /// Serial Bus connected ESC.
    ESC_CONNECTION_TYPE_SERIAL = 1,
    /// One Shot PPM ESC.
    ESC_CONNECTION_TYPE_ONESHOT = 2,
    /// I2C ESC.
    ESC_CONNECTION_TYPE_I2C = 3,
    /// CAN-Bus ESC.
    ESC_CONNECTION_TYPE_CAN = 4,
    /// DShot ESC.
    ESC_CONNECTION_TYPE_DSHOT = 5,
};

/// Enumeration for battery charge states.
pub const MAV_BATTERY_CHARGE_STATE = enum(u8) {
    /// Low battery state is not provided
    MAV_BATTERY_CHARGE_STATE_UNDEFINED = 0,
    /// Battery is not in low state. Normal operation.
    MAV_BATTERY_CHARGE_STATE_OK = 1,
    /// Battery state is low, warn and monitor close.
    MAV_BATTERY_CHARGE_STATE_LOW = 2,
    /// Battery state is critical, return or abort immediately.
    MAV_BATTERY_CHARGE_STATE_CRITICAL = 3,
    /// Battery state is too low for ordinary abort sequence. Perform fastest possible emergency stop to prevent damage.
    MAV_BATTERY_CHARGE_STATE_EMERGENCY = 4,
    /// Battery failed, damage unavoidable. Possible causes (faults) are listed in MAV_BATTERY_FAULT.
    MAV_BATTERY_CHARGE_STATE_FAILED = 5,
    /// Battery is diagnosed to be defective or an error occurred, usage is discouraged / prohibited. Possible causes (faults) are listed in MAV_BATTERY_FAULT.
    MAV_BATTERY_CHARGE_STATE_UNHEALTHY = 6,
    /// Battery is charging.
    MAV_BATTERY_CHARGE_STATE_CHARGING = 7,
};

/// List of possible failure type to inject.
pub const FAILURE_TYPE = enum(u32) {
    /// No failure injected, used to reset a previous failure.
    FAILURE_TYPE_OK = 0,
    /// Sets unit off, so completely non-responsive.
    FAILURE_TYPE_OFF = 1,
    /// Unit is stuck e.g. keeps reporting the same value.
    FAILURE_TYPE_STUCK = 2,
    /// Unit is reporting complete garbage.
    FAILURE_TYPE_GARBAGE = 3,
    /// Unit is consistently wrong.
    FAILURE_TYPE_WRONG = 4,
    /// Unit is slow, so e.g. reporting at slower than expected rate.
    FAILURE_TYPE_SLOW = 5,
    /// Data of unit is delayed in time.
    FAILURE_TYPE_DELAYED = 6,
    /// Unit is sometimes working, sometimes not.
    FAILURE_TYPE_INTERMITTENT = 7,
};

/// Flags for gimbal device (lower level) operation.
pub const GIMBAL_DEVICE_FLAGS = enum(u16) {
    /// Set to retracted safe position (no stabilization), takes precedence over all other flags.
    GIMBAL_DEVICE_FLAGS_RETRACT = 1,
    /// Set to neutral/default position, taking precedence over all other flags except RETRACT. Neutral is commonly forward-facing and horizontal (roll=pitch=yaw=0) but may be any orientation.
    GIMBAL_DEVICE_FLAGS_NEUTRAL = 2,
    /// Lock roll angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal.
    GIMBAL_DEVICE_FLAGS_ROLL_LOCK = 4,
    /// Lock pitch angle to absolute angle relative to horizon (not relative to vehicle). This is generally the default with a stabilizing gimbal.
    GIMBAL_DEVICE_FLAGS_PITCH_LOCK = 8,
    /// Lock yaw angle to absolute angle relative to North (not relative to vehicle). If this flag is set, the yaw angle and z component of angular velocity are relative to North (earth frame, x-axis pointing North), else they are relative to the vehicle heading (vehicle frame, earth frame rotated so that the x-axis is pointing forward).
    GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16,
    /// Yaw angle and z component of angular velocity are relative to the vehicle heading (vehicle frame, earth frame rotated such that the x-axis is pointing forward).
    GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME = 32,
    /// Yaw angle and z component of angular velocity are relative to North (earth frame, x-axis is pointing North).
    GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME = 64,
    /// Gimbal device can accept yaw angle inputs relative to North (earth frame). This flag is only for reporting (attempts to set this flag are ignored).
    GIMBAL_DEVICE_FLAGS_ACCEPTS_YAW_IN_EARTH_FRAME = 128,
    /// The gimbal orientation is set exclusively by the RC signals feed to the gimbal's radio control inputs. MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE) are ignored.
    GIMBAL_DEVICE_FLAGS_RC_EXCLUSIVE = 256,
    /// The gimbal orientation is determined by combining/mixing the RC signals feed to the gimbal's radio control inputs and the MAVLink messages for setting the gimbal orientation (GIMBAL_DEVICE_SET_ATTITUDE). How these two controls are combined or mixed is not defined by the protocol but is up to the implementation.
    GIMBAL_DEVICE_FLAGS_RC_MIXED = 512,
};

/// Specifies the datatype of a MAVLink parameter.
pub const MAV_PARAM_TYPE = enum(u8) {
    /// 8-bit unsigned integer
    MAV_PARAM_TYPE_UINT8 = 1,
    /// 8-bit signed integer
    MAV_PARAM_TYPE_INT8 = 2,
    /// 16-bit unsigned integer
    MAV_PARAM_TYPE_UINT16 = 3,
    /// 16-bit signed integer
    MAV_PARAM_TYPE_INT16 = 4,
    /// 32-bit unsigned integer
    MAV_PARAM_TYPE_UINT32 = 5,
    /// 32-bit signed integer
    MAV_PARAM_TYPE_INT32 = 6,
    /// 64-bit unsigned integer
    MAV_PARAM_TYPE_UINT64 = 7,
    /// 64-bit signed integer
    MAV_PARAM_TYPE_INT64 = 8,
    /// 32-bit floating-point
    MAV_PARAM_TYPE_REAL32 = 9,
    /// 64-bit floating-point
    MAV_PARAM_TYPE_REAL64 = 10,
};

/// Coordinate frames used by MAVLink. Not all frames are supported by all commands, messages, or vehicles.
/// 
///       Global frames use the following naming conventions:
///       - "GLOBAL": Global coordinate frame with WGS84 latitude/longitude and altitude positive over mean sea level (MSL) by default.
///         The following modifiers may be used with "GLOBAL":
///         - "RELATIVE_ALT": Altitude is relative to the vehicle home position rather than MSL.
///         - "TERRAIN_ALT": Altitude is relative to ground level rather than MSL.
///         - "INT": Latitude/longitude (in degrees) are scaled by multiplying by 1E7.
/// 
///       Local frames use the following naming conventions:
///       - "LOCAL": Origin of local frame is fixed relative to earth. Unless otherwise specified this origin is the origin of the vehicle position-estimator ("EKF").
///       - "BODY": Origin of local frame travels with the vehicle. NOTE, "BODY" does NOT indicate alignment of frame axis with vehicle attitude.
///       - "OFFSET": Deprecated synonym for "BODY" (origin travels with the vehicle). Not to be used for new frames.
/// 
///       Some deprecated frames do not follow these conventions (e.g. MAV_FRAME_BODY_NED and MAV_FRAME_BODY_OFFSET_NED).
pub const MAV_FRAME = enum(u8) {
    /// Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
    MAV_FRAME_GLOBAL = 0,
    /// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
    MAV_FRAME_LOCAL_NED = 1,
    /// NOT a coordinate frame, indicates a mission command.
    MAV_FRAME_MISSION = 2,
    /// Global (WGS84) coordinate frame + altitude relative to the home position.
    MAV_FRAME_GLOBAL_RELATIVE_ALT = 3,
    /// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
    MAV_FRAME_LOCAL_ENU = 4,
    /// Global (WGS84) coordinate frame (scaled) + altitude relative to mean sea level (MSL).
    MAV_FRAME_GLOBAL_INT = 5,
    /// Global (WGS84) coordinate frame (scaled) + altitude relative to the home position.
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6,
    /// NED local tangent frame (x: North, y: East, z: Down) with origin that travels with the vehicle.
    MAV_FRAME_LOCAL_OFFSET_NED = 7,
    /// Same as MAV_FRAME_LOCAL_NED when used to represent position values. Same as MAV_FRAME_BODY_FRD when used with velocity/acceleration values.
    MAV_FRAME_BODY_NED = 8,
    /// This is the same as MAV_FRAME_BODY_FRD.
    MAV_FRAME_BODY_OFFSET_NED = 9,
    /// Global (WGS84) coordinate frame with AGL altitude (altitude at ground level).
    MAV_FRAME_GLOBAL_TERRAIN_ALT = 10,
    /// Global (WGS84) coordinate frame (scaled) with AGL altitude (altitude at ground level).
    MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11,
    /// FRD local frame aligned to the vehicle's attitude (x: Forward, y: Right, z: Down) with an origin that travels with vehicle.
    MAV_FRAME_BODY_FRD = 12,
    /// MAV_FRAME_BODY_FLU - Body fixed frame of reference, Z-up (x: Forward, y: Left, z: Up).
    MAV_FRAME_RESERVED_13 = 13,
    /// MAV_FRAME_MOCAP_NED - Odometry local coordinate frame of data given by a motion capture system, Z-down (x: North, y: East, z: Down).
    MAV_FRAME_RESERVED_14 = 14,
    /// MAV_FRAME_MOCAP_ENU - Odometry local coordinate frame of data given by a motion capture system, Z-up (x: East, y: North, z: Up).
    MAV_FRAME_RESERVED_15 = 15,
    /// MAV_FRAME_VISION_NED - Odometry local coordinate frame of data given by a vision estimation system, Z-down (x: North, y: East, z: Down).
    MAV_FRAME_RESERVED_16 = 16,
    /// MAV_FRAME_VISION_ENU - Odometry local coordinate frame of data given by a vision estimation system, Z-up (x: East, y: North, z: Up).
    MAV_FRAME_RESERVED_17 = 17,
    /// MAV_FRAME_ESTIM_NED - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-down (x: North, y: East, z: Down).
    MAV_FRAME_RESERVED_18 = 18,
    /// MAV_FRAME_ESTIM_ENU - Odometry local coordinate frame of data given by an estimator running onboard the vehicle, Z-up (x: East, y: North, z: Up).
    MAV_FRAME_RESERVED_19 = 19,
    /// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
    MAV_FRAME_LOCAL_FRD = 20,
    /// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth. The forward axis is aligned to the front of the vehicle in the horizontal plane.
    MAV_FRAME_LOCAL_FLU = 21,
};

/// Source of information about this collision.
pub const MAV_COLLISION_SRC = enum(u8) {
    /// ID field references ADSB_VEHICLE packets
    MAV_COLLISION_SRC_ADSB = 0,
    /// ID field references MAVLink SRC ID
    MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT = 1,
};

/// Yaw behaviour during orbit flight.
pub const ORBIT_YAW_BEHAVIOUR = enum(u32) {
    /// Vehicle front points to the center (default).
    ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0,
    /// Vehicle front holds heading when message received.
    ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1,
    /// Yaw uncontrolled.
    ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2,
    /// Vehicle front follows flight path (tangential to circle).
    ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3,
    /// Yaw controlled by RC input.
    ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4,
    /// Vehicle uses current yaw behaviour (unchanged). The vehicle-default yaw behaviour is used if this value is specified when orbit is first commanded.
    ORBIT_YAW_BEHAVIOUR_UNCHANGED = 5,
};

/// Component supports locking control to a particular GCS independent of its system (via MAV_CMD_REQUEST_OPERATOR_CONTROL).
pub const MAV_PROTOCOL_CAPABILITY = enum(u64) {
    /// Autopilot supports the MISSION_ITEM float message type.
    ///           Note that MISSION_ITEM is deprecated, and autopilots should use MISSION_INT instead.
    MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT = 1,
    /// Autopilot supports the new param float message type.
    MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT = 2,
    /// Autopilot supports MISSION_ITEM_INT scaled integer message type.
    ///           Note that this flag must always be set if missions are supported, because missions must always use MISSION_ITEM_INT (rather than MISSION_ITEM, which is deprecated).
    MAV_PROTOCOL_CAPABILITY_MISSION_INT = 4,
    /// Autopilot supports COMMAND_INT scaled integer message type.
    MAV_PROTOCOL_CAPABILITY_COMMAND_INT = 8,
    /// Parameter protocol uses byte-wise encoding of parameter values into param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.
    ///           Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST should be set if the parameter protocol is supported.
    MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE = 16,
    /// Autopilot supports the File Transfer Protocol v1: https://mavlink.io/en/services/ftp.html.
    MAV_PROTOCOL_CAPABILITY_FTP = 32,
    /// Autopilot supports commanding attitude offboard.
    MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET = 64,
    /// Autopilot supports commanding position and velocity targets in local NED frame.
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED = 128,
    /// Autopilot supports commanding position and velocity targets in global scaled integers.
    MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT = 256,
    /// Autopilot supports terrain protocol / data handling.
    MAV_PROTOCOL_CAPABILITY_TERRAIN = 512,
    /// Reserved for future use.
    MAV_PROTOCOL_CAPABILITY_RESERVED3 = 1024,
    /// Autopilot supports the MAV_CMD_DO_FLIGHTTERMINATION command (flight termination).
    MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION = 2048,
    /// Autopilot supports onboard compass calibration.
    MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION = 4096,
    /// Autopilot supports MAVLink version 2.
    MAV_PROTOCOL_CAPABILITY_MAVLINK2 = 8192,
    /// Autopilot supports mission fence protocol.
    MAV_PROTOCOL_CAPABILITY_MISSION_FENCE = 16384,
    /// Autopilot supports mission rally point protocol.
    MAV_PROTOCOL_CAPABILITY_MISSION_RALLY = 32768,
    /// Reserved for future use.
    MAV_PROTOCOL_CAPABILITY_RESERVED2 = 65536,
    /// Parameter protocol uses C-cast of parameter values to set the param_value (float) fields: https://mavlink.io/en/services/parameter.html#parameter-encoding.
    ///           Note that either this flag or MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE should be set if the parameter protocol is supported.
    MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST = 131072,
    /// This component implements/is a gimbal manager. This means the GIMBAL_MANAGER_INFORMATION, and other messages can be requested.
    MAV_PROTOCOL_CAPABILITY_COMPONENT_IMPLEMENTS_GIMBAL_MANAGER = 262144,
};

/// Enumeration of the ADSB altimeter types
pub const ADSB_ALTITUDE_TYPE = enum(u8) {
    /// Altitude reported from a Baro source using QNH reference
    ADSB_ALTITUDE_TYPE_PRESSURE_QNH = 0,
    /// Altitude reported from a GNSS source
    ADSB_ALTITUDE_TYPE_GEOMETRIC = 1,
};

/// These flags indicate status such as data validity of each data source. Set = data valid
pub const ADSB_FLAGS = enum(u16) {
    ADSB_FLAGS_VALID_COORDS = 1,
    ADSB_FLAGS_VALID_ALTITUDE = 2,
    ADSB_FLAGS_VALID_HEADING = 4,
    ADSB_FLAGS_VALID_VELOCITY = 8,
    ADSB_FLAGS_VALID_CALLSIGN = 16,
    ADSB_FLAGS_VALID_SQUAWK = 32,
    ADSB_FLAGS_SIMULATED = 64,
    ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128,
    ADSB_FLAGS_BARO_VALID = 256,
    ADSB_FLAGS_SOURCE_UAT = 32768,
};

/// Camera tracking status flags
pub const CAMERA_TRACKING_STATUS_FLAGS = enum(u8) {
    /// Camera is not tracking
    CAMERA_TRACKING_STATUS_FLAGS_IDLE = 0,
    /// Camera is tracking
    CAMERA_TRACKING_STATUS_FLAGS_ACTIVE = 1,
    /// Camera tracking in error state
    CAMERA_TRACKING_STATUS_FLAGS_ERROR = 2,
};

/// MAV FTP error codes (https://mavlink.io/en/services/ftp.html)
pub const MAV_FTP_ERR = enum(u32) {
    /// None: No error
    MAV_FTP_ERR_NONE = 0,
    /// Fail: Unknown failure
    MAV_FTP_ERR_FAIL = 1,
    /// FailErrno: Command failed, Err number sent back in PayloadHeader.data[1].
    /// This is a file-system error number understood by the server operating system.
    MAV_FTP_ERR_FAILERRNO = 2,
    /// InvalidDataSize: Payload size is invalid
    MAV_FTP_ERR_INVALIDDATASIZE = 3,
    /// InvalidSession: Session is not currently open
    MAV_FTP_ERR_INVALIDSESSION = 4,
    /// NoSessionsAvailable: All available sessions are already in use
    MAV_FTP_ERR_NOSESSIONSAVAILABLE = 5,
    /// EOF: Offset past end of file for ListDirectory and ReadFile commands
    MAV_FTP_ERR_EOF = 6,
    /// UnknownCommand: Unknown command / opcode
    MAV_FTP_ERR_UNKNOWNCOMMAND = 7,
    /// FileExists: File/directory already exists
    MAV_FTP_ERR_FILEEXISTS = 8,
    /// FileProtected: File/directory is write protected
    MAV_FTP_ERR_FILEPROTECTED = 9,
    /// FileNotFound: File/directory not found
    MAV_FTP_ERR_FILENOTFOUND = 10,
};

/// Micro air vehicle / autopilot classes. This identifies the individual model.
pub const MAV_AUTOPILOT = enum(u8) {
    /// Generic autopilot, full support for everything
    MAV_AUTOPILOT_GENERIC = 0,
    /// Reserved for future use.
    MAV_AUTOPILOT_RESERVED = 1,
    /// SLUGS autopilot, http://slugsuav.soe.ucsc.edu
    MAV_AUTOPILOT_SLUGS = 2,
    /// ArduPilot - Plane/Copter/Rover/Sub/Tracker, https://ardupilot.org
    MAV_AUTOPILOT_ARDUPILOTMEGA = 3,
    /// OpenPilot, http://openpilot.org
    MAV_AUTOPILOT_OPENPILOT = 4,
    /// Generic autopilot only supporting simple waypoints
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY = 5,
    /// Generic autopilot supporting waypoints and other simple navigation commands
    MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY = 6,
    /// Generic autopilot supporting the full mission command set
    MAV_AUTOPILOT_GENERIC_MISSION_FULL = 7,
    /// No valid autopilot, e.g. a GCS or other MAVLink component
    MAV_AUTOPILOT_INVALID = 8,
    /// PPZ UAV - http://nongnu.org/paparazzi
    MAV_AUTOPILOT_PPZ = 9,
    /// UAV Dev Board
    MAV_AUTOPILOT_UDB = 10,
    /// FlexiPilot
    MAV_AUTOPILOT_FP = 11,
    /// PX4 Autopilot - http://px4.io/
    MAV_AUTOPILOT_PX4 = 12,
    /// SMACCMPilot - http://smaccmpilot.org
    MAV_AUTOPILOT_SMACCMPILOT = 13,
    /// AutoQuad -- http://autoquad.org
    MAV_AUTOPILOT_AUTOQUAD = 14,
    /// Armazila -- http://armazila.com
    MAV_AUTOPILOT_ARMAZILA = 15,
    /// Aerob -- http://aerob.ru
    MAV_AUTOPILOT_AEROB = 16,
    /// ASLUAV autopilot -- http://www.asl.ethz.ch
    MAV_AUTOPILOT_ASLUAV = 17,
    /// SmartAP Autopilot - http://sky-drones.com
    MAV_AUTOPILOT_SMARTAP = 18,
    /// AirRails - http://uaventure.com
    MAV_AUTOPILOT_AIRRAILS = 19,
    /// Fusion Reflex - https://fusion.engineering
    MAV_AUTOPILOT_REFLEX = 20,
};

/// Flags to indicate the status of camera storage.
pub const STORAGE_STATUS = enum(u8) {
    /// Storage is missing (no microSD card loaded for example.)
    STORAGE_STATUS_EMPTY = 0,
    /// Storage present but unformatted.
    STORAGE_STATUS_UNFORMATTED = 1,
    /// Storage present and ready.
    STORAGE_STATUS_READY = 2,
    /// Camera does not supply storage status information. Capacity information in STORAGE_INFORMATION fields will be ignored.
    STORAGE_STATUS_NOT_SUPPORTED = 3,
};

/// Mode properties.
pub const MAV_MODE_PROPERTY = enum(u32) {
    /// If set, this mode is an advanced mode.
    ///           For example a rate-controlled manual mode might be advanced, whereas a position-controlled manual mode is not.
    ///           A GCS can optionally use this flag to configure the UI for its intended users.
    MAV_MODE_PROPERTY_ADVANCED = 1,
    /// If set, this mode should not be added to the list of selectable modes.
    ///           The mode might still be selected by the FC directly (for example as part of a failsafe).
    MAV_MODE_PROPERTY_NOT_USER_SELECTABLE = 2,
    /// If set, this mode is automatically controlled (it may use but does not require a manual controller).
    ///           If unset the mode is a assumed to require user input (be a manual mode).
    MAV_MODE_PROPERTY_AUTO_MODE = 4,
};

/// Flags to indicate usage for a particular storage (see STORAGE_INFORMATION.storage_usage and MAV_CMD_SET_STORAGE_USAGE).
pub const STORAGE_USAGE_FLAG = enum(u8) {
    /// Always set to 1 (indicates STORAGE_INFORMATION.storage_usage is supported).
    STORAGE_USAGE_FLAG_SET = 1,
    /// Storage for saving photos.
    STORAGE_USAGE_FLAG_PHOTO = 2,
    /// Storage for saving videos.
    STORAGE_USAGE_FLAG_VIDEO = 4,
    /// Storage for saving logs.
    STORAGE_USAGE_FLAG_LOGS = 8,
};

/// Illuminator module error flags (bitmap, 0 means no error)
pub const ILLUMINATOR_ERROR_FLAGS = enum(u32) {
    /// Illuminator thermal throttling error.
    ILLUMINATOR_ERROR_FLAGS_THERMAL_THROTTLING = 1,
    /// Illuminator over temperature shutdown error.
    ILLUMINATOR_ERROR_FLAGS_OVER_TEMPERATURE_SHUTDOWN = 2,
    /// Illuminator thermistor failure.
    ILLUMINATOR_ERROR_FLAGS_THERMISTOR_FAILURE = 4,
};

/// Direction of VTOL transition
pub const VTOL_TRANSITION_HEADING = enum(u32) {
    /// Respect the heading configuration of the vehicle.
    VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT = 0,
    /// Use the heading pointing towards the next waypoint.
    VTOL_TRANSITION_HEADING_NEXT_WAYPOINT = 1,
    /// Use the heading on takeoff (while sitting on the ground).
    VTOL_TRANSITION_HEADING_TAKEOFF = 2,
    /// Use the specified heading in parameter 4.
    VTOL_TRANSITION_HEADING_SPECIFIED = 3,
    /// Use the current heading when reaching takeoff altitude (potentially facing the wind when weather-vaning is active).
    VTOL_TRANSITION_HEADING_ANY = 4,
};

/// Indicates the severity level, generally used for status messages to indicate their relative urgency. Based on RFC-5424 using expanded definitions at: http://www.kiwisyslog.com/kb/info:-syslog-message-levels/.
pub const MAV_SEVERITY = enum(u8) {
    /// System is unusable. This is a "panic" condition.
    MAV_SEVERITY_EMERGENCY = 0,
    /// Action should be taken immediately. Indicates error in non-critical systems.
    MAV_SEVERITY_ALERT = 1,
    /// Action must be taken immediately. Indicates failure in a primary system.
    MAV_SEVERITY_CRITICAL = 2,
    /// Indicates an error in secondary/redundant systems.
    MAV_SEVERITY_ERROR = 3,
    /// Indicates about a possible future error if this is not resolved within a given timeframe. Example would be a low battery warning.
    MAV_SEVERITY_WARNING = 4,
    /// An unusual event has occurred, though not an error condition. This should be investigated for the root cause.
    MAV_SEVERITY_NOTICE = 5,
    /// Normal operational messages. Useful for logging. No action is required for these messages.
    MAV_SEVERITY_INFO = 6,
    /// Useful non-operational messages that can assist in debugging. These should not occur during normal operation.
    MAV_SEVERITY_DEBUG = 7,
};

/// Bitmap to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 9 is set the floats afx afy afz should be interpreted as force instead of acceleration.
pub const POSITION_TARGET_TYPEMASK = enum(u16) {
    /// Ignore position x
    POSITION_TARGET_TYPEMASK_X_IGNORE = 1,
    /// Ignore position y
    POSITION_TARGET_TYPEMASK_Y_IGNORE = 2,
    /// Ignore position z
    POSITION_TARGET_TYPEMASK_Z_IGNORE = 4,
    /// Ignore velocity x
    POSITION_TARGET_TYPEMASK_VX_IGNORE = 8,
    /// Ignore velocity y
    POSITION_TARGET_TYPEMASK_VY_IGNORE = 16,
    /// Ignore velocity z
    POSITION_TARGET_TYPEMASK_VZ_IGNORE = 32,
    /// Ignore acceleration x
    POSITION_TARGET_TYPEMASK_AX_IGNORE = 64,
    /// Ignore acceleration y
    POSITION_TARGET_TYPEMASK_AY_IGNORE = 128,
    /// Ignore acceleration z
    POSITION_TARGET_TYPEMASK_AZ_IGNORE = 256,
    /// Use force instead of acceleration
    POSITION_TARGET_TYPEMASK_FORCE_SET = 512,
    /// Ignore yaw
    POSITION_TARGET_TYPEMASK_YAW_IGNORE = 1024,
    /// Ignore yaw rate
    POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE = 2048,
};

/// These values define the type of firmware release.  These values indicate the first version or release of this type.  For example the first alpha release would be 64, the second would be 65.
pub const FIRMWARE_VERSION_TYPE = enum(u32) {
    /// development release
    FIRMWARE_VERSION_TYPE_DEV = 0,
    /// alpha release
    FIRMWARE_VERSION_TYPE_ALPHA = 64,
    /// beta release
    FIRMWARE_VERSION_TYPE_BETA = 128,
    /// release candidate
    FIRMWARE_VERSION_TYPE_RC = 192,
    /// official stable release
    FIRMWARE_VERSION_TYPE_OFFICIAL = 255,
};

/// Specifies the conditions under which the MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command should be accepted.
pub const REBOOT_SHUTDOWN_CONDITIONS = enum(u32) {
    /// Reboot/Shutdown only if allowed by safety checks, such as being landed.
    REBOOT_SHUTDOWN_CONDITIONS_SAFETY_INTERLOCKED = 0,
    /// Force reboot/shutdown of the autopilot/component regardless of system state.
    REBOOT_SHUTDOWN_CONDITIONS_FORCE = 20190226,
};

/// Enumeration of estimator types
pub const MAV_ESTIMATOR_TYPE = enum(u8) {
    /// Unknown type of the estimator.
    MAV_ESTIMATOR_TYPE_UNKNOWN = 0,
    /// This is a naive estimator without any real covariance feedback.
    MAV_ESTIMATOR_TYPE_NAIVE = 1,
    /// Computer vision based estimate. Might be up to scale.
    MAV_ESTIMATOR_TYPE_VISION = 2,
    /// Visual-inertial estimate.
    MAV_ESTIMATOR_TYPE_VIO = 3,
    /// Plain GPS estimate.
    MAV_ESTIMATOR_TYPE_GPS = 4,
    /// Estimator integrating GPS and inertial sensing.
    MAV_ESTIMATOR_TYPE_GPS_INS = 5,
    /// Estimate from external motion capturing system.
    MAV_ESTIMATOR_TYPE_MOCAP = 6,
    /// Estimator based on lidar sensor input.
    MAV_ESTIMATOR_TYPE_LIDAR = 7,
    /// Estimator on autopilot.
    MAV_ESTIMATOR_TYPE_AUTOPILOT = 8,
};

/// Video stream types
pub const VIDEO_STREAM_TYPE = enum(u8) {
    /// Stream is RTSP
    VIDEO_STREAM_TYPE_RTSP = 0,
    /// Stream is RTP UDP (URI gives the port number)
    VIDEO_STREAM_TYPE_RTPUDP = 1,
    /// Stream is MPEG on TCP
    VIDEO_STREAM_TYPE_TCP_MPEG = 2,
    /// Stream is MPEG TS (URI gives the port number)
    VIDEO_STREAM_TYPE_MPEG_TS = 3,
};

pub const NAV_VTOL_LAND_OPTIONS = enum(u32) {
    /// Default autopilot landing behaviour.
    NAV_VTOL_LAND_OPTIONS_DEFAULT = 0,
    /// Descend in fixed wing mode, transitioning to multicopter mode for vertical landing when close to the ground.
    ///           The fixed wing descent pattern is at the discretion of the vehicle (e.g. transition altitude, loiter direction, radius, and speed, etc.).
    NAV_VTOL_LAND_OPTIONS_FW_DESCENT = 1,
    /// Land in multicopter mode on reaching the landing coordinates (the whole landing is by "hover descent").
    NAV_VTOL_LAND_OPTIONS_HOVER_DESCENT = 2,
};

pub const MAV_ODID_SPEED_ACC = enum(u8) {
    /// The speed accuracy is unknown.
    MAV_ODID_SPEED_ACC_UNKNOWN = 0,
    /// The speed accuracy is smaller than 10 meters per second.
    MAV_ODID_SPEED_ACC_10_METERS_PER_SECOND = 1,
    /// The speed accuracy is smaller than 3 meters per second.
    MAV_ODID_SPEED_ACC_3_METERS_PER_SECOND = 2,
    /// The speed accuracy is smaller than 1 meters per second.
    MAV_ODID_SPEED_ACC_1_METERS_PER_SECOND = 3,
    /// The speed accuracy is smaller than 0.3 meters per second.
    MAV_ODID_SPEED_ACC_0_3_METERS_PER_SECOND = 4,
};

/// Enumeration of battery types
pub const MAV_BATTERY_TYPE = enum(u8) {
    /// Not specified.
    MAV_BATTERY_TYPE_UNKNOWN = 0,
    /// Lithium polymer battery
    MAV_BATTERY_TYPE_LIPO = 1,
    /// Lithium-iron-phosphate battery
    MAV_BATTERY_TYPE_LIFE = 2,
    /// Lithium-ION battery
    MAV_BATTERY_TYPE_LION = 3,
    /// Nickel metal hydride battery
    MAV_BATTERY_TYPE_NIMH = 4,
};

/// Actions being taken to mitigate/prevent fence breach
pub const FENCE_MITIGATE = enum(u8) {
    /// Unknown
    FENCE_MITIGATE_UNKNOWN = 0,
    /// No actions being taken
    FENCE_MITIGATE_NONE = 1,
    /// Velocity limiting active to prevent breach
    FENCE_MITIGATE_VEL_LIMIT = 2,
};

/// Camera capability flags (Bitmap)
pub const CAMERA_CAP_FLAGS = enum(u32) {
    /// Camera is able to record video
    CAMERA_CAP_FLAGS_CAPTURE_VIDEO = 1,
    /// Camera is able to capture images
    CAMERA_CAP_FLAGS_CAPTURE_IMAGE = 2,
    /// Camera has separate Video and Image/Photo modes (MAV_CMD_SET_CAMERA_MODE)
    CAMERA_CAP_FLAGS_HAS_MODES = 4,
    /// Camera can capture images while in video mode
    CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8,
    /// Camera can capture videos while in Photo/Image mode
    CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16,
    /// Camera has image survey mode (MAV_CMD_SET_CAMERA_MODE)
    CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE = 32,
    /// Camera has basic zoom control (MAV_CMD_SET_CAMERA_ZOOM)
    CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM = 64,
    /// Camera has basic focus control (MAV_CMD_SET_CAMERA_FOCUS)
    CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS = 128,
    /// Camera has video streaming capabilities (request VIDEO_STREAM_INFORMATION with MAV_CMD_REQUEST_MESSAGE for video streaming info)
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM = 256,
    /// Camera supports tracking of a point on the camera view.
    CAMERA_CAP_FLAGS_HAS_TRACKING_POINT = 512,
    /// Camera supports tracking of a selection rectangle on the camera view.
    CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE = 1024,
    /// Camera supports tracking geo status (CAMERA_TRACKING_GEO_STATUS).
    CAMERA_CAP_FLAGS_HAS_TRACKING_GEO_STATUS = 2048,
    /// Camera supports absolute thermal range (request CAMERA_THERMAL_RANGE with MAV_CMD_REQUEST_MESSAGE).
    CAMERA_CAP_FLAGS_HAS_THERMAL_RANGE = 4096,
};

/// RTK GPS baseline coordinate system, used for RTK corrections
pub const RTK_BASELINE_COORDINATE_SYSTEM = enum(u8) {
    /// Earth-centered, Earth-fixed
    RTK_BASELINE_COORDINATE_SYSTEM_ECEF = 0,
    /// RTK basestation centered, north, east, down
    RTK_BASELINE_COORDINATE_SYSTEM_NED = 1,
};

/// Actuator configuration, used to change a setting on an actuator. Component information metadata can be used to know which outputs support which commands.
pub const ACTUATOR_CONFIGURATION = enum(u32) {
    /// Do nothing.
    ACTUATOR_CONFIGURATION_NONE = 0,
    /// Command the actuator to beep now.
    ACTUATOR_CONFIGURATION_BEEP = 1,
    /// Permanently set the actuator (ESC) to 3D mode (reversible thrust).
    ACTUATOR_CONFIGURATION_3D_MODE_ON = 2,
    /// Permanently set the actuator (ESC) to non 3D mode (non-reversible thrust).
    ACTUATOR_CONFIGURATION_3D_MODE_OFF = 3,
    /// Permanently set the actuator (ESC) to spin direction 1 (which can be clockwise or counter-clockwise).
    ACTUATOR_CONFIGURATION_SPIN_DIRECTION1 = 4,
    /// Permanently set the actuator (ESC) to spin direction 2 (opposite of direction 1).
    ACTUATOR_CONFIGURATION_SPIN_DIRECTION2 = 5,
};

/// Aircraft-rated danger from this threat.
pub const MAV_COLLISION_THREAT_LEVEL = enum(u8) {
    /// Not a threat
    MAV_COLLISION_THREAT_LEVEL_NONE = 0,
    /// Craft is mildly concerned about this threat
    MAV_COLLISION_THREAT_LEVEL_LOW = 1,
    /// Craft is panicking, and may take actions to avoid threat
    MAV_COLLISION_THREAT_LEVEL_HIGH = 2,
};

/// Actions for reading and writing plan information (mission, rally points, geofence) between persistent and volatile storage when using MAV_CMD_PREFLIGHT_STORAGE.
///         (Commonly missions are loaded from persistent storage (flash/EEPROM) into volatile storage (RAM) on startup and written back when they are changed.)
pub const PREFLIGHT_STORAGE_MISSION_ACTION = enum(u32) {
    /// Read current mission data from persistent storage
    MISSION_READ_PERSISTENT = 0,
    /// Write current mission data to persistent storage
    MISSION_WRITE_PERSISTENT = 1,
    /// Erase all mission data stored on the vehicle (both persistent and volatile storage)
    MISSION_RESET_DEFAULT = 2,
};

/// WiFi Mode.
pub const WIFI_CONFIG_AP_MODE = enum(i8) {
    /// WiFi mode is undefined.
    WIFI_CONFIG_AP_MODE_UNDEFINED = 0,
    /// WiFi configured as an access point.
    WIFI_CONFIG_AP_MODE_AP = 1,
    /// WiFi configured as a station connected to an existing local WiFi network.
    WIFI_CONFIG_AP_MODE_STATION = 2,
    /// WiFi disabled.
    WIFI_CONFIG_AP_MODE_DISABLED = 3,
};

/// Flags in the HIL_SENSOR message indicate which fields have updated since the last message
pub const HIL_SENSOR_UPDATED_FLAGS = enum(u32) {
    /// The value in the xacc field has been updated
    HIL_SENSOR_UPDATED_XACC = 1,
    /// The value in the yacc field has been updated
    HIL_SENSOR_UPDATED_YACC = 2,
    /// The value in the zacc field has been updated
    HIL_SENSOR_UPDATED_ZACC = 4,
    /// The value in the xgyro field has been updated
    HIL_SENSOR_UPDATED_XGYRO = 8,
    /// The value in the ygyro field has been updated
    HIL_SENSOR_UPDATED_YGYRO = 16,
    /// The value in the zgyro field has been updated
    HIL_SENSOR_UPDATED_ZGYRO = 32,
    /// The value in the xmag field has been updated
    HIL_SENSOR_UPDATED_XMAG = 64,
    /// The value in the ymag field has been updated
    HIL_SENSOR_UPDATED_YMAG = 128,
    /// The value in the zmag field has been updated
    HIL_SENSOR_UPDATED_ZMAG = 256,
    /// The value in the abs_pressure field has been updated
    HIL_SENSOR_UPDATED_ABS_PRESSURE = 512,
    /// The value in the diff_pressure field has been updated
    HIL_SENSOR_UPDATED_DIFF_PRESSURE = 1024,
    /// The value in the pressure_alt field has been updated
    HIL_SENSOR_UPDATED_PRESSURE_ALT = 2048,
    /// The value in the temperature field has been updated
    HIL_SENSOR_UPDATED_TEMPERATURE = 4096,
    /// Full reset of attitude/position/velocities/etc was performed in sim (Bit 31).
    HIL_SENSOR_UPDATED_RESET = 2147483648,
};

/// A data stream is not a fixed set of messages, but rather a
///      recommendation to the autopilot software. Individual autopilots may or may not obey
///      the recommended messages.
pub const MAV_DATA_STREAM = enum(u32) {
    /// Enable all data streams
    MAV_DATA_STREAM_ALL = 0,
    /// Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
    MAV_DATA_STREAM_RAW_SENSORS = 1,
    /// Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
    MAV_DATA_STREAM_EXTENDED_STATUS = 2,
    /// Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
    MAV_DATA_STREAM_RC_CHANNELS = 3,
    /// Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
    MAV_DATA_STREAM_RAW_CONTROLLER = 4,
    /// Enable LOCAL_POSITION, GLOBAL_POSITION_INT messages.
    MAV_DATA_STREAM_POSITION = 6,
    /// Dependent on the autopilot
    MAV_DATA_STREAM_EXTRA1 = 10,
    /// Dependent on the autopilot
    MAV_DATA_STREAM_EXTRA2 = 11,
    /// Dependent on the autopilot
    MAV_DATA_STREAM_EXTRA3 = 12,
};

/// ADSB classification for the type of vehicle emitting the transponder signal
pub const ADSB_EMITTER_TYPE = enum(u8) {
    ADSB_EMITTER_TYPE_NO_INFO = 0,
    ADSB_EMITTER_TYPE_LIGHT = 1,
    ADSB_EMITTER_TYPE_SMALL = 2,
    ADSB_EMITTER_TYPE_LARGE = 3,
    ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE = 4,
    ADSB_EMITTER_TYPE_HEAVY = 5,
    ADSB_EMITTER_TYPE_HIGHLY_MANUV = 6,
    ADSB_EMITTER_TYPE_ROTOCRAFT = 7,
    ADSB_EMITTER_TYPE_UNASSIGNED = 8,
    ADSB_EMITTER_TYPE_GLIDER = 9,
    ADSB_EMITTER_TYPE_LIGHTER_AIR = 10,
    ADSB_EMITTER_TYPE_PARACHUTE = 11,
    ADSB_EMITTER_TYPE_ULTRA_LIGHT = 12,
    ADSB_EMITTER_TYPE_UNASSIGNED2 = 13,
    ADSB_EMITTER_TYPE_UAV = 14,
    ADSB_EMITTER_TYPE_SPACE = 15,
    ADSB_EMITTER_TYPE_UNASSGINED3 = 16,
    ADSB_EMITTER_TYPE_EMERGENCY_SURFACE = 17,
    ADSB_EMITTER_TYPE_SERVICE_SURFACE = 18,
    ADSB_EMITTER_TYPE_POINT_OBSTACLE = 19,
};

/// Enumeration of possible mount operation modes. This message is used by obsolete/deprecated gimbal messages.
pub const MAV_MOUNT_MODE = enum(u32) {
    /// Load and keep safe position (Roll,Pitch,Yaw) from permanent memory and stop stabilization
    MAV_MOUNT_MODE_RETRACT = 0,
    /// Load and keep neutral position (Roll,Pitch,Yaw) from permanent memory.
    MAV_MOUNT_MODE_NEUTRAL = 1,
    /// Load neutral position and start MAVLink Roll,Pitch,Yaw control with stabilization
    MAV_MOUNT_MODE_MAVLINK_TARGETING = 2,
    /// Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
    MAV_MOUNT_MODE_RC_TARGETING = 3,
    /// Load neutral position and start to point to Lat,Lon,Alt
    MAV_MOUNT_MODE_GPS_POINT = 4,
    /// Gimbal tracks system with specified system ID
    MAV_MOUNT_MODE_SYSID_TARGET = 5,
    /// Gimbal tracks home position
    MAV_MOUNT_MODE_HOME_LOCATION = 6,
};

/// Sequence that motors are tested when using MAV_CMD_DO_MOTOR_TEST.
pub const MOTOR_TEST_ORDER = enum(u32) {
    /// Default autopilot motor test method.
    MOTOR_TEST_ORDER_DEFAULT = 0,
    /// Motor numbers are specified as their index in a predefined vehicle-specific sequence.
    MOTOR_TEST_ORDER_SEQUENCE = 1,
    /// Motor numbers are specified as the output as labeled on the board.
    MOTOR_TEST_ORDER_BOARD = 2,
};

pub const MAV_ODID_AUTH_TYPE = enum(u8) {
    /// No authentication type is specified.
    MAV_ODID_AUTH_TYPE_NONE = 0,
    /// Signature for the UAS (Unmanned Aircraft System) ID.
    MAV_ODID_AUTH_TYPE_UAS_ID_SIGNATURE = 1,
    /// Signature for the Operator ID.
    MAV_ODID_AUTH_TYPE_OPERATOR_ID_SIGNATURE = 2,
    /// Signature for the entire message set.
    MAV_ODID_AUTH_TYPE_MESSAGE_SET_SIGNATURE = 3,
    /// Authentication is provided by Network Remote ID.
    MAV_ODID_AUTH_TYPE_NETWORK_REMOTE_ID = 4,
    /// The exact authentication type is indicated by the first byte of authentication_data and these type values are managed by ICAO.
    MAV_ODID_AUTH_TYPE_SPECIFIC_AUTHENTICATION = 5,
};

/// Bitmap of options for the MAV_CMD_DO_REPOSITION
pub const MAV_DO_REPOSITION_FLAGS = enum(u32) {
    /// The aircraft should immediately transition into guided. This should not be set for follow me applications
    MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1,
};

pub const MAVLINK_DATA_STREAM_TYPE = enum(u8) {
    MAVLINK_DATA_STREAM_IMG_JPEG = 0,
    MAVLINK_DATA_STREAM_IMG_BMP = 1,
    MAVLINK_DATA_STREAM_IMG_RAW8U = 2,
    MAVLINK_DATA_STREAM_IMG_RAW32U = 3,
    MAVLINK_DATA_STREAM_IMG_PGM = 4,
    MAVLINK_DATA_STREAM_IMG_PNG = 5,
};

/// Gimbal manager high level capability flags (bitmap). The first 16 bits are identical to the GIMBAL_DEVICE_CAP_FLAGS. However, the gimbal manager does not need to copy the flags from the gimbal but can also enhance the capabilities and thus add flags.
pub const GIMBAL_MANAGER_CAP_FLAGS = enum(u32) {
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RETRACT.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_RETRACT = 1,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_NEUTRAL.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_NEUTRAL = 2,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_AXIS = 4,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_FOLLOW = 8,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_ROLL_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_ROLL_LOCK = 16,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_AXIS = 32,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_FOLLOW = 64,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_PITCH_LOCK = 128,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_AXIS = 256,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_FOLLOW.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_FOLLOW = 512,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_LOCK.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_YAW_LOCK = 1024,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_INFINITE_YAW.
    GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_INFINITE_YAW = 2048,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME.
    GIMBAL_MANAGER_CAP_FLAGS_SUPPORTS_YAW_IN_EARTH_FRAME = 4096,
    /// Based on GIMBAL_DEVICE_CAP_FLAGS_HAS_RC_INPUTS.
    GIMBAL_MANAGER_CAP_FLAGS_HAS_RC_INPUTS = 8192,
    /// Gimbal manager supports to point to a local position.
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_LOCAL = 65536,
    /// Gimbal manager supports to point to a global latitude, longitude, altitude position.
    GIMBAL_MANAGER_CAP_FLAGS_CAN_POINT_LOCATION_GLOBAL = 131072,
};

pub const MAV_ODID_DESC_TYPE = enum(u8) {
    /// Optional free-form text description of the purpose of the flight.
    MAV_ODID_DESC_TYPE_TEXT = 0,
    /// Optional additional clarification when status == MAV_ODID_STATUS_EMERGENCY.
    MAV_ODID_DESC_TYPE_EMERGENCY = 1,
    /// Optional additional clarification when status != MAV_ODID_STATUS_EMERGENCY.
    MAV_ODID_DESC_TYPE_EXTENDED_STATUS = 2,
};

