// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

/// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way.
pub const POSITION_TARGET_GLOBAL_INT = struct {
    pub const MSG_ID = 87;
    /// Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
    time_boot_ms: u32,

    /// Latitude in WGS84 frame
    lat_int: i32,

    /// Longitude in WGS84 frame
    lon_int: i32,

    /// Altitude (MSL, AGL or relative to home altitude, depending on frame)
    alt: f32,

    /// X velocity in NED frame
    vx: f32,

    /// Y velocity in NED frame
    vy: f32,

    /// Z velocity in NED frame
    vz: f32,

    /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afx: f32,

    /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afy: f32,

    /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afz: f32,

    /// yaw setpoint
    yaw: f32,

    /// yaw rate setpoint
    yaw_rate: f32,

    /// Bitmap to indicate which dimensions should be ignored by the vehicle.
    type_mask: enums.POSITION_TARGET_TYPEMASK.Type,

    /// Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
    coordinate_frame: enums.MAV_FRAME,

};

/// Cumulative distance traveled for each reported wheel.
pub const WHEEL_DISTANCE = struct {
    pub const MSG_ID = 9000;
    /// Distance reported by individual wheel encoders. Forward rotations increase values, reverse rotations decrease them. Not all wheels will necessarily have wheel encoders; the mapping of encoders to wheel positions must be agreed/understood by the endpoints.
    distance: [16]f64,

    /// Timestamp (synced to UNIX time or since system boot).
    time_usec: u64,

    /// Number of wheels reported.
    count: u8,

};

/// Metrics typically displayed on a HUD for fixed wing aircraft.
pub const VFR_HUD = struct {
    pub const MSG_ID = 74;
    /// Vehicle speed in form appropriate for vehicle type. For standard aircraft this is typically calibrated airspeed (CAS) or indicated airspeed (IAS) - either of which can be used by a pilot to estimate stall speed.
    airspeed: f32,

    /// Current ground speed.
    groundspeed: f32,

    /// Current altitude (MSL).
    alt: f32,

    /// Current climb rate.
    climb: f32,

    /// Current heading in compass units (0-360, 0=north).
    heading: i16,

    /// Current throttle setting (0 to 100).
    throttle: u16,

};

/// Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate.
pub const SERIAL_CONTROL = struct {
    pub const MSG_ID = 126;
    /// serial data
    data: [70]u8,

    /// Baudrate of transfer. Zero means no change.
    baudrate: u32,

    /// Timeout for reply data
    timeout: u16,

    /// Serial control device type.
    device: enums.SERIAL_CONTROL_DEV,

    /// Bitmap of serial control flags.
    flags: enums.SERIAL_CONTROL_FLAG.Type,

    /// how many bytes in this transfer
    count: u8,

    //Extension Field
    /// System ID
    target_system: u8,

    //Extension Field
    /// Component ID
    target_component: u8,

};

/// An OpenDroneID message pack is a container for multiple encoded OpenDroneID messages (i.e. not in the format given for the above message descriptions but after encoding into the compressed OpenDroneID byte format). Used e.g. when transmitting on Bluetooth 5.0 Long Range/Extended Advertising or on WiFi Neighbor Aware Networking or on WiFi Beacon.
pub const OPEN_DRONE_ID_MESSAGE_PACK = struct {
    pub const MSG_ID = 12915;
    /// Concatenation of encoded OpenDroneID messages. Shall be filled with nulls in the unused portion of the field.
    messages: [225]u8,

    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// This field must currently always be equal to 25 (bytes), since all encoded OpenDroneID messages are specified to have this length.
    single_message_size: u8,

    /// Number of encoded messages in the pack (not the number of bytes). Allowed range is 1 - 9.
    msg_pack_size: u8,

};

/// Publishes the GPS coordinates of the vehicle local origin (0,0,0) position. Emitted whenever a new GPS-Local position mapping is requested or set - e.g. following SET_GPS_GLOBAL_ORIGIN message.
pub const GPS_GLOBAL_ORIGIN = struct {
    pub const MSG_ID = 49;
    /// Latitude (WGS84)
    latitude: i32,

    /// Longitude (WGS84)
    longitude: i32,

    /// Altitude (MSL). Positive for up.
    altitude: i32,

    //Extension Field
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

};

/// Superseded by ACTUATOR_OUTPUT_STATUS. The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.
pub const SERVO_OUTPUT_RAW = struct {
    pub const MSG_ID = 36;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u32,

    /// Servo output 1 value
    servo1_raw: u16,

    /// Servo output 2 value
    servo2_raw: u16,

    /// Servo output 3 value
    servo3_raw: u16,

    /// Servo output 4 value
    servo4_raw: u16,

    /// Servo output 5 value
    servo5_raw: u16,

    /// Servo output 6 value
    servo6_raw: u16,

    /// Servo output 7 value
    servo7_raw: u16,

    /// Servo output 8 value
    servo8_raw: u16,

    /// Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
    port: u8,

    //Extension Field
    /// Servo output 9 value
    servo9_raw: u16,

    //Extension Field
    /// Servo output 10 value
    servo10_raw: u16,

    //Extension Field
    /// Servo output 11 value
    servo11_raw: u16,

    //Extension Field
    /// Servo output 12 value
    servo12_raw: u16,

    //Extension Field
    /// Servo output 13 value
    servo13_raw: u16,

    //Extension Field
    /// Servo output 14 value
    servo14_raw: u16,

    //Extension Field
    /// Servo output 15 value
    servo15_raw: u16,

    //Extension Field
    /// Servo output 16 value
    servo16_raw: u16,

};

/// This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!
pub const MISSION_WRITE_PARTIAL_LIST = struct {
    pub const MSG_ID = 38;
    /// Start index. Must be smaller / equal to the largest index of the current onboard list.
    start_index: i16,

    /// End index, equal or greater than start index.
    end_index: i16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Message that announces the sequence number of the current target mission item (that the system will fly towards/execute when the mission is running).
///         This message should be streamed all the time (nominally at 1Hz).
///         This message should be emitted following a call to MAV_CMD_DO_SET_MISSION_CURRENT or MISSION_SET_CURRENT.
pub const MISSION_CURRENT = struct {
    pub const MSG_ID = 42;
    /// Sequence
    seq: u16,

    //Extension Field
    /// Total number of mission items on vehicle (on last item, sequence == total). If the autopilot stores its home location as part of the mission this will be excluded from the total. 0: Not supported, UINT16_MAX if no mission is present on the vehicle.
    total: u16,

    //Extension Field
    /// Mission state machine state. MISSION_STATE_UNKNOWN if state reporting not supported.
    mission_state: enums.MISSION_STATE,

    //Extension Field
    /// Vehicle is in a mode that can execute mission items or suspended. 0: Unknown, 1: In mission mode, 2: Suspended (not in mission mode).
    mission_mode: u8,

    //Extension Field
    /// Id of current on-vehicle mission plan, or 0 if IDs are not supported or there is no mission loaded. GCS can use this to track changes to the mission plan type. The same value is returned on mission upload (in the MISSION_ACK).
    mission_id: u32,

    //Extension Field
    /// Id of current on-vehicle fence plan, or 0 if IDs are not supported or there is no fence loaded. GCS can use this to track changes to the fence plan type. The same value is returned on fence upload (in the MISSION_ACK).
    fence_id: u32,

    //Extension Field
    /// Id of current on-vehicle rally point plan, or 0 if IDs are not supported or there are no rally points loaded. GCS can use this to track changes to the rally point plan type. The same value is returned on rally point upload (in the MISSION_ACK).
    rally_points_id: u32,

};

/// An ack for a LOGGING_DATA_ACKED message
pub const LOGGING_ACK = struct {
    pub const MSG_ID = 268;
    /// sequence number (must match the one in LOGGING_DATA_ACKED)
    sequence: u16,

    /// system ID of the target
    target_system: u8,

    /// component ID of the target
    target_component: u8,

};

/// Component information message, which may be requested using MAV_CMD_REQUEST_MESSAGE.
pub const COMPONENT_INFORMATION = struct {
    pub const MSG_ID = 395;
    /// MAVLink FTP URI for the general metadata file (COMP_METADATA_TYPE_GENERAL), which may be compressed with xz. The file contains general component metadata, and may contain URI links for additional metadata (see COMP_METADATA_TYPE). The information is static from boot, and may be generated at compile time. The string needs to be zero terminated.
    general_metadata_uri: [100]u8,

    /// (Optional) MAVLink FTP URI for the peripherals metadata file (COMP_METADATA_TYPE_PERIPHERALS), which may be compressed with xz. This contains data about "attached components" such as UAVCAN nodes. The peripherals are in a separate file because the information must be generated dynamically at runtime. The string needs to be zero terminated.
    peripherals_metadata_uri: [100]u8,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// CRC32 of the general metadata file (general_metadata_uri).
    general_metadata_file_crc: u32,

    /// CRC32 of peripherals metadata file (peripherals_metadata_uri).
    peripherals_metadata_file_crc: u32,

};

/// Battery information. Updates GCS with flight controller battery status. Smart batteries also use this message, but may additionally send BATTERY_INFO.
pub const BATTERY_STATUS = struct {
    pub const MSG_ID = 147;
    /// Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).
    voltages: [10]u16,

    /// Consumed charge, -1: autopilot does not provide consumption estimate
    current_consumed: i32,

    /// Consumed energy, -1: autopilot does not provide energy consumption estimate
    energy_consumed: i32,

    /// Temperature of the battery. INT16_MAX for unknown temperature.
    temperature: i16,

    /// Battery current, -1: autopilot does not measure the current
    current_battery: i16,

    /// Battery ID
    id: u8,

    /// Function of the battery
    battery_function: enums.MAV_BATTERY_FUNCTION,

    /// Type (chemistry) of the battery
    type: enums.MAV_BATTERY_TYPE,

    /// Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.
    battery_remaining: i8,

    //Extension Field
    /// Remaining battery time, 0: autopilot does not provide remaining battery time estimate
    time_remaining: i32,

    //Extension Field
    /// State for extent of discharge, provided by autopilot for warning or external reactions
    charge_state: enums.MAV_BATTERY_CHARGE_STATE,

    //Extension Field
    /// Battery voltages for cells 11 to 14. Cells above the valid cell count for this battery should have a value of 0, where zero indicates not supported (note, this is different than for the voltages field and allows empty byte truncation). If the measured value is 0 then 1 should be sent instead.
    voltages_ext: [4]u16,

    //Extension Field
    /// Battery mode. Default (0) is that battery mode reporting is not supported or battery is in normal-use mode.
    mode: enums.MAV_BATTERY_MODE,

    //Extension Field
    /// Fault/health indications. These should be set when charge_state is MAV_BATTERY_CHARGE_STATE_FAILED or MAV_BATTERY_CHARGE_STATE_UNHEALTHY (if not, fault reporting is not supported).
    fault_bitmask: enums.MAV_BATTERY_FAULT.Type,

};

/// Sets the home position.
/// The home position is the default position that the system will return to and land on.
///         The position is set automatically by the system during the takeoff (and may also be set using this message).
///         The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.
///         Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach.
///         The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
///         Note: the current home position may be emitted in a HOME_POSITION message on request (using MAV_CMD_REQUEST_MESSAGE with param1=242).
pub const SET_HOME_POSITION = struct {
    pub const MSG_ID = 243;
    /// Latitude (WGS84)
    latitude: i32,

    /// Longitude (WGS84)
    longitude: i32,

    /// Altitude (MSL). Positive for up.
    altitude: i32,

    /// Local X position of this position in the local coordinate frame (NED)
    x: f32,

    /// Local Y position of this position in the local coordinate frame (NED)
    y: f32,

    /// Local Z position of this position in the local coordinate frame (NED: positive "down")
    z: f32,

    /// World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground
    q: [4]f32,

    /// Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    approach_x: f32,

    /// Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    approach_y: f32,

    /// Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    approach_z: f32,

    /// System ID.
    target_system: u8,

    //Extension Field
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

};

/// Play vehicle tone/tune (buzzer). Supersedes message PLAY_TUNE.
pub const PLAY_TUNE_V2 = struct {
    pub const MSG_ID = 400;
    /// Tune definition as a NULL-terminated string.
    tune: [248]u8,

    /// Tune format
    format: enums.TUNE_FORMAT,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint.
pub const MISSION_ITEM_REACHED = struct {
    pub const MSG_ID = 46;
    /// Sequence
    seq: u16,

};

/// Status generated by radio and injected into MAVLink stream.
pub const RADIO_STATUS = struct {
    pub const MSG_ID = 109;
    /// Count of radio packet receive errors (since boot).
    rxerrors: u16,

    /// Count of error corrected radio packets (since boot).
    fixed: u16,

    /// Local (message sender) received signal strength indication in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
    rssi: u8,

    /// Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
    remrssi: u8,

    /// Remaining free transmitter buffer space.
    txbuf: u8,

    /// Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], UINT8_MAX: invalid/unknown.
    noise: u8,

    /// Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], UINT8_MAX: invalid/unknown.
    remnoise: u8,

};

/// Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
pub const MEMORY_VECT = struct {
    pub const MSG_ID = 249;
    /// Memory contents at specified address
    value: [32]i8,

    /// Starting address of the debug variables
    address: u16,

    /// Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below
    ver: u8,

    /// Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14
    type: u8,

};

/// Information about a potential collision
pub const COLLISION = struct {
    pub const MSG_ID = 247;
    /// Unique identifier, domain based on src field
    id: u32,

    /// Estimated time until collision occurs
    time_to_minimum_delta: f32,

    /// Closest vertical distance between vehicle and object
    altitude_minimum_delta: f32,

    /// Closest horizontal distance between vehicle and object
    horizontal_minimum_delta: f32,

    /// Collision data source
    src: enums.MAV_COLLISION_SRC,

    /// Action that is being taken to avoid this collision
    action: enums.MAV_COLLISION_ACTION,

    /// How concerned the aircraft is about this collision
    threat_level: enums.MAV_COLLISION_THREAT_LEVEL,

};

/// Information about a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
pub const CAMERA_INFORMATION = struct {
    pub const MSG_ID = 259;
    /// Camera definition URI (if any, otherwise only basic functions will be available). HTTP- (http://) and MAVLink FTP- (mavlinkftp://) formatted URIs are allowed (and both must be supported by any GCS that implements the Camera Protocol). The definition file may be xz compressed, which will be indicated by the file extension .xml.xz (a GCS that implements the protocol must support decompressing the file). The string needs to be zero terminated.  Use a zero-length string if not known.
    cam_definition_uri: [140]u8,

    /// Name of the camera vendor
    vendor_name: [32]u8,

    /// Name of the camera model
    model_name: [32]u8,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Version of the camera firmware, encoded as: `(Dev & 0xff) << 24 \| (Patch & 0xff) << 16 \| (Minor & 0xff) << 8 \| (Major & 0xff)`. Use 0 if not known.
    firmware_version: u32,

    /// Focal length. Use NaN if not known.
    focal_length: f32,

    /// Image sensor size horizontal. Use NaN if not known.
    sensor_size_h: f32,

    /// Image sensor size vertical. Use NaN if not known.
    sensor_size_v: f32,

    /// Bitmap of camera capability flags.
    flags: enums.CAMERA_CAP_FLAGS.Type,

    /// Horizontal image resolution. Use 0 if not known.
    resolution_h: u16,

    /// Vertical image resolution. Use 0 if not known.
    resolution_v: u16,

    /// Camera definition version (iteration).  Use 0 if not known.
    cam_definition_version: u16,

    /// Reserved for a lens ID.  Use 0 if not known.
    lens_id: u8,

    //Extension Field
    /// Gimbal id of a gimbal associated with this camera. This is the component id of the gimbal device, or 1-6 for non mavlink gimbals. Use 0 if no gimbal is associated with the camera.
    gimbal_device_id: u8,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety.
pub const AUTH_KEY = struct {
    pub const MSG_ID = 7;
    /// key
    key: [32]u8,

};

/// RPM sensor data message.
pub const RAW_RPM = struct {
    pub const MSG_ID = 339;
    /// Indicated rate
    frequency: f32,

    /// Index of this RPM sensor (0-indexed)
    index: u8,

};

/// Set gimbal manager pitch and yaw angles (high rate message). This message is to be sent to the gimbal manager (e.g. from a ground station) and will be ignored by gimbal devices. Angles and rates can be set to NaN according to use case. Use MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW for low-rate adjustments that require confirmation.
pub const GIMBAL_MANAGER_SET_PITCHYAW = struct {
    pub const MSG_ID = 287;
    /// High level gimbal manager flags to use.
    flags: enums.GIMBAL_MANAGER_FLAGS.Type,

    /// Pitch angle (positive: up, negative: down, NaN to be ignored).
    pitch: f32,

    /// Yaw angle (positive: to the right, negative: to the left, NaN to be ignored).
    yaw: f32,

    /// Pitch angular rate (positive: up, negative: down, NaN to be ignored).
    pitch_rate: f32,

    /// Yaw angular rate (positive: to the right, negative: to the left, NaN to be ignored).
    yaw_rate: f32,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
    gimbal_device_id: u8,

};

/// The global position resulting from GPS and sensor fusion.
pub const UTM_GLOBAL_POSITION = struct {
    pub const MSG_ID = 340;
    /// Unique UAS ID.
    uas_id: [18]u8,

    /// Time of applicability of position (microseconds since UNIX epoch).
    time: u64,

    /// Latitude (WGS84)
    lat: i32,

    /// Longitude (WGS84)
    lon: i32,

    /// Altitude (WGS84)
    alt: i32,

    /// Altitude above ground
    relative_alt: i32,

    /// Next waypoint, latitude (WGS84)
    next_lat: i32,

    /// Next waypoint, longitude (WGS84)
    next_lon: i32,

    /// Next waypoint, altitude (WGS84)
    next_alt: i32,

    /// Ground X speed (latitude, positive north)
    vx: i16,

    /// Ground Y speed (longitude, positive east)
    vy: i16,

    /// Ground Z speed (altitude, positive down)
    vz: i16,

    /// Horizontal position uncertainty (standard deviation)
    h_acc: u16,

    /// Altitude uncertainty (standard deviation)
    v_acc: u16,

    /// Speed uncertainty (standard deviation)
    vel_acc: u16,

    /// Time until next update. Set to 0 if unknown or in data driven mode.
    update_rate: u16,

    /// Flight state
    flight_state: enums.UTM_FLIGHT_STATE,

    /// Bitwise OR combination of the data available flags.
    flags: enums.UTM_DATA_AVAIL_FLAGS.Type,

};

/// Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called. If there are no log files available this request shall be answered with one LOG_ENTRY message with id = 0 and num_logs = 0.
pub const LOG_REQUEST_LIST = struct {
    pub const MSG_ID = 117;
    /// First log id (0 for first available)
    start: u16,

    /// Last log id (0xffff for last available)
    end: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Data for filling the OpenDroneID Basic ID message. This and the below messages are primarily meant for feeding data to/from an OpenDroneID implementation. E.g. https://github.com/opendroneid/opendroneid-core-c. These messages are compatible with the ASTM F3411 Remote ID standard and the ASD-STAN prEN 4709-002 Direct Remote ID standard. Additional information and usage of these messages is documented at https://mavlink.io/en/services/opendroneid.html.
pub const OPEN_DRONE_ID_BASIC_ID = struct {
    pub const MSG_ID = 12900;
    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// UAS (Unmanned Aircraft System) ID following the format specified by id_type. Shall be filled with nulls in the unused portion of the field.
    uas_id: [20]u8,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// Indicates the format for the uas_id field of this message.
    id_type: enums.MAV_ODID_ID_TYPE,

    /// Indicates the type of UA (Unmanned Aircraft).
    ua_type: enums.MAV_ODID_UA_TYPE,

};

/// Message encoding a mission item. This message is emitted to announce
///                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN may be used to indicate an optional/default value (e.g. to use the system's current latitude or yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.
pub const MISSION_ITEM = struct {
    pub const MSG_ID = 39;
    /// PARAM1, see MAV_CMD enum
    param1: f32,

    /// PARAM2, see MAV_CMD enum
    param2: f32,

    /// PARAM3, see MAV_CMD enum
    param3: f32,

    /// PARAM4, see MAV_CMD enum
    param4: f32,

    /// PARAM5 / local: X coordinate, global: latitude
    x: f32,

    /// PARAM6 / local: Y coordinate, global: longitude
    y: f32,

    /// PARAM7 / local: Z coordinate, global: altitude (relative or absolute, depending on frame).
    z: f32,

    /// Sequence
    seq: u16,

    /// The scheduled action for the waypoint.
    command: enums.MAV_CMD,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// The coordinate system of the waypoint.
    frame: enums.MAV_FRAME,

    /// false:0, true:1
    current: u8,

    /// Autocontinue to next waypoint. 0: false, 1: true. Set false to pause mission after the item completes.
    autocontinue: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
pub const PARAM_VALUE = struct {
    pub const MSG_ID = 22;
    /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Onboard parameter value
    param_value: f32,

    /// Total number of onboard parameters
    param_count: u16,

    /// Index of this onboard parameter
    param_index: u16,

    /// Onboard parameter type.
    param_type: enums.MAV_PARAM_TYPE,

};

/// The IMU readings in SI units in NED body frame
pub const HIGHRES_IMU = struct {
    pub const MSG_ID = 105;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X acceleration
    xacc: f32,

    /// Y acceleration
    yacc: f32,

    /// Z acceleration
    zacc: f32,

    /// Angular speed around X axis
    xgyro: f32,

    /// Angular speed around Y axis
    ygyro: f32,

    /// Angular speed around Z axis
    zgyro: f32,

    /// X Magnetic field
    xmag: f32,

    /// Y Magnetic field
    ymag: f32,

    /// Z Magnetic field
    zmag: f32,

    /// Absolute pressure
    abs_pressure: f32,

    /// Differential pressure
    diff_pressure: f32,

    /// Altitude calculated from pressure
    pressure_alt: f32,

    /// Temperature
    temperature: f32,

    /// Bitmap for fields that have updated since last message
    fields_updated: enums.HIGHRES_IMU_UPDATED_FLAGS.Type,

    //Extension Field
    /// Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
    id: u8,

};

/// Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations.
pub const HIL_STATE = struct {
    pub const MSG_ID = 90;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Roll angle
    roll: f32,

    /// Pitch angle
    pitch: f32,

    /// Yaw angle
    yaw: f32,

    /// Body frame roll / phi angular speed
    rollspeed: f32,

    /// Body frame pitch / theta angular speed
    pitchspeed: f32,

    /// Body frame yaw / psi angular speed
    yawspeed: f32,

    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

    /// Altitude
    alt: i32,

    /// Ground X Speed (Latitude)
    vx: i16,

    /// Ground Y Speed (Longitude)
    vy: i16,

    /// Ground Z Speed (Altitude)
    vz: i16,

    /// X acceleration
    xacc: i16,

    /// Y acceleration
    yacc: i16,

    /// Z acceleration
    zacc: i16,

};

/// The current system altitude.
pub const ALTITUDE = struct {
    pub const MSG_ID = 141;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights.
    altitude_monotonic: f32,

    /// This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output MSL by default and not the WGS84 altitude.
    altitude_amsl: f32,

    /// This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive.
    altitude_local: f32,

    /// This is the altitude above the home position. It resets on each change of the current home position.
    altitude_relative: f32,

    /// This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown.
    altitude_terrain: f32,

    /// This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available.
    bottom_clearance: f32,

};

/// Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable signing
pub const SETUP_SIGNING = struct {
    pub const MSG_ID = 256;
    /// signing key
    secret_key: [32]u8,

    /// initial timestamp
    initial_timestamp: u64,

    /// system id of the target
    target_system: u8,

    /// component ID of the target
    target_component: u8,

};

/// Fuel status.
///         This message provides "generic" fuel level information for  in a GCS and for triggering failsafes in an autopilot.
///         The fuel type and associated units for fields in this message are defined in the enum MAV_FUEL_TYPE.
/// 
///         The reported `consumed_fuel` and `remaining_fuel` must only be supplied if measured: they must not be inferred from the `maximum_fuel` and the other value.
///         A recipient can assume that if these fields are supplied they are accurate.
///         If not provided, the recipient can infer `remaining_fuel` from `maximum_fuel` and `consumed_fuel` on the assumption that the fuel was initially at its maximum (this is what battery monitors assume).
///         Note however that this is an assumption, and the UI should prompt the user appropriately (i.e. notify user that they should fill the tank before boot).
/// 
///         This kind of information may also be sent in fuel-specific messages such as BATTERY_STATUS_V2.
///         If both messages are sent for the same fuel system, the ids and corresponding information must match.
/// 
///         This should be streamed (nominally at 0.1 Hz).
pub const FUEL_STATUS = struct {
    pub const MSG_ID = 371;
    /// Capacity when full. Must be provided.
    maximum_fuel: f32,

    /// Consumed fuel (measured). This value should not be inferred: if not measured set to NaN. NaN: field not provided.
    consumed_fuel: f32,

    /// Remaining fuel until empty (measured). The value should not be inferred: if not measured set to NaN. NaN: field not provided.
    remaining_fuel: f32,

    /// Positive value when emptying/using, and negative if filling/replacing. NaN: field not provided.
    flow_rate: f32,

    /// Fuel temperature. NaN: field not provided.
    temperature: f32,

    /// Fuel type. Defines units for fuel capacity and consumption fields above.
    fuel_type: enums.MAV_FUEL_TYPE,

    /// Fuel ID. Must match ID of other messages for same fuel system, such as BATTERY_STATUS_V2.
    id: u8,

    /// Percentage of remaining fuel, relative to full. Values: [0-100], UINT8_MAX: field not provided.
    percent_remaining: u8,

};

/// The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values.
pub const RAW_PRESSURE = struct {
    pub const MSG_ID = 28;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Absolute pressure (raw)
    press_abs: i16,

    /// Differential pressure 1 (raw, 0 if nonexistent)
    press_diff1: i16,

    /// Differential pressure 2 (raw, 0 if nonexistent)
    press_diff2: i16,

    /// Raw Temperature measurement (raw)
    temperature: i16,

};

/// Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz).
pub const STATUSTEXT = struct {
    pub const MSG_ID = 253;
    /// Status text message, without null termination character
    text: [50]u8,

    /// Severity of status. Relies on the definitions within RFC-5424.
    severity: enums.MAV_SEVERITY,

    //Extension Field
    /// Unique (opaque) identifier for this statustext message.  May be used to reassemble a logical long-statustext message from a sequence of chunks.  A value of zero indicates this is the only chunk in the sequence and the message can be emitted immediately.
    id: u16,

    //Extension Field
    /// This chunk's sequence number; indexing is from zero.  Any null character in the text field is taken to mean this was the last chunk.
    chunk_seq: u8,

};

/// Current status about a high level gimbal manager. This message should be broadcast at a low regular rate (e.g. 5Hz).
pub const GIMBAL_MANAGER_STATUS = struct {
    pub const MSG_ID = 281;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// High level gimbal manager flags currently applied.
    flags: enums.GIMBAL_MANAGER_FLAGS.Type,

    /// Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
    gimbal_device_id: u8,

    /// System ID of MAVLink component with primary control, 0 for none.
    primary_control_sysid: u8,

    /// Component ID of MAVLink component with primary control, 0 for none.
    primary_control_compid: u8,

    /// System ID of MAVLink component with secondary control, 0 for none.
    secondary_control_sysid: u8,

    /// Component ID of MAVLink component with secondary control, 0 for none.
    secondary_control_compid: u8,

};

/// Accept / deny control of this MAV
pub const CHANGE_OPERATOR_CONTROL_ACK = struct {
    pub const MSG_ID = 6;
    /// ID of the GCS this message 
    gcs_system_id: u8,

    /// 0: request control of this MAV, 1: Release control of this MAV
    control_request: u8,

    /// 0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control
    ack: u8,

};

/// Vibration levels and accelerometer clipping
pub const VIBRATION = struct {
    pub const MSG_ID = 241;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Vibration levels on X-axis
    vibration_x: f32,

    /// Vibration levels on Y-axis
    vibration_y: f32,

    /// Vibration levels on Z-axis
    vibration_z: f32,

    /// first accelerometer clipping count
    clipping_0: u32,

    /// second accelerometer clipping count
    clipping_1: u32,

    /// third accelerometer clipping count
    clipping_2: u32,

};

/// Request that the vehicle report terrain height at the given location (expected response is a TERRAIN_REPORT). Used by GCS to check if vehicle has all terrain data needed for a mission.
pub const TERRAIN_CHECK = struct {
    pub const MSG_ID = 135;
    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

};

/// Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
pub const NAMED_VALUE_FLOAT = struct {
    pub const MSG_ID = 251;
    /// Name of the debug variable
    name: [10]u8,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Floating point value
    value: f32,

};

/// Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows them to re-request missing parameters after a loss or timeout.
pub const PARAM_EXT_VALUE = struct {
    pub const MSG_ID = 322;
    /// Parameter value
    param_value: [128]u8,

    /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Total number of parameters
    param_count: u16,

    /// Index of this parameter
    param_index: u16,

    /// Parameter type.
    param_type: enums.MAV_PARAM_EXT_TYPE,

};

/// Data for filling the OpenDroneID Location message. The float data types are 32-bit IEEE 754. The Location message provides the location, altitude, direction and speed of the aircraft.
pub const OPEN_DRONE_ID_LOCATION = struct {
    pub const MSG_ID = 12901;
    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// Current latitude of the unmanned aircraft. If unknown: 0 (both Lat/Lon).
    latitude: i32,

    /// Current longitude of the unmanned aircraft. If unknown: 0 (both Lat/Lon).
    longitude: i32,

    /// The altitude calculated from the barometric pressure. Reference is against 29.92inHg or 1013.2mb. If unknown: -1000 m.
    altitude_barometric: f32,

    /// The geodetic altitude as defined by WGS84. If unknown: -1000 m.
    altitude_geodetic: f32,

    /// The current height of the unmanned aircraft above the take-off location or the ground as indicated by height_reference. If unknown: -1000 m.
    height: f32,

    /// Seconds after the full hour with reference to UTC time. Typically the GPS outputs a time-of-week value in milliseconds. First convert that to UTC and then convert for this field using ((float) (time_week_ms % (60*60*1000))) / 1000. If unknown: 0xFFFF.
    timestamp: f32,

    /// Direction over ground (not heading, but direction of movement) measured clockwise from true North: 0 - 35999 centi-degrees. If unknown: 36100 centi-degrees.
    direction: u16,

    /// Ground speed. Positive only. If unknown: 25500 cm/s. If speed is larger than 25425 cm/s, use 25425 cm/s.
    speed_horizontal: u16,

    /// The vertical speed. Up is positive. If unknown: 6300 cm/s. If speed is larger than 6200 cm/s, use 6200 cm/s. If lower than -6200 cm/s, use -6200 cm/s.
    speed_vertical: i16,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// Indicates whether the unmanned aircraft is on the ground or in the air.
    status: enums.MAV_ODID_STATUS,

    /// Indicates the reference point for the height field.
    height_reference: enums.MAV_ODID_HEIGHT_REF,

    /// The accuracy of the horizontal position.
    horizontal_accuracy: enums.MAV_ODID_HOR_ACC,

    /// The accuracy of the vertical position.
    vertical_accuracy: enums.MAV_ODID_VER_ACC,

    /// The accuracy of the barometric altitude.
    barometer_accuracy: enums.MAV_ODID_VER_ACC,

    /// The accuracy of the horizontal and vertical speed.
    speed_accuracy: enums.MAV_ODID_SPEED_ACC,

    /// The accuracy of the timestamps.
    timestamp_accuracy: enums.MAV_ODID_TIME_ACC,

};

/// High level message to control a gimbal manually. The angles or angular rates are unitless; the actual rates will depend on internal gimbal manager settings/configuration (e.g. set by parameters). This message is to be sent to the gimbal manager (e.g. from a ground station). Angles and rates can be set to NaN according to use case.
pub const GIMBAL_MANAGER_SET_MANUAL_CONTROL = struct {
    pub const MSG_ID = 288;
    /// High level gimbal manager flags.
    flags: enums.GIMBAL_MANAGER_FLAGS.Type,

    /// Pitch angle unitless (-1..1, positive: up, negative: down, NaN to be ignored).
    pitch: f32,

    /// Yaw angle unitless (-1..1, positive: to the right, negative: to the left, NaN to be ignored).
    yaw: f32,

    /// Pitch angular rate unitless (-1..1, positive: up, negative: down, NaN to be ignored).
    pitch_rate: f32,

    /// Yaw angular rate unitless (-1..1, positive: to the right, negative: to the left, NaN to be ignored).
    yaw_rate: f32,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
    gimbal_device_id: u8,

};

/// Send a command with up to seven parameters to the MAV, where params 5 and 6 are integers and the other values are floats. This is preferred over COMMAND_LONG as it allows the MAV_FRAME to be specified for interpreting positional information, such as altitude. COMMAND_INT is also preferred when sending latitude and longitude data in params 5 and 6, as it allows for greater precision. Param 5 and 6 encode positional data as scaled integers, where the scaling depends on the actual command value. NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value). The command microservice is documented at https://mavlink.io/en/services/command.html
pub const COMMAND_INT = struct {
    pub const MSG_ID = 75;
    /// PARAM1, see MAV_CMD enum
    param1: f32,

    /// PARAM2, see MAV_CMD enum
    param2: f32,

    /// PARAM3, see MAV_CMD enum
    param3: f32,

    /// PARAM4, see MAV_CMD enum
    param4: f32,

    /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
    x: i32,

    /// PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7
    y: i32,

    /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame).
    z: f32,

    /// The scheduled action for the mission item.
    command: enums.MAV_CMD,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// The coordinate system of the COMMAND.
    frame: enums.MAV_FRAME,

    /// Not used.
    current: u8,

    /// Not used (set 0).
    autocontinue: u8,

};

/// To debug something using a named 3D vector.
pub const DEBUG_VECT = struct {
    pub const MSG_ID = 250;
    /// Name
    name: [10]u8,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// x
    x: f32,

    /// y
    y: f32,

    /// z
    z: f32,

};

/// Camera absolute thermal range. This can be streamed when the associated VIDEO_STREAM_STATUS `flag` field bit VIDEO_STREAM_STATUS_FLAGS_THERMAL_RANGE_ENABLED is set, but a GCS may choose to only request it for the current active stream. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval (param3 indicates the stream id of the current camera, or 0 for all streams, param4 indicates the target camera_device_id for autopilot-attached cameras or 0 for MAVLink cameras).
pub const CAMERA_THERMAL_RANGE = struct {
    pub const MSG_ID = 277;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Temperature max.
    max: f32,

    /// Temperature max point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
    max_point_x: f32,

    /// Temperature max point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
    max_point_y: f32,

    /// Temperature min.
    min: f32,

    /// Temperature min point x value (normalized 0..1, 0 is left, 1 is right), NAN if unknown.
    min_point_x: f32,

    /// Temperature min point y value (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown.
    min_point_y: f32,

    /// Video Stream ID (1 for first, 2 for second, etc.)
    stream_id: u8,

    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N.
pub const DEBUG = struct {
    pub const MSG_ID = 254;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// DEBUG value
    value: f32,

    /// index of debug variable
    ind: u8,

};

/// Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval.
pub const CAMERA_TRACKING_IMAGE_STATUS = struct {
    pub const MSG_ID = 275;
    /// Current tracked point x value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is left, 1 is right), NAN if unknown
    point_x: f32,

    /// Current tracked point y value if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
    point_y: f32,

    /// Current tracked radius if CAMERA_TRACKING_MODE_POINT (normalized 0..1, 0 is image left, 1 is image right), NAN if unknown
    radius: f32,

    /// Current tracked rectangle top x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
    rec_top_x: f32,

    /// Current tracked rectangle top y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
    rec_top_y: f32,

    /// Current tracked rectangle bottom x value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is left, 1 is right), NAN if unknown
    rec_bottom_x: f32,

    /// Current tracked rectangle bottom y value if CAMERA_TRACKING_MODE_RECTANGLE (normalized 0..1, 0 is top, 1 is bottom), NAN if unknown
    rec_bottom_y: f32,

    /// Current tracking status
    tracking_status: enums.CAMERA_TRACKING_STATUS_FLAGS,

    /// Current tracking mode
    tracking_mode: enums.CAMERA_TRACKING_MODE,

    /// Defines location of target data
    target_data: enums.CAMERA_TRACKING_TARGET_DATA.Type,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units
pub const SCALED_IMU2 = struct {
    pub const MSG_ID = 116;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X acceleration
    xacc: i16,

    /// Y acceleration
    yacc: i16,

    /// Z acceleration
    zacc: i16,

    /// Angular speed around X axis
    xgyro: i16,

    /// Angular speed around Y axis
    ygyro: i16,

    /// Angular speed around Z axis
    zgyro: i16,

    /// X Magnetic field
    xmag: i16,

    /// Y Magnetic field
    ymag: i16,

    /// Z Magnetic field
    zmag: i16,

    //Extension Field
    /// Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
    temperature: i16,

};

/// Wind estimate from vehicle. Note that despite the name, this message does not actually contain any covariances but instead variability and accuracy fields in terms of standard deviation (1-STD).
pub const WIND_COV = struct {
    pub const MSG_ID = 231;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Wind in North (NED) direction (NAN if unknown)
    wind_x: f32,

    /// Wind in East (NED) direction (NAN if unknown)
    wind_y: f32,

    /// Wind in down (NED) direction (NAN if unknown)
    wind_z: f32,

    /// Variability of wind in XY, 1-STD estimated from a 1 Hz lowpassed wind estimate (NAN if unknown)
    var_horiz: f32,

    /// Variability of wind in Z, 1-STD estimated from a 1 Hz lowpassed wind estimate (NAN if unknown)
    var_vert: f32,

    /// Altitude (MSL) that this measurement was taken at (NAN if unknown)
    wind_alt: f32,

    /// Horizontal speed 1-STD accuracy (0 if unknown)
    horiz_accuracy: f32,

    /// Vertical speed 1-STD accuracy (0 if unknown)
    vert_accuracy: f32,

};

/// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/services/mission.html
pub const MISSION_REQUEST_INT = struct {
    pub const MSG_ID = 51;
    /// Sequence
    seq: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It
///                is designed as scaled integer message since the resolution of float is not sufficient.
pub const GLOBAL_POSITION_INT = struct {
    pub const MSG_ID = 33;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Latitude, expressed
    lat: i32,

    /// Longitude, expressed
    lon: i32,

    /// Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
    alt: i32,

    /// Altitude above home
    relative_alt: i32,

    /// Ground X Speed (Latitude, positive north)
    vx: i16,

    /// Ground Y Speed (Longitude, positive east)
    vy: i16,

    /// Ground Z Speed (Altitude, positive down)
    vz: i16,

    /// Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    hdg: u16,

};

/// Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations.
pub const HIL_STATE_QUATERNION = struct {
    pub const MSG_ID = 115;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
    attitude_quaternion: [4]f32,

    /// Body frame roll / phi angular speed
    rollspeed: f32,

    /// Body frame pitch / theta angular speed
    pitchspeed: f32,

    /// Body frame yaw / psi angular speed
    yawspeed: f32,

    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

    /// Altitude
    alt: i32,

    /// Ground X Speed (Latitude)
    vx: i16,

    /// Ground Y Speed (Longitude)
    vy: i16,

    /// Ground Z Speed (Altitude)
    vz: i16,

    /// Indicated airspeed
    ind_airspeed: u16,

    /// True airspeed
    true_airspeed: u16,

    /// X acceleration
    xacc: i16,

    /// Y acceleration
    yacc: i16,

    /// Z acceleration
    zacc: i16,

};

/// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
pub const ATTITUDE_QUATERNION = struct {
    pub const MSG_ID = 31;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Quaternion component 1, w (1 in null-rotation)
    q1: f32,

    /// Quaternion component 2, x (0 in null-rotation)
    q2: f32,

    /// Quaternion component 3, y (0 in null-rotation)
    q3: f32,

    /// Quaternion component 4, z (0 in null-rotation)
    q4: f32,

    /// Roll angular speed
    rollspeed: f32,

    /// Pitch angular speed
    pitchspeed: f32,

    /// Yaw angular speed
    yawspeed: f32,

    //Extension Field
    /// Rotation offset by which the attitude quaternion and angular speed vector should be rotated for user display (quaternion with [w, x, y, z] order, zero-rotation is [1, 0, 0, 0], send [0, 0, 0, 0] if field not supported). This field is intended for systems in which the reference attitude may change during flight. For example, tailsitters VTOLs rotate their reference attitude by 90 degrees between hover mode and fixed wing mode, thus repr_offset_q is equal to [1, 0, 0, 0] in hover mode and equal to [0.7071, 0, 0.7071, 0] in fixed wing mode.
    repr_offset_q: [4]f32,

};

/// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
pub const GPS_RTK = struct {
    pub const MSG_ID = 127;
    /// Time since boot of last baseline message received.
    time_last_baseline_ms: u32,

    /// GPS Time of Week of last baseline
    tow: u32,

    /// Current baseline in ECEF x or NED north component.
    baseline_a_mm: i32,

    /// Current baseline in ECEF y or NED east component.
    baseline_b_mm: i32,

    /// Current baseline in ECEF z or NED down component.
    baseline_c_mm: i32,

    /// Current estimate of baseline accuracy.
    accuracy: u32,

    /// Current number of integer ambiguity hypotheses.
    iar_num_hypotheses: i32,

    /// GPS Week Number of last baseline
    wn: u16,

    /// Identification of connected RTK receiver.
    rtk_receiver_id: u8,

    /// GPS-specific health report for RTK data.
    rtk_health: u8,

    /// Rate of baseline messages being received by GPS
    rtk_rate: u8,

    /// Current number of sats used for RTK calculation.
    nsats: u8,

    /// Coordinate system of baseline
    baseline_coords_type: enums.RTK_BASELINE_COORDINATE_SYSTEM,

};

/// Request all parameters of this component. After this request, all parameters are emitted. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html
pub const PARAM_REQUEST_LIST = struct {
    pub const MSG_ID = 21;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)
pub const OPTICAL_FLOW_RAD = struct {
    pub const MSG_ID = 106;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
    integration_time_us: u32,

    /// Flow around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
    integrated_x: f32,

    /// Flow around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
    integrated_y: f32,

    /// RH rotation around X axis
    integrated_xgyro: f32,

    /// RH rotation around Y axis
    integrated_ygyro: f32,

    /// RH rotation around Z axis
    integrated_zgyro: f32,

    /// Time since the distance was sampled.
    time_delta_distance_us: u32,

    /// Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
    distance: f32,

    /// Temperature
    temperature: i16,

    /// Sensor ID
    sensor_id: u8,

    /// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
    quality: u8,

};

/// Power supply status
pub const POWER_STATUS = struct {
    pub const MSG_ID = 125;
    /// 5V rail voltage.
    Vcc: u16,

    /// Servo rail voltage.
    Vservo: u16,

    /// Bitmap of power supply status flags.
    flags: enums.MAV_POWER_STATUS.Type,

};

/// Data packet for images sent using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
pub const ENCAPSULATED_DATA = struct {
    pub const MSG_ID = 131;
    /// image data bytes
    data: [253]u8,

    /// sequence number (starting with 0 on every transmission)
    seqnr: u16,

};

/// Handshake message to initiate, control and stop image streaming when using the Image Transmission Protocol: https://mavlink.io/en/services/image_transmission.html.
pub const DATA_TRANSMISSION_HANDSHAKE = struct {
    pub const MSG_ID = 130;
    /// total data size (set on ACK only).
    size: u32,

    /// Width of a matrix or image.
    width: u16,

    /// Height of a matrix or image.
    height: u16,

    /// Number of packets being sent (set on ACK only).
    packets: u16,

    /// Type of requested/acknowledged data.
    type: enums.MAVLINK_DATA_STREAM_TYPE,

    /// Payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only).
    payload: u8,

    /// JPEG quality. Values: [1-100].
    jpg_quality: u8,

};

/// Smart Battery information (static/infrequent update). Use for updates from: smart battery to flight stack, flight stack to GCS. Use BATTERY_STATUS for the frequent battery updates.
pub const SMART_BATTERY_INFO = struct {
    pub const MSG_ID = 370;
    /// Static device name in ASCII characters, 0 terminated. All 0: field not provided. Encode as manufacturer name then product name separated using an underscore.
    device_name: [50]u8,

    /// Serial number in ASCII characters, 0 terminated. All 0: field not provided.
    serial_number: [16]u8,

    /// Capacity when full according to manufacturer, -1: field not provided.
    capacity_full_specification: i32,

    /// Capacity when full (accounting for battery degradation), -1: field not provided.
    capacity_full: i32,

    /// Charge/discharge cycle count. UINT16_MAX: field not provided.
    cycle_count: u16,

    /// Battery weight. 0: field not provided.
    weight: u16,

    /// Minimum per-cell voltage when discharging. If not supplied set to UINT16_MAX value.
    discharge_minimum_voltage: u16,

    /// Minimum per-cell voltage when charging. If not supplied set to UINT16_MAX value.
    charging_minimum_voltage: u16,

    /// Minimum per-cell voltage when resting. If not supplied set to UINT16_MAX value.
    resting_minimum_voltage: u16,

    /// Battery ID
    id: u8,

    /// Function of the battery
    battery_function: enums.MAV_BATTERY_FUNCTION,

    /// Type (chemistry) of the battery
    type: enums.MAV_BATTERY_TYPE,

    //Extension Field
    /// Maximum per-cell voltage when charged. 0: field not provided.
    charging_maximum_voltage: u16,

    //Extension Field
    /// Number of battery cells in series. 0: field not provided.
    cells_in_series: u8,

    //Extension Field
    /// Maximum pack discharge current. 0: field not provided.
    discharge_maximum_current: u32,

    //Extension Field
    /// Maximum pack discharge burst current. 0: field not provided.
    discharge_maximum_burst_current: u32,

    //Extension Field
    /// Manufacture date (DD/MM/YYYY) in ASCII characters, 0 terminated. All 0: field not provided.
    manufacture_date: [11]u8,

};

/// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
pub const LOCAL_POSITION_NED = struct {
    pub const MSG_ID = 32;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X Position
    x: f32,

    /// Y Position
    y: f32,

    /// Z Position
    z: f32,

    /// X Speed
    vx: f32,

    /// Y Speed
    vy: f32,

    /// Z Speed
    vz: f32,

};

/// The state of the navigation and position controller.
pub const NAV_CONTROLLER_OUTPUT = struct {
    pub const MSG_ID = 62;
    /// Current desired roll
    nav_roll: f32,

    /// Current desired pitch
    nav_pitch: f32,

    /// Current altitude error
    alt_error: f32,

    /// Current airspeed error
    aspd_error: f32,

    /// Current crosstrack error on x-y plane
    xtrack_error: f32,

    /// Current desired heading
    nav_bearing: i16,

    /// Bearing to current waypoint/target
    target_bearing: i16,

    /// Distance to active waypoint
    wp_dist: u16,

};

/// Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/services/mission.html
pub const MISSION_REQUEST = struct {
    pub const MSG_ID = 40;
    /// Sequence
    seq: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Set a parameter value (write new value to permanent storage).
///         The receiving component should acknowledge the new parameter value by broadcasting a PARAM_VALUE message (broadcasting ensures that multiple GCS all have an up-to-date list of all parameters). If the sending GCS did not receive a PARAM_VALUE within its timeout time, it should re-send the PARAM_SET message. The parameter microservice is documented at https://mavlink.io/en/services/parameter.html.
pub const PARAM_SET = struct {
    pub const MSG_ID = 23;
    /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Onboard parameter value
    param_value: f32,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Onboard parameter type.
    param_type: enums.MAV_PARAM_TYPE,

};

/// The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field.
pub const SCALED_PRESSURE = struct {
    pub const MSG_ID = 29;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Absolute pressure
    press_abs: f32,

    /// Differential pressure 1
    press_diff: f32,

    /// Absolute pressure temperature
    temperature: i16,

    //Extension Field
    /// Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
    temperature_press_diff: i16,

};

/// Request for terrain data and terrain status. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
pub const TERRAIN_REQUEST = struct {
    pub const MSG_ID = 133;
    /// Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)
    mask: u64,

    /// Latitude of SW corner of first grid
    lat: i32,

    /// Longitude of SW corner of first grid
    lon: i32,

    /// Grid spacing
    grid_spacing: u16,

};

/// Illuminator status
pub const ILLUMINATOR_STATUS = struct {
    pub const MSG_ID = 440;
    /// Time since the start-up of the illuminator in ms
    uptime_ms: u32,

    /// Errors
    error_status: enums.ILLUMINATOR_ERROR_FLAGS.Type,

    /// Illuminator brightness
    brightness: f32,

    /// Illuminator strobing period in seconds
    strobe_period: f32,

    /// Illuminator strobing duty cycle
    strobe_duty_cycle: f32,

    /// Temperature in Celsius
    temp_c: f32,

    /// Minimum strobing period in seconds
    min_strobe_period: f32,

    /// Maximum strobing period in seconds
    max_strobe_period: f32,

    /// 0: Illuminators OFF, 1: Illuminators ON
    enable: u8,

    /// Supported illuminator modes
    mode_bitmask: enums.ILLUMINATOR_MODE,

    /// Illuminator mode
    mode: enums.ILLUMINATOR_MODE,

};

/// RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting
pub const GPS2_RTK = struct {
    pub const MSG_ID = 128;
    /// Time since boot of last baseline message received.
    time_last_baseline_ms: u32,

    /// GPS Time of Week of last baseline
    tow: u32,

    /// Current baseline in ECEF x or NED north component.
    baseline_a_mm: i32,

    /// Current baseline in ECEF y or NED east component.
    baseline_b_mm: i32,

    /// Current baseline in ECEF z or NED down component.
    baseline_c_mm: i32,

    /// Current estimate of baseline accuracy.
    accuracy: u32,

    /// Current number of integer ambiguity hypotheses.
    iar_num_hypotheses: i32,

    /// GPS Week Number of last baseline
    wn: u16,

    /// Identification of connected RTK receiver.
    rtk_receiver_id: u8,

    /// GPS-specific health report for RTK data.
    rtk_health: u8,

    /// Rate of baseline messages being received by GPS
    rtk_rate: u8,

    /// Current number of sats used for RTK calculation.
    nsats: u8,

    /// Coordinate system of baseline
    baseline_coords_type: enums.RTK_BASELINE_COORDINATE_SYSTEM,

};

/// Drone IMU data. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
pub const AVSS_DRONE_IMU = struct {
    pub const MSG_ID = 60052;
    /// Timestamp (time since FC boot).
    time_boot_ms: u32,

    /// Quaternion component 1, w (1 in null-rotation)
    q1: f32,

    /// Quaternion component 2, x (0 in null-rotation)
    q2: f32,

    /// Quaternion component 3, y (0 in null-rotation)
    q3: f32,

    /// Quaternion component 4, z (0 in null-rotation)
    q4: f32,

    /// X acceleration
    xacc: f32,

    /// Y acceleration
    yacc: f32,

    /// Z acceleration
    zacc: f32,

    /// Angular speed around X axis
    xgyro: f32,

    /// Angular speed around Y axis
    ygyro: f32,

    /// Angular speed around Z axis
    zgyro: f32,

};

/// Camera tracking status, sent while in active tracking. Use MAV_CMD_SET_MESSAGE_INTERVAL to define message interval.
pub const CAMERA_TRACKING_GEO_STATUS = struct {
    pub const MSG_ID = 276;
    /// Latitude of tracked object
    lat: i32,

    /// Longitude of tracked object
    lon: i32,

    /// Altitude of tracked object(AMSL, WGS84)
    alt: f32,

    /// Horizontal accuracy. NAN if unknown
    h_acc: f32,

    /// Vertical accuracy. NAN if unknown
    v_acc: f32,

    /// North velocity of tracked object. NAN if unknown
    vel_n: f32,

    /// East velocity of tracked object. NAN if unknown
    vel_e: f32,

    /// Down velocity of tracked object. NAN if unknown
    vel_d: f32,

    /// Velocity accuracy. NAN if unknown
    vel_acc: f32,

    /// Distance between camera and tracked object. NAN if unknown
    dist: f32,

    /// Heading in radians, in NED. NAN if unknown
    hdg: f32,

    /// Accuracy of heading, in NED. NAN if unknown
    hdg_acc: f32,

    /// Current tracking status
    tracking_status: enums.CAMERA_TRACKING_STATUS_FLAGS,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// Reports results of completed compass calibration. Sent until MAG_CAL_ACK received.
pub const MAG_CAL_REPORT = struct {
    pub const MSG_ID = 192;
    /// RMS milligauss residuals.
    fitness: f32,

    /// X offset.
    ofs_x: f32,

    /// Y offset.
    ofs_y: f32,

    /// Z offset.
    ofs_z: f32,

    /// X diagonal (matrix 11).
    diag_x: f32,

    /// Y diagonal (matrix 22).
    diag_y: f32,

    /// Z diagonal (matrix 33).
    diag_z: f32,

    /// X off-diagonal (matrix 12 and 21).
    offdiag_x: f32,

    /// Y off-diagonal (matrix 13 and 31).
    offdiag_y: f32,

    /// Z off-diagonal (matrix 32 and 23).
    offdiag_z: f32,

    /// Compass being calibrated.
    compass_id: u8,

    /// Bitmask of compasses being calibrated.
    cal_mask: u8,

    /// Calibration Status.
    cal_status: enums.MAG_CAL_STATUS,

    /// 0=requires a MAV_CMD_DO_ACCEPT_MAG_CAL, 1=saved to parameters.
    autosaved: u8,

    //Extension Field
    /// Confidence in orientation (higher is better).
    orientation_confidence: f32,

    //Extension Field
    /// orientation before calibration.
    old_orientation: enums.MAV_SENSOR_ORIENTATION,

    //Extension Field
    /// orientation after calibration.
    new_orientation: enums.MAV_SENSOR_ORIENTATION,

    //Extension Field
    /// field radius correction factor
    scale_factor: f32,

};

/// Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/services/parameter.html for a full documentation of QGroundControl and IMU code.
pub const PARAM_REQUEST_READ = struct {
    pub const MSG_ID = 20;
    /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)
    param_index: i16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Message for transporting "arbitrary" variable-length data from one component to another (broadcast is not forbidden, but discouraged). The encoding of the data is usually extension specific, i.e. determined by the source, and is usually not documented as part of the MAVLink specification.
pub const TUNNEL = struct {
    pub const MSG_ID = 385;
    /// Variable length payload. The payload length is defined by payload_length. The entire content of this block is opaque unless you understand the encoding specified by payload_type.
    payload: [128]u8,

    /// A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
    payload_type: enums.MAV_TUNNEL_PAYLOAD_TYPE,

    /// System ID (can be 0 for broadcast, but this is discouraged)
    target_system: u8,

    /// Component ID (can be 0 for broadcast, but this is discouraged)
    target_component: u8,

    /// Length of the data transported in payload
    payload_length: u8,

};

/// Modify the filter of what CAN messages to forward over the mavlink. This can be used to make CAN forwarding work well on low bandwidth links. The filtering is applied on bits 8 to 24 of the CAN id (2nd and 3rd bytes) which corresponds to the DroneCAN message ID for DroneCAN. Filters with more than 16 IDs can be constructed by sending multiple CAN_FILTER_MODIFY messages.
pub const CAN_FILTER_MODIFY = struct {
    pub const MSG_ID = 388;
    /// filter IDs, length num_ids
    ids: [16]u16,

    /// System ID.
    target_system: u8,

    /// Component ID.
    target_component: u8,

    /// bus number
    bus: u8,

    /// what operation to perform on the filter list. See CAN_FILTER_OP enum.
    operation: enums.CAN_FILTER_OP,

    /// number of IDs in filter list
    num_ids: u8,

};

/// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints.
pub const MISSION_COUNT = struct {
    pub const MSG_ID = 44;
    /// Number of mission items in the sequence
    count: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

    //Extension Field
    /// Id of current on-vehicle mission, fence, or rally point plan (on download from vehicle).
    ///         This field is used when downloading a plan from a vehicle to a GCS.
    ///         0 on upload to the vehicle from GCS.
    ///         0 if plan ids are not supported.
    ///         The current on-vehicle plan ids are streamed in `MISSION_CURRENT`, allowing a GCS to determine if any part of the plan has changed and needs to be re-uploaded.
    ///         The ids are recalculated by the vehicle when any part of the on-vehicle plan changes (when a new plan is uploaded, the vehicle returns the new id to the GCS in MISSION_ACK).
    ///       
    opaque_id: u32,

};

/// The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0).
pub const ATTITUDE_QUATERNION_COV = struct {
    pub const MSG_ID = 61;
    /// Row-major representation of a 3x3 attitude covariance matrix (states: roll, pitch, yaw; first three entries are the first ROW, next three entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [9]f32,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
    q: [4]f32,

    /// Roll angular speed
    rollspeed: f32,

    /// Pitch angular speed
    pitchspeed: f32,

    /// Yaw angular speed
    yawspeed: f32,

};

/// General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service "uavcan.protocol.GetNodeInfo" for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org.
pub const UAVCAN_NODE_INFO = struct {
    pub const MSG_ID = 311;
    /// Node name string. For example, "sapog.px4.io".
    name: [80]u8,

    /// Hardware unique 128-bit ID.
    hw_unique_id: [16]u8,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Time since the start-up of the node.
    uptime_sec: u32,

    /// Version control system (VCS) revision identifier (e.g. git short commit hash). 0 if unknown.
    sw_vcs_commit: u32,

    /// Hardware major version number.
    hw_version_major: u8,

    /// Hardware minor version number.
    hw_version_minor: u8,

    /// Software major version number.
    sw_version_major: u8,

    /// Software minor version number.
    sw_version_minor: u8,

};

/// Large debug/prototyping array. The message uses the maximum available payload for data. The array_id and name fields are used to discriminate between messages in code and in user interfaces (respectively). Do not use in production code.
pub const DEBUG_FLOAT_ARRAY = struct {
    pub const MSG_ID = 350;
    /// Name, for human-friendly display in a Ground Control Station
    name: [10]u8,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Unique ID used to discriminate between arrays
    array_id: u16,

    //Extension Field
    /// data
    data: [58]f32,

};

/// The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units
pub const SCALED_IMU = struct {
    pub const MSG_ID = 26;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X acceleration
    xacc: i16,

    /// Y acceleration
    yacc: i16,

    /// Z acceleration
    zacc: i16,

    /// Angular speed around X axis
    xgyro: i16,

    /// Angular speed around Y axis
    ygyro: i16,

    /// Angular speed around Z axis
    zgyro: i16,

    /// X Magnetic field
    xmag: i16,

    /// Y Magnetic field
    ymag: i16,

    /// Z Magnetic field
    zmag: i16,

    //Extension Field
    /// Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
    temperature: i16,

};

/// The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
pub const RC_CHANNELS_RAW = struct {
    pub const MSG_ID = 35;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// RC channel 1 value.
    chan1_raw: u16,

    /// RC channel 2 value.
    chan2_raw: u16,

    /// RC channel 3 value.
    chan3_raw: u16,

    /// RC channel 4 value.
    chan4_raw: u16,

    /// RC channel 5 value.
    chan5_raw: u16,

    /// RC channel 6 value.
    chan6_raw: u16,

    /// RC channel 7 value.
    chan7_raw: u16,

    /// RC channel 8 value.
    chan8_raw: u16,

    /// Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
    port: u8,

    /// Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
    rssi: u8,

};

/// Drone operation mode.
pub const AVSS_DRONE_OPERATION_MODE = struct {
    pub const MSG_ID = 60053;
    /// Timestamp (time since FC boot).
    time_boot_ms: u32,

    /// DJI M300 operation mode
    M300_operation_mode: u8,

    /// horsefly operation mode
    horsefly_operation_mode: u8,

};

/// The interval between messages for a particular MAVLink message ID.
///         This message is sent in response to the MAV_CMD_REQUEST_MESSAGE command with param1=244 (this message) and param2=message_id (the id of the message for which the interval is required).
/// It may also be sent in response to MAV_CMD_GET_MESSAGE_INTERVAL.
/// This interface replaces DATA_STREAM.
pub const MESSAGE_INTERVAL = struct {
    pub const MSG_ID = 244;
    /// The interval between two messages. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent.
    interval_us: i32,

    /// The ID of the requested MAVLink message. v1.0 is limited to 254 messages.
    message_id: u16,

};

/// Set the mission item with sequence number seq as the current item and emit MISSION_CURRENT (whether or not the mission number changed).
///         If a mission is currently being executed, the system will continue to this new mission item on the shortest path, skipping any intermediate mission items.
///         Note that mission jump repeat counters are not reset (see MAV_CMD_DO_JUMP param2).
/// 
///         This message may trigger a mission state-machine change on some systems: for example from MISSION_STATE_NOT_STARTED or MISSION_STATE_PAUSED to MISSION_STATE_ACTIVE.
///         If the system is in mission mode, on those systems this command might therefore start, restart or resume the mission.
///         If the system is not in mission mode this message must not trigger a switch to mission mode.
pub const MISSION_SET_CURRENT = struct {
    pub const MSG_ID = 41;
    /// Sequence
    seq: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output.
pub const NAMED_VALUE_INT = struct {
    pub const MSG_ID = 252;
    /// Name of the debug variable
    name: [10]u8,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Signed integer value
    value: i32,

};

/// File transfer protocol message: https://mavlink.io/en/services/ftp.html.
pub const FILE_TRANSFER_PROTOCOL = struct {
    pub const MSG_ID = 110;
    /// Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields. The content/format of this block is defined in https://mavlink.io/en/services/ftp.html.
    payload: [251]u8,

    /// Network ID (0 for broadcast)
    target_network: u8,

    /// System ID (0 for broadcast)
    target_system: u8,

    /// Component ID (0 for broadcast)
    target_component: u8,

};

/// Bind a RC channel to a parameter. The parameter should change according to the RC channel value.
pub const PARAM_MAP_RC = struct {
    pub const MSG_ID = 50;
    /// Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Initial parameter value
    param_value0: f32,

    /// Scale, maps the RC range [-1, 1] to a parameter value
    scale: f32,

    /// Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)
    param_value_min: f32,

    /// Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)
    param_value_max: f32,

    /// Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index.
    param_index: i16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Index of parameter RC channel. Not equal to the RC channel id. Typically corresponds to a potentiometer-knob on the RC.
    parameter_rc_channel_index: u8,

};

/// The global position, as returned by the Global Positioning System (GPS). This is
///                  NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION_INT for the global position estimate.
pub const HIL_GPS = struct {
    pub const MSG_ID = 113;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Latitude (WGS84)
    lat: i32,

    /// Longitude (WGS84)
    lon: i32,

    /// Altitude (MSL). Positive for up.
    alt: i32,

    /// GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    eph: u16,

    /// GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    epv: u16,

    /// GPS ground speed. If unknown, set to: UINT16_MAX
    vel: u16,

    /// GPS velocity in north direction in earth-fixed NED frame
    vn: i16,

    /// GPS velocity in east direction in earth-fixed NED frame
    ve: i16,

    /// GPS velocity in down direction in earth-fixed NED frame
    vd: i16,

    /// Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    cog: u16,

    /// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
    fix_type: u8,

    /// Number of satellites visible. If unknown, set to UINT8_MAX
    satellites_visible: u8,

    //Extension Field
    /// GPS ID (zero indexed). Used for multiple GPS inputs
    id: u8,

    //Extension Field
    /// Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
    yaw: u16,

};

/// Temperature and humidity from hygrometer.
pub const HYGROMETER_SENSOR = struct {
    pub const MSG_ID = 12920;
    /// Temperature
    temperature: i16,

    /// Humidity
    humidity: u16,

    /// Hygrometer ID
    id: u8,

};

/// The location and information of an AIS vessel
pub const AIS_VESSEL = struct {
    pub const MSG_ID = 301;
    /// The vessel name
    name: [20]u8,

    /// The vessel callsign
    callsign: [7]u8,

    /// Mobile Marine Service Identifier, 9 decimal digits
    MMSI: u32,

    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

    /// Course over ground
    COG: u16,

    /// True heading
    heading: u16,

    /// Speed over ground
    velocity: u16,

    /// Distance from lat/lon location to bow
    dimension_bow: u16,

    /// Distance from lat/lon location to stern
    dimension_stern: u16,

    /// Time since last communication in seconds
    tslc: u16,

    /// Bitmask to indicate various statuses including valid data fields
    flags: enums.AIS_FLAGS.Type,

    /// Turn rate, 0.1 degrees per second
    turn_rate: i8,

    /// Navigational status
    navigational_status: enums.AIS_NAV_STATUS,

    /// Type of vessels
    type: enums.AIS_TYPE,

    /// Distance from lat/lon location to port side
    dimension_port: u8,

    /// Distance from lat/lon location to starboard side
    dimension_starboard: u8,

};

/// Reply to LOG_REQUEST_DATA
pub const LOG_DATA = struct {
    pub const MSG_ID = 120;
    /// log data
    data: [90]u8,

    /// Offset into the log
    ofs: u32,

    /// Log id (from LOG_ENTRY reply)
    id: u16,

    /// Number of bytes (zero for end of log)
    count: u8,

};

/// A message containing logged data (see also MAV_CMD_LOGGING_START)
pub const LOGGING_DATA = struct {
    pub const MSG_ID = 266;
    /// logged data
    data: [249]u8,

    /// sequence number (can wrap)
    sequence: u16,

    /// system ID of the target
    target_system: u8,

    /// component ID of the target
    target_component: u8,

    /// data length
    length: u8,

    /// offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to UINT8_MAX if no start exists).
    first_message_offset: u8,

};

/// Information about a high level gimbal manager. This message should be requested by a ground station using MAV_CMD_REQUEST_MESSAGE.
pub const GIMBAL_MANAGER_INFORMATION = struct {
    pub const MSG_ID = 280;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Bitmap of gimbal capability flags.
    cap_flags: enums.GIMBAL_MANAGER_CAP_FLAGS.Type,

    /// Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
    roll_min: f32,

    /// Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left)
    roll_max: f32,

    /// Minimum pitch angle (positive: up, negative: down)
    pitch_min: f32,

    /// Maximum pitch angle (positive: up, negative: down)
    pitch_max: f32,

    /// Minimum yaw angle (positive: to the right, negative: to the left)
    yaw_min: f32,

    /// Maximum yaw angle (positive: to the right, negative: to the left)
    yaw_max: f32,

    /// Gimbal device ID that this gimbal manager is responsible for. Component ID of gimbal device (or 1-6 for non-MAVLink gimbal).
    gimbal_device_id: u8,

};

/// Request a chunk of a log
pub const LOG_REQUEST_DATA = struct {
    pub const MSG_ID = 119;
    /// Offset into the log
    ofs: u32,

    /// Number of bytes
    count: u32,

    /// Log id (from LOG_ENTRY reply)
    id: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Sets the GPS coordinates of the vehicle local origin (0,0,0) position. Vehicle should emit GPS_GLOBAL_ORIGIN irrespective of whether the origin is changed. This enables transform between the local coordinate frame and the global (GPS) coordinate frame, which may be necessary when (for example) indoor and outdoor settings are connected and the MAV should move from in- to outdoor.
pub const SET_GPS_GLOBAL_ORIGIN = struct {
    pub const MSG_ID = 48;
    /// Latitude (WGS84)
    latitude: i32,

    /// Longitude (WGS84)
    longitude: i32,

    /// Altitude (MSL). Positive for up.
    altitude: i32,

    /// System ID
    target_system: u8,

    //Extension Field
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

};

/// Configure cellular modems.
///         This message is re-emitted as an acknowledgement by the modem.
///         The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE.
pub const CELLULAR_CONFIG = struct {
    pub const MSG_ID = 336;
    /// Name of the cellular APN. Blank to leave it unchanged. Current APN when sent back as a response.
    apn: [32]u8,

    /// PIN sent to the SIM card. Blank when PIN is disabled. Empty when message is sent back as a response.
    pin: [16]u8,

    /// New PIN when changing the PIN. Blank to leave it unchanged. Empty when message is sent back as a response.
    new_pin: [16]u8,

    /// Required PUK code in case the user failed to authenticate 3 times with the PIN. Empty when message is sent back as a response.
    puk: [16]u8,

    /// Enable/disable LTE. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
    enable_lte: u8,

    /// Enable/disable PIN on the SIM card. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
    enable_pin: u8,

    /// Enable/disable roaming. 0: setting unchanged, 1: disabled, 2: enabled. Current setting when sent back as a response.
    roaming: u8,

    /// Message acceptance response (sent back to GS).
    response: enums.CELLULAR_CONFIG_RESPONSE,

};

/// Set the vehicle attitude and body angular rates.
pub const SET_ACTUATOR_CONTROL_TARGET = struct {
    pub const MSG_ID = 139;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
    controls: [8]f32,

    /// Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
    group_mlx: u8,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Status of the Iridium SBD link.
pub const ISBD_LINK_STATUS = struct {
    pub const MSG_ID = 335;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    timestamp: u64,

    /// Timestamp of the last successful sbd session. The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    last_heartbeat: u64,

    /// Number of failed SBD sessions.
    failed_sessions: u16,

    /// Number of successful SBD sessions.
    successful_sessions: u16,

    /// Signal quality equal to the number of bars displayed on the ISU signal strength indicator. Range is 0 to 5, where 0 indicates no signal and 5 indicates maximum signal strength.
    signal_quality: u8,

    /// 1: Ring call pending, 0: No call pending.
    ring_pending: u8,

    /// 1: Transmission session pending, 0: No transmission session pending.
    tx_session_pending: u8,

    /// 1: Receiving session pending, 0: No receiving session pending.
    rx_session_pending: u8,

};

/// Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system).
pub const SET_ATTITUDE_TARGET = struct {
    pub const MSG_ID = 82;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0) from MAV_FRAME_LOCAL_NED to MAV_FRAME_BODY_FRD
    q: [4]f32,

    /// Body roll rate
    body_roll_rate: f32,

    /// Body pitch rate
    body_pitch_rate: f32,

    /// Body yaw rate
    body_yaw_rate: f32,

    /// Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
    thrust: f32,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Bitmap to indicate which dimensions should be ignored by the vehicle.
    type_mask: enums.ATTITUDE_TARGET_TYPEMASK.Type,

    //Extension Field
    /// 3D thrust setpoint in the body NED frame, normalized to -1 .. 1
    thrust_body: [3]f32,

};

/// Provides state for additional features
pub const EXTENDED_SYS_STATE = struct {
    pub const MSG_ID = 245;
    /// The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration.
    vtol_state: enums.MAV_VTOL_STATE,

    /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    landed_state: enums.MAV_LANDED_STATE,

};

/// Request all parameters of this component. All parameters should be emitted in response as PARAM_EXT_VALUE.
pub const PARAM_EXT_REQUEST_LIST = struct {
    pub const MSG_ID = 321;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Orientation of a mount
pub const MOUNT_ORIENTATION = struct {
    pub const MSG_ID = 265;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Roll in global frame (set to NaN for invalid).
    roll: f32,

    /// Pitch in global frame (set to NaN for invalid).
    pitch: f32,

    /// Yaw relative to vehicle (set to NaN for invalid).
    yaw: f32,

    //Extension Field
    /// Yaw in absolute frame relative to Earth's North, north is 0 (set to NaN for invalid).
    yaw_absolute: f32,

};

/// The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset.
pub const GLOBAL_POSITION_INT_COV = struct {
    pub const MSG_ID = 63;
    /// Row-major representation of a 6x6 position and velocity 6x6 cross-covariance matrix (states: lat, lon, alt, vx, vy, vz; first six entries are the first ROW, next six entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [36]f32,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

    /// Altitude in meters above MSL
    alt: i32,

    /// Altitude above ground
    relative_alt: i32,

    /// Ground X Speed (Latitude)
    vx: f32,

    /// Ground Y Speed (Longitude)
    vy: f32,

    /// Ground Z Speed (Altitude)
    vz: f32,

    /// Class id of the estimator this estimate originated from.
    estimator_type: enums.MAV_ESTIMATOR_TYPE,

};

/// A forwarded CANFD frame as requested by MAV_CMD_CAN_FORWARD. These are separated from CAN_FRAME as they need different handling (eg. TAO handling)
pub const CANFD_FRAME = struct {
    pub const MSG_ID = 387;
    /// Frame data
    data: [64]u8,

    /// Frame ID
    id: u32,

    /// System ID.
    target_system: u8,

    /// Component ID.
    target_component: u8,

    /// bus number
    bus: u8,

    /// Frame length
    len: u8,

};

/// Data stream status information.
pub const DATA_STREAM = struct {
    pub const MSG_ID = 67;
    /// The message rate
    message_rate: u16,

    /// The ID of the requested data stream
    stream_id: u8,

    /// 1 stream is enabled, 0 stream is stopped.
    on_off: u8,

};

/// Global position/attitude estimate from a vision source.
pub const GLOBAL_VISION_POSITION_ESTIMATE = struct {
    pub const MSG_ID = 101;
    /// Timestamp (UNIX time or since system boot)
    usec: u64,

    /// Global X position
    x: f32,

    /// Global Y position
    y: f32,

    /// Global Z position
    z: f32,

    /// Roll angle
    roll: f32,

    /// Pitch angle
    pitch: f32,

    /// Yaw angle
    yaw: f32,

    //Extension Field
    /// Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x_global, y_global, z_global, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [21]f32,

    //Extension Field
    /// Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
    reset_counter: u8,

};

/// Get the current mode.
///         This should be emitted on any mode change, and broadcast at low rate (nominally 0.5 Hz).
///         It may be requested using MAV_CMD_REQUEST_MESSAGE.
///         See https://mavlink.io/en/services/standard_modes.html
pub const CURRENT_MODE = struct {
    pub const MSG_ID = 436;
    /// A bitfield for use for autopilot-specific flags
    custom_mode: u32,

    /// The custom_mode of the mode that was last commanded by the user (for example, with MAV_CMD_DO_SET_STANDARD_MODE, MAV_CMD_DO_SET_MODE or via RC). This should usually be the same as custom_mode. It will be different if the vehicle is unable to enter the intended mode, or has left that mode due to a failsafe condition. 0 indicates the intended custom mode is unknown/not supplied
    intended_custom_mode: u32,

    /// Standard mode.
    standard_mode: enums.MAV_STANDARD_MODE,

};

/// Information about a captured image. This is emitted every time a message is captured.
///         MAV_CMD_REQUEST_MESSAGE can be used to (re)request this message for a specific sequence number or range of sequence numbers:
///         MAV_CMD_REQUEST_MESSAGE.param2 indicates the sequence number the first image to send, or set to -1 to send the message for all sequence numbers.
///         MAV_CMD_REQUEST_MESSAGE.param3 is used to specify a range of messages to send:
///         set to 0 (default) to send just the the message for the sequence number in param 2,
///         set to -1 to send the message for the sequence number in param 2 and all the following sequence numbers,
///         set to the sequence number of the final message in the range.
pub const CAMERA_IMAGE_CAPTURED = struct {
    pub const MSG_ID = 263;
    /// URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
    file_url: [205]u8,

    /// Timestamp (time since UNIX epoch) in UTC. 0 for unknown.
    time_utc: u64,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Latitude where image was taken
    lat: i32,

    /// Longitude where capture was taken
    lon: i32,

    /// Altitude (MSL) where image was taken
    alt: i32,

    /// Altitude above ground
    relative_alt: i32,

    /// Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    q: [4]f32,

    /// Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)
    image_index: i32,

    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id). Field name is usually camera_device_id.
    camera_id: u8,

    /// Image was captured successfully (MAV_BOOL_TRUE). Values not equal to 0 or 1 are invalid.
    capture_result: enums.MAV_BOOL.Type,

};

/// Version and capability of autopilot software. This should be emitted in response to a request with MAV_CMD_REQUEST_MESSAGE.
pub const AUTOPILOT_VERSION = struct {
    pub const MSG_ID = 148;
    /// Bitmap of capabilities
    capabilities: enums.MAV_PROTOCOL_CAPABILITY.Type,

    /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    flight_custom_version: [8]u8,

    /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    middleware_custom_version: [8]u8,

    /// Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases.
    os_custom_version: [8]u8,

    /// UID if provided by hardware (see uid2)
    uid: u64,

    /// Firmware version number.
    ///         The field must be encoded as 4 bytes, where each byte (shown from MSB to LSB) is part of a semantic version: (major) (minor) (patch) (FIRMWARE_VERSION_TYPE).
    ///       
    flight_sw_version: u32,

    /// Middleware version number
    middleware_sw_version: u32,

    /// Operating system version number
    os_sw_version: u32,

    /// HW / board version (last 8 bits should be silicon ID, if any). The first 16 bits of this field specify a board type from an enumeration stored at https://github.com/PX4/PX4-Bootloader/blob/master/board_types.txt and with extensive additions at https://github.com/ArduPilot/ardupilot/blob/master/Tools/AP_Bootloader/board_types.txt
    board_version: u32,

    /// ID of the board vendor
    vendor_id: u16,

    /// ID of the product
    product_id: u16,

    //Extension Field
    /// UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)
    uid2: [18]u8,

};

/// Winch status.
pub const WINCH_STATUS = struct {
    pub const MSG_ID = 9005;
    /// Timestamp (synced to UNIX time or since system boot).
    time_usec: u64,

    /// Length of line released. NaN if unknown
    line_length: f32,

    /// Speed line is being released or retracted. Positive values if being released, negative values if being retracted, NaN if unknown
    speed: f32,

    /// Tension on the line. NaN if unknown
    tension: f32,

    /// Voltage of the battery supplying the winch. NaN if unknown
    voltage: f32,

    /// Current draw from the winch. NaN if unknown
    current: f32,

    /// Status flags
    status: enums.MAV_WINCH_STATUS_FLAG.Type,

    /// Temperature of the motor. INT16_MAX if unknown
    temperature: i16,

};

/// Request a partial list of mission items from the system/component. https://mavlink.io/en/services/mission.html. If start and end index are the same, just send one waypoint.
pub const MISSION_REQUEST_PARTIAL_LIST = struct {
    pub const MSG_ID = 37;
    /// Start index
    start_index: i16,

    /// End index, -1 by default (-1: send list to end). Else a valid index of the list
    end_index: i16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Distance sensor information for an onboard rangefinder.
pub const DISTANCE_SENSOR = struct {
    pub const MSG_ID = 132;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Minimum distance the sensor can measure
    min_distance: u16,

    /// Maximum distance the sensor can measure
    max_distance: u16,

    /// Current distance reading
    current_distance: u16,

    /// Type of distance sensor.
    type: enums.MAV_DISTANCE_SENSOR,

    /// Onboard ID of the sensor
    id: u8,

    /// Direction the sensor faces. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270
    orientation: enums.MAV_SENSOR_ORIENTATION,

    /// Measurement variance. Max standard deviation is 6cm. UINT8_MAX if unknown.
    covariance: u8,

    //Extension Field
    /// Horizontal Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.
    horizontal_fov: f32,

    //Extension Field
    /// Vertical Field of View (angle) where the distance measurement is valid and the field of view is known. Otherwise this is set to 0.
    vertical_fov: f32,

    //Extension Field
    /// Quaternion of the sensor orientation in vehicle body frame (w, x, y, z order, zero-rotation is 1, 0, 0, 0). Zero-rotation is along the vehicle body x-axis. This field is required if the orientation is set to MAV_SENSOR_ROTATION_CUSTOM. Set it to 0 if invalid."
    quaternion: [4]f32,

    //Extension Field
    /// Signal quality of the sensor. Specific to each sensor type, representing the relation of the signal strength with the target reflectivity, distance, size or aspect, but normalised as a percentage. 0 = unknown/unset signal quality, 1 = invalid signal, 100 = perfect signal.
    signal_quality: u8,

};

/// The location of a landing target. See: https://mavlink.io/en/services/landing_target.html
pub const LANDING_TARGET = struct {
    pub const MSG_ID = 149;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X-axis angular offset of the target from the center of the image
    angle_x: f32,

    /// Y-axis angular offset of the target from the center of the image
    angle_y: f32,

    /// Distance to the target from the vehicle
    distance: f32,

    /// Size of target along x-axis
    size_x: f32,

    /// Size of target along y-axis
    size_y: f32,

    /// The ID of the target if multiple targets are present
    target_num: u8,

    /// Coordinate frame used for following fields.
    frame: enums.MAV_FRAME,

    //Extension Field
    /// X Position of the landing target in MAV_FRAME
    x: f32,

    //Extension Field
    /// Y Position of the landing target in MAV_FRAME
    y: f32,

    //Extension Field
    /// Z Position of the landing target in MAV_FRAME
    z: f32,

    //Extension Field
    /// Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    q: [4]f32,

    //Extension Field
    /// Type of landing target
    type: enums.LANDING_TARGET_TYPE,

    //Extension Field
    /// Position fields (x, y, z, q, type) contain valid target position information (MAV_BOOL_FALSE: invalid values). Values not equal to 0 or 1 are invalid.
    position_valid: enums.MAV_BOOL.Type,

};

/// A message containing logged data which requires a LOGGING_ACK to be sent back
pub const LOGGING_DATA_ACKED = struct {
    pub const MSG_ID = 267;
    /// logged data
    data: [249]u8,

    /// sequence number (can wrap)
    sequence: u16,

    /// system ID of the target
    target_system: u8,

    /// component ID of the target
    target_component: u8,

    /// data length
    length: u8,

    /// offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to UINT8_MAX if no start exists).
    first_message_offset: u8,

};

/// The autopilot is requesting a resource (file, binary, other type of data)
pub const RESOURCE_REQUEST = struct {
    pub const MSG_ID = 142;
    /// The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)
    uri: [120]u8,

    /// The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP).
    storage: [120]u8,

    /// Request ID. This ID should be reused when sending back URI contents
    request_id: u8,

    /// The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary
    uri_type: u8,

    /// The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream.
    transfer_type: u8,

};

/// Describe a trajectory using an array of up-to 5 bezier control points in the local frame (MAV_FRAME_LOCAL_NED).
pub const TRAJECTORY_REPRESENTATION_BEZIER = struct {
    pub const MSG_ID = 333;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X-coordinate of bezier control points. Set to NaN if not being used
    pos_x: [5]f32,

    /// Y-coordinate of bezier control points. Set to NaN if not being used
    pos_y: [5]f32,

    /// Z-coordinate of bezier control points. Set to NaN if not being used
    pos_z: [5]f32,

    /// Bezier time horizon. Set to NaN if velocity/acceleration should not be incorporated
    delta: [5]f32,

    /// Yaw. Set to NaN for unchanged
    pos_yaw: [5]f32,

    /// Number of valid control points (up-to 5 points are possible)
    valid_points: u8,

};

/// The scaled values of the RC channels received: (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to INT16_MAX.
pub const RC_CHANNELS_SCALED = struct {
    pub const MSG_ID = 34;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// RC channel 1 value scaled.
    chan1_scaled: i16,

    /// RC channel 2 value scaled.
    chan2_scaled: i16,

    /// RC channel 3 value scaled.
    chan3_scaled: i16,

    /// RC channel 4 value scaled.
    chan4_scaled: i16,

    /// RC channel 5 value scaled.
    chan5_scaled: i16,

    /// RC channel 6 value scaled.
    chan6_scaled: i16,

    /// RC channel 7 value scaled.
    chan7_scaled: i16,

    /// RC channel 8 value scaled.
    chan8_scaled: i16,

    /// Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.
    port: u8,

    /// Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
    rssi: u8,

};

/// Report status of a command. Includes feedback whether the command was executed. The command microservice is documented at https://mavlink.io/en/services/command.html
pub const COMMAND_ACK = struct {
    pub const MSG_ID = 77;
    /// Command ID (of acknowledged command).
    command: enums.MAV_CMD,

    /// Result of command.
    result: enums.MAV_RESULT,

    //Extension Field
    /// The progress percentage when result is MAV_RESULT_IN_PROGRESS. Values: [0-100], or UINT8_MAX if the progress is unknown.
    progress: u8,

    //Extension Field
    /// Additional result information. Can be set with a command-specific enum containing command-specific error reasons for why the command might be denied. If used, the associated enum must be documented in the corresponding MAV_CMD (this enum should have a 0 value to indicate "unused" or "unknown").
    result_param2: i32,

    //Extension Field
    /// System ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK is an acknowledgement.
    target_system: u8,

    //Extension Field
    /// Component ID of the target recipient. This is the ID of the system that sent the command for which this COMMAND_ACK is an acknowledgement.
    target_component: u8,

};

/// Information about the status of a video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE.
pub const VIDEO_STREAM_STATUS = struct {
    pub const MSG_ID = 270;
    /// Frame rate
    framerate: f32,

    /// Bit rate
    bitrate: u32,

    /// Bitmap of stream status flags
    flags: enums.VIDEO_STREAM_STATUS_FLAGS.Type,

    /// Horizontal resolution
    resolution_h: u16,

    /// Vertical resolution
    resolution_v: u16,

    /// Video image rotation clockwise
    rotation: u16,

    /// Horizontal Field of view
    hfov: u16,

    /// Video Stream ID (1 for first, 2 for second, etc.)
    stream_id: u8,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// Settings of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
pub const CAMERA_SETTINGS = struct {
    pub const MSG_ID = 260;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Camera mode
    mode_id: enums.CAMERA_MODE,

    //Extension Field
    /// Current zoom level as a percentage of the full range (0.0 to 100.0, NaN if not known)
    zoomLevel: f32,

    //Extension Field
    /// Current focus level as a percentage of the full range (0.0 to 100.0, NaN if not known)
    focusLevel: f32,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
pub const LOCAL_POSITION_NED_COV = struct {
    pub const MSG_ID = 64;
    /// Row-major representation of position, velocity and acceleration 9x9 cross-covariance matrix upper right triangle (states: x, y, z, vx, vy, vz, ax, ay, az; first nine entries are the first ROW, next eight entries are the second row, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [45]f32,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X Position
    x: f32,

    /// Y Position
    y: f32,

    /// Z Position
    z: f32,

    /// X Speed
    vx: f32,

    /// Y Speed
    vy: f32,

    /// Z Speed
    vz: f32,

    /// X Acceleration
    ax: f32,

    /// Y Acceleration
    ay: f32,

    /// Z Acceleration
    az: f32,

    /// Class id of the estimator this estimate originated from.
    estimator_type: enums.MAV_ESTIMATOR_TYPE,

};

/// The RAW values of the RC channels sent to the MAV to override info received from the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.  Note carefully the semantic differences between the first 8 channels and the subsequent channels
pub const RC_CHANNELS_OVERRIDE = struct {
    pub const MSG_ID = 70;
    /// RC channel 1 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan1_raw: u16,

    /// RC channel 2 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan2_raw: u16,

    /// RC channel 3 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan3_raw: u16,

    /// RC channel 4 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan4_raw: u16,

    /// RC channel 5 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan5_raw: u16,

    /// RC channel 6 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan6_raw: u16,

    /// RC channel 7 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan7_raw: u16,

    /// RC channel 8 value. A value of UINT16_MAX means to ignore this field. A value of 0 means to release this channel back to the RC radio.
    chan8_raw: u16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// RC channel 9 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan9_raw: u16,

    //Extension Field
    /// RC channel 10 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan10_raw: u16,

    //Extension Field
    /// RC channel 11 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan11_raw: u16,

    //Extension Field
    /// RC channel 12 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan12_raw: u16,

    //Extension Field
    /// RC channel 13 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan13_raw: u16,

    //Extension Field
    /// RC channel 14 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan14_raw: u16,

    //Extension Field
    /// RC channel 15 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan15_raw: u16,

    //Extension Field
    /// RC channel 16 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan16_raw: u16,

    //Extension Field
    /// RC channel 17 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan17_raw: u16,

    //Extension Field
    /// RC channel 18 value. A value of 0 or UINT16_MAX means to ignore this field. A value of UINT16_MAX-1 means to release this channel back to the RC radio.
    chan18_raw: u16,

};

/// A change to the sequence number indicates that the set of AVAILABLE_MODES has changed.
///         A receiver must re-request all available modes whenever the sequence number changes.
///         This is only emitted after the first change and should then be broadcast at low rate (nominally 0.3 Hz) and on change.
///         See https://mavlink.io/en/services/standard_modes.html
pub const AVAILABLE_MODES_MONITOR = struct {
    pub const MSG_ID = 437;
    /// Sequence number. The value iterates sequentially whenever AVAILABLE_MODES changes (e.g. support for a new mode is added/removed dynamically).
    seq: u8,

};

/// Erase all logs
pub const LOG_ERASE = struct {
    pub const MSG_ID = 121;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Information about the field of view of a camera. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
pub const CAMERA_FOV_STATUS = struct {
    pub const MSG_ID = 271;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Latitude of camera (INT32_MAX if unknown).
    lat_camera: i32,

    /// Longitude of camera (INT32_MAX if unknown).
    lon_camera: i32,

    /// Altitude (MSL) of camera (INT32_MAX if unknown).
    alt_camera: i32,

    /// Latitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
    lat_image: i32,

    /// Longitude of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
    lon_image: i32,

    /// Altitude (MSL) of center of image (INT32_MAX if unknown, INT32_MIN if at infinity, not intersecting with horizon).
    alt_image: i32,

    /// Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    q: [4]f32,

    /// Horizontal field of view (NaN if unknown).
    hfov: f32,

    /// Vertical field of view (NaN if unknown).
    vfov: f32,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// Second GPS data.
pub const GPS2_RAW = struct {
    pub const MSG_ID = 124;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Latitude (WGS84)
    lat: i32,

    /// Longitude (WGS84)
    lon: i32,

    /// Altitude (MSL). Positive for up.
    alt: i32,

    /// Age of DGPS info
    dgps_age: u32,

    /// GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    eph: u16,

    /// GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    epv: u16,

    /// GPS ground speed. If unknown, set to: UINT16_MAX
    vel: u16,

    /// Course over ground (NOT heading, but direction of movement): 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    cog: u16,

    /// GPS fix type.
    fix_type: enums.GPS_FIX_TYPE,

    /// Number of satellites visible. If unknown, set to UINT8_MAX
    satellites_visible: u8,

    /// Number of DGPS satellites
    dgps_numch: u8,

    //Extension Field
    /// Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
    yaw: u16,

    //Extension Field
    /// Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
    alt_ellipsoid: i32,

    //Extension Field
    /// Position uncertainty.
    h_acc: u32,

    //Extension Field
    /// Altitude uncertainty.
    v_acc: u32,

    //Extension Field
    /// Speed uncertainty.
    vel_acc: u32,

    //Extension Field
    /// Heading / track uncertainty
    hdg_acc: u32,

};

/// Status of simulation environment, if used
pub const SIM_STATE = struct {
    pub const MSG_ID = 108;
    /// True attitude quaternion component 1, w (1 in null-rotation)
    q1: f32,

    /// True attitude quaternion component 2, x (0 in null-rotation)
    q2: f32,

    /// True attitude quaternion component 3, y (0 in null-rotation)
    q3: f32,

    /// True attitude quaternion component 4, z (0 in null-rotation)
    q4: f32,

    /// Attitude roll expressed as Euler angles, not recommended except for human-readable outputs
    roll: f32,

    /// Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs
    pitch: f32,

    /// Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs
    yaw: f32,

    /// X acceleration
    xacc: f32,

    /// Y acceleration
    yacc: f32,

    /// Z acceleration
    zacc: f32,

    /// Angular speed around X axis
    xgyro: f32,

    /// Angular speed around Y axis
    ygyro: f32,

    /// Angular speed around Z axis
    zgyro: f32,

    /// Latitude (lower precision). Both this and the lat_int field should be set.
    lat: f32,

    /// Longitude (lower precision). Both this and the lon_int field should be set.
    lon: f32,

    /// Altitude
    alt: f32,

    /// Horizontal position standard deviation
    std_dev_horz: f32,

    /// Vertical position standard deviation
    std_dev_vert: f32,

    /// True velocity in north direction in earth-fixed NED frame
    vn: f32,

    /// True velocity in east direction in earth-fixed NED frame
    ve: f32,

    /// True velocity in down direction in earth-fixed NED frame
    vd: f32,

    //Extension Field
    /// Latitude (higher precision). If 0, recipients should use the lat field value (otherwise this field is preferred).
    lat_int: i32,

    //Extension Field
    /// Longitude (higher precision). If 0, recipients should use the lon field value (otherwise this field is preferred).
    lon_int: i32,

};

/// The smoothed, monotonic system state used to feed the control loops of the system.
pub const CONTROL_SYSTEM_STATE = struct {
    pub const MSG_ID = 146;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X acceleration in body frame
    x_acc: f32,

    /// Y acceleration in body frame
    y_acc: f32,

    /// Z acceleration in body frame
    z_acc: f32,

    /// X velocity in body frame
    x_vel: f32,

    /// Y velocity in body frame
    y_vel: f32,

    /// Z velocity in body frame
    z_vel: f32,

    /// X position in local frame
    x_pos: f32,

    /// Y position in local frame
    y_pos: f32,

    /// Z position in local frame
    z_pos: f32,

    /// Airspeed, set to -1 if unknown
    airspeed: f32,

    /// The attitude, represented as Quaternion
    q: [4]f32,

    /// Angular rate in roll axis
    roll_rate: f32,

    /// Angular rate in pitch axis
    pitch_rate: f32,

    /// Angular rate in yaw axis
    yaw_rate: f32,

    /// Variance of body velocity estimate
    vel_variance: [3]f32,

    /// Variance in local position
    pos_variance: [3]f32,

};

/// Motion capture attitude and position
pub const ATT_POS_MOCAP = struct {
    pub const MSG_ID = 138;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    q: [4]f32,

    /// X position (NED)
    x: f32,

    /// Y position (NED)
    y: f32,

    /// Z position (NED)
    z: f32,

    //Extension Field
    /// Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [21]f32,

};

/// Message appropriate for high latency connections like Iridium (version 2)
pub const HIGH_LATENCY2 = struct {
    pub const MSG_ID = 235;
    /// Timestamp (milliseconds since boot or Unix epoch)
    timestamp: u32,

    /// Latitude
    latitude: i32,

    /// Longitude
    longitude: i32,

    /// A bitfield for use for autopilot-specific flags (2 byte version).
    custom_mode: u16,

    /// Altitude above mean sea level
    altitude: i16,

    /// Altitude setpoint
    target_altitude: i16,

    /// Distance to target waypoint or position
    target_distance: u16,

    /// Current waypoint number
    wp_num: u16,

    /// Bitmap of failure flags.
    failure_flags: enums.HL_FAILURE_FLAG.Type,

    /// Type of the MAV (quadrotor, helicopter, etc.)
    type: enums.MAV_TYPE,

    /// Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
    autopilot: enums.MAV_AUTOPILOT,

    /// Heading
    heading: u8,

    /// Heading setpoint
    target_heading: u8,

    /// Throttle
    throttle: u8,

    /// Airspeed
    airspeed: u8,

    /// Airspeed setpoint
    airspeed_sp: u8,

    /// Groundspeed
    groundspeed: u8,

    /// Windspeed
    windspeed: u8,

    /// Wind heading
    wind_heading: u8,

    /// Maximum error horizontal position since last message
    eph: u8,

    /// Maximum error vertical position since last message
    epv: u8,

    /// Air temperature
    temperature_air: i8,

    /// Maximum climb rate magnitude since last message
    climb_rate: i8,

    /// Battery level (-1 if field not provided).
    battery: i8,

    /// Field for custom payload.
    custom0: i8,

    /// Field for custom payload.
    custom1: i8,

    /// Field for custom payload.
    custom2: i8,

};

/// Local position/attitude estimate from a vision source.
pub const VISION_POSITION_ESTIMATE = struct {
    pub const MSG_ID = 102;
    /// Timestamp (UNIX time or time since system boot)
    usec: u64,

    /// Local X position
    x: f32,

    /// Local Y position
    y: f32,

    /// Local Z position
    z: f32,

    /// Roll angle
    roll: f32,

    /// Pitch angle
    pitch: f32,

    /// Yaw angle
    yaw: f32,

    //Extension Field
    /// Row-major representation of pose 6x6 cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [21]f32,

    //Extension Field
    /// Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
    reset_counter: u8,

};

/// Global position estimate from a Vicon motion system source.
pub const VICON_POSITION_ESTIMATE = struct {
    pub const MSG_ID = 104;
    /// Timestamp (UNIX time or time since system boot)
    usec: u64,

    /// Global X position
    x: f32,

    /// Global Y position
    y: f32,

    /// Global Z position
    z: f32,

    /// Roll angle
    roll: f32,

    /// Pitch angle
    pitch: f32,

    /// Yaw angle
    yaw: f32,

    //Extension Field
    /// Row-major representation of 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [21]f32,

};

/// Message reporting the status of a gimbal device.
///   This message should be broadcast by a gimbal device component at a low regular rate (e.g. 5 Hz).
///   For the angles encoded in the quaternion and the angular velocities holds:
///   If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME is set, then they are relative to the vehicle heading (vehicle frame).
///   If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME is set, then they are relative to absolute North (earth frame).
///   If neither of these flags are set, then (for backwards compatibility) it holds:
///   If the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, then they are relative to absolute North (earth frame),
///   else they are relative to the vehicle heading (vehicle frame).
///   Other conditions of the flags are not allowed.
///   The quaternion and angular velocities in the other frame can be calculated from delta_yaw and delta_yaw_velocity as
///   q_earth = q_delta_yaw * q_vehicle and w_earth = w_delta_yaw_velocity + w_vehicle (if not NaN).
///   If neither the GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME nor the GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME flag is set,
///   then (for backwards compatibility) the data in the delta_yaw and delta_yaw_velocity fields are to be ignored.
///   New implementations should always set either GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME or GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME,
///   and always should set delta_yaw and delta_yaw_velocity either to the proper value or NaN.
pub const GIMBAL_DEVICE_ATTITUDE_STATUS = struct {
    pub const MSG_ID = 285;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description.
    q: [4]f32,

    /// X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN if unknown.
    angular_velocity_x: f32,

    /// Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN if unknown.
    angular_velocity_y: f32,

    /// Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN if unknown.
    angular_velocity_z: f32,

    /// Failure flags (0 for no failure)
    failure_flags: enums.GIMBAL_DEVICE_ERROR_FLAGS.Type,

    /// Current gimbal flags set.
    flags: enums.GIMBAL_DEVICE_FLAGS.Type,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Yaw angle relating the quaternions in earth and body frames (see message description). NaN if unknown.
    delta_yaw: f32,

    //Extension Field
    /// Yaw angular velocity relating the angular velocities in earth and body frames (see message description). NaN if unknown.
    delta_yaw_velocity: f32,

    //Extension Field
    /// This field is to be used if the gimbal manager and the gimbal device are the same component and hence have the same component ID. This field is then set a number between 1-6. If the component ID is separate, this field is not required and must be set to 0.
    gimbal_device_id: u8,

};

/// Control vehicle tone generation (buzzer).
pub const PLAY_TUNE = struct {
    pub const MSG_ID = 258;
    /// tune in board specific format
    tune: [30]u8,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// tune extension (appended to tune)
    tune2: [200]u8,

};

/// The attitude in the aeronautical frame (right-handed, Z-down, Y-right, X-front, ZYX, intrinsic).
pub const ATTITUDE = struct {
    pub const MSG_ID = 30;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Roll angle (-pi..+pi)
    roll: f32,

    /// Pitch angle (-pi..+pi)
    pitch: f32,

    /// Yaw angle (-pi..+pi)
    yaw: f32,

    /// Roll angular speed
    rollspeed: f32,

    /// Pitch angular speed
    pitchspeed: f32,

    /// Yaw angular speed
    yawspeed: f32,

};

/// Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way.
pub const POSITION_TARGET_LOCAL_NED = struct {
    pub const MSG_ID = 85;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X Position in NED frame
    x: f32,

    /// Y Position in NED frame
    y: f32,

    /// Z Position in NED frame (note, altitude is negative in NED)
    z: f32,

    /// X velocity in NED frame
    vx: f32,

    /// Y velocity in NED frame
    vy: f32,

    /// Z velocity in NED frame
    vz: f32,

    /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afx: f32,

    /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afy: f32,

    /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afz: f32,

    /// yaw setpoint
    yaw: f32,

    /// yaw rate setpoint
    yaw_rate: f32,

    /// Bitmap to indicate which dimensions should be ignored by the vehicle.
    type_mask: enums.POSITION_TARGET_TYPEMASK.Type,

    /// Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
    coordinate_frame: enums.MAV_FRAME,

};

/// Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations.
pub const SAFETY_SET_ALLOWED_AREA = struct {
    pub const MSG_ID = 54;
    /// x position 1 / Latitude 1
    p1x: f32,

    /// y position 1 / Longitude 1
    p1y: f32,

    /// z position 1 / Altitude 1
    p1z: f32,

    /// x position 2 / Latitude 2
    p2x: f32,

    /// y position 2 / Longitude 2
    p2y: f32,

    /// z position 2 / Altitude 2
    p2z: f32,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    frame: enums.MAV_FRAME,

};

/// Time synchronization message.
///         The message is used for both timesync requests and responses.
///         The request is sent with `ts1=syncing component timestamp` and `tc1=0`, and may be broadcast or targeted to a specific system/component.
///         The response is sent with `ts1=syncing component timestamp` (mirror back unchanged), and `tc1=responding component timestamp`, with the `target_system` and `target_component` set to ids of the original request.
///         Systems can determine if they are receiving a request or response based on the value of `tc`.
///         If the response has `target_system==target_component==0` the remote system has not been updated to use the component IDs and cannot reliably timesync; the requester may report an error.
///         Timestamps are UNIX Epoch time or time since system boot in nanoseconds (the timestamp format can be inferred by checking for the magnitude of the number; generally it doesn't matter as only the offset is used).
///         The message sequence is repeated numerous times with results being filtered/averaged to estimate the offset.
///         See also: https://mavlink.io/en/services/timesync.html.
pub const TIMESYNC = struct {
    pub const MSG_ID = 111;
    /// Time sync timestamp 1. Syncing: 0. Responding: Timestamp of responding component.
    tc1: i64,

    /// Time sync timestamp 2. Timestamp of syncing component (mirrored in response).
    ts1: i64,

    //Extension Field
    /// Target system id. Request: 0 (broadcast) or id of specific system. Response must contain system id of the requesting component.
    target_system: u8,

    //Extension Field
    /// Target component id. Request: 0 (broadcast) or id of specific component. Response must contain component id of the requesting component.
    target_component: u8,

};

/// Data for filling the OpenDroneID Operator ID message, which contains the CAA (Civil Aviation Authority) issued operator ID.
pub const OPEN_DRONE_ID_OPERATOR_ID = struct {
    pub const MSG_ID = 12905;
    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
    operator_id: [20]u8,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// Indicates the type of the operator_id field.
    operator_id_type: enums.MAV_ODID_OPERATOR_ID_TYPE,

};

/// Request to read the value of a parameter with either the param_id string id or param_index. PARAM_EXT_VALUE should be emitted in response.
pub const PARAM_EXT_REQUEST_READ = struct {
    pub const MSG_ID = 320;
    /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)
    param_index: i16,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovation test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovation test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user.
pub const ESTIMATOR_STATUS = struct {
    pub const MSG_ID = 230;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Velocity innovation test ratio
    vel_ratio: f32,

    /// Horizontal position innovation test ratio
    pos_horiz_ratio: f32,

    /// Vertical position innovation test ratio
    pos_vert_ratio: f32,

    /// Magnetometer innovation test ratio
    mag_ratio: f32,

    /// Height above terrain innovation test ratio
    hagl_ratio: f32,

    /// True airspeed innovation test ratio
    tas_ratio: f32,

    /// Horizontal position 1-STD accuracy relative to the EKF local origin
    pos_horiz_accuracy: f32,

    /// Vertical position 1-STD accuracy relative to the EKF local origin
    pos_vert_accuracy: f32,

    /// Bitmap indicating which EKF outputs are valid.
    flags: enums.ESTIMATOR_STATUS_FLAGS.Type,

};

/// Send a command with up to seven parameters to the MAV. COMMAND_INT is generally preferred when sending MAV_CMD commands that include positional information; it offers higher precision and allows the MAV_FRAME to be specified (which may otherwise be ambiguous, particularly for altitude). The command microservice is documented at https://mavlink.io/en/services/command.html
pub const COMMAND_LONG = struct {
    pub const MSG_ID = 76;
    /// Parameter 1 (for the specific command).
    param1: f32,

    /// Parameter 2 (for the specific command).
    param2: f32,

    /// Parameter 3 (for the specific command).
    param3: f32,

    /// Parameter 4 (for the specific command).
    param4: f32,

    /// Parameter 5 (for the specific command).
    param5: f32,

    /// Parameter 6 (for the specific command).
    param6: f32,

    /// Parameter 7 (for the specific command).
    param7: f32,

    /// Command ID (of command to send).
    command: enums.MAV_CMD,

    /// System which should execute the command
    target_system: u8,

    /// Component which should execute the command, 0 for all components
    target_component: u8,

    /// 0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)
    confirmation: u8,

};

/// General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message "uavcan.protocol.NodeStatus" for the background information. The UAVCAN specification is available at http://uavcan.org.
pub const UAVCAN_NODE_STATUS = struct {
    pub const MSG_ID = 310;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Time since the start-up of the node.
    uptime_sec: u32,

    /// Vendor-specific status information.
    vendor_specific_status_code: u16,

    /// Generalized node health status.
    health: enums.UAVCAN_NODE_HEALTH,

    /// Generalized operating mode.
    mode: enums.UAVCAN_NODE_MODE,

    /// Not used currently.
    sub_mode: u8,

};

/// Information about a low level gimbal. This message should be requested by the gimbal manager or a ground station using MAV_CMD_REQUEST_MESSAGE. The maximum angles and rates are the limits by hardware. However, the limits by software used are likely different/smaller and dependent on mode/settings/etc..
pub const GIMBAL_DEVICE_INFORMATION = struct {
    pub const MSG_ID = 283;
    /// Name of the gimbal vendor.
    vendor_name: [32]u8,

    /// Name of the gimbal model.
    model_name: [32]u8,

    /// Custom name of the gimbal given to it by the user.
    custom_name: [32]u8,

    /// UID of gimbal hardware (0 if unknown).
    uid: u64,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Version of the gimbal firmware, encoded as: `(Dev & 0xff) << 24 \| (Patch & 0xff) << 16 \| (Minor & 0xff) << 8 \| (Major & 0xff)`.
    firmware_version: u32,

    /// Version of the gimbal hardware, encoded as: `(Dev & 0xff) << 24 \| (Patch & 0xff) << 16 \| (Minor & 0xff) << 8 \| (Major & 0xff)`.
    hardware_version: u32,

    /// Minimum hardware roll angle (positive: rolling to the right, negative: rolling to the left). NAN if unknown.
    roll_min: f32,

    /// Maximum hardware roll angle (positive: rolling to the right, negative: rolling to the left). NAN if unknown.
    roll_max: f32,

    /// Minimum hardware pitch angle (positive: up, negative: down). NAN if unknown.
    pitch_min: f32,

    /// Maximum hardware pitch angle (positive: up, negative: down). NAN if unknown.
    pitch_max: f32,

    /// Minimum hardware yaw angle (positive: to the right, negative: to the left). NAN if unknown.
    yaw_min: f32,

    /// Maximum hardware yaw angle (positive: to the right, negative: to the left). NAN if unknown.
    yaw_max: f32,

    /// Bitmap of gimbal capability flags.
    cap_flags: enums.GIMBAL_DEVICE_CAP_FLAGS.Type,

    /// Bitmap for use for gimbal-specific capability flags.
    custom_cap_flags: u16,

    //Extension Field
    /// This field is to be used if the gimbal manager and the gimbal device are the same component and hence have the same component ID. This field is then set to a number between 1-6. If the component ID is separate, this field is not required and must be set to 0.
    gimbal_device_id: u8,

};

/// The RAW IMU readings for a 9DOF sensor, which is identified by the id (default IMU1). This message should always contain the true raw values without any scaling to allow data capture and system debugging.
pub const RAW_IMU = struct {
    pub const MSG_ID = 27;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X acceleration (raw)
    xacc: i16,

    /// Y acceleration (raw)
    yacc: i16,

    /// Z acceleration (raw)
    zacc: i16,

    /// Angular speed around X axis (raw)
    xgyro: i16,

    /// Angular speed around Y axis (raw)
    ygyro: i16,

    /// Angular speed around Z axis (raw)
    zgyro: i16,

    /// X Magnetic field (raw)
    xmag: i16,

    /// Y Magnetic field (raw)
    ymag: i16,

    /// Z Magnetic field (raw)
    zmag: i16,

    //Extension Field
    /// Id. Ids are numbered from 0 and map to IMUs numbered from 1 (e.g. IMU1 will have a message with id=0)
    id: u8,

    //Extension Field
    /// Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
    temperature: i16,

};

/// The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units
pub const SCALED_IMU3 = struct {
    pub const MSG_ID = 129;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X acceleration
    xacc: i16,

    /// Y acceleration
    yacc: i16,

    /// Z acceleration
    zacc: i16,

    /// Angular speed around X axis
    xgyro: i16,

    /// Angular speed around Y axis
    ygyro: i16,

    /// Angular speed around Z axis
    zgyro: i16,

    /// X Magnetic field
    xmag: i16,

    /// Y Magnetic field
    ymag: i16,

    /// Z Magnetic field
    zmag: i16,

    //Extension Field
    /// Temperature, 0: IMU does not provide temperature values. If the IMU is at 0C it must send 1 (0.01C).
    temperature: i16,

};

/// Vehicle status report that is sent out while orbit execution is in progress (see MAV_CMD_DO_ORBIT).
pub const ORBIT_EXECUTION_STATUS = struct {
    pub const MSG_ID = 360;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise.
    radius: f32,

    /// X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
    x: i32,

    /// Y coordinate of center point.  Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
    y: i32,

    /// Altitude of center point. Coordinate system depends on frame field.
    z: f32,

    /// The coordinate system of the fields: x, y, z.
    frame: enums.MAV_FRAME,

};

/// A forwarded CAN frame as requested by MAV_CMD_CAN_FORWARD.
pub const CAN_FRAME = struct {
    pub const MSG_ID = 386;
    /// Frame data
    data: [8]u8,

    /// Frame ID
    id: u32,

    /// System ID.
    target_system: u8,

    /// Component ID.
    target_component: u8,

    /// Bus number
    bus: u8,

    /// Frame length
    len: u8,

};

/// Delete all mission items at once.
pub const MISSION_CLEAR_ALL = struct {
    pub const MSG_ID = 45;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Read out the safety zone the MAV currently assumes.
pub const SAFETY_ALLOWED_AREA = struct {
    pub const MSG_ID = 55;
    /// x position 1 / Latitude 1
    p1x: f32,

    /// y position 1 / Longitude 1
    p1y: f32,

    /// z position 1 / Altitude 1
    p1z: f32,

    /// x position 2 / Latitude 2
    p2x: f32,

    /// y position 2 / Longitude 2
    p2y: f32,

    /// z position 2 / Altitude 2
    p2z: f32,

    /// Coordinate frame. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down.
    frame: enums.MAV_FRAME,

};

/// Data for injecting into the onboard GPS (used for DGPS)
pub const GPS_INJECT_DATA = struct {
    pub const MSG_ID = 123;
    /// Raw data (110 is enough for 12 satellites of RTCMv2)
    data: [110]u8,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Data length
    len: u8,

};

/// Flight information.
///         This includes time since boot for arm, takeoff, and land, and a flight number.
///         Takeoff and landing values reset to zero on arm.
///         This can be requested using MAV_CMD_REQUEST_MESSAGE.
///         Note, some fields are misnamed - timestamps are from boot (not UTC) and the flight_uuid is a sequence number.
pub const FLIGHT_INFORMATION = struct {
    pub const MSG_ID = 264;
    /// Timestamp at arming (since system boot). Set to 0 on boot. Set value on arming. Note, field is misnamed UTC.
    arming_time_utc: u64,

    /// Timestamp at takeoff (since system boot). Set to 0 at boot and on arming. Note, field is misnamed UTC.
    takeoff_time_utc: u64,

    /// Flight number. Note, field is misnamed UUID.
    flight_uuid: u64,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    //Extension Field
    /// Timestamp at landing (in ms since system boot). Set to 0 at boot and on arming.
    landing_time: u32,

};

/// The raw values of the actuator outputs (e.g. on Pixhawk, from MAIN, AUX ports). This message supersedes SERVO_OUTPUT_RAW.
pub const ACTUATOR_OUTPUT_STATUS = struct {
    pub const MSG_ID = 375;
    /// Servo / motor output array values. Zero values indicate unused channels.
    actuator: [32]f32,

    /// Timestamp (since system boot).
    time_usec: u64,

    /// Active outputs
    active: u32,

};

/// Camera-IMU triggering and synchronisation message.
pub const CAMERA_TRIGGER = struct {
    pub const MSG_ID = 112;
    /// Timestamp for image frame (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Image frame sequence
    seq: u32,

};

/// The system time is the time of the master clock.
///         This can be emitted by flight controllers, onboard computers, or other components in the MAVLink network.
///         Components that are using a less reliable time source, such as a battery-backed real time clock, can choose to match their system clock to that of a SYSTEM_TYPE that indicates a more recent time.
///         This allows more broadly accurate date stamping of logs, and so on.
///         If precise time synchronization is needed then use TIMESYNC instead.
pub const SYSTEM_TIME = struct {
    pub const MSG_ID = 2;
    /// Timestamp (UNIX epoch time).
    time_unix_usec: u64,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

};

/// Stop log transfer and resume normal logging
pub const LOG_REQUEST_END = struct {
    pub const MSG_ID = 122;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Data for filling the OpenDroneID System message. The System Message contains general system information including the operator location/altitude and possible aircraft group and/or category/class information.
pub const OPEN_DRONE_ID_SYSTEM = struct {
    pub const MSG_ID = 12904;
    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// Latitude of the operator. If unknown: 0 (both Lat/Lon).
    operator_latitude: i32,

    /// Longitude of the operator. If unknown: 0 (both Lat/Lon).
    operator_longitude: i32,

    /// Area Operations Ceiling relative to WGS84. If unknown: -1000 m. Used only for swarms/multiple UA.
    area_ceiling: f32,

    /// Area Operations Floor relative to WGS84. If unknown: -1000 m. Used only for swarms/multiple UA.
    area_floor: f32,

    /// Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
    operator_altitude_geo: f32,

    /// 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
    timestamp: u32,

    /// Number of aircraft in the area, group or formation (default 1). Used only for swarms/multiple UA.
    area_count: u16,

    /// Radius of the cylindrical area of the group or formation (default 0). Used only for swarms/multiple UA.
    area_radius: u16,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// Specifies the operator location type.
    operator_location_type: enums.MAV_ODID_OPERATOR_LOCATION_TYPE,

    /// Specifies the classification type of the UA.
    classification_type: enums.MAV_ODID_CLASSIFICATION_TYPE,

    /// When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the category of the UA.
    category_eu: enums.MAV_ODID_CATEGORY_EU,

    /// When classification_type is MAV_ODID_CLASSIFICATION_TYPE_EU, specifies the class of the UA.
    class_eu: enums.MAV_ODID_CLASS_EU,

};

/// Transmitter (remote ID system) is enabled and ready to start sending location and other required information. This is streamed by transmitter. A flight controller uses it as a condition to arm.
pub const OPEN_DRONE_ID_ARM_STATUS = struct {
    pub const MSG_ID = 12918;
    /// Text error message, should be empty if status is good to arm. Fill with nulls in unused portion.
    @"error": [50]u8,

    /// Status level indicating if arming is allowed.
    status: enums.MAV_ODID_ARM_STATUS,

};

/// Current motion information from a designated system
pub const FOLLOW_TARGET = struct {
    pub const MSG_ID = 144;
    /// Timestamp (time since system boot).
    timestamp: u64,

    /// button states or switches of a tracker device
    custom_state: u64,

    /// Latitude (WGS84)
    lat: i32,

    /// Longitude (WGS84)
    lon: i32,

    /// Altitude (MSL)
    alt: f32,

    /// (0 0 0 0 for unknown)
    attitude_q: [4]f32,

    /// target velocity (0,0,0) for unknown
    vel: [3]f32,

    /// linear target acceleration (0,0,0) for unknown
    acc: [3]f32,

    /// (0 0 0 for unknown)
    rates: [3]f32,

    /// eph epv
    position_cov: [3]f32,

    /// bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)
    est_capabilities: u8,

};

/// The global position, as returned by the Global Positioning System (GPS). This is
///                 NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION_INT for the global position estimate.
pub const GPS_RAW_INT = struct {
    pub const MSG_ID = 24;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Latitude (WGS84, EGM96 ellipsoid)
    lat: i32,

    /// Longitude (WGS84, EGM96 ellipsoid)
    lon: i32,

    /// Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.
    alt: i32,

    /// GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    eph: u16,

    /// GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX
    epv: u16,

    /// GPS ground speed. If unknown, set to: UINT16_MAX
    vel: u16,

    /// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
    cog: u16,

    /// GPS fix type.
    fix_type: enums.GPS_FIX_TYPE,

    /// Number of satellites visible. If unknown, set to UINT8_MAX
    satellites_visible: u8,

    //Extension Field
    /// Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
    alt_ellipsoid: i32,

    //Extension Field
    /// Position uncertainty.
    h_acc: u32,

    //Extension Field
    /// Altitude uncertainty.
    v_acc: u32,

    //Extension Field
    /// Speed uncertainty.
    vel_acc: u32,

    //Extension Field
    /// Heading / track uncertainty
    hdg_acc: u32,

    //Extension Field
    /// Yaw in earth frame from north. Use 0 if this GPS does not provide yaw. Use UINT16_MAX if this GPS is configured to provide yaw and is currently unable to provide it. Use 36000 for north.
    yaw: u16,

};

/// Sent from autopilot to simulation. Hardware in the loop control outputs. Alternative to HIL_ACTUATOR_CONTROLS.
pub const HIL_CONTROLS = struct {
    pub const MSG_ID = 91;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Control output -1 .. 1
    roll_ailerons: f32,

    /// Control output -1 .. 1
    pitch_elevator: f32,

    /// Control output -1 .. 1
    yaw_rudder: f32,

    /// Throttle 0 .. 1
    throttle: f32,

    /// Aux 1, -1 .. 1
    aux1: f32,

    /// Aux 2, -1 .. 1
    aux2: f32,

    /// Aux 3, -1 .. 1
    aux3: f32,

    /// Aux 4, -1 .. 1
    aux4: f32,

    /// System mode.
    mode: enums.MAV_MODE,

    /// Navigation mode (MAV_NAV_MODE)
    nav_mode: u8,

};

/// Message appropriate for high latency connections like Iridium
pub const HIGH_LATENCY = struct {
    pub const MSG_ID = 234;
    /// A bitfield for use for autopilot-specific flags.
    custom_mode: u32,

    /// Latitude
    latitude: i32,

    /// Longitude
    longitude: i32,

    /// roll
    roll: i16,

    /// pitch
    pitch: i16,

    /// heading
    heading: u16,

    /// heading setpoint
    heading_sp: i16,

    /// Altitude above mean sea level
    altitude_amsl: i16,

    /// Altitude setpoint relative to the home position
    altitude_sp: i16,

    /// distance to target
    wp_distance: u16,

    /// Bitmap of enabled system modes.
    base_mode: enums.MAV_MODE_FLAG.Type,

    /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    landed_state: enums.MAV_LANDED_STATE,

    /// throttle (percentage)
    throttle: i8,

    /// airspeed
    airspeed: u8,

    /// airspeed setpoint
    airspeed_sp: u8,

    /// groundspeed
    groundspeed: u8,

    /// climb rate
    climb_rate: i8,

    /// Number of satellites visible. If unknown, set to UINT8_MAX
    gps_nsat: u8,

    /// GPS Fix type.
    gps_fix_type: enums.GPS_FIX_TYPE,

    /// Remaining battery (percentage)
    battery_remaining: u8,

    /// Autopilot temperature (degrees C)
    temperature: i8,

    /// Air temperature (degrees C) from airspeed sensor
    temperature_air: i8,

    /// failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)
    failsafe: u8,

    /// current waypoint number
    wp_num: u8,

};

/// EFI status output
pub const EFI_STATUS = struct {
    pub const MSG_ID = 225;
    /// ECU index
    ecu_index: f32,

    /// RPM
    rpm: f32,

    /// Fuel consumed
    fuel_consumed: f32,

    /// Fuel flow rate
    fuel_flow: f32,

    /// Engine load
    engine_load: f32,

    /// Throttle position
    throttle_position: f32,

    /// Spark dwell time
    spark_dwell_time: f32,

    /// Barometric pressure
    barometric_pressure: f32,

    /// Intake manifold pressure(
    intake_manifold_pressure: f32,

    /// Intake manifold temperature
    intake_manifold_temperature: f32,

    /// Cylinder head temperature
    cylinder_head_temperature: f32,

    /// Ignition timing (Crank angle degrees)
    ignition_timing: f32,

    /// Injection time
    injection_time: f32,

    /// Exhaust gas temperature
    exhaust_gas_temperature: f32,

    /// Output throttle
    throttle_out: f32,

    /// Pressure/temperature compensation
    pt_compensation: f32,

    /// EFI health status
    health: u8,

    //Extension Field
    /// Supply voltage to EFI sparking system.  Zero in this value means "unknown", so if the supply voltage really is zero volts use 0.0001 instead.
    ignition_voltage: f32,

    //Extension Field
    /// Fuel pressure. Zero in this value means "unknown", so if the fuel pressure really is zero kPa use 0.0001 instead.
    fuel_pressure: f32,

};

/// Basic component information data. Should be requested using MAV_CMD_REQUEST_MESSAGE on startup, or when required.
pub const COMPONENT_INFORMATION_BASIC = struct {
    pub const MSG_ID = 396;
    /// Name of the component vendor. Needs to be zero terminated. The field is optional and can be empty/all zeros.
    vendor_name: [32]u8,

    /// Name of the component model. Needs to be zero terminated. The field is optional and can be empty/all zeros.
    model_name: [32]u8,

    /// Hardware serial number. The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
    serial_number: [32]u8,

    /// Software version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
    software_version: [24]u8,

    /// Hardware version. The recommended format is SEMVER: 'major.minor.patch'  (any format may be used). The field must be zero terminated if it has a value. The field is optional and can be empty/all zeros.
    hardware_version: [24]u8,

    /// Component capability flags
    capabilities: enums.MAV_PROTOCOL_CAPABILITY.Type,

    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Date of manufacture as a UNIX Epoch time (since 1.1.1970) in seconds.
    time_manufacture_s: u32,

};

/// Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system).
pub const SET_POSITION_TARGET_GLOBAL_INT = struct {
    pub const MSG_ID = 86;
    /// Timestamp (time since system boot). The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency.
    time_boot_ms: u32,

    /// Latitude in WGS84 frame
    lat_int: i32,

    /// Longitude in WGS84 frame
    lon_int: i32,

    /// Altitude (MSL, Relative to home, or AGL - depending on frame)
    alt: f32,

    /// X velocity in NED frame
    vx: f32,

    /// Y velocity in NED frame
    vy: f32,

    /// Z velocity in NED frame
    vz: f32,

    /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afx: f32,

    /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afy: f32,

    /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afz: f32,

    /// yaw setpoint
    yaw: f32,

    /// yaw rate setpoint
    yaw_rate: f32,

    /// Bitmap to indicate which dimensions should be ignored by the vehicle.
    type_mask: enums.POSITION_TARGET_TYPEMASK.Type,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Valid options are: MAV_FRAME_GLOBAL = 0, MAV_FRAME_GLOBAL_RELATIVE_ALT = 3, MAV_FRAME_GLOBAL_TERRAIN_ALT = 10 (MAV_FRAME_GLOBAL_INT, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT are allowed synonyms, but have been deprecated)
    coordinate_frame: enums.MAV_FRAME,

};

/// Information about a flight mode.
/// 
///         The message can be enumerated to get information for all modes, or requested for a particular mode, using MAV_CMD_REQUEST_MESSAGE.
///         Specify 0 in param2 to request that the message is emitted for all available modes or the specific index for just one mode.
///         The modes must be available/settable for the current vehicle/frame type.
///         Each mode should only be emitted once (even if it is both standard and custom).
///         Note that the current mode should be emitted in CURRENT_MODE, and that if the mode list can change then AVAILABLE_MODES_MONITOR must be emitted on first change and subsequently streamed.
///         See https://mavlink.io/en/services/standard_modes.html
pub const AVAILABLE_MODES = struct {
    pub const MSG_ID = 435;
    /// Name of custom mode, with null termination character. Should be omitted for standard modes.
    mode_name: [35]u8,

    /// A bitfield for use for autopilot-specific flags
    custom_mode: u32,

    /// Mode properties.
    properties: enums.MAV_MODE_PROPERTY.Type,

    /// The total number of available modes for the current vehicle type.
    number_modes: u8,

    /// The current mode index within number_modes, indexed from 1. The index is not guaranteed to be persistent, and may change between reboots or if the set of modes change.
    mode_index: u8,

    /// Standard mode.
    standard_mode: enums.MAV_STANDARD_MODE,

};

/// Tune formats supported by vehicle. This should be emitted as response to MAV_CMD_REQUEST_MESSAGE.
pub const SUPPORTED_TUNES = struct {
    pub const MSG_ID = 401;
    /// Bitfield of supported tune formats.
    format: enums.TUNE_FORMAT,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Report button state change.
pub const BUTTON_CHANGE = struct {
    pub const MSG_ID = 257;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Time of last change of button state.
    last_change_ms: u32,

    /// Bitmap for state of buttons.
    state: u8,

};

/// The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)
pub const LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET = struct {
    pub const MSG_ID = 89;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X Position
    x: f32,

    /// Y Position
    y: f32,

    /// Z Position
    z: f32,

    /// Roll
    roll: f32,

    /// Pitch
    pitch: f32,

    /// Yaw
    yaw: f32,

};

/// Obstacle distances in front of the sensor, starting from the left in increment degrees to the right
pub const OBSTACLE_DISTANCE = struct {
    pub const MSG_ID = 330;
    /// Distance of obstacles around the vehicle with index 0 corresponding to north + angle_offset, unless otherwise specified in the frame. A value of 0 is valid and means that the obstacle is practically touching the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm.
    distances: [72]u16,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Minimum distance the sensor can measure.
    min_distance: u16,

    /// Maximum distance the sensor can measure.
    max_distance: u16,

    /// Class id of the distance sensor type.
    sensor_type: enums.MAV_DISTANCE_SENSOR,

    /// Angular width in degrees of each array element. Increment direction is clockwise. This field is ignored if increment_f is non-zero.
    increment: u8,

    //Extension Field
    /// Angular width in degrees of each array element as a float. If non-zero then this value is used instead of the uint8_t increment field. Positive is clockwise direction, negative is counter-clockwise.
    increment_f: f32,

    //Extension Field
    /// Relative angle offset of the 0-index element in the distances array. Value of 0 corresponds to forward. Positive is clockwise direction, negative is counter-clockwise.
    angle_offset: f32,

    //Extension Field
    /// Coordinate frame of reference for the yaw rotation and offset of the sensor data. Defaults to MAV_FRAME_GLOBAL, which is north aligned. For body-mounted sensors use MAV_FRAME_BODY_FRD, which is vehicle front aligned.
    frame: enums.MAV_FRAME,

};

/// Reply to LOG_REQUEST_LIST
pub const LOG_ENTRY = struct {
    pub const MSG_ID = 118;
    /// UTC timestamp of log since 1970, or 0 if not available
    time_utc: u32,

    /// Size of the log (may be approximate)
    size: u32,

    /// Log id
    id: u16,

    /// Total number of logs
    num_logs: u16,

    /// High log number
    last_log_num: u16,

};

/// The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occurred. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occurred it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout.
pub const SYS_STATUS = struct {
    pub const MSG_ID = 1;
    /// Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
    onboard_control_sensors_present: enums.MAV_SYS_STATUS_SENSOR.Type,

    /// Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
    onboard_control_sensors_enabled: enums.MAV_SYS_STATUS_SENSOR.Type,

    /// Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
    onboard_control_sensors_health: enums.MAV_SYS_STATUS_SENSOR.Type,

    /// Maximum usage in percent of the mainloop time. Values: [0-1000] - should always be below 1000
    load: u16,

    /// Battery voltage, UINT16_MAX: Voltage not sent by autopilot
    voltage_battery: u16,

    /// Battery current, -1: Current not sent by autopilot
    current_battery: i16,

    /// Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
    drop_rate_comm: u16,

    /// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
    errors_comm: u16,

    /// Autopilot-specific errors
    errors_count1: u16,

    /// Autopilot-specific errors
    errors_count2: u16,

    /// Autopilot-specific errors
    errors_count3: u16,

    /// Autopilot-specific errors
    errors_count4: u16,

    /// Battery energy remaining, -1: Battery remaining energy not sent by autopilot
    battery_remaining: i8,

    //Extension Field
    /// Bitmap showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present.
    onboard_control_sensors_present_extended: enums.MAV_SYS_STATUS_SENSOR_EXTENDED.Type,

    //Extension Field
    /// Bitmap showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled.
    onboard_control_sensors_enabled_extended: enums.MAV_SYS_STATUS_SENSOR_EXTENDED.Type,

    //Extension Field
    /// Bitmap showing which onboard controllers and sensors have an error (or are operational). Value of 0: error. Value of 1: healthy.
    onboard_control_sensors_health_extended: enums.MAV_SYS_STATUS_SENSOR_EXTENDED.Type,

};

/// This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled and buttons states are transmitted as individual on/off bits of a bitmask
pub const MANUAL_CONTROL = struct {
    pub const MSG_ID = 69;
    /// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
    x: i16,

    /// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
    y: i16,

    /// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust.
    z: i16,

    /// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
    r: i16,

    /// A bitfield corresponding to the joystick buttons' 0-15 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
    buttons: u16,

    /// The system to be controlled.
    target: u8,

    //Extension Field
    /// A bitfield corresponding to the joystick buttons' 16-31 current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 16.
    buttons2: u16,

    //Extension Field
    /// Set bits to 1 to indicate which of the following extension fields contain valid data: bit 0: pitch, bit 1: roll, bit 2: aux1, bit 3: aux2, bit 4: aux3, bit 5: aux4, bit 6: aux5, bit 7: aux6
    enabled_extensions: u8,

    //Extension Field
    /// Pitch-only-axis, normalized to the range [-1000,1000]. Generally corresponds to pitch on vehicles with additional degrees of freedom. Valid if bit 0 of enabled_extensions field is set. Set to 0 if invalid.
    s: i16,

    //Extension Field
    /// Roll-only-axis, normalized to the range [-1000,1000]. Generally corresponds to roll on vehicles with additional degrees of freedom. Valid if bit 1 of enabled_extensions field is set. Set to 0 if invalid.
    t: i16,

    //Extension Field
    /// Aux continuous input field 1. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 2 of enabled_extensions field is set. 0 if bit 2 is unset.
    aux1: i16,

    //Extension Field
    /// Aux continuous input field 2. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 3 of enabled_extensions field is set. 0 if bit 3 is unset.
    aux2: i16,

    //Extension Field
    /// Aux continuous input field 3. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 4 of enabled_extensions field is set. 0 if bit 4 is unset.
    aux3: i16,

    //Extension Field
    /// Aux continuous input field 4. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 5 of enabled_extensions field is set. 0 if bit 5 is unset.
    aux4: i16,

    //Extension Field
    /// Aux continuous input field 5. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 6 of enabled_extensions field is set. 0 if bit 6 is unset.
    aux5: i16,

    //Extension Field
    /// Aux continuous input field 6. Normalized in the range [-1000,1000]. Purpose defined by recipient. Valid data if bit 7 of enabled_extensions field is set. 0 if bit 7 is unset.
    aux6: i16,

};

/// Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)
pub const HIL_OPTICAL_FLOW = struct {
    pub const MSG_ID = 114;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Integration time. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
    integration_time_us: u32,

    /// Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
    integrated_x: f32,

    /// Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
    integrated_y: f32,

    /// RH rotation around X axis
    integrated_xgyro: f32,

    /// RH rotation around Y axis
    integrated_ygyro: f32,

    /// RH rotation around Z axis
    integrated_zgyro: f32,

    /// Time since the distance was sampled.
    time_delta_distance_us: u32,

    /// Distance to the center of the flow field. Positive value (including zero): distance known. Negative value: Unknown distance.
    distance: f32,

    /// Temperature
    temperature: i16,

    /// Sensor ID
    sensor_id: u8,

    /// Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
    quality: u8,

};

/// Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component.
pub const SET_MODE = struct {
    pub const MSG_ID = 11;
    /// The new autopilot-specific mode. This field can be ignored by an autopilot.
    custom_mode: u32,

    /// The system setting the mode
    target_system: u8,

    /// The new base mode.
    base_mode: enums.MAV_MODE,

};

/// Drone position.
pub const AVSS_DRONE_POSITION = struct {
    pub const MSG_ID = 60051;
    /// Timestamp (time since FC boot).
    time_boot_ms: u32,

    /// Latitude, expressed
    lat: i32,

    /// Longitude, expressed
    lon: i32,

    /// Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.
    alt: i32,

    /// Altitude above ground, This altitude is measured by a ultrasound, Laser rangefinder or millimeter-wave radar
    ground_alt: f32,

    /// This altitude is measured by a barometer
    barometer_alt: f32,

};

/// Setpoint in roll, pitch, yaw and thrust from the operator
pub const MANUAL_SETPOINT = struct {
    pub const MSG_ID = 81;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Desired roll rate
    roll: f32,

    /// Desired pitch rate
    pitch: f32,

    /// Desired yaw rate
    yaw: f32,

    /// Collective thrust, normalized to 0 .. 1
    thrust: f32,

    /// Flight mode switch position, 0.. 255
    mode_switch: u8,

    /// Override mode switch position, 0.. 255
    manual_override_switch: u8,

};

/// RTCM message for injecting into the onboard GPS (used for DGPS)
pub const GPS_RTCM_DATA = struct {
    pub const MSG_ID = 233;
    /// RTCM message (may be fragmented)
    data: [180]u8,

    /// LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.
    flags: u8,

    /// data length
    len: u8,

};

/// Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way.
pub const ATTITUDE_TARGET = struct {
    pub const MSG_ID = 83;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
    q: [4]f32,

    /// Body roll rate
    body_roll_rate: f32,

    /// Body pitch rate
    body_pitch_rate: f32,

    /// Body yaw rate
    body_yaw_rate: f32,

    /// Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
    thrust: f32,

    /// Bitmap to indicate which dimensions should be ignored by the vehicle.
    type_mask: enums.ATTITUDE_TARGET_TYPEMASK.Type,

};

/// Set the vehicle attitude and body angular rates.
pub const ACTUATOR_CONTROL_TARGET = struct {
    pub const MSG_ID = 140;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs.
    controls: [8]f32,

    /// Actuator group. The "_mlx" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances.
    group_mlx: u8,

};

/// Contains the home position.
/// The home position is the default position that the system will return to and land on.
/// The position must be set automatically by the system during the takeoff, and may also be explicitly set using MAV_CMD_DO_SET_HOME.
/// The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface.
/// Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach.
/// The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector.
///         Note: this message can be requested by sending the MAV_CMD_REQUEST_MESSAGE with param1=242 (or the deprecated MAV_CMD_GET_HOME_POSITION command).
pub const HOME_POSITION = struct {
    pub const MSG_ID = 242;
    /// Latitude (WGS84)
    latitude: i32,

    /// Longitude (WGS84)
    longitude: i32,

    /// Altitude (MSL). Positive for up.
    altitude: i32,

    /// Local X position of this position in the local coordinate frame (NED)
    x: f32,

    /// Local Y position of this position in the local coordinate frame (NED)
    y: f32,

    /// Local Z position of this position in the local coordinate frame (NED: positive "down")
    z: f32,

    /// 
    ///         Quaternion indicating world-to-surface-normal and heading transformation of the takeoff position.
    ///         Used to indicate the heading and slope of the ground.
    ///         All fields should be set to NaN if an accurate quaternion for both heading and surface slope cannot be supplied.
    ///       
    q: [4]f32,

    /// Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    approach_x: f32,

    /// Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    approach_y: f32,

    /// Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone.
    approach_z: f32,

    //Extension Field
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

};

/// Data for filling the OpenDroneID Authentication message. The Authentication Message defines a field that can provide a means of authenticity for the identity of the UAS (Unmanned Aircraft System). The Authentication message can have two different formats. For data page 0, the fields PageCount, Length and TimeStamp are present and AuthData is only 17 bytes. For data page 1 through 15, PageCount, Length and TimeStamp are not present and the size of AuthData is 23 bytes.
pub const OPEN_DRONE_ID_AUTHENTICATION = struct {
    pub const MSG_ID = 12902;
    /// Opaque authentication data. For page 0, the size is only 17 bytes. For other pages, the size is 23 bytes. Shall be filled with nulls in the unused portion of the field.
    authentication_data: [23]u8,

    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// This field is only present for page 0. 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
    timestamp: u32,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// Indicates the type of authentication.
    authentication_type: enums.MAV_ODID_AUTH_TYPE,

    /// Allowed range is 0 - 15.
    data_page: u8,

    /// This field is only present for page 0. Allowed range is 0 - 15. See the description of struct ODID_Auth_data at https://github.com/opendroneid/opendroneid-core-c/blob/master/libopendroneid/opendroneid.h.
    last_page_index: u8,

    /// This field is only present for page 0. Total bytes of authentication_data from all data pages. See the description of struct ODID_Auth_data at https://github.com/opendroneid/opendroneid-core-c/blob/master/libopendroneid/opendroneid.h.
    length: u8,

};

/// The IMU readings in SI units in NED body frame
pub const HIL_SENSOR = struct {
    pub const MSG_ID = 107;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X acceleration
    xacc: f32,

    /// Y acceleration
    yacc: f32,

    /// Z acceleration
    zacc: f32,

    /// Angular speed around X axis in body frame
    xgyro: f32,

    /// Angular speed around Y axis in body frame
    ygyro: f32,

    /// Angular speed around Z axis in body frame
    zgyro: f32,

    /// X Magnetic field
    xmag: f32,

    /// Y Magnetic field
    ymag: f32,

    /// Z Magnetic field
    zmag: f32,

    /// Absolute pressure
    abs_pressure: f32,

    /// Differential pressure (airspeed)
    diff_pressure: f32,

    /// Altitude calculated from pressure
    pressure_alt: f32,

    /// Temperature
    temperature: f32,

    /// Bitmap for fields that have updated since last message
    fields_updated: enums.HIL_SENSOR_UPDATED_FLAGS.Type,

    //Extension Field
    /// Sensor ID (zero indexed). Used for multiple sensor inputs
    id: u8,

};

/// Describe a trajectory using an array of up-to 5 waypoints in the local frame (MAV_FRAME_LOCAL_NED).
pub const TRAJECTORY_REPRESENTATION_WAYPOINTS = struct {
    pub const MSG_ID = 332;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X-coordinate of waypoint, set to NaN if not being used
    pos_x: [5]f32,

    /// Y-coordinate of waypoint, set to NaN if not being used
    pos_y: [5]f32,

    /// Z-coordinate of waypoint, set to NaN if not being used
    pos_z: [5]f32,

    /// X-velocity of waypoint, set to NaN if not being used
    vel_x: [5]f32,

    /// Y-velocity of waypoint, set to NaN if not being used
    vel_y: [5]f32,

    /// Z-velocity of waypoint, set to NaN if not being used
    vel_z: [5]f32,

    /// X-acceleration of waypoint, set to NaN if not being used
    acc_x: [5]f32,

    /// Y-acceleration of waypoint, set to NaN if not being used
    acc_y: [5]f32,

    /// Z-acceleration of waypoint, set to NaN if not being used
    acc_z: [5]f32,

    /// Yaw angle, set to NaN if not being used
    pos_yaw: [5]f32,

    /// Yaw rate, set to NaN if not being used
    vel_yaw: [5]f32,

    /// MAV_CMD command id of waypoint, set to UINT16_MAX if not being used.
    command: enums.MAV_CMD,

    /// Number of valid points (up-to 5 waypoints are possible)
    valid_points: u8,

};

/// Request a data stream.
pub const REQUEST_DATA_STREAM = struct {
    pub const MSG_ID = 66;
    /// The requested message rate
    req_message_rate: u16,

    /// The target requested to send the message stream.
    target_system: u8,

    /// The target requested to send the message stream.
    target_component: u8,

    /// The ID of the requested data stream
    req_stream_id: u8,

    /// 1 to start sending, 0 to stop sending.
    start_stop: u8,

};

/// Message encoding a mission item. This message is emitted to announce
///                 the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). NaN or INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current latitude, yaw rather than a specific value). See also https://mavlink.io/en/services/mission.html.
pub const MISSION_ITEM_INT = struct {
    pub const MSG_ID = 73;
    /// PARAM1, see MAV_CMD enum
    param1: f32,

    /// PARAM2, see MAV_CMD enum
    param2: f32,

    /// PARAM3, see MAV_CMD enum
    param3: f32,

    /// PARAM4, see MAV_CMD enum
    param4: f32,

    /// PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7
    x: i32,

    /// PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7
    y: i32,

    /// PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame.
    z: f32,

    /// Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
    seq: u16,

    /// The scheduled action for the waypoint.
    command: enums.MAV_CMD,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// The coordinate system of the waypoint.
    frame: enums.MAV_FRAME,

    /// false:0, true:1
    current: u8,

    /// Autocontinue to next waypoint. 0: false, 1: true. Set false to pause mission after the item completes.
    autocontinue: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Speed estimate from a vision source.
pub const VISION_SPEED_ESTIMATE = struct {
    pub const MSG_ID = 103;
    /// Timestamp (UNIX time or time since system boot)
    usec: u64,

    /// Global X speed
    x: f32,

    /// Global Y speed
    y: f32,

    /// Global Z speed
    z: f32,

    //Extension Field
    /// Row-major representation of 3x3 linear velocity covariance matrix (states: vx, vy, vz; 1st three entries - 1st row, etc.). If unknown, assign NaN value to first element in the array.
    covariance: [9]f32,

    //Extension Field
    /// Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
    reset_counter: u8,

};

/// The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION_INT for the global position estimate. This message can contain information for up to 20 satellites.
pub const GPS_STATUS = struct {
    pub const MSG_ID = 25;
    /// Global satellite ID
    satellite_prn: [20]u8,

    /// 0: Satellite not used, 1: used for localization
    satellite_used: [20]u8,

    /// Elevation (0: right on top of receiver, 90: on the horizon) of satellite
    satellite_elevation: [20]u8,

    /// Direction of satellite, 0: 0 deg, 255: 360 deg.
    satellite_azimuth: [20]u8,

    /// Signal to noise ratio of satellite
    satellite_snr: [20]u8,

    /// Number of satellites visible
    satellites_visible: u8,

};

/// Sent from autopilot to simulation. Hardware in the loop control outputs. Alternative to HIL_CONTROLS.
pub const HIL_ACTUATOR_CONTROLS = struct {
    pub const MSG_ID = 93;
    /// Control outputs -1 .. 1. Channel assignment depends on the simulated hardware.
    controls: [16]f32,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Flags bitmask.
    flags: enums.HIL_ACTUATOR_CONTROLS_FLAGS.Type,

    /// System mode. Includes arming state.
    mode: enums.MAV_MODE_FLAG.Type,

};

/// Request the overall list of mission items from the system/component.
pub const MISSION_REQUEST_LIST = struct {
    pub const MSG_ID = 43;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

};

/// Optical flow from a flow sensor (e.g. optical mouse sensor)
pub const OPTICAL_FLOW = struct {
    pub const MSG_ID = 100;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// Flow in x-sensor direction, angular-speed compensated
    flow_comp_m_x: f32,

    /// Flow in y-sensor direction, angular-speed compensated
    flow_comp_m_y: f32,

    /// Ground distance. Positive value: distance known. Negative value: Unknown distance
    ground_distance: f32,

    /// Flow in x-sensor direction
    flow_x: i16,

    /// Flow in y-sensor direction
    flow_y: i16,

    /// Sensor ID
    sensor_id: u8,

    /// Optical flow quality / confidence. 0: bad, 255: maximum quality
    quality: u8,

    //Extension Field
    /// Flow rate about X axis
    flow_rate_x: f32,

    //Extension Field
    /// Flow rate about Y axis
    flow_rate_y: f32,

};

/// Barometer readings for 2nd barometer
pub const SCALED_PRESSURE2 = struct {
    pub const MSG_ID = 137;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Absolute pressure
    press_abs: f32,

    /// Differential pressure
    press_diff: f32,

    /// Absolute pressure temperature
    temperature: i16,

    //Extension Field
    /// Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
    temperature_press_diff: i16,

};

/// Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
pub const TERRAIN_DATA = struct {
    pub const MSG_ID = 134;
    /// Terrain data MSL
    data: [16]i16,

    /// Latitude of SW corner of first grid
    lat: i32,

    /// Longitude of SW corner of first grid
    lon: i32,

    /// Grid spacing
    grid_spacing: u16,

    /// bit within the terrain request mask
    gridbit: u8,

};

/// GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the system.
pub const GPS_INPUT = struct {
    pub const MSG_ID = 232;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// GPS time (from start of GPS week)
    time_week_ms: u32,

    /// Latitude (WGS84)
    lat: i32,

    /// Longitude (WGS84)
    lon: i32,

    /// Altitude (MSL). Positive for up.
    alt: f32,

    /// GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX
    hdop: f32,

    /// GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX
    vdop: f32,

    /// GPS velocity in north direction in earth-fixed NED frame
    vn: f32,

    /// GPS velocity in east direction in earth-fixed NED frame
    ve: f32,

    /// GPS velocity in down direction in earth-fixed NED frame
    vd: f32,

    /// GPS speed accuracy
    speed_accuracy: f32,

    /// GPS horizontal accuracy
    horiz_accuracy: f32,

    /// GPS vertical accuracy
    vert_accuracy: f32,

    /// Bitmap indicating which GPS input flags fields to ignore.  All other fields must be provided.
    ignore_flags: enums.GPS_INPUT_IGNORE_FLAGS.Type,

    /// GPS week number
    time_week: u16,

    /// ID of the GPS for multiple GPS inputs
    gps_id: u8,

    /// 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
    fix_type: u8,

    /// Number of satellites visible.
    satellites_visible: u8,

    //Extension Field
    /// Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north
    yaw: u16,

};

/// The location and information of an ADSB vehicle
pub const ADSB_VEHICLE = struct {
    pub const MSG_ID = 246;
    /// The callsign, 8+null
    callsign: [9]u8,

    /// ICAO address
    ICAO_address: u32,

    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

    /// Altitude(ASL)
    altitude: i32,

    /// Course over ground
    heading: u16,

    /// The horizontal velocity
    hor_velocity: u16,

    /// The vertical velocity. Positive is up
    ver_velocity: i16,

    /// Bitmap to indicate various statuses including valid data fields
    flags: enums.ADSB_FLAGS.Type,

    /// Squawk code. Note that the code is in decimal: e.g. 7700 (general emergency) is encoded as binary 0b0001_1110_0001_0100, not(!) as 0b0000_111_111_000_000
    squawk: u16,

    /// ADSB altitude type.
    altitude_type: enums.ADSB_ALTITUDE_TYPE,

    /// ADSB emitter type.
    emitter_type: enums.ADSB_EMITTER_TYPE,

    /// Time since last communication in seconds
    tslc: u8,

};

/// Information about the status of a capture. Can be requested with a MAV_CMD_REQUEST_MESSAGE command.
pub const CAMERA_CAPTURE_STATUS = struct {
    pub const MSG_ID = 262;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Image capture interval
    image_interval: f32,

    /// Elapsed time since recording started (0: Not supported/available). A GCS should compute recording time and use non-zero values of this field to correct any discrepancy.
    recording_time_ms: u32,

    /// Available storage capacity.
    available_capacity: f32,

    /// Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)
    image_status: u8,

    /// Current status of video capturing (0: idle, 1: capture in progress)
    video_status: u8,

    //Extension Field
    /// Total number of images captured ('forever', or until reset using MAV_CMD_STORAGE_FORMAT).
    image_count: i32,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

/// AVSS PRS system status.
pub const AVSS_PRS_SYS_STATUS = struct {
    pub const MSG_ID = 60050;
    /// Timestamp (time since PRS boot).
    time_boot_ms: u32,

    /// PRS error statuses
    error_status: u32,

    /// Estimated battery run-time without a remote connection and PRS battery voltage
    battery_status: u32,

    /// PRS arm statuses
    arm_status: u8,

    /// PRS battery charge statuses
    charge_status: u8,

};

/// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
pub const HEARTBEAT = struct {
    pub const MSG_ID = 0;
    /// A bitfield for use for autopilot-specific flags
    custom_mode: u32,

    /// Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
    type: enums.MAV_TYPE,

    /// Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
    autopilot: enums.MAV_AUTOPILOT,

    /// System mode bitmap.
    base_mode: enums.MAV_MODE_FLAG.Type,

    /// System status flag.
    system_status: enums.MAV_STATE,

    /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
    mavlink_version: u8,

};

/// Message implementing parts of the V2 payload specs in V1 frames for transitional support.
pub const V2_EXTENSION = struct {
    pub const MSG_ID = 248;
    /// Variable length payload. The length must be encoded in the payload as part of the message_type protocol, e.g. by including the length as payload data, or by terminating the payload data with a non-zero marker. This is required in order to reconstruct zero-terminated payloads that are (or otherwise would be) trimmed by MAVLink 2 empty-byte truncation. The entire content of the payload block is opaque unless you understand the encoding message_type. The particular encoding used can be extension specific and might not always be documented as part of the MAVLink specification.
    payload: [249]u8,

    /// A code that identifies the software component that understands this message (analogous to USB device classes or mime type strings). If this code is less than 32768, it is considered a 'registered' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/definition_files/extension_message_ids.xml. Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
    message_type: u16,

    /// Network ID (0 for broadcast)
    target_network: u8,

    /// System ID (0 for broadcast)
    target_system: u8,

    /// Component ID (0 for broadcast)
    target_component: u8,

};

/// Barometer readings for 3rd barometer
pub const SCALED_PRESSURE3 = struct {
    pub const MSG_ID = 143;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Absolute pressure
    press_abs: f32,

    /// Differential pressure
    press_diff: f32,

    /// Absolute pressure temperature
    temperature: i16,

    //Extension Field
    /// Differential pressure temperature (0, if not available). Report values of 0 (or 1) as 1 cdegC.
    temperature_press_diff: i16,

};

/// Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when setting a parameter value and the new value is the same as the current value, you will immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive a PARAM_ACK_IN_PROGRESS in response.
pub const PARAM_EXT_SET = struct {
    pub const MSG_ID = 323;
    /// Parameter value
    param_value: [128]u8,

    /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Parameter type.
    param_type: enums.MAV_PARAM_EXT_TYPE,

};

/// Information about a storage medium. This message is sent in response to a request with MAV_CMD_REQUEST_MESSAGE and whenever the status of the storage changes (STORAGE_STATUS). Use MAV_CMD_REQUEST_MESSAGE.param2 to indicate the index/id of requested storage: 0 for all, 1 for first, 2 for second, etc.
pub const STORAGE_INFORMATION = struct {
    pub const MSG_ID = 261;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// Total capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
    total_capacity: f32,

    /// Used capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
    used_capacity: f32,

    /// Available storage capacity. If storage is not ready (STORAGE_STATUS_READY) value will be ignored.
    available_capacity: f32,

    /// Read speed.
    read_speed: f32,

    /// Write speed.
    write_speed: f32,

    /// Storage ID (1 for first, 2 for second, etc.)
    storage_id: u8,

    /// Number of storage devices
    storage_count: u8,

    /// Status of storage
    status: enums.STORAGE_STATUS,

    //Extension Field
    /// Type of storage
    type: enums.STORAGE_TYPE,

    //Extension Field
    /// Textual storage name to be used in UI (microSD 1, Internal Memory, etc.) This is a NULL terminated string. If it is exactly 32 characters long, add a terminating NULL. If this string is empty, the generic type is shown to the user.
    name: [32]u8,

    //Extension Field
    /// Flags indicating whether this instance is preferred storage for photos, videos, etc.
    ///         Note: Implementations should initially set the flags on the system-default storage id used for saving media (if possible/supported).
    ///         This setting can then be overridden using MAV_CMD_SET_STORAGE_USAGE.
    ///         If the media usage flags are not set, a GCS may assume storage ID 1 is the default storage for all media types.
    storage_usage: enums.STORAGE_USAGE_FLAG.Type,

};

/// Acknowledgment message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero).
pub const MISSION_ACK = struct {
    pub const MSG_ID = 47;
    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Mission result.
    type: enums.MAV_MISSION_RESULT,

    //Extension Field
    /// Mission type.
    mission_type: enums.MAV_MISSION_TYPE,

    //Extension Field
    /// Id of new on-vehicle mission, fence, or rally point plan (on upload to vehicle).
    ///         The id is calculated and returned by a vehicle when a new plan is uploaded by a GCS.
    ///         The only requirement on the id is that it must change when there is any change to the on-vehicle plan type (there is no requirement that the id be globally unique).
    ///         0 on download from the vehicle to the GCS (on download the ID is set in MISSION_COUNT).
    ///         0 if plan ids are not supported.
    ///         The current on-vehicle plan ids are streamed in `MISSION_CURRENT`, allowing a GCS to determine if any part of the plan has changed and needs to be re-uploaded.
    ///       
    opaque_id: u32,

};

/// Update the data in the OPEN_DRONE_ID_SYSTEM message with new location information. This can be sent to update the location information for the operator when no other information in the SYSTEM message has changed. This message allows for efficient operation on radio links which have limited uplink bandwidth while meeting requirements for update frequency of the operator location.
pub const OPEN_DRONE_ID_SYSTEM_UPDATE = struct {
    pub const MSG_ID = 12919;
    /// Latitude of the operator. If unknown: 0 (both Lat/Lon).
    operator_latitude: i32,

    /// Longitude of the operator. If unknown: 0 (both Lat/Lon).
    operator_longitude: i32,

    /// Geodetic altitude of the operator relative to WGS84. If unknown: -1000 m.
    operator_altitude_geo: f32,

    /// 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
    timestamp: u32,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

};

/// Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification.
pub const HIL_RC_INPUTS_RAW = struct {
    pub const MSG_ID = 92;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// RC channel 1 value
    chan1_raw: u16,

    /// RC channel 2 value
    chan2_raw: u16,

    /// RC channel 3 value
    chan3_raw: u16,

    /// RC channel 4 value
    chan4_raw: u16,

    /// RC channel 5 value
    chan5_raw: u16,

    /// RC channel 6 value
    chan6_raw: u16,

    /// RC channel 7 value
    chan7_raw: u16,

    /// RC channel 8 value
    chan8_raw: u16,

    /// RC channel 9 value
    chan9_raw: u16,

    /// RC channel 10 value
    chan10_raw: u16,

    /// RC channel 11 value
    chan11_raw: u16,

    /// RC channel 12 value
    chan12_raw: u16,

    /// Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
    rssi: u8,

};

/// Low level message containing autopilot state relevant for a gimbal device. This message is to be sent from the autopilot to the gimbal device component. The data of this message are for the gimbal device's estimator corrections, in particular horizon compensation, as well as indicates autopilot control intentions, e.g. feed forward angular control in the z-axis.
pub const AUTOPILOT_STATE_FOR_GIMBAL_DEVICE = struct {
    pub const MSG_ID = 286;
    /// Timestamp (time since system boot).
    time_boot_us: u64,

    /// Quaternion components of autopilot attitude: w, x, y, z (1 0 0 0 is the null-rotation, Hamilton convention).
    q: [4]f32,

    /// Estimated delay of the attitude data. 0 if unknown.
    q_estimated_delay_us: u32,

    /// X Speed in NED (North, East, Down). NAN if unknown.
    vx: f32,

    /// Y Speed in NED (North, East, Down). NAN if unknown.
    vy: f32,

    /// Z Speed in NED (North, East, Down). NAN if unknown.
    vz: f32,

    /// Estimated delay of the speed data. 0 if unknown.
    v_estimated_delay_us: u32,

    /// Feed forward Z component of angular velocity (positive: yawing to the right). NaN to be ignored. This is to indicate if the autopilot is actively yawing.
    feed_forward_angular_velocity_z: f32,

    /// Bitmap indicating which estimator outputs are valid.
    estimator_status: enums.ESTIMATOR_STATUS_FLAGS.Type,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown.
    landed_state: enums.MAV_LANDED_STATE,

    //Extension Field
    /// Z component of angular velocity in NED (North, East, Down). NaN if unknown.
    angular_velocity_z: f32,

};

/// Request to control this MAV
pub const CHANGE_OPERATOR_CONTROL = struct {
    pub const MSG_ID = 5;
    /// Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and "!?,.-"
    passkey: [25]u8,

    /// System the GCS requests control for
    target_system: u8,

    /// 0: request control of this MAV, 1: Release control of this MAV
    control_request: u8,

    /// 0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch.
    version: u8,

};

/// Streamed from drone to report progress of terrain map download (initiated by TERRAIN_REQUEST), or sent as a response to a TERRAIN_CHECK request. See terrain protocol docs: https://mavlink.io/en/services/terrain.html
pub const TERRAIN_REPORT = struct {
    pub const MSG_ID = 136;
    /// Latitude
    lat: i32,

    /// Longitude
    lon: i32,

    /// Terrain height MSL
    terrain_height: f32,

    /// Current vehicle height above lat/lon terrain height
    current_height: f32,

    /// grid spacing (zero if terrain at this location unavailable)
    spacing: u16,

    /// Number of 4x4 terrain blocks waiting to be received or read from disk
    pending: u16,

    /// Number of 4x4 terrain blocks in memory
    loaded: u16,

};

/// The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%.  A value of UINT16_MAX implies the channel is unused. Individual receivers/transmitters might violate this specification.
pub const RC_CHANNELS = struct {
    pub const MSG_ID = 65;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// RC channel 1 value.
    chan1_raw: u16,

    /// RC channel 2 value.
    chan2_raw: u16,

    /// RC channel 3 value.
    chan3_raw: u16,

    /// RC channel 4 value.
    chan4_raw: u16,

    /// RC channel 5 value.
    chan5_raw: u16,

    /// RC channel 6 value.
    chan6_raw: u16,

    /// RC channel 7 value.
    chan7_raw: u16,

    /// RC channel 8 value.
    chan8_raw: u16,

    /// RC channel 9 value.
    chan9_raw: u16,

    /// RC channel 10 value.
    chan10_raw: u16,

    /// RC channel 11 value.
    chan11_raw: u16,

    /// RC channel 12 value.
    chan12_raw: u16,

    /// RC channel 13 value.
    chan13_raw: u16,

    /// RC channel 14 value.
    chan14_raw: u16,

    /// RC channel 15 value.
    chan15_raw: u16,

    /// RC channel 16 value.
    chan16_raw: u16,

    /// RC channel 17 value.
    chan17_raw: u16,

    /// RC channel 18 value.
    chan18_raw: u16,

    /// Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available.
    chancount: u8,

    /// Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.
    rssi: u8,

};

/// Odometry message to communicate odometry information with an external interface. Fits ROS REP 147 standard for aerial vehicles (http://www.ros.org/reps/rep-0147.html).
pub const ODOMETRY = struct {
    pub const MSG_ID = 331;
    /// Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
    pose_covariance: [21]f32,

    /// Row-major representation of a 6x6 velocity cross-covariance matrix upper right triangle (states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.). If unknown, assign NaN value to first element in the array.
    velocity_covariance: [21]f32,

    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// X Position
    x: f32,

    /// Y Position
    y: f32,

    /// Z Position
    z: f32,

    /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)
    q: [4]f32,

    /// X linear speed
    vx: f32,

    /// Y linear speed
    vy: f32,

    /// Z linear speed
    vz: f32,

    /// Roll angular speed
    rollspeed: f32,

    /// Pitch angular speed
    pitchspeed: f32,

    /// Yaw angular speed
    yawspeed: f32,

    /// Coordinate frame of reference for the pose data.
    frame_id: enums.MAV_FRAME,

    /// Coordinate frame of reference for the velocity in free space (twist) data.
    child_frame_id: enums.MAV_FRAME,

    //Extension Field
    /// Estimate reset counter. This should be incremented when the estimate resets in any of the dimensions (position, velocity, attitude, angular speed). This is designed to be used when e.g an external SLAM system detects a loop-closure and the estimate jumps.
    reset_counter: u8,

    //Extension Field
    /// Type of estimator that is providing the odometry.
    estimator_type: enums.MAV_ESTIMATOR_TYPE,

    //Extension Field
    /// Optional odometry quality metric as a percentage. -1 = odometry has failed, 0 = unknown/unset quality, 1 = worst quality, 100 = best quality
    quality: i8,

};

/// Configure WiFi AP SSID, password, and mode. This message is re-emitted as an acknowledgement by the AP. The message may also be explicitly requested using MAV_CMD_REQUEST_MESSAGE
pub const WIFI_CONFIG_AP = struct {
    pub const MSG_ID = 299;
    /// Password. Blank for an open AP. MD5 hash when message is sent back as a response.
    password: [64]u8,

    /// Name of Wi-Fi network (SSID). Blank to leave it unchanged when setting. Current SSID when sent back as a response.
    ssid: [32]u8,

    //Extension Field
    /// WiFi Mode.
    mode: enums.WIFI_CONFIG_AP_MODE,

    //Extension Field
    /// Message acceptance response (sent back to GS).
    response: enums.WIFI_CONFIG_AP_RESPONSE,

};

/// A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections. The ping microservice is documented at https://mavlink.io/en/services/ping.html
pub const PING = struct {
    pub const MSG_ID = 4;
    /// Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
    time_usec: u64,

    /// PING sequence
    seq: u32,

    /// 0: request ping from all receiving systems. If greater than 0: message is a ping response and number is the system id of the requesting system
    target_system: u8,

    /// 0: request ping from all receiving components. If greater than 0: message is a ping response and number is the component id of the requesting component.
    target_component: u8,

};

/// High level message to control a gimbal's attitude. This message is to be sent to the gimbal manager (e.g. from a ground station). Angles and rates can be set to NaN according to use case.
pub const GIMBAL_MANAGER_SET_ATTITUDE = struct {
    pub const MSG_ID = 282;
    /// High level gimbal manager flags to use.
    flags: enums.GIMBAL_MANAGER_FLAGS.Type,

    /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation, the frame is depends on whether the flag GIMBAL_MANAGER_FLAGS_YAW_LOCK is set)
    q: [4]f32,

    /// X component of angular velocity, positive is rolling to the right, NaN to be ignored.
    angular_velocity_x: f32,

    /// Y component of angular velocity, positive is pitching up, NaN to be ignored.
    angular_velocity_y: f32,

    /// Z component of angular velocity, positive is yawing to the right, NaN to be ignored.
    angular_velocity_z: f32,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).
    gimbal_device_id: u8,

};

/// Telemetry of power generation system. Alternator or mechanical generator.
pub const GENERATOR_STATUS = struct {
    pub const MSG_ID = 373;
    /// Status flags.
    status: enums.MAV_GENERATOR_STATUS_FLAG.Type,

    /// Current into/out of battery. Positive for out. Negative for in. NaN: field not provided.
    battery_current: f32,

    /// Current going to the UAV. If battery current not available this is the DC current from the generator. Positive for out. Negative for in. NaN: field not provided
    load_current: f32,

    /// The power being generated. NaN: field not provided
    power_generated: f32,

    /// Voltage of the bus seen at the generator, or battery bus if battery bus is controlled by generator and at a different voltage to main bus.
    bus_voltage: f32,

    /// The target battery current. Positive for out. Negative for in. NaN: field not provided
    bat_current_setpoint: f32,

    /// Seconds this generator has run since it was rebooted. UINT32_MAX: field not provided.
    runtime: u32,

    /// Seconds until this generator requires maintenance.  A negative value indicates maintenance is past-due. INT32_MAX: field not provided.
    time_until_maintenance: i32,

    /// Speed of electrical generator or alternator. UINT16_MAX: field not provided.
    generator_speed: u16,

    /// The temperature of the rectifier or power converter. INT16_MAX: field not provided.
    rectifier_temperature: i16,

    /// The temperature of the mechanical motor, fuel cell core or generator. INT16_MAX: field not provided.
    generator_temperature: i16,

};

/// Report current used cellular network status
pub const CELLULAR_STATUS = struct {
    pub const MSG_ID = 334;
    /// Mobile country code. If unknown, set to UINT16_MAX
    mcc: u16,

    /// Mobile network code. If unknown, set to UINT16_MAX
    mnc: u16,

    /// Location area code. If unknown, set to 0
    lac: u16,

    /// Cellular modem status
    status: enums.CELLULAR_STATUS_FLAG,

    /// Failure reason when status in in CELLULAR_STATUS_FLAG_FAILED
    failure_reason: enums.CELLULAR_NETWORK_FAILED_REASON,

    /// Cellular network radio type: gsm, cdma, lte...
    type: enums.CELLULAR_NETWORK_RADIO_TYPE,

    /// Signal quality in percent. If unknown, set to UINT8_MAX
    quality: u8,

};

/// Data for filling the OpenDroneID Self ID message. The Self ID Message is an opportunity for the operator to (optionally) declare their identity and purpose of the flight. This message can provide additional information that could reduce the threat profile of a UA (Unmanned Aircraft) flying in a particular area or manner. This message can also be used to provide optional additional clarification in an emergency/remote ID system failure situation.
pub const OPEN_DRONE_ID_SELF_ID = struct {
    pub const MSG_ID = 12903;
    /// Text description or numeric value expressed as ASCII characters. Shall be filled with nulls in the unused portion of the field.
    description: [23]u8,

    /// Only used for drone ID data received from other UAs. See detailed description at https://mavlink.io/en/services/opendroneid.html. 
    id_or_mac: [20]u8,

    /// System ID (0 for broadcast).
    target_system: u8,

    /// Component ID (0 for broadcast).
    target_component: u8,

    /// Indicates the type of the description field.
    description_type: enums.MAV_ODID_DESC_TYPE,

};

/// Response from a PARAM_EXT_SET message.
pub const PARAM_EXT_ACK = struct {
    pub const MSG_ID = 324;
    /// Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)
    param_value: [128]u8,

    /// Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
    param_id: [16]u8,

    /// Parameter type.
    param_type: enums.MAV_PARAM_EXT_TYPE,

    /// Result code.
    param_result: enums.PARAM_ACK,

};

/// Low level message to control a gimbal device's attitude.
///   This message is to be sent from the gimbal manager to the gimbal device component.
///   The quaternion and angular velocities can be set to NaN according to use case.
///   For the angles encoded in the quaternion and the angular velocities holds:
///   If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME is set, then they are relative to the vehicle heading (vehicle frame).
///   If the flag GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME is set, then they are relative to absolute North (earth frame).
///   If neither of these flags are set, then (for backwards compatibility) it holds:
///   If the flag GIMBAL_DEVICE_FLAGS_YAW_LOCK is set, then they are relative to absolute North (earth frame),
///   else they are relative to the vehicle heading (vehicle frame).
///   Setting both GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME and GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME is not allowed.
///   These rules are to ensure backwards compatibility.
///   New implementations should always set either GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME or GIMBAL_DEVICE_FLAGS_YAW_IN_EARTH_FRAME.
pub const GIMBAL_DEVICE_SET_ATTITUDE = struct {
    pub const MSG_ID = 284;
    /// Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation). The frame is described in the message description. Set fields to NaN to be ignored.
    q: [4]f32,

    /// X component of angular velocity (positive: rolling to the right). The frame is described in the message description. NaN to be ignored.
    angular_velocity_x: f32,

    /// Y component of angular velocity (positive: pitching up). The frame is described in the message description. NaN to be ignored.
    angular_velocity_y: f32,

    /// Z component of angular velocity (positive: yawing to the right). The frame is described in the message description. NaN to be ignored.
    angular_velocity_z: f32,

    /// Low level gimbal flags.
    flags: enums.GIMBAL_DEVICE_FLAGS.Type,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

};

/// Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system).
pub const SET_POSITION_TARGET_LOCAL_NED = struct {
    pub const MSG_ID = 84;
    /// Timestamp (time since system boot).
    time_boot_ms: u32,

    /// X Position in NED frame
    x: f32,

    /// Y Position in NED frame
    y: f32,

    /// Z Position in NED frame (note, altitude is negative in NED)
    z: f32,

    /// X velocity in NED frame
    vx: f32,

    /// Y velocity in NED frame
    vy: f32,

    /// Z velocity in NED frame
    vz: f32,

    /// X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afx: f32,

    /// Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afy: f32,

    /// Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
    afz: f32,

    /// yaw setpoint
    yaw: f32,

    /// yaw rate setpoint
    yaw_rate: f32,

    /// Bitmap to indicate which dimensions should be ignored by the vehicle.
    type_mask: enums.POSITION_TARGET_TYPEMASK.Type,

    /// System ID
    target_system: u8,

    /// Component ID
    target_component: u8,

    /// Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
    coordinate_frame: enums.MAV_FRAME,

};

/// Status of geo-fencing. Sent in extended status stream when fencing enabled.
pub const FENCE_STATUS = struct {
    pub const MSG_ID = 162;
    /// Time (since boot) of last breach.
    breach_time: u32,

    /// Number of fence breaches.
    breach_count: u16,

    /// Breach status (0 if currently inside fence, 1 if outside).
    breach_status: u8,

    /// Last breach type.
    breach_type: enums.FENCE_BREACH,

    //Extension Field
    /// Active action to prevent fence breach
    breach_mitigation: enums.FENCE_MITIGATE,

};

/// Information about video stream. It may be requested using MAV_CMD_REQUEST_MESSAGE, where param2 indicates the video stream id: 0 for all streams, 1 for first, 2 for second, etc.
pub const VIDEO_STREAM_INFORMATION = struct {
    pub const MSG_ID = 269;
    /// Video stream URI (TCP or RTSP URI ground station should connect to) or port number (UDP port ground station should listen to).
    uri: [160]u8,

    /// Stream name.
    name: [32]u8,

    /// Frame rate.
    framerate: f32,

    /// Bit rate.
    bitrate: u32,

    /// Bitmap of stream status flags.
    flags: enums.VIDEO_STREAM_STATUS_FLAGS.Type,

    /// Horizontal resolution.
    resolution_h: u16,

    /// Vertical resolution.
    resolution_v: u16,

    /// Video image rotation clockwise.
    rotation: u16,

    /// Horizontal Field of view.
    hfov: u16,

    /// Video Stream ID (1 for first, 2 for second, etc.)
    stream_id: u8,

    /// Number of streams available.
    count: u8,

    /// Type of stream.
    type: enums.VIDEO_STREAM_TYPE,

    //Extension Field
    /// Encoding of stream.
    encoding: enums.VIDEO_STREAM_ENCODING,

    //Extension Field
    /// Camera id of a non-MAVLink camera attached to an autopilot (1-6).  0 if the component is a MAVLink camera (with its own component id).
    camera_device_id: u8,

};

