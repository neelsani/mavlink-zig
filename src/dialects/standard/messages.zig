// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

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

