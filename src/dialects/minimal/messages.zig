// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

/// Version and capability of protocol version. This message can be requested with MAV_CMD_REQUEST_MESSAGE and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to a request for PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly.
pub const PROTOCOL_VERSION = struct {
    pub const MSG_ID = 300;
    /// Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc.
    version: u16,

    /// Minimum MAVLink version supported
    min_version: u16,

    /// Maximum MAVLink version supported (set to the same value as version by default)
    max_version: u16,

    /// The first 8 bytes (not characters printed in hex!) of the git hash.
    spec_version_hash: [8]u8,

    /// The first 8 bytes (not characters printed in hex!) of the git hash.
    library_version_hash: [8]u8,

};

/// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
pub const HEARTBEAT = struct {
    pub const MSG_ID = 0;
    /// Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
    type: enums.MAV_TYPE,

    /// Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
    autopilot: enums.MAV_AUTOPILOT,

    /// System mode bitmap.
    base_mode: u8,

    /// A bitfield for use for autopilot-specific flags
    custom_mode: u32,

    /// System status flag.
    system_status: enums.MAV_STATE,

    /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
    mavlink_version: u8,

};

