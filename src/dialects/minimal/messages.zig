// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

/// The heartbeat message shows that a system or component is present and responding. The type and autopilot fields (along with the message component id), allow the receiving system to treat further messages from this system appropriately (e.g. by laying out the user interface based on the autopilot). This microservice is documented at https://mavlink.io/en/services/heartbeat.html
pub const HEARTBEAT = struct {
    pub const MSG_ID = 0;
    /// Vehicle or component type. For a flight controller component the vehicle type (quadrotor, helicopter, etc.). For other components the component type (e.g. camera, gimbal, etc.). This should be used in preference to component id for identifying the component type.
    type: enums.MAV_TYPE,

    /// Autopilot type / class. Use MAV_AUTOPILOT_INVALID for components that are not flight controllers.
    autopilot: enums.MAV_AUTOPILOT,

    /// System mode bitmap.
    base_mode: enums.MAV_MODE_FLAG,

    /// A bitfield for use for autopilot-specific flags
    custom_mode: u32,

    /// System status flag.
    system_status: enums.MAV_STATE,

    /// MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
    mavlink_version: u8,

};

