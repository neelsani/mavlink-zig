// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

/// Authorization package
pub const AIRLINK_AUTH = struct {
    pub const MSG_ID = 52000;
    /// Login
    login: [50]u8,

    /// Password
    password: [50]u8,

};

/// Response to the authorization request
pub const AIRLINK_AUTH_RESPONSE = struct {
    pub const MSG_ID = 52001;
    /// Response type
    resp_type: enums.AIRLINK_AUTH_RESPONSE_TYPE,

};

