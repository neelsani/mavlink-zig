// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

/// Kinematic multi bands (track) output from Daidalus
pub const ICAROUS_KINEMATIC_BANDS = struct {
    pub const MSG_ID = 42001;
    /// min angle (degrees)
    min1: f32,

    /// max angle (degrees)
    max1: f32,

    /// min angle (degrees)
    min2: f32,

    /// max angle (degrees)
    max2: f32,

    /// min angle (degrees)
    min3: f32,

    /// max angle (degrees)
    max3: f32,

    /// min angle (degrees)
    min4: f32,

    /// max angle (degrees)
    max4: f32,

    /// min angle (degrees)
    min5: f32,

    /// max angle (degrees)
    max5: f32,

    /// Number of track bands
    numBands: i8,

    /// See the TRACK_BAND_TYPES enum.
    type1: enums.ICAROUS_TRACK_BAND_TYPES,

    /// See the TRACK_BAND_TYPES enum.
    type2: enums.ICAROUS_TRACK_BAND_TYPES,

    /// See the TRACK_BAND_TYPES enum.
    type3: enums.ICAROUS_TRACK_BAND_TYPES,

    /// See the TRACK_BAND_TYPES enum.
    type4: enums.ICAROUS_TRACK_BAND_TYPES,

    /// See the TRACK_BAND_TYPES enum.
    type5: enums.ICAROUS_TRACK_BAND_TYPES,

};

/// ICAROUS heartbeat
pub const ICAROUS_HEARTBEAT = struct {
    pub const MSG_ID = 42000;
    /// See the FMS_STATE enum.
    status: enums.ICAROUS_FMS_STATE,

};

