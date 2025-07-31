// Auto-generated MAVLink messages
// DO NOT EDIT MANUALLY

const enums = @import("enums.zig");

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

/// Composite EFI and Governor data from Loweheiser equipment.  This message is created by the EFI unit based on its own data and data received from a governor attached to that EFI unit.
pub const LOWEHEISER_GOV_EFI = struct {
    pub const MSG_ID = 10151;
    /// Generator Battery voltage.
    volt_batt: f32,

    /// Generator Battery current.
    curr_batt: f32,

    /// Current being produced by generator.
    curr_gen: f32,

    /// Load current being consumed by the UAV (sum of curr_gen and curr_batt)
    curr_rot: f32,

    /// Generator fuel remaining in litres.
    fuel_level: f32,

    /// Throttle Output.
    throttle: f32,

    /// Seconds this generator has run since it was rebooted.
    runtime: u32,

    /// Seconds until this generator requires maintenance.  A negative value indicates maintenance is past due.
    until_maintenance: i32,

    /// The Temperature of the rectifier.
    rectifier_temp: f32,

    /// The temperature of the mechanical motor, fuel cell core or generator.
    generator_temp: f32,

    ///  EFI Supply Voltage.
    efi_batt: f32,

    /// Motor RPM.
    efi_rpm: f32,

    /// Injector pulse-width in milliseconds.
    efi_pw: f32,

    /// Fuel flow rate in litres/hour.
    efi_fuel_flow: f32,

    /// Fuel consumed.
    efi_fuel_consumed: f32,

    /// Atmospheric pressure.
    efi_baro: f32,

    /// Manifold Air Temperature.
    efi_mat: f32,

    /// Cylinder Head Temperature.
    efi_clt: f32,

    /// Throttle Position.
    efi_tps: f32,

    /// Exhaust gas temperature.
    efi_exhaust_gas_temperature: f32,

    /// Generator status.
    generator_status: u16,

    /// EFI status.
    efi_status: u16,

    /// EFI index.
    efi_index: u8,

};

