// Auto-generated MAVLink enums
// DO NOT EDIT MANUALLY

/// These flags encode the MAV mode.
pub const MAV_MODE_FLAG = packed struct {
    pub const is_bitmask = true;
    bits: u8,

    pub const Type = u8;

    pub inline fn toInt(self: @This()) Type {
        return self.bits;
    }

    pub inline fn fromInt(bits: Type) @This() {
        return @This(){ .bits = bits };
    }

    pub inline fn isSet(self: @This(), comptime flag: @This()) bool {
        return (self.bits & flag.bits) != 0;
    }

    pub inline fn set(self: *@This(), comptime flag: @This()) void {
        self.bits |= flag.bits;
    }

    pub inline fn unset(self: *@This(), comptime flag: @This()) void {
        self.bits &= ~flag.bits;
    }

    pub inline fn toggle(self: *@This(), comptime flag: @This()) void {
        self.bits ^= flag.bits;
    }
    /// 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly. Additional note: this flag is to be ignore when sent in the command MAV_CMD_DO_SET_MODE and MAV_CMD_COMPONENT_ARM_DISARM shall be used instead. The flag can still be used to report the armed state.
    pub const MAV_MODE_FLAG_SAFETY_ARMED: @This() = .{ .bits = 128 };
    /// 0b01000000 remote control input is enabled.
    pub const MAV_MODE_FLAG_MANUAL_INPUT_ENABLED: @This() = .{ .bits = 64 };
    /// 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
    pub const MAV_MODE_FLAG_HIL_ENABLED: @This() = .{ .bits = 32 };
    /// 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
    pub const MAV_MODE_FLAG_STABILIZE_ENABLED: @This() = .{ .bits = 16 };
    /// 0b00001000 guided mode enabled, system flies waypoints / mission items.
    pub const MAV_MODE_FLAG_GUIDED_ENABLED: @This() = .{ .bits = 8 };
    /// 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
    pub const MAV_MODE_FLAG_AUTO_ENABLED: @This() = .{ .bits = 4 };
    /// 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
    pub const MAV_MODE_FLAG_TEST_ENABLED: @This() = .{ .bits = 2 };
    /// 0b00000001 Reserved for future use.
    pub const MAV_MODE_FLAG_CUSTOM_MODE_ENABLED: @This() = .{ .bits = 1 };
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

pub const MAV_CMD = enum(u32) {
    /// Set Loweheiser desired states
    MAV_CMD_LOWEHEISER_SET_STATE = 10151,
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
    /// VTOL hybrid of helicopter and autogyro. It has a main rotor for lift and separate propellers for forward flight. The rotor must be powered for hover but can autorotate in cruise flight. See: https://en.wikipedia.org/wiki/Gyrodyne
    MAV_TYPE_VTOL_GYRODYNE = 47,
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

/// These values encode the bit positions of the decode position. These values can be used to read the value of a flag bit by combining the base_mode variable with AND with the flag position value. The result will be either 0 or 1, depending on if the flag is set or not.
pub const MAV_MODE_FLAG_DECODE_POSITION = packed struct {
    pub const is_bitmask = true;
    bits: u32,

    pub const Type = u32;

    pub inline fn toInt(self: @This()) Type {
        return self.bits;
    }

    pub inline fn fromInt(bits: Type) @This() {
        return @This(){ .bits = bits };
    }

    pub inline fn isSet(self: @This(), comptime flag: @This()) bool {
        return (self.bits & flag.bits) != 0;
    }

    pub inline fn set(self: *@This(), comptime flag: @This()) void {
        self.bits |= flag.bits;
    }

    pub inline fn unset(self: *@This(), comptime flag: @This()) void {
        self.bits &= ~flag.bits;
    }

    pub inline fn toggle(self: *@This(), comptime flag: @This()) void {
        self.bits ^= flag.bits;
    }
    /// First bit:  10000000
    pub const MAV_MODE_FLAG_DECODE_POSITION_SAFETY: @This() = .{ .bits = 128 };
    /// Second bit: 01000000
    pub const MAV_MODE_FLAG_DECODE_POSITION_MANUAL: @This() = .{ .bits = 64 };
    /// Third bit:  00100000
    pub const MAV_MODE_FLAG_DECODE_POSITION_HIL: @This() = .{ .bits = 32 };
    /// Fourth bit: 00010000
    pub const MAV_MODE_FLAG_DECODE_POSITION_STABILIZE: @This() = .{ .bits = 16 };
    /// Fifth bit:  00001000
    pub const MAV_MODE_FLAG_DECODE_POSITION_GUIDED: @This() = .{ .bits = 8 };
    /// Sixth bit:   00000100
    pub const MAV_MODE_FLAG_DECODE_POSITION_AUTO: @This() = .{ .bits = 4 };
    /// Seventh bit: 00000010
    pub const MAV_MODE_FLAG_DECODE_POSITION_TEST: @This() = .{ .bits = 2 };
    /// Eighth bit: 00000001
    pub const MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE: @This() = .{ .bits = 1 };
};

/// Legacy component ID values for particular types of hardware/software that might make up a MAVLink system (autopilot, cameras, servos, avoidance systems etc.).
///       
///         Components are not required or expected to use IDs with names that correspond to their type or function, but may choose to do so.
///         Using an ID that matches the type may slightly reduce the chances of component id clashes, as, for historical reasons, it is less likely to be used by some other type of component.
///         System integration will still need to ensure that all components have unique IDs.
/// 
///         Component IDs are used for addressing messages to a particular component within a system.
///         A component can use any unique ID between 1 and 255 (MAV_COMP_ID_ALL value is the broadcast address, used to send to all components).
///         
///         Historically component ID were also used for identifying the type of component.
///         New code must not use component IDs to infer the component type, but instead check the MAV_TYPE in the HEARTBEAT message!
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

