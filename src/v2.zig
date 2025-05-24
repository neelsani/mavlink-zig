const std = @import("std");
const print = std.debug.print;
const serde = @import("serde.zig");
const MavlinkCrc = @import("crc.zig");

// MAVLink v2 protocol constants
pub const MAVLINK_STX_V2: u8 = 0xFD; // MAVLink v2 start byte marker
pub const MAVLINK_HEADER_LEN: u8 = 10; // Length of MAVLink v2 header (STX + 9 bytes)
pub const MAVLINK_CHECKSUM_LEN: u8 = 2; // Length of CRC checksum
pub const MAVLINK_SIGNATURE_LEN: u8 = 13; // Length of signature block
pub const MAVLINK_MAX_PAYLOAD_LEN: u8 = 255; // Maximum payload length
pub const MAVLINK_MAX_PACKET_SIZE: usize = 280; // Maximum possible packet size (header + max payload + crc + signature)

/// Parser state machine states
const ParseState = enum {
    WaitingForStart, // Waiting for STX byte
    GotHeader, // Received STX, collecting header
    ReadingPayload, // Reading payload bytes
    ReadingCrcLow, // Reading CRC low byte
    ReadingCrcHigh, // Reading CRC high byte
    ReadingSignature, // Reading signature (if present)
};

/// MAVLink v2 packet structure
/// Represents a complete MAVLink v2 message with all fields
pub const MavlinkPacket = struct {
    stx: u8, // Start byte (0xFD for v2)
    len: u8, // Payload length
    incompat_flags: u8, // Incompatible flags
    compat_flags: u8, // Compatible flags
    seq: u8, // Sequence number
    sysid: u8, // System ID
    compid: u8, // Component ID
    msgid: u24, // Message ID (stored in u32 since Zig doesn't have u24)
    payload: [MAVLINK_MAX_PAYLOAD_LEN]u8, // Payload data
    checksum: u16, // Message checksum
    signature: [MAVLINK_SIGNATURE_LEN]u8, // Optional signature
    has_signature: bool, // Whether signature is present

    /// Custom formatter for pretty-printing MavlinkPacket
    /// Implements the std.fmt.format interface
    pub fn format(
        self: MavlinkPacket,
        comptime fmt: []const u8,
        options: std.fmt.FormatOptions,
        writer: anytype,
    ) !void {
        _ = fmt; // Unused format string
        _ = options; // Unused format options

        // Print basic packet information
        try writer.print(
            \\MavlinkPacket:
            \\  STX: 0x{X}
            \\  Length: {}
            \\  Incompat Flags: 0x{X}
            \\  Compat Flags: 0x{X}
            \\  Seq: {}
            \\  SysID: {}
            \\  CompID: {}
            \\  MsgID: {}
            \\  Payload Length: {}
            \\  Checksum: 0x{X}
            \\  Has Signature: {}
            \\
        , .{
            self.stx,
            self.len,
            self.incompat_flags,
            self.compat_flags,
            self.seq,
            self.sysid,
            self.compid,
            self.msgid,
            self.len,
            self.checksum,
            self.has_signature,
        });

        // Print payload if present
        if (self.len > 0) {
            try writer.writeAll("  Payload: ");
            for (self.payload[0..self.len]) |byte| {
                try writer.print("{X:0>2} ", .{byte});
            }
            try writer.writeAll("\n");
        }

        // Print signature if present
        if (self.has_signature) {
            try writer.writeAll("  Signature: ");
            for (self.signature) |byte| {
                try writer.print("{X:0>2} ", .{byte});
            }
            try writer.writeAll("\n");
        }
    }
};

/// Temporary sequence number generator
/// TODO: Implement proper sequence number management
fn nextSeq() u8 {
    return 0;
}

/// Serializes a MAVLink message into a byte buffer
///
/// # Parameters
/// - `buf`: Destination buffer (must be large enough for the complete message)
/// - `msg`: The message to serialize (must have MSG_ID declared)
///
/// # Returns
/// Number of bytes written or error if:
/// - Message type has no MSG_ID
/// - Invalid message ID
/// - Buffer too small
pub fn writeMessageToSlice(buf: []u8, msg: anytype) !usize {
    const T = @TypeOf(msg);

    // Compile-time check for required MSG_ID declaration
    comptime {
        if (!@hasDecl(T, "MSG_ID")) {
            @compileError("writeMessage: type " ++ @typeName(T) ++ " has no MSG_ID");
        }
    }
    const MSG_ID: u24 = @field(T, "MSG_ID");

    // MAVLink v2 header fields with default values
    const STX: u8 = MAVLINK_STX_V2; // Start byte
    const INCOMP: u8 = 0; // No incompatible flags
    const COMPAT: u8 = 0; // No compatible flags
    const SEQ: u8 = nextSeq(); // Sequence number
    const SYS_ID: u8 = 255; // Default system ID
    const COMP_ID: u8 = 255; // Default component ID

    // Get message info from CRC table
    const info = MavlinkCrc.get_msg_info(MSG_ID) orelse return error.InvalidMsgId;
    const PAYLOAD_LEN: u8 = info.max_payload_len;
    const CRC_EXTRA: u8 = info.crc_extra;

    // Calculate total message size
    const total_size = 1 + 9 + PAYLOAD_LEN + 2; // STX + header + payload + CRC
    if (buf.len < total_size) return error.BufferTooSmall;

    // Write STX (start byte)
    buf[0] = STX;
    var pos: usize = 1;

    // Write header fields
    buf[pos] = PAYLOAD_LEN; // Payload length
    pos += 1;
    buf[pos] = INCOMP; // Incompatible flags
    pos += 1;
    buf[pos] = COMPAT; // Compatible flags
    pos += 1;
    buf[pos] = SEQ; // Sequence number
    pos += 1;
    buf[pos] = SYS_ID; // System ID
    pos += 1;
    buf[pos] = COMP_ID; // Component ID
    pos += 1;

    // Write 3-byte little-endian message ID
    buf[pos] = @intCast(MSG_ID & 0xFF);
    pos += 1;
    buf[pos] = @intCast((MSG_ID >> 8) & 0xFF);
    pos += 1;
    buf[pos] = @intCast((MSG_ID >> 16) & 0xFF);
    pos += 1;

    // Serialize payload directly into buffer
    const payload_slice = buf[pos .. pos + PAYLOAD_LEN];
    _ = try serde.serialize(msg, payload_slice);
    pos += PAYLOAD_LEN;

    // Calculate CRC over header (except STX) + payload, then append CRC_EXTRA
    var crc = MavlinkCrc.crc_init();
    crc = MavlinkCrc.crc_accumulate_buf(crc, buf[1..pos]); // Skip STX
    crc = MavlinkCrc.crc_accumulate(CRC_EXTRA, crc);

    // Write CRC (little-endian)
    buf[pos] = @intCast(crc & 0xFF); // CRC low byte
    pos += 1;
    buf[pos] = @intCast((crc >> 8) & 0xFF); // CRC high byte
    pos += 1;

    return pos; // Return total bytes written
}

// Parser instance state
state: ParseState, // Current parsing state
packet: MavlinkPacket, // Packet being constructed
payload_idx: u8, // Current payload byte index
signature_idx: u8, // Current signature byte index
crc_calc: MavlinkCrc, // CRC calculator
header_bytes: [MAVLINK_HEADER_LEN]u8, // Buffer for header bytes
header_idx: u8, // Current header byte index

const Self = @This(); // Type alias for the parser type

/// Creates a new MAVLink parser in initial state
pub fn init() Self {
    return Self{
        .state = ParseState.WaitingForStart,
        .packet = std.mem.zeroes(MavlinkPacket),
        .payload_idx = 0,
        .signature_idx = 0,
        .crc_calc = MavlinkCrc.init(),
        .header_bytes = [_]u8{0} ** MAVLINK_HEADER_LEN,
        .header_idx = 0,
    };
}

/// Resets the parser to initial state
fn reset(self: *Self) void {
    self.state = ParseState.WaitingForStart;
    self.packet = std.mem.zeroes(MavlinkPacket);
    self.payload_idx = 0;
    self.signature_idx = 0;
    self.crc_calc = MavlinkCrc.init();
    self.header_idx = 0;
}

/// Parses a single byte, returning a complete packet if one is finished
///
/// # Parameters
/// - `c`: The byte to parse
///
/// # Returns
/// - `MavlinkPacket` if a complete packet was parsed
/// - `null` if more bytes are needed
pub fn parseChar(self: *Self, c: u8) ?MavlinkPacket {
    switch (self.state) {
        .WaitingForStart => {
            // Looking for STX byte to begin a new packet
            if (c == MAVLINK_STX_V2) {
                self.reset();
                self.packet.stx = c;
                self.header_bytes[0] = c;
                self.header_idx = 1;
                self.state = .GotHeader;
            }
        },

        .GotHeader => {
            // Collecting header bytes
            self.header_bytes[self.header_idx] = c;
            self.header_idx += 1;

            // Once we have full header, parse it
            if (self.header_idx >= MAVLINK_HEADER_LEN) {
                // Parse header fields
                self.packet.len = self.header_bytes[1];
                self.packet.incompat_flags = self.header_bytes[2];
                self.packet.compat_flags = self.header_bytes[3];
                self.packet.seq = self.header_bytes[4];
                self.packet.sysid = self.header_bytes[5];
                self.packet.compid = self.header_bytes[6];
                self.packet.msgid = @as(u24, self.header_bytes[7]) |
                    (@as(u24, self.header_bytes[8]) << 8) |
                    (@as(u24, self.header_bytes[9]) << 16);
                self.packet.has_signature = (self.packet.incompat_flags & 0x01) != 0;

                // Validate payload length
                if (self.packet.len > MAVLINK_MAX_PAYLOAD_LEN) {
                    self.reset();
                    return null;
                }

                // Initialize CRC with header bytes (except STX)
                self.crc_calc = MavlinkCrc.init();
                for (self.header_bytes[1..MAVLINK_HEADER_LEN]) |b| {
                    self.crc_calc.accumulate(b);
                }

                // Transition to payload reading or CRC if no payload
                if (self.packet.len > 0) {
                    self.state = .ReadingPayload;
                    self.payload_idx = 0;
                } else {
                    self.state = .ReadingCrcLow;
                }
            }
        },

        .ReadingPayload => {
            // Collect payload bytes
            if (self.payload_idx < self.packet.len) {
                self.packet.payload[self.payload_idx] = c;
                self.crc_calc.accumulate(c);
                self.payload_idx += 1;
            }

            // Check if payload is complete
            if (self.payload_idx >= self.packet.len) {
                // Pad with zeros if payload is shorter than expected
                if (MavlinkCrc.get_msg_info(self.packet.msgid)) |info| {
                    const expected_len = info.max_payload_len;
                    if (self.packet.len < expected_len) {
                        // Fill remaining payload with zeros (but don't add to CRC)
                        for (self.packet.len..expected_len) |i| {
                            self.packet.payload[i] = 0;
                        }
                        self.packet.len = expected_len;
                    }
                }
                self.state = .ReadingCrcLow;
            }
        },

        .ReadingCrcLow => {
            // Store CRC low byte
            self.packet.checksum = c;
            self.state = .ReadingCrcHigh;
        },

        .ReadingCrcHigh => {
            // Store CRC high byte and validate
            self.packet.checksum |= @as(u16, c) << 8;
            const calculated_crc = self.crc_calc.finalize(self.packet.msgid);

            if (calculated_crc != self.packet.checksum) {
                self.reset();
                return null;
            }

            // Check if we need to read signature
            if (self.packet.has_signature) {
                self.signature_idx = 0;
                self.state = .ReadingSignature;
            } else {
                const result = self.packet;
                self.reset();
                return result;
            }
        },

        .ReadingSignature => {
            // Collect signature bytes
            if (self.signature_idx < MAVLINK_SIGNATURE_LEN) {
                self.packet.signature[self.signature_idx] = c;
                self.signature_idx += 1;
            }

            // Return packet when signature is complete
            if (self.signature_idx >= MAVLINK_SIGNATURE_LEN) {
                const result = self.packet;
                self.reset();
                return result;
            }
        },
    }
    return null;
}

const testing = std.testing;

test "writeMessageToSlice returns BufferTooSmall when buffer is too short" {
    var small: [4]u8 = undefined;
    // Use a type whose MSG_ID is in the CRC table (e.g. 1)
    const Msg1 = struct {
        pub const MSG_ID = 1;
        dummy: u8,
    };
    const msg = Msg1{ .dummy = 0 };
    const result = writeMessageToSlice(small[0..], msg);
    try testing.expect(result == error.BufferTooSmall);
}

test "parseChar builds a zero-length-payload packet" {
    // Build header for msgid=1 with payload length=0
    const hdr = [_]u8{
        MAVLINK_STX_V2, // STX
        0, // len=0
        0, // incompat_flags
        0, // compat_flags
        5, // seq
        17, // sysid
        23, // compid
        1,
        0,
        0, // msgid = 1 (little endian)
    };
    // Compute CRC over header (except STX) + CRC_EXTRA
    const header_slice = hdr[1..];
    const crc_body = MavlinkCrc.crc_accumulate_buf(MavlinkCrc.crc_init(), header_slice);
    const extra = MavlinkCrc.get_msg_info(1).?.crc_extra;
    const final_crc = MavlinkCrc.crc_accumulate(extra, crc_body);
    const crc_low = @as(u8, @intCast(final_crc & 0xFF));
    const crc_high = @as(u8, @intCast((final_crc >> 8) & 0xFF));

    var parser = init();
    var maybe_pkt: ?MavlinkPacket = null;

    // Feed header bytes
    for (hdr) |b| {
        if (parser.parseChar(b)) |pkt| maybe_pkt = pkt;
    }
    try testing.expect(maybe_pkt == null);

    // Feed CRC bytes
    if (parser.parseChar(crc_low)) |pkt| maybe_pkt = pkt;
    if (parser.parseChar(crc_high)) |pkt| maybe_pkt = pkt;

    // Now we should have a complete packet
    const pkt = maybe_pkt.?;
    try testing.expectEqual(pkt.stx, MAVLINK_STX_V2);
    try testing.expectEqual(pkt.len, 0);
    try testing.expectEqual(pkt.seq, 5);
    try testing.expectEqual(pkt.sysid, 17);
    try testing.expectEqual(pkt.compid, 23);
    try testing.expectEqual(pkt.msgid, 1);
    try testing.expect(!pkt.has_signature);
}

test "parseChar ignores bytes until STX" {
    var parser = init();
    // Feed random bytes
    _ = parser.parseChar(0x00);
    _ = parser.parseChar(0xAB);
    _ = parser.parseChar(0x7F);
    // Still waiting
    try testing.expect(parser.state == ParseState.WaitingForStart);

    // Now feed STX and we transition
    _ = parser.parseChar(MAVLINK_STX_V2);
    try testing.expect(parser.state == ParseState.GotHeader);
}
