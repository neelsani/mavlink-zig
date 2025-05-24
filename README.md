# MAVLink Zig Library ✈️

[![Zig Version](https://img.shields.io/badge/Zig-0.14.0-%23f7a41d.svg)](https://ziglang.org/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Build Status](https://github.com/neelsani/mavlink-zig/actions/workflows/build.yaml/badge.svg)](https://github.com/neelsani/mavlink-zig/actions)

A lightweight, zero-dependency, zero-allocation MAVLink v2.0 protocol implementation in Zig, designed for embedded systems and high-performance applications.

## Features ✨

- 🚀 Full MAVLink v2.0 protocol support
- ⚡ Zero allocations during packet processing
- 🔒 End-to-end type safety
- 🔋 Suitable for resource-constrained systems
- ✨ Pure Zig :)

## Installation

Developers tend to either use
* The latest tagged release of Zig
* The latest build of Zigs master branch

Depending on which developer you are, you need to run different `zig fetch` commands:

```sh
# Version of zig-clap that works with a tagged release of Zig
# Replace `<REPLACE ME>` with the version of zig-clap that you want to use
# See: https://github.com/neelsani/mavlink-zig/releases
zig fetch --save https://github.com/neelsani/mavlink-zig/archive/refs/tags/<REPLACE ME>.tar.gz

# Version of zig-clap that works with latest build of Zigs master branch
zig fetch --save git+https://github.com/neelsani/mavlink-zig
```

Then add the following to `build.zig`:

```zig
const mavlink = b.dependency("mavlink", .{});
exe.root_module.addImport("mavlink", mavlink.module("mavlink"));
```


## Acknowledgements
- [zig-xml](https://github.com/ianprime0509/zig-xml) — the XML parsing library used by the code generator to transform MAVLink XML definitions into Zig code.  

## Support

For support, email neel@neels.dev.

