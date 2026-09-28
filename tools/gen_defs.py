#!/usr/bin/env python3
"""
Turn profile/ardudeck.xml into src/ad_msg_defs.h.

Nothing about the wire is hand-written. Message ids, wire field order, payload lengths
and CRC extras are all computed here from the XML, because a mistyped CRC extra produces
frames that every receiver silently drops, which looks exactly like a network fault and
costs a day to find.

The generator proves itself before it emits anything: it recomputes the CRC extras for a
set of standard MAVLink messages whose published values are known, and refuses to run if
any of them disagree. If the algorithm is right for HEARTBEAT and for MISSION_COUNT (which
has extension fields, the case people get wrong), it is right for ours.

    python3 tools/gen_defs.py --common <path to common.xml>
    python3 tools/gen_defs.py --verify            # check only, write nothing

The generated header is committed, so building the SDK needs neither Python nor this file.
"""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Where a checkout of the MAVLink definitions usually lives. Any of these will do; the
# file is only needed when regenerating, never when building.
COMMON_CANDIDATES = [
    os.path.join(ROOT, "profile", "common.xml"),
    os.path.expanduser("~/work/ardudeck/MissionPlanner-ref/ExtLibs/Mavlink/message_definitions/common.xml"),
    "/usr/local/share/mavlink/message_definitions/v1.0/common.xml",
]

# Published CRC extras, cross-checked against ArduDeck's own MESSAGE_REGISTRY. These are
# the check, not the output: reproduce all of them and the algorithm is reproducing ours
# correctly too. Half of these carry extension fields, which is the case worth proving.
KNOWN_EXTRAS = {
    "HEARTBEAT": 50,
    "SYS_STATUS": 124,
    "GPS_RAW_INT": 24,
    "ATTITUDE": 39,
    "GLOBAL_POSITION_INT": 104,
    "VFR_HUD": 20,
    "PARAM_VALUE": 220,
    "PARAM_REQUEST_READ": 214,
    "PARAM_REQUEST_LIST": 159,
    "PARAM_SET": 168,
    "COMMAND_LONG": 152,
    "COMMAND_ACK": 143,
    "MISSION_ITEM_INT": 38,
    "MISSION_REQUEST_INT": 196,
    "MISSION_REQUEST_LIST": 132,
    "MISSION_COUNT": 221,
    "MISSION_ACK": 153,
    "MISSION_CLEAR_ALL": 232,
    "MISSION_CURRENT": 28,
    "RC_CHANNELS": 118,
    "STATUSTEXT": 83,
    "HOME_POSITION": 104,
    "SET_MODE": 89,
}

# Standard messages the SDK itself emits or consumes. Pulled from common.xml so their
# lengths and extras come from the same place as everything else.
STANDARD_USED = [
    "HEARTBEAT", "SYS_STATUS", "GPS_RAW_INT", "ATTITUDE", "GLOBAL_POSITION_INT",
    "VFR_HUD", "PARAM_VALUE", "PARAM_REQUEST_READ", "PARAM_REQUEST_LIST", "PARAM_SET",
    "COMMAND_LONG", "COMMAND_ACK", "MISSION_ITEM_INT", "MISSION_REQUEST_INT",
    "MISSION_REQUEST_LIST", "MISSION_COUNT", "MISSION_ACK", "MISSION_CLEAR_ALL",
    "MISSION_CURRENT", "RC_CHANNELS", "STATUSTEXT", "HOME_POSITION", "SET_MODE",
]

TYPE_SIZE = {
    "char": 1, "int8_t": 1, "uint8_t": 1, "uint8_t_mavlink_version": 1,
    "int16_t": 2, "uint16_t": 2,
    "int32_t": 4, "uint32_t": 4, "float": 4,
    "int64_t": 8, "uint64_t": 8, "double": 8,
}


class Field:
    __slots__ = ("name", "base_type", "array_len", "extension")

    def __init__(self, name, type_str, extension):
        self.name = name
        self.extension = extension
        m = re.match(r"^([A-Za-z0-9_]+)\[(\d+)\]$", type_str)
        if m:
            self.base_type, self.array_len = m.group(1), int(m.group(2))
        else:
            self.base_type, self.array_len = type_str, 0
        if self.base_type not in TYPE_SIZE:
            raise SystemExit("unknown field type %r on %s" % (type_str, name))

    @property
    def unit_size(self):
        return TYPE_SIZE[self.base_type]

    @property
    def size(self):
        return self.unit_size * (self.array_len or 1)

    @property
    def crc_type(self):
        # mavgen writes the declared type minus the array suffix, and the magic
        # version field is checksummed as the plain integer it really is.
        return "uint8_t" if self.base_type == "uint8_t_mavlink_version" else self.base_type

    def c_type(self):
        return "uint8_t" if self.base_type == "uint8_t_mavlink_version" else self.base_type


class Message:
    def __init__(self, msg_id, name, fields):
        self.id = msg_id
        self.name = name
        self.fields = fields

    @property
    def base_fields(self):
        return [f for f in self.fields if not f.extension]

    @property
    def wire_fields(self):
        """Wire order: base fields widest first, stable, then extensions as declared."""
        base = sorted(self.base_fields, key=lambda f: -f.unit_size)
        return base + [f for f in self.fields if f.extension]

    @property
    def length(self):
        """Base payload only.

        The SDK never populates extension fields of a standard message. Receivers vary in
        what maximum length they accept, and a frame longer than one expects is dropped
        without a word, so it only ever builds the payload every receiver agrees on.
        """
        return sum(f.size for f in self.base_fields)

    @property
    def crc_extra(self):
        crc = X25()
        crc.feed_str(self.name + " ")
        # Extensions are excluded on purpose: adding one must not change the extra, or
        # every already-deployed receiver would drop the message.
        for f in sorted(self.base_fields, key=lambda f: -f.unit_size):
            crc.feed_str(f.crc_type + " ")
            crc.feed_str(f.name + " ")
            if f.array_len:
                crc.feed(bytes([f.array_len]))
        return (crc.value & 0xFF) ^ (crc.value >> 8)


class X25:
    """The CRC-16/MCRF4XX that MAVLink calls the X.25 checksum."""

    def __init__(self):
        self.value = 0xFFFF

    def feed(self, data):
        for b in data:
            tmp = b ^ (self.value & 0xFF)
            tmp = (tmp ^ (tmp << 4)) & 0xFF
            self.value = ((self.value >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

    def feed_str(self, s):
        self.feed(s.encode("ascii"))


def parse(path, search=(), _seen=None):
    """Read a dialect into {name: Message}, following <include> the way mavgen does."""
    seen = _seen if _seen is not None else set()
    real = os.path.realpath(path)
    if real in seen:
        return {}
    seen.add(real)

    root = ET.parse(path).getroot()
    out = {}
    for inc in root.findall("./include"):
        name = inc.text.strip()
        for base in (os.path.dirname(real),) + tuple(search):
            candidate = os.path.join(base, name)
            if os.path.exists(candidate):
                out.update(parse(candidate, search, seen))
                break
        else:
            raise SystemExit("%s includes %s, which is not next to it or in --common's "
                             "directory" % (os.path.basename(real), name))

    for m in root.findall("./messages/message"):
        fields, extension = [], False
        for child in m:
            if child.tag == "extensions":
                extension = True
            elif child.tag == "field":
                fields.append(Field(child.get("name"), child.get("type"), extension))
        msg = Message(int(m.get("id")), m.get("name"), fields)
        out[msg.name] = msg
    return out


def find_common(explicit):
    for path in ([explicit] if explicit else []) + COMMON_CANDIDATES:
        if path and os.path.exists(path):
            return path
    raise SystemExit(
        "common.xml not found. Pass --common <path>, or drop a copy at profile/common.xml.\n"
        "It is only needed to regenerate; src/ad_msg_defs.h is committed."
    )


def verify(common):
    """Refuse to generate unless the algorithm reproduces every published extra."""
    bad = []
    for name, expected in sorted(KNOWN_EXTRAS.items()):
        msg = common.get(name)
        if msg is None:
            bad.append("%s missing from common.xml" % name)
        elif msg.crc_extra != expected:
            bad.append("%s extra %d, expected %d" % (name, msg.crc_extra, expected))
    if bad:
        print("CRC algorithm does not reproduce known values:", file=sys.stderr)
        for line in bad:
            print("  " + line, file=sys.stderr)
        raise SystemExit(1)
    print("verified %d known CRC extras" % len(KNOWN_EXTRAS))


def c_name(name):
    return "AD_MSG_" + name.replace("ARDUDECK_", "")


def emit(messages, out_path):
    lines = [
        "/*",
        " * Generated by tools/gen_defs.py from profile/ardudeck.xml. Do not edit.",
        " *",
        " * Every id, payload length and CRC extra here was computed, never typed. The",
        " * generator refuses to run unless it first reproduces the published extras for",
        " * the standard messages, so these values are as trustworthy as those are.",
        " */",
        "",
        "#ifndef AD_MSG_DEFS_H",
        "#define AD_MSG_DEFS_H",
        "",
        "#include <stdint.h>",
        "",
        "typedef struct {",
        "  uint32_t id;",
        "  uint8_t  length;   /* base payload; the SDK never fills extensions */",
        "  uint8_t  crc_extra;",
        "} ad_msg_def_t;",
        "",
    ]

    widest = max(len(c_name(m.name)) for m in messages)
    for m in messages:
        lines.append(
            "#define %-*s ((ad_msg_def_t){ %5d, %3d, %3d })  /* %s */"
            % (widest, c_name(m.name), m.id, m.length, m.crc_extra, m.name)
        )

    lines += [
        "",
        "/* Largest payload the SDK ever builds, so the frame buffer can be sized exactly. */",
        "#define AD_MAX_PAYLOAD %d" % max(m.length for m in messages),
        "",
        "#endif /* AD_MSG_DEFS_H */",
        "",
    ]
    with open(out_path, "w") as fh:
        fh.write("\n".join(lines))
    print("wrote %s (%d messages, max payload %d)"
          % (os.path.relpath(out_path, ROOT), len(messages), max(m.length for m in messages)))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--common", help="path to a MAVLink common.xml")
    ap.add_argument("--verify", action="store_true", help="check the algorithm, write nothing")
    args = ap.parse_args()

    common_path = find_common(args.common)
    common = parse(common_path)
    print("common.xml: %s" % common_path)
    verify(common)

    if args.verify:
        return

    ardudeck = parse(os.path.join(ROOT, "profile", "ardudeck.xml"),
                     search=(os.path.dirname(common_path),))
    ardudeck = {n: m for n, m in ardudeck.items() if n.startswith("ARDUDECK_")}

    for name, msg in sorted(ardudeck.items()):
        if msg.length > 255:
            raise SystemExit("%s payload is %d bytes, the limit is 255" % (name, msg.length))

    messages = [common[n] for n in STANDARD_USED] + [ardudeck[n] for n in sorted(ardudeck)]
    emit(messages, os.path.join(ROOT, "src", "ad_msg_defs.h"))

    print("\nardudeck dialect:")
    for name in sorted(ardudeck):
        m = ardudeck[name]
        print("  %-28s id=%5d len=%3d extra=%3d" % (m.name, m.id, m.length, m.crc_extra))


if __name__ == "__main__":
    main()
