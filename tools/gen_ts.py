#!/usr/bin/env python3
"""
Turn profile/ardudeck.xml into a TypeScript dialect for @ardudeck/mavlink-ts.

The same XML that generates the C side generates this, so the two can never disagree
about a field offset or a CRC extra. Without it the ARDUDECK_* messages reach ArduDeck's
inspector as MSG_42000 with no fields, because an id the registry does not know cannot
be decoded.

    python3 tools/gen_ts.py --out ~/work/ardudeck/packages/mavlink-ts/src/dialects/ardudeck.ts

Field offsets come from the same wire ordering the C uses: base fields widest first,
stable within a size, extensions last.
"""

import argparse
import os
import pathlib
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_defs import ROOT, find_common, parse, verify  # noqa: E402


def camel(name):
    head, *rest = name.split("_")
    return head + "".join(w.capitalize() for w in rest)


def pascal(name):
    return "".join(w.capitalize() for w in name.split("_"))


def const_name(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()


READERS = {
    "uint8_t": ("payload[{off}]", "buffer[{off}] = {v} & 0xff;"),
    "int8_t": ("view.getInt8({off})", "view.setInt8({off}, {v});"),
    "uint16_t": ("view.getUint16({off}, true)", "view.setUint16({off}, {v}, true);"),
    "int16_t": ("view.getInt16({off}, true)", "view.setInt16({off}, {v}, true);"),
    "uint32_t": ("view.getUint32({off}, true)", "view.setUint32({off}, {v}, true);"),
    "int32_t": ("view.getInt32({off}, true)", "view.setInt32({off}, {v}, true);"),
    "float": ("view.getFloat32({off}, true)", "view.setFloat32({off}, {v}, true);"),
}


def ts_type(f):
    if f.base_type == "char":
        return "string"
    return "number[]" if f.array_len else "number"


def emit_message(m):
    iface = pascal(m.name)
    fields = m.wire_fields
    out = []

    out.append("/**")
    out.append(" * %s" % m.name)
    out.append(" * Message ID: %d" % m.id)
    out.append(" * CRC Extra: %d" % m.crc_extra)
    out.append(" */")
    out.append("export interface %s {" % iface)
    for f in fields:
        out.append("  %s: %s;" % (camel(f.name), ts_type(f)))
    out.append("}")
    out.append("")
    out.append("export const %s_ID = %d;" % (const_name(iface), m.id))
    out.append("export const %s_CRC_EXTRA = %d;" % (const_name(iface), m.crc_extra))
    out.append("export const %s_LENGTH = %d;" % (const_name(iface), m.length))
    out.append("")

    # serialize
    out.append("export function serialize%s(msg: %s): Uint8Array {" % (iface, iface))
    out.append("  const buffer = new Uint8Array(%d);" % m.length)
    out.append("  const view = new DataView(buffer.buffer);")
    out.append("")
    off = 0
    for f in fields:
        name = camel(f.name)
        if f.base_type == "char":
            out.append("  buffer.set(new TextEncoder().encode(msg.%s || '')"
                       ".slice(0, %d), %d);" % (name, f.array_len, off))
        elif f.array_len:
            _, w = READERS[f.base_type]
            out.append("  for (let i = 0; i < %d; i++) {" % f.array_len)
            out.append("    " + w.format(off="%d + i * %d" % (off, f.unit_size),
                                         v="msg.%s?.[i] ?? 0" % name))
            out.append("  }")
        else:
            _, w = READERS[f.base_type]
            out.append("  " + w.format(off=off, v="msg.%s" % name))
        off += f.size
    out.append("")
    out.append("  return buffer;")
    out.append("}")
    out.append("")

    # deserialize
    out.append("export function deserialize%s(payload: Uint8Array): %s {" % (iface, iface))
    out.append("  const view = new DataView(payload.buffer, payload.byteOffset, "
               "payload.byteLength);")
    out.append("")
    out.append("  return {")
    off = 0
    for f in fields:
        name = camel(f.name)
        if f.base_type == "char":
            out.append("    %s: new TextDecoder().decode(payload.slice(%d, %d))"
                       ".replace(/\\0.*$/, '')," % (name, off, off + f.array_len))
        elif f.array_len:
            r, _ = READERS[f.base_type]
            expr = r.format(off="%d + i * %d" % (off, f.unit_size))
            out.append("    %s: Array.from({ length: %d }, (_, i) => %s),"
                       % (name, f.array_len, expr))
        else:
            r, _ = READERS[f.base_type]
            out.append("    %s: %s," % (name, r.format(off=off)))
        off += f.size
    out.append("  };")
    out.append("}")
    out.append("")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", required=True)
    ap.add_argument("--common")
    args = ap.parse_args()

    common_path = find_common(args.common)
    verify(parse(common_path))

    messages = parse(os.path.join(ROOT, "profile", "ardudeck.xml"),
                     search=(os.path.dirname(common_path),))
    messages = {n: m for n, m in messages.items() if n.startswith("ARDUDECK_")}
    ordered = [messages[n] for n in sorted(messages, key=lambda n: messages[n].id)]

    lines = [
        "/**",
        " * The ArduDeck dialect, generated from the Vehicle SDK's profile/ardudeck.xml.",
        " *",
        " * DO NOT EDIT. Regenerate with:",
        " *   python3 tools/gen_ts.py --out <this file>",
        " * in the ardudeck-vehicle-sdk repository, which is upstream of this one: the",
        " * contract a vendor implements cannot change because the application changed.",
        " *",
        " * Without these the messages still reach the parser (unknown ids are queued, not",
        " * dropped) but show up as MSG_42000 with no fields, because a message the registry",
        " * does not know cannot be decoded.",
        " */",
        "",
        "import type { MessageInfo } from '../core/types.js';",
        "import { MESSAGE_REGISTRY } from '../generated/message-registry.js';",
        "",
        "export const ARDUDECK_PROFILE_VERSION = 1;",
        "",
    ]

    for m in ordered:
        lines += emit_message(m)

    lines.append("/** Every message this dialect adds, keyed by id. */")
    lines.append("export const ARDUDECK_MESSAGES: ReadonlyArray<MessageInfo> = [")
    for m in ordered:
        iface = pascal(m.name)
        lines.append("  {")
        lines.append("    msgid: %d," % m.id)
        lines.append("    name: '%s'," % m.name)
        lines.append("    crcExtra: %d," % m.crc_extra)
        lines.append("    minLength: %d," % m.length)
        lines.append("    maxLength: %d," % m.length)
        lines.append("    serialize: serialize%s as (msg: unknown) => Uint8Array," % iface)
        lines.append("    deserialize: deserialize%s as (payload: Uint8Array) => unknown,"
                     % iface)
        lines.append("  },")
    lines.append("];")
    lines.append("")
    lines += [
        "/**",
        " * Merge the dialect into the shared registry.",
        " *",
        " * Idempotent, and it never replaces a standard message: a dialect that shadowed a",
        " * common id would break every vehicle that is not using it.",
        " */",
        "export function registerArduDeckDialect(): void {",
        "  for (const info of ARDUDECK_MESSAGES) {",
        "    if (MESSAGE_REGISTRY.has(info.msgid)) continue;",
        "    MESSAGE_REGISTRY.set(info.msgid, info);",
        "  }",
        "}",
        "",
    ]

    out = pathlib.Path(os.path.expanduser(args.out))
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines))
    print("wrote %s (%d messages)" % (out, len(ordered)))
    for m in ordered:
        print("  %-28s id=%5d len=%3d extra=%3d" % (m.name, m.id, m.length, m.crc_extra))


if __name__ == "__main__":
    main()
