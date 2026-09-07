#!/usr/bin/env python3
"""
transform_company_ids.py - Transform Bluetooth SIG company_ids.json into multiple formats:
  1. C++ Header / PROGMEM array (for direct compilation into firmware)
  2. Binary Database (company_ids.bin) for SD card / LittleFS with O(log N) binary search
  3. Text Database (company_ids.txt) for SD card / LittleFS
"""

import os
import sys
import json
import struct
import argparse

def sanitize_name(name: str, max_len: int = 28) -> str:
    """Clean company name by stripping excessive legal suffixes and trimming length."""
    cleaned = name.strip()
    # Common replacements for compact embedded display
    replacements = [
        (" Technologies International, Ltd. (QTIL)", ""),
        (" International Industries, Inc.", ""),
        (" Mobile Communications", ""),
        (" Semiconductor Corporation", " Semi"),
        (" Semiconductor ASA", " Semi"),
        (" Semiconductor", " Semi"),
        (" Technologies AG", ""),
        (" Technologies, Inc.", ""),
        (" Technologies Inc.", ""),
        (" Corporation", " Corp."),
        (" Holdings Corporation", " Corp."),
        (" Holding Co., Ltd.", ""),
        (" Co., Ltd.", ""),
        (" Co.,Ltd.", ""),
        (" Ltd.", ""),
        (" LLC", ""),
        (" Inc.", ""),
        (" Inc", ""),
        (" AB", ""),
        (" AG", ""),
        (" B.V.", ""),
        (" S.A.S", ""),
        (" S.L.", ""),
        (" OY", ""),
    ]
    for old, new in replacements:
        if cleaned.endswith(old) or old in cleaned:
            cleaned = cleaned.replace(old, new)
    
    cleaned = cleaned.strip(" ,.")
    if len(cleaned) > max_len:
        cleaned = cleaned[:max_len].rstrip(" ,.")
    return cleaned

def generate_cpp_header(entries, output_path, var_name="BLE_SIG_ALL_COMPANIES"):
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("#ifndef BLE_COMPANY_IDS_DATA_H\n")
        f.write("#define BLE_COMPANY_IDS_DATA_H\n\n")
        f.write("#include <Arduino.h>\n\n")
        f.write("struct BleCompanyIdEntry {\n")
        f.write("    uint16_t id;\n")
        f.write("    const char *name;\n")
        f.write("};\n\n")
        f.write(f"// Bluetooth SIG Assigned Numbers ({len(entries)} entries)\n")
        f.write(f"static const BleCompanyIdEntry {var_name}[] PROGMEM = {{\n")
        for code, name in entries:
            escaped_name = name.replace('\\', '\\\\').replace('"', '\\"')
            f.write(f'    {{0x{code:04X}, "{escaped_name}"}},\n')
        f.write("};\n\n")
        f.write(f"static const size_t {var_name}_COUNT = sizeof({var_name}) / sizeof({var_name}[0]);\n\n")
        f.write("#endif // BLE_COMPANY_IDS_DATA_H\n")

def generate_binary_db(entries, output_path, record_size=32):
    """
    Generate sorted binary file.
    Record format (32 bytes fixed):
      - 2 bytes: uint16_t Company ID (Big-Endian)
      - 30 bytes: ASCII null-terminated company name
    """
    name_len = record_size - 2
    with open(output_path, "wb") as f:
        for code, name in entries:
            code_bytes = struct.pack(">H", code)
            name_bytes = name.encode("ascii", errors="replace")[:name_len - 1]
            padded_name = name_bytes + b"\x00" * (name_len - len(name_bytes))
            f.write(code_bytes + padded_name)

def generate_text_db(entries, output_path):
    with open(output_path, "w", encoding="utf-8") as f:
        f.write("# Bluetooth SIG Company Identifiers\n")
        f.write("# Format: <HEX_ID> <COMPANY_NAME>\n")
        for code, name in entries:
            f.write(f"0x{code:04X} {name}\n")

def main():
    parser = argparse.ArgumentParser(description="Transform Bluetooth SIG company_ids.json")
    parser.add_argument("-i", "--input", default="company_ids.json", help="Path to company_ids.json input")
    parser.add_argument("-o", "--outdir", default=".", help="Output directory")
    parser.add_argument("--compact", action="store_true", help="Sanitize and compact company names")
    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Error: input file {args.input} not found.")
        sys.exit(1)

    with open(args.input, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Process entries
    entries = []
    for item in data:
        code = item.get("code") if "code" in item else item.get("value")
        raw_name = item.get("name", "")
        if code is None or not raw_name:
            continue
        name = sanitize_name(raw_name) if args.compact else raw_name.strip()
        entries.append((int(code), name))

    # Sort numerically by code for binary search
    entries.sort(key=lambda x: x[0])

    os.makedirs(args.outdir, exist_ok=True)
    cpp_out = os.path.join(args.outdir, "ble_company_ids_data.h")
    bin_out = os.path.join(args.outdir, "company_ids.bin")
    txt_out = os.path.join(args.outdir, "company_ids.txt")

    generate_cpp_header(entries, cpp_out)
    generate_binary_db(entries, bin_out)
    generate_text_db(entries, txt_out)

    print(f"Successfully transformed {len(entries)} company IDs:")
    print(f"  -> C++ Header: {cpp_out}")
    print(f"  -> Binary DB:  {bin_out} ({os.path.getsize(bin_out)} bytes)")
    print(f"  -> Text DB:    {txt_out} ({os.path.getsize(txt_out)} bytes)")

if __name__ == "__main__":
    main()
