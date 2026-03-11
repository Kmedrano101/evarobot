#!/usr/bin/env python3
"""Capture raw UART data and try to find/decode CRSF frames."""
import os, time, struct, fcntl

TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
BOTHER = 0o10000

# CRC8 DVB-S2 table
CRC_TABLE = [0]*256
for i in range(256):
    crc = i
    for _ in range(8):
        crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    CRC_TABLE[i] = crc

def crc8(data):
    crc = 0
    for b in data:
        crc = CRC_TABLE[crc ^ b]
    return crc

fd = os.open('/dev/ttyAMA0', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

buf = bytearray(44)
fcntl.ioctl(fd, TCGETS2, buf)
c_cflag = struct.unpack_from('I', buf, 8)[0]
c_cflag = (c_cflag & ~0o10017) | BOTHER
struct.pack_into('I', buf, 8, c_cflag)
struct.pack_into('I', buf, 36, 420000)
struct.pack_into('I', buf, 40, 420000)
fcntl.ioctl(fd, TCSETS2, buf)

try:
    os.read(fd, 4096)
except:
    pass

print("Capturing at 420000 baud for 2 seconds...")
raw = bytearray()
start = time.time()
while time.time() - start < 2.0:
    try:
        data = os.read(fd, 512)
        if data:
            raw.extend(data)
    except BlockingIOError:
        pass
    time.sleep(0.01)
os.close(fd)
print(f"Captured {len(raw)} bytes\n")

# Byte frequency
freq = {}
for b in raw:
    freq[b] = freq.get(b, 0) + 1
print("Top 20 byte values:")
for b, count in sorted(freq.items(), key=lambda x: -x[1])[:20]:
    print(f"  0x{b:02X}: {count}")

# Scan for CRSF frames: try every byte as potential sync
print("\n--- Scanning for valid CRSF frames (any sync byte) ---")
found_rc = 0
found_link = 0
for i in range(len(raw) - 3):
    length = raw[i+1]
    ftype = raw[i+2]
    if length < 2 or length > 62:
        continue
    end = i + 2 + length
    if end > len(raw):
        continue
    frame_data = raw[i+2 : i+1+length]  # type + payload (no crc)
    frame_crc = raw[i+1+length]
    if crc8(frame_data) != frame_crc:
        continue
    sync = raw[i]
    if ftype == 0x16 and found_rc < 3:
        payload = raw[i+3 : i+1+length]
        bits = int.from_bytes(payload[:22], 'little')
        ch = [(bits >> (11*c)) & 0x7FF for c in range(16)]
        print(f"  RC_CHANNELS @ {i}, sync=0x{sync:02X}, CRC OK")
        print(f"    CH0-7:  {ch[0]:4} {ch[1]:4} {ch[2]:4} {ch[3]:4} {ch[4]:4} {ch[5]:4} {ch[6]:4} {ch[7]:4}")
        print(f"    CH8-15: {ch[8]:4} {ch[9]:4} {ch[10]:4} {ch[11]:4} {ch[12]:4} {ch[13]:4} {ch[14]:4} {ch[15]:4}")
        found_rc += 1
    elif ftype == 0x14 and found_link < 2:
        rssi = -raw[i+3]
        lq = raw[i+5]
        print(f"  LINK_STATS @ {i}, sync=0x{sync:02X}, RSSI={rssi}dBm LQ={lq}%")
        found_link += 1
    elif found_rc == 0 and found_link == 0:
        print(f"  Frame @ {i}, sync=0x{sync:02X}, len={length}, type=0x{ftype:02X}, CRC OK")

if found_rc == 0 and found_link == 0:
    print("  No valid frames found.")
    print("\n--- Raw hex (first 300 bytes) ---")
    for r in range(0, min(300, len(raw)), 26):
        h = ' '.join(f'{raw[r+j]:02X}' for j in range(min(26, len(raw)-r)))
        print(f"  {r:04d}: {h}")
