#!/usr/bin/env python3
"""Scan baud rates to find correct one for ELRS CRSF receiver."""
import os, time, struct, fcntl

TCGETS2 = 0x802C542A
TCSETS2 = 0x402C542B
BOTHER = 0o10000

fd = os.open('/dev/ttyAMA0', os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

def set_baud(fd, baud):
    buf = bytearray(44)
    fcntl.ioctl(fd, TCGETS2, buf)
    c_cflag = struct.unpack_from('I', buf, 8)[0]
    c_cflag = (c_cflag & ~0o10017) | BOTHER
    struct.pack_into('I', buf, 8, c_cflag)
    struct.pack_into('I', buf, 36, baud)
    struct.pack_into('I', buf, 40, baud)
    fcntl.ioctl(fd, TCSETS2, buf)

print("Scanning baud rates for CRSF sync byte (0xC8)...\n")

for baud in [416666, 420000, 400000, 410000, 430000, 440000, 460800, 380000, 450000, 500000]:
    set_baud(fd, baud)
    try:
        while True:
            d = os.read(fd, 4096)
            if not d:
                break
    except:
        pass
    time.sleep(0.5)

    total = 0
    syncs = 0
    sync_ee = 0
    sample = b''
    start = time.time()
    while time.time() - start < 1.0:
        try:
            data = os.read(fd, 512)
            if data:
                total += len(data)
                syncs += data.count(b'\xc8')
                sync_ee += data.count(b'\xee')
                if not sample:
                    sample = data[:20]
        except BlockingIOError:
            pass
        time.sleep(0.02)

    if total > 0:
        print(f'{baud:>7}: {total:4} bytes  0xC8={syncs:3}  0xEE={sync_ee:3}  hex={sample.hex(" ")}')
    else:
        print(f'{baud:>7}: no data')

os.close(fd)
print("\nThe correct baud rate will have the most 0xC8 sync bytes.")
