"""Embed an explicitly reviewed VME; never reinterpret its programming data."""
import hashlib
import sys
from pathlib import Path

source, manifest, output = map(Path, sys.argv[1:])
data = source.read_bytes()
digest = hashlib.sha256(data).hexdigest()
if digest != manifest.read_text().strip():
    raise SystemExit('VME SHA-256 is not reviewed. Audit the new VME before updating reviewed-vme.sha256.')
rows = [','.join(f'0x{x:02x}' for x in data[i:i+16]) for i in range(0, len(data), 16)]
output.write_text('#include <stdint.h>\nstatic const uint8_t vme_image[] = {\n' +
                  ',\n'.join(rows) + '\n};\n#define VME_SHA256 "' + digest + '"\n')
