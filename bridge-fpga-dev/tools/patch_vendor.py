"""Reproduce the small, reviewed changes to the preserved Lattice core."""
from pathlib import Path

root = Path(__file__).resolve().parents[2]
source = (root / 'bridge-docs/FPGA-JTAG-EXAMPLE/ivm_core.c').read_bytes()
text = source.decode('cp1252').replace('\r\n', '\n')
start = text.index('signed char ispVMShift( signed char a_cCode )\n{')
end = text.index('signed char ispVMAmble( signed char Code )\n{', start)
part = text[start:end]
part = part.replace('g_usiDataSize = (unsigned short) ispVMDataSize();',
                    'g_usiDataSize = (unsigned short) ispVMDataSize();\n'
                    '\tif (!g_usiDataSize || g_usiDataSize > g_usMaxSize || '
                    'g_usMaxSize > SCANMAX) return VME_INVALID_FILE;', 1)
part = part.replace('#ifdef VME_DEBUG\n\tprintf( "%d ", g_usiDataSize );',
                    'scan_begin();\n\n#ifdef VME_DEBUG\n\tprintf( "%d ", g_usiDataSize );', 1)
part = part.replace('\treturn ( cRetCode );\n}', '\tscan_end(cRetCode);\n\treturn ( cRetCode );\n}', 1)
text = text[:start] + part + text[end:]
# Bound the legacy signed-long variable-length decode on both ILP32 and LP64.
text = text.replace('while ( ( cCurrentByte = GetByte() ) & 0x80 ) {',
                    'while ( ( cCurrentByte = GetByte() ) & 0x80 ) {\n'
                    '\t\tif (cIndex >= 28) fail(VME_INVALID_FILE);', 1)
text = text.replace('cIndex += 7;\n\t}',
                    'cIndex += 7;\n\t\tif (cIndex > 28) fail(VME_INVALID_FILE);\n\t}', 1)
text = text.replace('iSize |= ( ( long int ) ( cCurrentByte & 0x7F ) ) << cIndex;\n\treturn iSize;',
                    'if (cIndex == 28 && (cCurrentByte & 0x78)) fail(VME_INVALID_FILE);\n'
                    '\tiSize |= ( ( long int ) ( cCurrentByte & 0x7F ) ) << cIndex;\n\treturn iSize;', 1)
# Correct a vendor printf type mismatch; checksum algorithm stays unchanged.
text = text.replace('sprintf(StrChecksum,"Data Checksum: %.4X\\n\\n",g_usChecksum);',
                    'sprintf(StrChecksum,"Data Checksum: %.4lX\\n\\n",g_usChecksum);')
text = text.replace('usErrorCount++;',
                    'compare_failed(usDataSizeIndex, (cDataByte >> (7 - cByteIndex)) & 1, cCurBit);\n'
                    '\t\t\t\t\t\tusErrorCount++;', 1)
text = text.replace('g_usDataType &= ~LHEAP_IN;\n\treturn cRetCode;',
                    'g_usDataType &= ~LHEAP_IN;\n'
                    '\tif (cRetCode < 0) poll_exhausted(a_usCountSize);\n\treturn cRetCode;', 1)
(root / 'bridge-fpga-dev/src/ispvm/ivm_core.c').write_bytes(text.replace('\n', '\r\n').encode('cp1252'))
