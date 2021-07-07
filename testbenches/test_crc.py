
CRC_poly = 0x04C11DB7

def bitreverse(size, value):
    out = 0
    for i in range(size):
        if (value & (1 << i)): out |= 1 << (size - i - 1)
    return out

def create_table():
    a = [0 for i in range(16)]
    for i in range(16):
        k = i
        crc = 0
        value = bitreverse(4,i)
        for j in range(4):
            bit_in = (value ^ (crc >> 31)) & 1
            value >>= 1
            crc = ((crc << 1) ^ (CRC_poly * bit_in)) & 0xFFFFFFFF
        a[i] = crc
    return(a)

def calculate_crc(value, nbits):
    crc = 0xFFFFFFFF
    for i in range(nbits//4):
        k = ((crc >> 28) ^ bitreverse(4, value)) & 0xF
        value >>= 4
        crc = ((crc << 4) ^ crc_table[k]) & 0xFFFFFFFF
        print(f"{crc&0xF:X}", end=' ')
    return crc ^ 0xFFFFFFFF

crc_table = create_table()

for item in crc_table:
    print(f"0x{item:08X},", end=' ')
print()




### 1611 A509 1062 4E02 F5DB
##
##
###  BD5F 20E4 2601 905A 1161 | 1611 A509 1062 4E02 F5DB
##
###61 11 5A 90 01 26
###16 11 A5 09 10 62
##
###E4 20 5F BD
###4E 02 F5 DB
##value = 0xBD5F20E4_2601905A1161
###value =          0x2601905A1161
##nbits = 10*8
##
##crc = calculate_crc(value, nbits)
##rev = bitreverse(32, crc)
##brev = bytebitreverse(32, crc)
##
#           BD5F20E4      4E02F5DB8
#          8BD5F20E4_2601905A1161                         1611A50910626A02F5CA8
##value = 0xBD5F20E4_2601905A1161
##nbits = 10*8
##print(f"normal: {calculate_crc(value, nbits):08X}    bitreverse: {bitreverse(32, crc):08X}  bytebit reverse: {bytebitreverse(32, crc):08X}")
##
##value = 0xBD5F20E4_2601905A1161
##nbits = 10*8-1
##print(f"normal: {calculate_crc(value, nbits):08X}    bitreverse: {bitreverse(32, calculate_crc(value, nbits)):08X}  bytebit reverse: {bytebitreverse(32, calculate_crc(value, nbits)):08X}")


#print(hex(crc_update(b"\x61\x11\x5A\x90\x01\x26\xE4\x20\x5F\xBD", 0)))


##0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,

##0123456789AB, 67E1550B full: 67E1550B0123456789AB result: C704DD7B
##2601905A1161, BD5F20E4 full: BD5F20E42601905A1161 result: C704DD7B
##0123456789AB, 67E1550B full: 67E1550B0123456789AB result: C704DD7B
##2601905A1161, BD5F20E4 full: BD5F20E42601905A1161 result: C704DD7B

##             5815590ff8f2d4cd001b
## 5815590ff8f25815590ff8f2d4cd001b


value = 0x0123456789AB
crc = bitreverse(32, calculate_crc(value, 6*8))
full_value = value | (crc << 6*8)
result = 0xFFFFFFFF ^ calculate_crc(full_value, 10*8)
print(f"{value:012X}, {crc:08X} full: {full_value:020X} result: {result:08X}")

value = 0x2601905A1161
crc = bitreverse(32, calculate_crc(value, 6*8))
full_value = value | (crc << 6*8)
result = 0xFFFFFFFF ^ calculate_crc(full_value, 10*8)
print(f"{value:012X}, {crc:08X} full: {full_value:020X} result: {result:08X}")


def calculate_crc(value, nbits):
    crc = 0xFFFFFFFF
    for i in range(nbits):
        bit_in = (value ^ (crc >> 31)) & 1
        value >>= 1
        crc = ((crc << 1) & 0xFFFFFFFF) ^ (CRC_poly * bit_in)
    return(0xFFFFFFFF ^ crc)

value = 0x0123456789AB
crc = bitreverse(32, calculate_crc(value, 6*8))
full_value = value | (crc << 6*8)
result = 0xFFFFFFFF ^ calculate_crc(full_value, 10*8)
print(f"{value:012X}, {crc:08X} full: {full_value:020X} result: {result:08X}")


value = 0x2601905A1161
crc = bitreverse(32, calculate_crc(value, 6*8))
full_value = value | (crc << 6*8)
result = 0xFFFFFFFF ^ calculate_crc(full_value, 10*8)
print(f"{value:012X}, {crc:08X} full: {full_value:020X} result: {result:08X}")
