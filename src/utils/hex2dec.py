import math

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0:          # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)                 # compute negative value
    return ((val * 0.35 * math.pi)/(16 * 60))    # return positive value as is

right_hex_string = '0x000001F4' # or whatever... '0x' prefix doesn't matter
right_out = twos_comp(int(right_hex_string,16), 32)

left_hex_string = '0xFFFFFE0C'
left_out = twos_comp(int(left_hex_string,16), 32)

print("R: ", right_out, " L: ", left_out)# or whatever... '0x' prefix doesn't matter