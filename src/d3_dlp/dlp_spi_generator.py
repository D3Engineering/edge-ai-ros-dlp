def VCM_START_ADDR1(START_ADDR1):
    """Take the Video 1 Start Address in Flash (int) and return corresponding DLP Command"""
    START_ADDR1 = cast_to_int(START_ADDR1)
    START_ADDR1 = hex(int(START_ADDR1)).upper()
    return generate_spi_command_arr("startaddr", START_ADDR1)


def VCM_CONFIG1(LOOP_CNT1, FRAME_CNT1):
    """Take the Loop and Frame Counts (int) and return corresponding DLP Command"""
    LOOP_CNT1 = cast_to_int(LOOP_CNT1)
    FRAME_CNT1 = cast_to_int(FRAME_CNT1)
    LOOP_CNT1 = hex(int(LOOP_CNT1))[2:].upper()
    FRAME_CNT1 = hex(int(FRAME_CNT1))[2:].upper()
    return generate_spi_command_arr("vidconf1", "0x" + LOOP_CNT1 + FRAME_CNT1)


def VCM_FRAME_RATE(fps):
    """Take the desired Framerate in Frames per Second (int) and return corresponding DLP Command"""
    fps = cast_to_int(fps)
    fps = 40000000 / int(fps)
    VCM_FRAME_RATE = hex(int(fps))[2:].upper()
    return generate_spi_command_arr("vidfps", "0x" + VCM_FRAME_RATE)


def FMT_FLIP(long_flip, short_flip):
    """Take the desired flips (bool) and return corresponding DLP Command"""
    cmd = 0
    if short_flip:
        cmd |= 0b1
    if long_flip:
        cmd |= 0b10000
    return generate_spi_command_arr("fmtflip", hex(cmd).upper())


def VCM_CONTROL(enable_playback):
    """Take whether we want to play video (bool) and return corresponding DLP Command"""
    cmd = 0
    if enable_playback:
        cmd |= 0b10101
    else:
        cmd |= 0b10
    print(bin(cmd))
    print(hex(cmd))
    return generate_spi_command_arr("vidctl", hex(cmd).upper())


def cast_to_int(hex_or_int_str):
    hex_or_int_str = str(hex_or_int_str)
    if len(hex_or_int_str) >= 2 and hex_or_int_str[0:2] == "0x":
        return int(hex_or_int_str, 16)
    else:
        return int(hex_or_int_str)


def main(cmd=""):
    print("")
    print("DLP SPI Command Generator")
    print("Commands: exit, startaddr, vidconf1, vidfps, fmtflip, vidctl")
    if cmd == "":
        cmd = input("> ").lower()
    if cmd == "exit":
        return False
    elif cmd == "startaddr":
        startaddr = input("Enter the Video Start Address Offset: ")
        VCM_START_ADDR1(startaddr)
    elif cmd == "vidconf1":
        loopcount = input("Loop Count (0 for Inf): ")
        framecount = input("Frame Count: ")
        VCM_CONFIG1(loopcount, framecount)
    elif cmd == "vidfps":
        fps = input("Framerate (FPS): ")
        VCM_FRAME_RATE(fps)
    elif cmd == "fmtflip":
        ssf = ""
        while ssf != "y" and ssf != "n":
            ssf = input("Short Side Flip? [y/n]: ")
        lsf = ""
        while lsf != "y" and lsf != "n":
            lsf = input("Long Side Flip? [y/n]: ")
        ssf = ssf == "y"
        lsf = lsf == "y"
        FMT_FLIP(lsf, ssf)
    elif cmd == "vidctl":
        play = ""
        while play != "y" and play != "n":
            play = input("Play Video? [y/n]: ")
        play = play == "y"
        VCM_CONTROL(play)
    return True


def generate_spi_command_arr(cmdtype, data):
    result = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    checksumint = 0
    if cmdtype == "startaddr":
        checksumint = 0x64
        result[1] = 0x64
    elif cmdtype == "vidconf1":
        checksumint = 0x68
        result[1] = 0x68
    elif cmdtype == "vidfps":
        checksumint = 0x60
        result[1] = 0x60
    elif cmdtype == "fmtflip":
        checksumint = 0x20
        result[1] = 0x20
    elif cmdtype == "vidctl":
        checksumint = 0x74
        result[1] = 0x74
    arrindex = 0
    strindex = 2
    arrbitlen = 0
    if len(data) % 2 == 0:
        arrbitlen = 2
    else:
        arrbitlen = 1

    if len(data) < 3:
        print("data value too small")
    elif 3 <= len(data) <= 4:
        arrindex = -5
    elif 5 <= len(data) <= 6:
        arrindex = -4
    elif 7 <= len(data) <= 8:
        arrindex = -3
    elif 9 <= len(data) <= 10:
        arrindex = -2
    else:
        print("data value too large")

    tempstr = ""
    for char in data[2:]:
        if arrbitlen == 1:
            rint = int("0x0" + char, base=16)
            result[arrindex] = rint
            checksumint += rint
            arrbitlen = 2
            arrindex -= 1
        else:
            if tempstr == "":
                tempstr = "0x" + char
            else:
                tempstr += char
                rint = int(tempstr, base=16)
                result[arrindex] = rint
                checksumint += rint
                tempstr = ""
                arrindex -= 1
    if checksumint > 255:
        checksumint &= 0xFF
    result[-1] = checksumint
    print_address_arr(result)
    return result


def print_address_arr(arr):
    result = ""
    for i in range(len(arr)):
        if i == 0:
            result += "[" + ("0x{:02x}".format(arr[i])) + ", "
        elif i < len(arr) - 1:
            result += "0x{:02x}".format(arr[i]) + ", "
        else:
            result += "0x{:02x}".format(arr[i]) + "]"
    print(result)


if __name__ == "__main__":
    running = True
    while (running):
        running = main()
