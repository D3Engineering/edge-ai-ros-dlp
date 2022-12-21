def main(cmd=""):
	print("")
	print("DLP SPI Command Generator")
	print("Commands: exit, startaddr, vidconf1")
	if cmd == "":
		cmd = input("> ").lower()
	if cmd == "exit":
		return False
	elif cmd == "startaddr":
		startaddr = input("Enter the Video Start Address Offset in hex format (0xABCDEF...): ")
		if startaddr[0:2] == "0x":
			generate_spi_command_arr(cmd, startaddr)
		else:
			print("Invalid Video Start Address Offset")
			main(cmd)
	elif cmd == "vidconf1":
		framecount = input("Enter the Video Frame Count in hex format (0xABCDEF...): ")
		if framecount[0:2] == "0x":
			generate_spi_command_arr(cmd, framecount)
		else:
			print("Invalid Video Frame Count")
			main(cmd)
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
			rint = int("0x0"+char, base=16)
			result[arrindex] = rint
			checksumint += rint
			arrbitlen = 2
			arrindex-=1
		else:
			if tempstr == "":
				tempstr = "0x"+char
			else:
				tempstr += char
				rint = int(tempstr, base=16)
				result[arrindex] = rint
				checksumint += rint
				tempstr = ""
				arrindex-=1
	if checksumint > 255:
		checksumint &= 0xFF
	result[-1] = checksumint
	print_address_arr(result)
	

def print_address_arr(arr):
	result = ""
	for i in range(len(arr)):
		if i == 0:
			result += "["+("0x{:02x}".format(arr[i])) + ", "
		elif i < len(arr)-1:
			result += "0x{:02x}".format(arr[i]) + ", "
		else:
			result += "0x{:02x}".format(arr[i]) + "]"
	print(result)


if __name__ == "__main__":
	running = True
	while(running):
		running = main()
