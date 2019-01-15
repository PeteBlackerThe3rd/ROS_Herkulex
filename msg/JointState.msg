#--------------------------------------
#
# Herkulex package custom joint state message
#
# represents the current state of a set of
# named herkulex servos
#
#--------------------------------------

Header header

string[] name
float64[] position
float64[] velocity
float64[] effort

# the addres 0x0 - 0x253 of each servo
char[] servo_id

# The mode of each servo
string[] type

# The firmware version string of each servo
string[] firmware

float64[] voltage
float64[] temperature

# Servo status, 2x bytes
# This matches the statusError and statusDetail bytes as described in the Herkulex protocol documentation 
int16[] status

# The following bit masks can be used to access the status information
int16 STATUS_VOLTAGE_EXCEEDED = 		1		# 0x0001
int16 STATUS_RANGE_EXCEEDED =			2		# 0x0002
int16 STATUS_TEMPERATURE_EXCEEDED = 	4		# 0x0004
int16 STATUS_INVALID_PACKET	= 			8 		# 0x0008
int16 STATUS_OVERLOAD_DETECTED = 		16		# 0x0010
int16 STATUS_EEG_REGISTER_DISTORTED =	64		# 0x0040

int16 STATUS_MOVING = 					256		# 0x0100
int16 STATUS_IN_POSITION = 				512		# 0x0200
int16 STATUS_CHECKSUM_ERROR	= 			1024	# 0x0400
int16 STATUS_UNKNOWN_COMMAND = 			2048	# 0x0800
int16 STATUS_EXCEEDED_REG_RANGE	= 		4096	# 0x1000
int16 STATUS_GARBAGE_DETECTED = 		8192	# 0x2000
int16 STATUS_TORQUE_ON = 				16384	# 0x4000