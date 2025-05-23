#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "confdata" (robot configuration data).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Rotating axis:
# - The current quadrant of axis 1, 4 and 6,
#   expressed as a positive or negative integer.
# Linear axis:
# - The current meter interval of axis 1, 4 and 6,
#   expressed as a positive or negative integer.
float32 cf1
float32 cf4
float32 cf6

# Rotating axis:
# - For serial link robots, the current robot configuration,
#   expressed as an integer in the range from 0 to 7.
# - For SCARA robots, the current robot configuration,
#   expressed as an integer in the range from 0 to 1.
# - For 7-axis robots, the current robot configuration,
#   expressed as an integer in the range from 0 to 7.
# - For paint robots, the current quadrant of axis 5,
#   expressed as a positive or negative integer.
# - For other robots, using the current quadrant of axis 2,
#   expressed as a positive or negative integer.
# Linear axis:
# - The current meter interval of axis 2,
#   expressed as a positive or negative integer.
float32 cfx
