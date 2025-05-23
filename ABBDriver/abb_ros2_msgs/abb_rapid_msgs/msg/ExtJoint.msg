#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "extjoint" (position of external joints).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# The position of the external logical axis "a"-"f",
# expressed in degrees or mm (depending on the type of axis).
float32 eax_a
float32 eax_b
float32 eax_c
float32 eax_d
float32 eax_e
float32 eax_f
