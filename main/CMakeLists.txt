#idf_component_register(
	#SRCS "wireless.cpp" "LIS2.cpp" "LSM6.cpp" "common.cpp" "AD7680.cpp" "I2CDevice.cpp" "main.cpp" "AD5933.cpp" "Fusion/FusionAhrs.c"
	#INCLUDE_DIRS ""
	#)
set(COMPONENT_SRCDIRS "." "Fusion/")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
