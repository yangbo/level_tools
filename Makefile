#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

PROJECT_NAME := level_tools

#这个变量会被 managed_components 中的CMakeLists.txt覆盖，所以没有用
#EXTRA_COMPONENT_DIRS = ./components/SensorLib

include $(IDF_PATH)/make/project.mk
