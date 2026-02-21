# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Extra component directories (Codec2 vocoder)
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(walkie_talkie)

# Reduce BT SDP discovery buffer from 8000→4096 bytes.
# HFP-AG only needs to discover HFP-HF service; 4 KB is plenty.
idf_build_set_property(COMPILE_DEFINITIONS "BTA_DM_SDP_DB_SIZE=4096" APPEND)
