# ===========================================================================
# ESP-IDF component for the Codec2 vocoder
# ===========================================================================
#
# SETUP (run once before build):
#   cd <project_root>
#   bash components/codec2/fetch_codec2.sh
#
# This builds only the subset of codec2 source files needed for the core
# encode/decode API (no freedv, no OFDM modem, no demo apps).
# ===========================================================================

cmake_minimum_required(VERSION 3.16)

set(CODEC2_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/codec2-src/src")

if(NOT EXISTS "${CODEC2_SRC_DIR}/codec2.c")
    message(WARNING
        "Codec2 source not found at ${CODEC2_SRC_DIR}.\n"
        "Run:  bash components/codec2/fetch_codec2.sh\n"
        "Falling back to μ-law passthrough stub.")
    # Register empty component so the build succeeds without codec2
    idf_component_register()
    return()
endif()

# Core source files required for codec2_create / encode / decode
# NOTE: Only mode 3200 is compiled (CODEC2_MODE_3200_ONLY).
# Other modes and their codebooks are excluded to save ~130 KB.
set(CODEC2_SOURCES
    "${CODEC2_SRC_DIR}/codec2.c"
    "${CODEC2_SRC_DIR}/codec2_fft.c"
    "${CODEC2_SRC_DIR}/lpc.c"
    "${CODEC2_SRC_DIR}/lsp.c"
    "${CODEC2_SRC_DIR}/quantise.c"
    "${CODEC2_SRC_DIR}/pack.c"
    "${CODEC2_SRC_DIR}/sine.c"
    "${CODEC2_SRC_DIR}/nlp.c"
    "${CODEC2_SRC_DIR}/interp.c"
    "${CODEC2_SRC_DIR}/postfilter.c"
    "${CODEC2_SRC_DIR}/phase.c"
    "${CODEC2_SRC_DIR}/ak2lsp.c"
    "${CODEC2_SRC_DIR}/kiss_fft.c"
    "${CODEC2_SRC_DIR}/kiss_fftr.c"
    # Codebooks needed by mode 3200 only
    "${CODEC2_SRC_DIR}/codebookd.c"
)

# Only add dump.c if not compiled as NDEBUG (it can be omitted)
# filter out files that don't exist in this codec2 version
set(CODEC2_FILTERED_SOURCES "")
foreach(src ${CODEC2_SOURCES})
    if(EXISTS "${src}")
        list(APPEND CODEC2_FILTERED_SOURCES "${src}")
    endif()
endforeach()

idf_component_register(
    SRCS            ${CODEC2_FILTERED_SOURCES}
                    "${CMAKE_CURRENT_SOURCE_DIR}/codec2_esp_malloc.c"
    INCLUDE_DIRS    "${CODEC2_SRC_DIR}"
)

# Suppress codec2 warnings that would cause -Werror failures
target_compile_options(${COMPONENT_LIB} PRIVATE
    -D__EMBEDDED__        # Ensure codebook float arrays are 'const' → stored in flash
    -DCODEC2_MODE_3200_ONLY  # Only compile mode 3200 to save memory
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-maybe-uninitialized
    -Wno-sign-compare
    -Wno-float-conversion
    -Wno-implicit-function-declaration
    -Wno-misleading-indentation
    -ffast-math           # Significant speed-up on ESP32 FPU
)

# Tell the wrapper that the real library is present
target_compile_definitions(${COMPONENT_LIB} PUBLIC CONFIG_CODEC2_ENABLED=1)

message(STATUS "Codec2 component configured with ${CMAKE_CURRENT_SOURCE_DIR}/codec2-src")
