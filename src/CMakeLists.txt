########################################################################
# Project setup
########################################################################

cmake_minimum_required(VERSION 2.8)
project(dabtools C CXX)

# Select the release build type by default to get optimization flags
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Debug")
   message(STATUS "Build type not specified: defaulting to debug.")
endif(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

# Set the version information here
set(VERSION_INFO_MAJOR_VERSION 0)
set(VERSION_INFO_API_COMPAT    0)
set(VERSION_INFO_MINOR_VERSION 0)
set(VERSION_INFO_MAINT_VERSION git)


########################################################################
# Compiler specific setup
########################################################################

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_CLANGCC)
    add_definitions(-Wall)
    add_definitions(-Wno-unused)
endif()


########################################################################
# Find build dependencies
########################################################################

find_package(PkgConfig)
# pthreads
find_package(Threads REQUIRED)
# rtlsdr
pkg_check_modules(RTLSDR librtlsdr REQUIRED)
include_directories(${RTLSDR_INCLUDE_DIRS})
# hackrf
pkg_check_modules(HACKRF libhackrf REQUIRED)
include_directories(${HACKRF_INCLUDE_DIRS})
# fftw3
pkg_check_modules(FFTW3 fftw3 REQUIRED )
include_directories(${FFTW3_INCLUDE_DIRS})


########################################################################
# Setup apps
########################################################################

list(APPEND dabtools_sources
    dab.cpp
    dab_tables.c
    depuncture.cpp
    fic.cpp
    input_sdr.c
    input_wf.c
    misc.cpp
    sdr_fifo.c
#    sdr_prstab.c
    sdr_sync.c
    wf_maths.c
    wf_prstables.c
    wf_sync.c
    dab2eti.cpp
    conv/ConvEncoder.cpp
    conv/PuncturingRule.cpp
    conv/PuncturingEncoder.cpp
    conv/SubchannelSource.cpp
    ber.cpp
    )

if(ENABLE_SPIRAL_VITERBI)
    list(APPEND dabtools_sources
        ${dabtools_sources}
        viterbi_spiral.c
        viterbi_spiral_sse16.c
        )
    add_definitions(
        -DENABLE_SPIRAL_VITERBI
        -msse2
        -msse3
        )
else()
    list(APPEND dabtools_sources
        ${dabtools_sources}
        viterbi.c)
endif()

add_executable(dab2eti ${dabtools_sources})
#add_executable(eti2mpa eti2mpa.c)
target_link_libraries(dab2eti m ${CMAKE_THREAD_LIBS_INIT} ${FFTW3_LIBRARIES} ${RTLSDR_LIBRARIES} ${HACKRF_LIBRARIES})

set_property(TARGET dab2eti PROPERTY CXX_STANDARD 11)


install(TARGETS
    dab2eti
    #eti2mpa
    DESTINATION bin
)

