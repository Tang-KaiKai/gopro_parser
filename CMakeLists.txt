##################################### 基本编译配置 #####################################

cmake_minimum_required( VERSION 3.16 )
project( gopro_parser )

set( CMAKE_CXX_STANDARD 17 )

set( CMAKE_BUILD_TYPE "Release")
# set( CMAKE_BUILD_TYPE "Debug")
message( "Build type: ${CMAKE_BUILD_TYPE}" )

add_compile_options( -g )  # 编译时添加调试信息

set( CMAKE_CXX_FLAGS_RELEASE "$ENV{CMAKE_CXX_FLAGS} -O3 -Wall" )
set( CMAKE_CXX_FLAGS_DEBUG "$ENV{CMAKE_CXX_FLAGS} -O0 -Wall -g -ggdb" )

include_directories( ${PROJECT_SOURCE_DIR}/gpmf_parser/include )  # 来自官方的 C 代码
include_directories( ${PROJECT_SOURCE_DIR}/include )

# 设置输出目录
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin )


##################################### 包含第三方开源库 #####################################

## FFmpeg
set( FFMPEG_LIB_DIR "/usr/lib/x86_64-linux-gnu" )
set( FFMPEG_INCLUDE_DIRS "/usr/include/x86_64-linux-gnu" )
set( FFMPEG_LIBRARIES "-L${FFMPEG_LIB_DIR} -lavcodec -lavformat -lpostproc -lavdevice -lswresample -lavfilter -lswscale -lavutil" )
string( STRIP "${FFMPEG_LIBRARIES}" FFMPEG_LIBRARIES )
include_directories( ${FFMPEG_INCLUDE_DIRS} )


## OpenCV
find_package( OpenCV 4.2.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
message( "OpenCV_VERSION: ${OpenCV_VERSION}" ) 


##################################### 编译库文件 #####################################

file( GLOB c_src "gpmf_parser/src/*.c" )
file( GLOB cpp_src "src/*.cpp" )

add_library( ${PROJECT_NAME} SHARED ${c_src} ${cpp_src} )
target_link_libraries( ${PROJECT_NAME} PRIVATE ${FFMPEG_LIBRARIES} ${OpenCV_LIBS} )


##################################### 编译可执行文件 #####################################

add_subdirectory( app )



