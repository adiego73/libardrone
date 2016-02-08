cmake_minimum_required(VERSION 2.8.4)

project(libardrone)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")

add_library(ardrone SHARED libardrone.cpp 
                            src/message_client.cpp 
                            src/classes/pid.cpp
                            src/classes/pid_rp.cpp
                            src/classes/pid_y.cpp
                            src/classes/pid_z.cpp 
                            )

add_subdirectory(src)

set(USER_LIB_PATH "/usr/local/lib")
set(BOOST_LIB_PATH "/opt/boost_1_57_0/lib")
set(CVDRONE_LIB_PATH "/opt/cvdrone/lib")

include_directories(./include/ 
                    /usr/local/include 
                    /opt/boost_1_57_0/include 
                    /opt/cvdrone/include )
                    
                    
# pthread
find_library(PTHREAD_LIB NAMES pthread)

# boost
find_library(BOOST_THREAD_LIB NAMES boost_thread PATHS ${BOOST_LIB_PATH})
find_library(BOOST_SYSTEM_LIB NAMES boost_system PATHS ${BOOST_LIB_PATH})

# cvdrone
find_library(CVDRONE_LIB NAMES cvdrone PATHS ${CVDRONE_LIB_PATH})

# ffmpeg
find_library(AVDEVICE_LIB NAMES avdevice PATHS ${USER_LIB_PATH})
find_library(AVFILTER_LIB NAMES avfilter PATHS ${USER_LIB_PATH})
find_library(SWSCALE_LIB NAMES swscale PATHS ${USER_LIB_PATH})
find_library(AVFORMAT_LIB NAMES avformat PATHS ${USER_LIB_PATH})
find_library(AVCODEC_LIB NAMES avcodec PATHS ${USER_LIB_PATH})
find_library(XCB_SHM_LIB NAMES xcb-shm PATHS ${USER_LIB_PATH})
find_library(XCB_LIB NAMES xcb PATHS ${USER_LIB_PATH})
find_library(X11_LIB NAMES X11 PATHS ${USER_LIB_PATH})
find_library(SDL_LIB NAMES SDL PATHS ${USER_LIB_PATH})
find_library(BZ2_LIB NAMES bz2 PATHS ${USER_LIB_PATH})
find_library(Z_LIB NAMES z PATHS ${USER_LIB_PATH})
find_library(AVUTIL_LIB NAMES avutil PATHS ${USER_LIB_PATH})
find_library(RT_LIB NAMES rt PATHS ${USER_LIB_PATH})
find_library(M_LIB NAMES m PATHS ${USER_LIB_PATH})

target_link_libraries(ardrone ${PTHREAD_LIB} 
                                 ${OPENCV_CORE_LIB} 
                                 ${OPENCV_HIGHGUI_LIB} 
                                 ${OPENCV_IMGPROC_LIB} 
                                 ${OPENCV_VIDEO_LIB}
                                 ${BOOST_THREAD_LIB} 
                                 ${BOOST_SYSTEM_LIB} 
                                 ${CVDRONE_LIB} 
                                 ${AVDEVICE_LIB} 
                                 ${AVUTIL_LIB} 
                                 ${AVFILTER_LIB} 
                                 ${SWSCALE_LIB} 
                                 ${AVFORMAT_LIB} 
                                 ${AVCODEC_LIB} 
                             )    