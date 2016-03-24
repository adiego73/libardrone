cmake_minimum_required(VERSION 2.8.4)

project(libardrone CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y -fPIC")
set(CMAKE_INCLUDE_PATH ${CMAKE_INCLUDE_PATH} "/opt/boost_1_57_0/include")
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} "/opt/boost_1_57_0/lib")

#set(Boost_USE_STATIC_LIBS   ON)

add_library(ardrone SHARED libardrone.cpp 
                            src/util/util.cpp
                            src/classes/robot_configuration.cpp 
                            src/classes/pid.cpp
                            src/classes/pid_rp.cpp
                            src/classes/pid_y.cpp
                            src/classes/pid_z.cpp 
                            )

add_subdirectory(src)

set(INSTALL_DESTINATION "/home/emiliano/tesis/framework/tesis-ardrone/bin")

set(USER_LIB_PATH "/usr/local/lib")
set(CVDRONE_LIB_PATH "/opt/cvdrone/lib")

find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(./include/ 
                    /usr/local/include 
                    /opt/boost_1_57_0/include 
                    /opt/cvdrone/include )
                    
                    
# pthread
find_library(PTHREAD_LIB NAMES pthread)

# boost
#find_library(BOOST_SYSTEM_LIB NAMES boost_system PATHS ${BOOST_LIB_PATH})
#find_library(BOOST_THREAD_LIB NAMES boost_thread PATHS ${BOOST_LIB_PATH})

# cvdrone
find_library(CVDRONE_LIB NAMES cvdrone PATHS ${CVDRONE_LIB_PATH})

target_link_libraries(ardrone ${PTHREAD_LIB} 
                                 ${Boost_LIBRARIES}
                                 ${CVDRONE_LIB} 
                             )    

install(TARGETS ardrone DESTINATION "${INSTALL_DESTINATION}/lib/"
                        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(FILES ./config/libardrone.json DESTINATION "${INSTALL_DESTINATION}/config/")