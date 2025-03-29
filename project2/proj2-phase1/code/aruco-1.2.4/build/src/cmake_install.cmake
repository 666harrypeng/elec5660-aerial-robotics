# Install script for directory: /root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE FILES
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/src/libaruco.so.1.2.4"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/src/libaruco.so.1.2"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/build/src/libaruco.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so.1.2"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libaruco.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/opt/ros/kinetic/lib/x86_64-linux-gnu:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/aruco" TYPE FILE FILES
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/cameraparameters.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/arucofidmarkers.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/exports.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/marker.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/aruco.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/cvdrawingutils.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/markerdetector.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/boarddetector.h"
    "/root/elec5660_onedrive/projects/ref_proj2phase1/src/aruco-1.2.4/src/board.h"
    )
endif()

