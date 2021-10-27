#cmake_minimum_required（VERSION 2.8）
project(UserDecoderLib)
include_directories(../decoder)
include_directories(./)

set(CMAKE_GENERATOR "NMake Makefiles" CACHE INTERNAL "" FORCE)
set(CAMKE_BUILD_TYPE RELEASE)

#set(CMAKE_CXX_COMPILER "F:/soft_install/Microsoft Visual Studio/2019/Community/VC/Tools/MSVC/14.29.30133/bin/Hostx86/x86/cl.exe"  CACHE INTERNAL "" FORCE)

#set(CMAKE_C_COMPILER "F:/soft_install/Microsoft Visual Studio/2019/Community/VC/Tools/MSVC/14.29.30133/bin/Hostx86/x86/cl.exe"  CACHE INTERNAL "" FORCE)


file(GLOB core_src  *.c ../decoder/*.c *.cpp ../decoder/*.cpp)
file(GLOB core_hdr *.h ../decoder/*.h)
add_definitions(-DUSERDECODERLIB_EXPORTS -DWIN32 -DUNICODE -D_UNICODE -D_WINDOWS -D_USRDLL -D_WINDLL)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99")  

add_library(${PROJECT_NAME}  SHARED ${core_src} ${core_hdr})
