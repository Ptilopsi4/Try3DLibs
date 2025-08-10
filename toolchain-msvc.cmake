set(CMAKE_SYSTEM_NAME Windows)

set(CMAKE_C_COMPILER "cl.exe")
set(CMAKE_CXX_COMPILER "cl.exe")

set(CMAKE_SYSTEM_PROCESSOR "x64")
set(CMAKE_SYSROOT "E:/Windows Kits/10")

set(CMAKE_TOOLCHAIN_FILE "E:/DevTools/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

set(Qt6_DIR "E:/Qt/6.6.3/msvc2019_64/lib/cmake/Qt6" CACHE PATH "Path to Qt6 CMake files")