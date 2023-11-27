# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/juans/esp/esp-idf/components/bootloader/subproject"
  "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader"
  "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix"
  "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix/tmp"
  "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix/src"
  "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/juans/OneDrive/Documents/PlatformIO/Projects/Tamagotchi/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
