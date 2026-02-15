cmake_minimum_required(VERSION 3.20)
project(gbaemu LANGUAGES C ASM_NASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

option(GBAEMU_ENABLE_ASAN "Enable AddressSanitizer for host C code" OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED sdl2)

add_library(gba_core STATIC
    src/asm/core.asm
    src/asm/cpu/step.asm
    src/asm/ppu/scanline.asm
    src/asm/apu/mixer.asm
    src/asm/sys/timer.asm
    src/host/host_bridge.c
)

target_include_directories(gba_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${SDL2_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src/asm
)

target_compile_options(gba_core PRIVATE
    $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra -Wpedantic>
)

add_executable(gbaemu
    src/host/main.c
)

target_include_directories(gbaemu PRIVATE ${SDL2_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(gbaemu PRIVATE gba_core ${SDL2_LIBRARIES} m)

target_compile_options(gbaemu PRIVATE -Wall -Wextra -Wpedantic)

if(GBAEMU_ENABLE_ASAN)
  target_compile_options(gba_core PRIVATE
      $<$<COMPILE_LANGUAGE:C>:-fsanitize=address -fno-omit-frame-pointer>
  )
  target_compile_options(gbaemu PRIVATE -fsanitize=address -fno-omit-frame-pointer)
  target_link_options(gbaemu PRIVATE -fsanitize=address)
endif()

include(CTest)
if(BUILD_TESTING)
  add_test(
    NAME headless_smoke
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tests/smoke.sh
            $<TARGET_FILE:gbaemu>
            ${CMAKE_CURRENT_SOURCE_DIR}/gba_bios.bin
            ${CMAKE_CURRENT_SOURCE_DIR}/SuperMarioAdvance.gba
  )
endif()
