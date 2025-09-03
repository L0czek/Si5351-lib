
cmake_minimum_required(VERSION 3.23)
project(Si5351 CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Si5351_c_file ${CMAKE_CURRENT_SOURCE_DIR}/src/si5351.c)
set(Si5351_h_file ${CMAKE_CURRENT_SOURCE_DIR}/inc/si5351.h)

add_library(Si5351 STATIC
    ${Si5351_c_file}
)

function(replace_in_file filepath pattern replacement)
    file(READ "${filepath}" file_content)
    string(REPLACE "${pattern}" "${replacement}" new_content "${file_content}")
    file(WRITE "${filepath}" "${new_content}")
endfunction()

replace_in_file(${Si5351_c_file} "stm32f1x" "${chip_family}")

target_include_directories(Si5351 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

set(Si5351_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/inc PARENT_SCOPE)
