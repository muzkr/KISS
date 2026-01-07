cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME KISS)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C CXX ASM)

add_subdirectory(ext)
add_subdirectory(fat)
add_subdirectory(bl2)
add_subdirectory(app)


# ---------------------------
#       bl2

add_executable(bl2)
target_link_libraries(bl2 BL2)
target_compile_definitions(bl2 PUBLIC $<$<CONFIG:Debug>:DEBUG>)

target_link_options(bl2 PUBLIC "-Wl,-Map=bl2.map")

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(bl2 PROPERTIES ADDITIONAL_CLEAN_FILES bl2.map)

# Print memory size
add_custom_command(
    TARGET bl2
    POST_BUILD
    COMMAND echo "Memory size of bl2.elf: "
    COMMAND ${CMAKE_SIZE} bl2.elf
    # COMMAND ${CMAKE_SIZE} -A bl2.elf
    COMMENT "Print memory size of bl2.elf"
)

# Generate .bin 
add_custom_command(
    TARGET bl2
    POST_BUILD
    COMMAND echo "Generating bl2.bin"
    COMMAND ${CMAKE_OBJCOPY} -O binary bl2.elf bl2.bin
    COMMAND echo "Generating bl2.uf2"
    COMMAND python3 ${CMAKE_SOURCE_DIR}/uf2/utils/uf2conv.py -c -b 0x08002800 -o bl2.uf2 bl2.bin
    COMMENT "Generate bl2.bin"
)

# ---------------------------
#       app

add_executable(app)

# Add linked libraries
target_link_libraries(app App)
target_compile_definitions(app PUBLIC $<$<CONFIG:Debug>:DEBUG>)

target_link_options(app PUBLIC "-Wl,-Map=app.map")

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(app PROPERTIES ADDITIONAL_CLEAN_FILES app.map)

# Print memory size
add_custom_command(
    TARGET app
    POST_BUILD
    COMMAND echo "Memory size of app.elf: "
    COMMAND ${CMAKE_SIZE} app.elf
    # COMMAND ${CMAKE_SIZE} -A app.elf
    COMMENT "Print memory size of app.elf"
)

# Generate .bin 
add_custom_command(
    TARGET app
    POST_BUILD
    COMMAND echo "Generating app.bin"
    COMMAND ${CMAKE_OBJCOPY} -O binary app.elf app.bin
    COMMENT "Generate app.bin"
)

# ---------------------------

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# ---------------------------

# # Generate .uf2
# add_custom_command(
#     TARGET ${CMAKE_PROJECT_NAME}
#     POST_BUILD
#     COMMAND python3 ${CMAKE_SOURCE_DIR}/uf2/utils/uf2conv.py -c -b 0x08002800 -o ${CMAKE_PROJECT_NAME}.uf2 ${CMAKE_PROJECT_NAME}.bin
#     COMMENT "Generate .bin and .uf2"
# )
