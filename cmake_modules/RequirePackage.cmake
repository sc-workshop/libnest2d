include(CMakeDependentOption)

set(RP_REPOSITORY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external CACHE STRING "Package repository location")
cmake_dependent_option(RP_FORCE_DOWNLOADING "Force downloading packages even if found." OFF "RP_ENABLE_DOWNLOADING" OFF)

macro(require_package RP_ARGS_PACKAGE)    
    set(options REQUIRED QUIET)
    set(oneValueArgs "VERSION")
    set(multiValueArgs "")
    cmake_parse_arguments(RP_ARGS 
        "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT RP_FORCE_DOWNLOADING)
        #message("Finding ${RP_ARGS_PACKAGE} ${RP_ARGS_VERSION}...")
        find_package(${RP_ARGS_PACKAGE} ${RP_ARGS_VERSION} QUIET ${RP_ARGS_UNPARSED_ARGUMENTS})
    endif()
    
    if(NOT ${RP_ARGS_PACKAGE}_FOUND)
        #message("Downloading ${RP_ARGS_PACKAGE} using FetchContent...")
        add_subdirectory(${RP_REPOSITORY_DIR}/${RP_ARGS_PACKAGE} ${RP_ARGS_PACKAGE}) 
        # download_package(${RP_ARGS_PACKAGE} ${RP_ARGS_VERSION} ${_QUIET} ${_REQUIRED} ${RP_ARGS_UNPARSED_ARGUMENTS} )
    endif()
    
endmacro()
