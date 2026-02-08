add_library(clipperBackend INTERFACE)

require_package(Clipper 6.1 REQUIRED) 
target_link_libraries(clipperBackend INTERFACE Clipper::Clipper)

# Clipper backend is not enough on its own, it still need some functions
# from Boost geometry
require_package(Boost VERSION 1.86 REQUIRED COMPONENTS geometry)

if(Boost_INCLUDE_DIRS_FOUND OR Boost_INCLUDE_DIRS)
    target_include_directories(clipperBackend INTERFACE $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}> )
else()
    if (TARGET Boost::geometry)
        target_link_libraries(clipperBackend INTERFACE Boost::geometry)
    else() 
        target_link_libraries(clipperBackend INTERFACE Boost::geometry)
    endif()
endif()

list(APPEND LIBNEST2D_SRCFILES 
    include/libnest2d/backends/clipper/geometries.hpp
    include/libnest2d/backends/clipper/clipper_polygon.hpp
)
set(LIBNEST2D_SRCFILES ${LIBNEST2D_SRCFILES} PARENT_SCOPE)

#install(TARGETS clipperBackend EXPORT Libnest2DTargets INCLUDES DESTINATION include)
#set(LIBNEST2D_PUBLIC_PACKAGES "${LIBNEST2D_PUBLIC_PACKAGES};Boost;Clipper" CACHE INTERNAL "")
