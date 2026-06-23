include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

set(BOOST_INCLUDE_LIBRARIES geometry)
set(BOOST_ENABLE_CMAKE ON)

set (BOOST_VERSION 1.91.0-1)

FetchContent_Declare(
    Boost
    URL https://github.com/boostorg/boost/releases/download/boost-${BOOST_VERSION}/boost-${BOOST_VERSION}-cmake.7z
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    FIND_PACKAGE_ARGS NAMES Boost
)

FetchContent_MakeAvailable(Boost)
