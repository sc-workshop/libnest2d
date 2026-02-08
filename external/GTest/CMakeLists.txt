include(FetchContent)

set(BUILD_GTEST OFF)
set(BUILD_GMOCK OFF)
FetchContent_Declare(
    GTest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG release-1.10.0
    FIND_PACKAGE_ARGS NAMES googletest
)
FetchContent_MakeAvailable(GTest)

if (NOT TARGET GTest::GTest)
    add_library(GTest::GTest ALIAS GTest)
endif()