include(FetchContent)

set(CMAKE_POLICY_VERSION_MINIMUM "3.5")
FetchContent_Declare(
    Clipper
    GIT_REPOSITORY https://github.com/tamasmeszaros/libpolyclipping.git
)
FetchContent_MakeAvailable(Clipper)

target_include_directories(polyclipping PUBLIC ${clipper_SOURCE_DIR}) 
add_library(Clipper::Clipper ALIAS polyclipping)
