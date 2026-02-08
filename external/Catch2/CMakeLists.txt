include(FetchContent)

set(CATCH_INSTALL_DOCS OFF)
set(CATCH_BUILD_TESTING OFF)
set(CATCH_INSTALL_HELPERS ON)
FetchContent_Declare(
    Catch2
    URL "https://github.com/catchorg/Catch2/archive/refs/tags/v2.13.10.zip"
    URL_HASH SHA256=121e7488912c2ce887bfe4699ebfb983d0f2e0d68bcd60434cdfd6bb0cf78b43
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(Catch2)

if (NOT TARGET Catch2::Catch2)
    add_library(Catch2::Catch2 ALIAS Catch2)
endif()