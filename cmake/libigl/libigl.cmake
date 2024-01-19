# License: ./LICENSE
# Licensed under: Mozilla Public License 2.0
# Referenced from: https://github.com/libigl/libigl-example-project/blob/8bffa8b1722f937734d06f8762f813cc4022abcd/cmake/libigl.cmake

if(TARGET igl::core)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    libigl
    GIT_REPOSITORY https://github.com/libigl/libigl.git
    GIT_TAG v2.5.0
)

# Note: In libigl v3.0.0, the following will become a one-liner:
FetchContent_MakeAvailable(libigl)