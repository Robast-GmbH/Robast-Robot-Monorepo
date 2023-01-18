Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        ab6c7375be9a8e71ee84c6f8537113f9f47daf99 # should equal tag v3.2.1
)
FetchContent_MakeAvailable(Catch2)
add_executable(test_main test.cpp)
target_link_libraries(test_main PRIVATE Catch2::Catch2 Catch2::Catch2WithMain)
include(CTest)
include(Catch)
catch_discover_tests(test_main)