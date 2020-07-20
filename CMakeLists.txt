project(k4a-starter)
cmake_minimum_required(VERSION 2.8)
find_package(k4a)
add_executable(fastpcd
    main.cpp)
target_link_libraries(fastpcd PRIVATE 
    k4a::k4a)