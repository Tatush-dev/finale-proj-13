cmake_minimum_required(VERSION 3.16)
project(ImintDrone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(ImintDrone
    src/main.cpp
    src/Model/NavigationModel.cpp
    src/Model/IntelligenceManager.cpp
    src/Model/IntelData.cpp
    src/Utils/SpatialUtilities.cpp
    src/Utils/Base64Codec.cpp
    src/View/MissionView.cpp
    src/Controller/MissionController.cpp
)

target_include_directories(ImintDrone PRIVATE src)
