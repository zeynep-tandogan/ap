cmake_minimum_required(VERSION 3.10)
project(DirectVO)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# Optimizasyon bayrakları
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Sophus - ORB_SLAM3'teki Sophus'u kullan
set(SOPHUS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/../ORB_SLAM3/Thirdparty/Sophus)
include_directories(${SOPHUS_INCLUDE_DIR})

# Include dizinleri
include_directories(${PROJECT_SOURCE_DIR}/include)

# Kaynak dosyaları
set(SOURCES
    src/CameraModel.cpp
    src/PointSelector.cpp
    src/DirectTracker.cpp
    src/DepthFilter.cpp
    src/KeyframeManager.cpp
    src/VisualOdometry.cpp
)


add_library(direct_vo SHARED ${SOURCES})
target_link_libraries(direct_vo
    ${OpenCV_LIBS}
    Eigen3::Eigen
)


add_executable(run_vo main.cpp)
target_link_libraries(run_vo
    direct_vo
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

# Install
install(TARGETS run_vo DESTINATION bin)
install(TARGETS direct_vo DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
