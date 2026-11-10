# ESP32-S31-Korvo-1 DVP 카메라 웹 스트리밍 (ESP-IDF v6.1)
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# main 컴포넌트와 그 의존 컴포넌트만 빌드한다.
idf_build_set_property(MINIMAL_BUILD ON)
project(s31_dvp_webcam)
