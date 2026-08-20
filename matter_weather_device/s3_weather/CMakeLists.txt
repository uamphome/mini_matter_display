# ESP32-S3 weather front-end for the Thread Border Router board.
# Plain ESP-IDF app (NO esp-matter): it owns Wi-Fi, fetches the forecast from
# Open-Meteo, runs the arrow-key "setup" TUI over USB serial, and streams the
# readings to the on-board ESP32-H2 (Matter/Thread device) over UART.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s3_weather)
