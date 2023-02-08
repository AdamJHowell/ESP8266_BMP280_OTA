cmake_minimum_required( VERSION 3.24 )
project( ESP8266_BMP280_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( . )

add_executable( ESP8266_BMP280_OTA
		ESP8266_BMP280_OTA.ino
		privateInfo.h )

set_target_properties( ESP8266_BMP280_OTA PROPERTIES LINKER_LANGUAGE CXX )
