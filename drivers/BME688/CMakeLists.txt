target_sources(ugfx INTERFACE 
    BME680.h
    bme68x_defs.h
    bme68x.h
    BME68XBSEC.h
    bme68x.c
    BME680.c
    BME68XBSEC.c
    )
target_include_directories(ugfx INTERFACE .
    Bosch-BSEC2-Library/src/inc
    )

find_library(algobsec NAMES libalgobsec.a PATHS "${CMAKE_CURRENT_LIST_DIR}/Bosch-BSEC2-Library/src/cortex-m0plus" REQUIRED NO_CMAKE_FIND_ROOT_PATH)
target_link_libraries(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/Bosch-BSEC2-Library/src/cortex-m0plus/libalgobsec.a)