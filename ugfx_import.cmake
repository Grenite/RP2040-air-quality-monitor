include(ugfx/cmake/Findugfx.cmake)
include_directories("ugfx/")
add_subdirectory(drivers)
target_include_directories(
	ugfx
    INTERFACE
    ugfx/drivers/gdisp/ILI9341
)
