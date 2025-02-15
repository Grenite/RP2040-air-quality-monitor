target_sources(ugfx INTERFACE 
    board_ILI9341.h
    gdisp_lld_ILI9341.c
    )
target_include_directories(ugfx INTERFACE .)