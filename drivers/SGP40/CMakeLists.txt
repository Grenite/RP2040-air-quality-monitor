target_sources(ugfx INTERFACE 
    SGP40.h
    sensirion_arch_config.h
    sensirion_gas_index_algorithm.h
    SGP40.c
    sensirion_gas_index_algorithm.c)
target_include_directories(ugfx INTERFACE .)