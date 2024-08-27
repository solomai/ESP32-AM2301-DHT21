idf_component_register(
    SRCS "am2301.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "driver"
    REQUIRES "esp_timer"
)