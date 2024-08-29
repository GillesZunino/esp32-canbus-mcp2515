set(srcs
    "src/canbus_mcp2515.c"
    "src/canbus_mcp2515_isr.c"
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       LDFRAGMENTS "canbus_mcp2515.lf"
                       REQUIRES "driver esp_driver_gpio")
