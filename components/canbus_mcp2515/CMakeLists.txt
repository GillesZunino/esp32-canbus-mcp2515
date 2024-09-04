set(srcs
    "src/canbus_mcp2515.c"
    "src/canbus_mcp2515_isr.c"
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       LDFRAGMENTS "canbus_mcp2515.lf"
                       REQUIRES "driver esp_driver_gpio")


if(CONFIG_CANBUS_MCP2515_SANITIZER)
    idf_component_get_property(lib canbus_mcp2515 COMPONENT_LIB)
    target_compile_options(${lib} PRIVATE "-fsanitize=undefined" "-fno-sanitize=shift-base")
endif()
