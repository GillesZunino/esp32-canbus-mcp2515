set(srcs
    "src/canbus_mcp2515_logtag.c"
    "src/canbus_mcp2515_handle.c"
    "src/canbus_mcp2515_instructions.c"
    "src/canbus_mcp2515_registers.c"

    "src/canbus_mcp2515.c"
    "src/canbus_mcp2515_isr.c"
    "src/canbus_mcp2515_task.c"
    "src/canbus_mcp2515_log_utils.c"
    "src/canid_mcp2515_coding_private.c"
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       LDFRAGMENTS "canbus_mcp2515.lf"
                       REQUIRES "driver esp_driver_gpio")


if(CONFIG_MCP2515_SANITIZER)
    idf_component_get_property(lib canbus_mcp2515 COMPONENT_LIB)
    target_compile_options(${lib} PRIVATE "-fsanitize=undefined" "-fno-sanitize=shift-base")
endif()
