set(srcs
    "src/canbus_mcp2515.c"
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "include"
                       REQUIRES "driver")
