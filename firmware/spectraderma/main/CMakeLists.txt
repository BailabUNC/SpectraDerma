idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG")