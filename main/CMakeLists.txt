idf_component_register(
        SRCS "app_main.c" "pn532/task.c" "pn532/532.c"
        "common/dto.c" "uart/tool.c" "uart/parse.c" "mqtt/mqtts.c"
        INCLUDE_DIRS "." "pn532" "common" "uart" "mqtt"
)
