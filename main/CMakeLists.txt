idf_component_register(SRCS "bt_classic_bug.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver bt esp_timer esp_hid nvs_flash)
