idf_component_register(SRCS "main.c"
                            "spp_task.c"
                            "neo6m/neo6m.c"
				    "max30100/max30100.c"
                    INCLUDE_DIRS "." "neo6m" "max30100"
                    REQUIRES bt driver freertos nvs_flash)
