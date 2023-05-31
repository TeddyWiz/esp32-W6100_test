set(srcs "W6100_Test_main.c")

idf_component_register(SRCS ${srcs}
                            "Application/loopback/loopback.c"
                            "W6100/w6100.c"
                            "wizchip_conf.c"
                            "socket.c"
                    INCLUDE_DIRS ".")
