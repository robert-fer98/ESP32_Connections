set(srcs "nimble-server.c"
        "gatt_svr.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS ".")
