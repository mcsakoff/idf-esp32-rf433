set(COMPONENT_SRCS
        "src/rf433_driver.c"
        "src/rf433_parser.c"
        "src/rf433_pulse_parser.c"
        "src/rf433_nec_parser.c"
        )
set(COMPONENT_ADD_INCLUDEDIRS "include")
set(COMPONENT_PRIV_INCLUDEDIRS "private_include")
register_component()
