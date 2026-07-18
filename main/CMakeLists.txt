idf_component_register(
    SRCS
        "main.c"
        "power_ui.c"
        "rs485_rx.c"
        "triumph_logo.c"
        "lv_font_montserrat_bold_28.c"
        "lv_font_montserrat_bold_36.c"
        "lv_font_montserrat_bold_44.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        waveshare__esp32_p4_wifi6_touch_lcd_xc)
