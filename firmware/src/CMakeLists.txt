idf_component_register(
    SRCS "main.cpp" "i2c_soft.cpp" "vl6180x.cpp" "tof050c.cpp" "bola_detector.cpp"
         "kicker_sim.cpp" "kicker_sim_testes.cpp" "kicker_sim_firmware.cpp"
         "autorizacao_chute.cpp" "autorizacao_chute_testes.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_gpio esp_driver_i2c esp_driver_usb_serial_jtag esp_timer
)

