idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_adc esp_driver_gpio esp_driver_i2c esp_event esp_http_server esp_netif esp_timer esp_wifi nvs_flash
)

