set(SOURCES main.cpp ../espAds1x.cpp)
idf_component_register(SRCS ${SOURCES}
                      INCLUDE_DIRS . ../include
                      REQUIRES nvs_flash esp_timer driver)
