set(src_dirs        ./)

set(include_dirs    ./)

set(requires        coco_pose
                esp_video)

if (IDF_TARGET STREQUAL "esp32s3")
    list(APPEND requires esp32_s3_eye_noglib
                         esp_lcd)
elseif (IDF_TARGET STREQUAL "esp32p4")
    list(APPEND requires esp32_p4_function_ev_board_noglib
                         esp_lcd)
endif()

idf_build_get_property(component_targets __COMPONENT_TARGETS)
if ("___idf_espressif__esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
elseif("___idf_esp-dl" IN_LIST component_targets)
    idf_component_get_property(espdl_dir esp-dl COMPONENT_DIR)
endif()
include(${espdl_dir}/fbs_loader/cmake/utilities.cmake)

idf_component_register(
    SRCS
         "app_main.cpp"
         "app_lcd.c"
         "app_video.c"
    INCLUDE_DIRS "."
    REQUIRES esp-dl esp_video
)