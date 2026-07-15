idf_component_register(
    SRCS
        "main.cpp"
        "app_video.c"
        "app_lcd.c"
    INCLUDE_DIRS "."
    EMBED_FILES
        "yolo11n_pose.espdl"   # ← 正确！放在 EMBED_FILES 里
)