/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include <cstdlib>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_video_init.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ppa.h"
#include <cmath>
extern "C" {
#include "app_lcd.h"
#include "app_video.h"
}

#include "coco_pose.hpp"

#define ALIGN_UP(num, align)    (((num) + ((align) - 1)) & ~((align) - 1))

// ---------- 模型输入尺寸（请根据实际模型调整） ----------
#define MODEL_INPUT_W  360
#define MODEL_INPUT_H  280
// 注意：如果摄像头输出是 RGB565 且模型输入也要求 RGB565，则改为 *2；否则保持 *3（RGB888）
#define MODEL_INPUT_BYTES (MODEL_INPUT_W * MODEL_INPUT_H * 3)   // RGB888

static const char *TAG = "app_main";

static esp_lcd_panel_handle_t display_panel;
static ppa_client_handle_t ppa_srm_handle = NULL;
static size_t data_cache_line_size = 0;
static void *lcd_buffer[EXAMPLE_LCD_BUF_NUM];

// 摄像头单独缓冲区（避免与 LCD 共用）
static void *camera_buf[EXAMPLE_CAM_BUF_NUM];

// 模型输入预分配缓冲区（全局静态，避免每帧 malloc）
static uint8_t *model_buf = NULL;
static size_t model_buf_aligned_size = 0;

#if CONFIG_EXAMPLE_ENABLE_PRINT_FPS_RATE_VALUE
static int fps_count;
static int64_t start_time;
#endif

static COCOPose *pose = nullptr;
static QueueHandle_t inference_queue = NULL;

// 前向声明
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index,
                                         uint32_t camera_buf_hes, uint32_t camera_buf_ves,
                                         size_t camera_buf_len, void *user_data);
static void inference_task(void *arg);

extern "C" void app_main(void)
{
    // 1. 初始化 LCD
    ESP_ERROR_CHECK(app_lcd_init(&display_panel));

    // 2. 初始化 PPA
    ppa_client_config_t ppa_srm_config = {};
    ppa_srm_config.oper_type = PPA_OPERATION_SRM;
    ESP_ERROR_CHECK(ppa_register_client(&ppa_srm_config, &ppa_srm_handle));
    ESP_ERROR_CHECK(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size));

    // 3. 初始化摄像头
    esp_err_t ret = app_video_main(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "video main init failed with error 0x%x", ret);
        return;
    }

    // 4. 打开摄像头设备
    // 强制转换以消除 const char* 警告（如函数原型为 char*，可改头文件）
    int video_cam_fd0 = app_video_open((char *)EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    if (video_cam_fd0 < 0) {
        ESP_LOGE(TAG, "video cam open failed");
        return;
    }

    // 5. 获取 LCD 帧缓冲区（仅用于显示）
#if EXAMPLE_LCD_BUF_NUM == 2
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(display_panel, 2,
                    &lcd_buffer[0], &lcd_buffer[1]));
#else
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(display_panel, 3,
                    &lcd_buffer[0], &lcd_buffer[1], &lcd_buffer[2]));
#endif

    // 6. 为摄像头单独分配缓冲区（非零拷贝，避免 PPA 输入输出冲突）
    size_t cam_buf_size = app_video_get_buf_size();
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        camera_buf[i] = heap_caps_aligned_calloc(data_cache_line_size, 1,
                                                 cam_buf_size, MALLOC_CAP_SPIRAM);
        if (camera_buf[i] == NULL) {
            ESP_LOGE(TAG, "Failed to allocate camera buffer %d", i);
            return;
        }
    }
    ESP_ERROR_CHECK(app_video_set_bufs(video_cam_fd0, EXAMPLE_CAM_BUF_NUM,
                                        (const void **)camera_buf));

    // 7. 预分配模型输入缓冲区（一次分配，重复使用）
    model_buf_aligned_size = ALIGN_UP(MODEL_INPUT_BYTES, data_cache_line_size);
    model_buf = (uint8_t *)heap_caps_aligned_alloc(data_cache_line_size,
                                                   model_buf_aligned_size,
                                                   MALLOC_CAP_SPIRAM);
    if (model_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate model input buffer");
        return;
    }
    ESP_LOGI(TAG, "Model buffer allocated (size %zu bytes)", model_buf_aligned_size);

    // 8. 初始化姿态估计模型（加载量化 .espdl）
    pose = new COCOPose();
    if (pose == nullptr) {
        ESP_LOGE(TAG, "Failed to create COCOPose instance");
        return;
    }
    ESP_LOGI(TAG, "COCOPose model loaded (quantized)");

    // 9. 创建推理队列
    inference_queue = xQueueCreate(1, sizeof(uint8_t *));
    if (inference_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create inference queue");
        return;
    }

    // 10. 创建推理任务
    xTaskCreatePinnedToCore(inference_task, "inference", 16384, NULL, 5,
                            NULL, 1);

    // 11. 注册帧回调
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(camera_video_frame_operation));

    // 12. 启动摄像头流
    ESP_ERROR_CHECK(app_video_stream_task_start(video_cam_fd0, 0, NULL));

#if CONFIG_EXAMPLE_ENABLE_PRINT_FPS_RATE_VALUE
    start_time = esp_timer_get_time();
#endif
}

// ---------- 帧回调：显示 + PPA 缩放 + 入队 ----------
static void camera_video_frame_operation(uint8_t *camera_buf, uint8_t camera_buf_index,
                                         uint32_t camera_buf_hes, uint32_t camera_buf_ves,
                                         size_t camera_buf_len, void *user_data)
{
    // 1. 显示到 LCD（使用 LCD 帧缓冲区，与 camera_buf 独立）
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(display_panel, 0, 0,
                    camera_buf_hes, camera_buf_ves, camera_buf));

#if CONFIG_EXAMPLE_ENABLE_PRINT_FPS_RATE_VALUE
    fps_count++;
    if (fps_count == 50) {
        int64_t end_time = esp_timer_get_time();
        ESP_LOGI(TAG, "display fps: %f", 1000000.0 / ((end_time - start_time) / 50.0));
        start_time = end_time;
        fps_count = 0;
    }
#endif

    // 2. 配置 PPA 缩放（输入 camera_buf，输出 model_buf）
    // 注意：输入和输出 buffer 必须不同
    ppa_srm_oper_config_t srm_config = {};
    srm_config.in.buffer = camera_buf;
    srm_config.in.pic_w = camera_buf_hes;
    srm_config.in.pic_h = camera_buf_ves;
    srm_config.in.block_w = camera_buf_hes;
    srm_config.in.block_h = camera_buf_ves;
    srm_config.in.block_offset_x = 0;
    srm_config.in.block_offset_y = 0;
    // 输入色彩模式：根据 APP_VIDEO_FMT 设置（如果摄像头输出 RGB565 则选 RGB565，否则 RGB888）
    srm_config.in.srm_cm = (APP_VIDEO_FMT == APP_VIDEO_FMT_RGB565) ?
                            PPA_SRM_COLOR_MODE_RGB565 : PPA_SRM_COLOR_MODE_RGB888;

    srm_config.out.buffer = model_buf;
    srm_config.out.buffer_size = model_buf_aligned_size;
    srm_config.out.pic_w = MODEL_INPUT_W;
    srm_config.out.pic_h = MODEL_INPUT_H;
    srm_config.out.block_offset_x = 0;
    srm_config.out.block_offset_y = 0;
    // 输出色彩模式：模型要求 RGB888，因此强制设为 RGB888
    // 如果 PPA 支持从 RGB565 到 RGB888 的转换，则自动完成；否则需调整
    srm_config.out.srm_cm = PPA_SRM_COLOR_MODE_RGB888;

    srm_config.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
    srm_config.scale_x = (float)MODEL_INPUT_W / camera_buf_hes;
    srm_config.scale_y = (float)MODEL_INPUT_H / camera_buf_ves;
    srm_config.rgb_swap = 0;
    srm_config.byte_swap = 0;
    srm_config.mode = PPA_TRANS_MODE_BLOCKING;

    esp_err_t err = ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PPA scaling failed: 0x%x", err);
        return;
    }

    // 3. 将缩放后的帧指针（model_buf）发送给推理任务（队列容量1，覆盖旧帧）
    xQueueOverwrite(inference_queue, &model_buf);
}

// ---------- 独立推理任务（仅输出三个欧拉角） ----------
static void inference_task(void *arg)
{
    uint8_t *buf = NULL;

    while (1) {
        if (xQueueReceive(inference_queue, &buf, portMAX_DELAY) == pdTRUE) {
            if (pose != nullptr && buf != nullptr) {
                dl::image::img_t img = {};
                img.data      = buf;
                img.width     = MODEL_INPUT_W;
                img.height    = MODEL_INPUT_H;
                img.pix_type  = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

                auto &results = pose->run(img);

                for (const auto &res : results) {
                    // 提取鼻子、左眼、右眼坐标
                    int nose_x = res.keypoint[2 * 0];
                    int nose_y = res.keypoint[2 * 0 + 1];
                    int lex = res.keypoint[2 * 1];
                    int ley = res.keypoint[2 * 1 + 1];
                    int rex = res.keypoint[2 * 2];
                    int rey = res.keypoint[2 * 2 + 1];

                    // 检查关键点有效性（必须鼻子和双眼都有效）
                    if (!(nose_x > 0 && nose_y > 0 && lex > 0 && ley > 0 && rex > 0 && rey > 0)) {
                        continue;   // 静默跳过
                    }

                    // 计算双眼中心和瞳距
                    float cx_eye = (lex + rex) / 2.0f;
                    float cy_eye = (ley + rey) / 2.0f;
                    float eye_dist = std::sqrt((rex - lex) * (rex - lex) + (rey - ley) * (rey - ley));
                    if (eye_dist < 1.0f) continue;   // 防止除零

                    float dx = nose_x - cx_eye;
                    float dy = nose_y - cy_eye;

                    // 计算 Yaw 和 Pitch
                    float yaw_rad = std::atan2(dx, eye_dist);
                    float pitch_rad = std::atan2(dy, eye_dist);
                    float yaw = yaw_rad * 180.0f / M_PI;
                    float pitch = pitch_rad * 180.0f / M_PI;

                    // 限幅（可选）
                    if (yaw > 60.0f) yaw = 60.0f;
                    if (yaw < -60.0f) yaw = -60.0f;
                    if (pitch > 40.0f) pitch = 40.0f;
                    if (pitch < -40.0f) pitch = -40.0f;

                    // 计算 Roll
                    float roll = std::atan2(rey - ley, rex - lex) * 180.0f / M_PI;
                    if (roll > 90.0f) {
                        roll -= 180.0f;
                    } else if (roll < -90.0f) {
                        roll += 180.0f;
                    }

                    // 仅输出三个角度
                    printf("%.1f %.1f %.1f\n", yaw, pitch, roll);
                }   
            }
        }
    }
}