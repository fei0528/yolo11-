#include "coco_pose.hpp"
#include "dl_image_jpeg.hpp"
#include "esp_log.h"
#include <cmath>
#include <vector>

/ LCD 相关头文件
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_ek79007.h"
#include "esp_ldo_regulator.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "driver/ppa.h"
// 视频和摄像头头文件
#include "app_video.h"
#include "app_lcd.h"

#define ALIGN_UP(num, align) (((num) + ((align) - 1)) & ~((align) - 1))

static const char *TAG = "app_main";

static esp_lcd_panel_handle_t display_panel;
static ppa_client_handle_t ppa_srm_handle = NULL;
static size_t data_cache_line_size = 0;
static void *lcd_buffer[2]; // 双缓冲

// 帧处理回调：将摄像头图像写入 LCD
static void camera_video_frame_operation(uint8_t *camera_buf,
    uint8_t camera_buf_index, uint32_t camera_buf_hes,
    uint32_t camera_buf_ves, size_t camera_buf_len, void *user_data)
{
    // 分辨率不匹配时使用 PPA 裁剪缩放
    if (camera_buf_hes > EXAMPLE_LCD_H_RES || camera_buf_ves > EXAMPLE_LCD_V_RES) {
        ppa_srm_oper_config_t srm_config = {
            .in.buffer = camera_buf,
            .in.pic_w = camera_buf_hes,
            .in.pic_h = camera_buf_ves,
            .in.block_w = (camera_buf_hes > EXAMPLE_LCD_H_RES) ? EXAMPLE_LCD_H_RES : camera_buf_hes,
            .in.block_h = (camera_buf_ves > EXAMPLE_LCD_V_RES) ? EXAMPLE_LCD_V_RES : camera_buf_ves,
            .in.block_offset_x = (camera_buf_hes > EXAMPLE_LCD_H_RES) ? (camera_buf_hes - EXAMPLE_LCD_H_RES) / 2 : 0,
            .in.block_offset_y = (camera_buf_ves > EXAMPLE_LCD_V_RES) ? (camera_buf_ves - EXAMPLE_LCD_V_RES) / 2 : 0,
            .in.srm_cm = PPA_SRM_COLOR_MODE_RGB888,
            .out.buffer = lcd_buffer[camera_buf_index],
            .out.buffer_size = ALIGN_UP(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES * 3, data_cache_line_size),
            .out.pic_w = EXAMPLE_LCD_H_RES,
            .out.pic_h = EXAMPLE_LCD_V_RES,
            .out.block_offset_x = 0,
            .out.block_offset_y = 0,
            .out.srm_cm = PPA_SRM_COLOR_MODE_RGB888,
            .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
            .scale_x = 1,
            .scale_y = 1,
            .mode = PPA_TRANS_MODE_BLOCKING,
        };
        ESP_ERROR_CHECK(ppa_do_scale_rotate_mirror(ppa_srm_handle, &srm_config));
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(display_panel, 0, 0,
                        srm_config.in.block_w, srm_config.in.block_h,
                        lcd_buffer[camera_buf_index]));
    } else {
        // 分辨率匹配，直接写入 LCD
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(display_panel, 0, 0,
                        camera_buf_hes, camera_buf_ves, camera_buf));
    }
}

extern "C" void app_main(void)
{
    // ---------- 1. 初始化 LCD ----------
    ESP_ERROR_CHECK(app_lcd_init(&display_panel));

    // ---------- 2. 初始化 PPA（用于图像缩放/裁剪）----------
    ppa_client_config_t ppa_srm_config = {
        .oper_type = PPA_OPERATION_SRM,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_srm_config, &ppa_srm_handle));
    ESP_ERROR_CHECK(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &data_cache_line_size));

    // ---------- 3. 初始化摄像头 ----------
    esp_err_t ret = app_video_main(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "video main init failed: 0x%x", ret);
        return;
    }

    // ---------- 4. 打开摄像头设备 ----------
    int video_cam_fd0 = app_video_open(EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    if (video_cam_fd0 < 0) {
        ESP_LOGE(TAG, "video cam open failed");
        return;
    }

    // ---------- 5. 获取 LCD 帧缓冲区 ----------
    ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(display_panel, 2,
                    &lcd_buffer[0], &lcd_buffer[1]));

    // ---------- 6. 设置摄像头缓冲区 ----------
    ESP_ERROR_CHECK(app_video_set_bufs(video_cam_fd0, EXAMPLE_CAM_BUF_NUM, NULL));

    // ---------- 7. 创建姿态估计模型 ----------
    COCOPose *pose = new COCOPose();

    // ---------- 8. 注册帧处理回调（显示到 LCD）----------
    ESP_ERROR_CHECK(app_video_register_frame_operation_cb(camera_video_frame_operation));

    // ---------- 9. 启动摄像头流任务 ----------
    ESP_ERROR_CHECK(app_video_stream_task_start(video_cam_fd0, 0, NULL));

    // ---------- 3. 主循环：捕获并处理帧 ----------
    while (1) {
        // 获取一帧（阻塞等待）
        esp_video_frame_t *frame = NULL;
        esp_err_t ret = esp_video_get_frame(video_handle, &frame, portMAX_DELAY);
        if (ret != ESP_OK || frame == NULL) {
            ESP_LOGE(TAG, "Get frame failed");
            continue;
        }

        // 将帧数据转换为 dl::image::img_t（要求 RGB888）
        // 注意：若摄像头输出格式不是 RGB888，需先转换（例如使用 ISP 或在代码中转换）
        // 假设已配置 ISP 输出 RGB888，则直接包装
        dl::image::img_t img = {
            .data = (uint8_t*)frame->buffer,
            .width = frame->width,
            .height = frame->height,
            .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888,
            .free_data = false,   // 不由 img 析构释放，由 esp_video_return_frame 释放
        };

        // 运行推理
        auto &pose_results = pose->run(img);

        // 打印结果（与之前相同，但移除 PnP 部分）
        for (const auto &res : pose_results) {
            ESP_LOGI(TAG,
                     "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                     res.score,
                     res.box[0], res.box[1], res.box[2], res.box[3]);

            // 打印所有关键点（可选）
            char log_buf[512];
            char *p = log_buf;
            const char *kpt_names[17] = {...}; // 同前
            for (int i = 0; i < 17; ++i) {
                p += sprintf(p, "%s: [%d, %d] ", kpt_names[i],
                             res.keypoint[2 * i], res.keypoint[2 * i + 1]);
            }
            ESP_LOGI(TAG, "%s", log_buf);
        }

        // 归还帧缓冲区给驱动
        esp_video_return_frame(video_handle, frame);
    }

    // 清理（实际上不会执行到，因为 while 死循环）
    esp_video_stop(video_handle);
    esp_video_deinit(video_handle);
    delete pose;
}