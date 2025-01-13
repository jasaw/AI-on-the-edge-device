#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include "esp_camera.h"
#include "ov5640_autofocus.h"
#include "ov5640_autofocus_config.h"


static bool is_ov5640(sensor_t *sensor)
{
    uint8_t vid, pid;
    vid = sensor->get_reg(sensor, OV5640_CHIPID_HIGH, 0xff);
    pid = sensor->get_reg(sensor, OV5640_CHIPID_LOW, 0xff);

    return (vid == 0x56) && (pid == 0x40);
}

int ov5640_autofocus_init(sensor_t *sensor)
{
    if (is_ov5640(sensor))
    {
        uint16_t i;
        uint16_t addr = 0x8000;
        uint8_t state = 0x8F;
        uint8_t rc = 0;
        rc = sensor->set_reg(sensor, OV5640_SYSTEM_RESET00, 0xff, 0x20);  // reset MCU
        if (rc < 0) return -1;

        for (i = 0; i < sizeof(OV5640_AF_Config); i++) {
            rc = sensor->set_reg(sensor, addr, 0xff, OV5640_AF_Config[i]);
            if (rc < 0) return -1;
            addr++;
            if ((i & 0x7f) == 0x7f) {
                vTaskDelay(1); // to avoid watchdog trigger
            }
        }

        sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_ACK, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_PARA0, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_PARA1, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_PARA2, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_PARA3, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_PARA4, 0xff, 0x00);
        sensor->set_reg(sensor, OV5640_CMD_FW_STATUS, 0xff, 0x7f);
        sensor->set_reg(sensor, OV5640_SYSTEM_RESET00, 0xff, 0x00);

        i = 0;
        do {
            state = sensor->get_reg(sensor, OV5640_CMD_FW_STATUS, 0xff);
            // vTaskDelay(pdMS_TO_TICKS(5));
            vTaskDelay(5 / portTICK_PERIOD_MS);
            i++;
            if (i > 1000) return 1;
        } while (state != FW_STATUS_S_IDLE);

        return 0;
    }
    return -1;
}

uint8_t ov5640_autofocus_set_mode(sensor_t *sensor, uint8_t mode)
{
    if ((mode != AF_TRIG_SINGLE_AUTO_FOCUS) && (mode != AF_CONTINUE_AUTO_FOCUS) && (mode != AF_LOCK_FOCUS))
        return -1;

    uint8_t rc = 0;
    uint8_t temp = 0;
    uint16_t retry = 0;

    // Note: The MCU will auto clear CMD_MAIN to zero after the command is receipt, and auto clear
    // CMD_ACK to zero when the command is completed.

    // rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, 0x01);
    // rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, 0x08);
    // do {
    //     temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
    //     retry++;
    //     if (retry > 1000) return 1;
    //     // vTaskDelay(pdMS_TO_TICKS(5));
    //     vTaskDelay(5 / portTICK_PERIOD_MS);
    // } while (temp != 0x00);
    rc = sensor->set_reg(sensor, OV5640_CMD_ACK, 0xff, 0x01);
    rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, mode);
    retry = 0;
    do {
        temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
        retry++;
        if (retry > 1000) return 2;
        // vTaskDelay(pdMS_TO_TICKS(5));
        // vTaskDelay(5 / portTICK_PERIOD_MS);
        vTaskDelay(30 / portTICK_PERIOD_MS);
    } while (temp != 0x00);
    return 0;
}

// returns FW_STATUS_S_FOCUSED or FW_STATUS_S_FOCUSING
uint8_t ov5640_autofocus_get_status(sensor_t *sensor, uint8_t *S_Zone, int S_Zone_len)
{
    uint8_t rc = 0;
    uint8_t temp = 0;
    uint16_t retry = 0;
    rc = sensor->set_reg(sensor, OV5640_CMD_ACK, 0xff, 0x01);
    rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, 0x07);
    retry = 0;
    do {
        temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
        retry++;
        if (retry > 1000) return 2;
        // vTaskDelay(pdMS_TO_TICKS(5));
        vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (temp != 0x00);
    if (S_Zone_len > 5)
        S_Zone_len = 5;
    for (int i = 0; i < S_Zone_len; i++) {
        S_Zone[i] = sensor->get_reg(sensor, OV5640_CMD_PARA0 + i, 0xff);
    }
    // for (int i = 0; i < S_Zone_len; i++) {
    //     if (S_Zone[i] == 0) {
    //         // focused
    //         return FW_STATUS_S_FOCUSED;
    //     }
    // }
    // // focus failed
    // return FW_STATUS_S_FOCUSING;
    return sensor->get_reg(sensor, OV5640_CMD_FW_STATUS, 0xff);
}

uint8_t ov5640_release_autofocus(sensor_t *sensor)
{
    uint8_t rc = 0;
    uint8_t temp = 0;
    uint16_t retry = 0;
    // rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, 0x01);
    // rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, 0x08);
    // do {
    //     temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
    //     retry++;
    //     if (retry > 1000) return 1;
    //     // vTaskDelay(pdMS_TO_TICKS(5));
    //     vTaskDelay(5 / portTICK_PERIOD_MS);
    // } while (temp != 0x00);
    rc = sensor->set_reg(sensor, OV5640_CMD_ACK, 0xff, 0x01);
    rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, AF_RELEASE_FOCUS);
    retry = 0;
    do {
        temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
        retry++;
        if (retry > 1000) return 2;
        // vTaskDelay(pdMS_TO_TICKS(5));
        vTaskDelay(5 / portTICK_PERIOD_MS);
        // vTaskDelay(30 / portTICK_PERIOD_MS);
    } while (temp != 0x00);
    return 0;
}
