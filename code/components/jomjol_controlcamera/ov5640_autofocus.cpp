#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_camera.h"
#include "ov5640_autofocus.h"
#include "ov5640_autofocus_config.h"

static const char *TAG = "CAM";

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

    rc = sensor->set_reg(sensor, OV5640_CMD_ACK, 0xff, 0x01);
    rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, mode);
    retry = 0;
    do {
        temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
        retry++;
        if (retry > 1000) return 2;
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
        vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (temp != 0x00);
    if (S_Zone_len > 5)
        S_Zone_len = 5;
    for (int i = 0; i < S_Zone_len; i++) {
        S_Zone[i] = sensor->get_reg(sensor, OV5640_CMD_PARA0 + i, 0xff);
    }
    return sensor->get_reg(sensor, OV5640_CMD_FW_STATUS, 0xff);
}

uint8_t ov5640_autofocus_release(sensor_t *sensor)
{
    uint8_t rc = 0;
    uint8_t temp = 0;
    uint16_t retry = 0;
    rc = sensor->set_reg(sensor, OV5640_CMD_ACK, 0xff, 0x01);
    rc = sensor->set_reg(sensor, OV5640_CMD_MAIN, 0xff, AF_RELEASE_FOCUS);
    retry = 0;
    do {
        temp = sensor->get_reg(sensor, OV5640_CMD_ACK, 0xff);
        retry++;
        if (retry > 1000) return 2;
        vTaskDelay(5 / portTICK_PERIOD_MS);
    } while (temp != 0x00);
    return 0;
}

uint8_t ov5640_manual_focus_set(sensor_t *sensor, uint16_t focusLevel)
{
    uint8_t rc = 0;

    uint8_t pd = 0;
    uint8_t slew_rate = 0x04;
    uint16_t vcm_target = focusLevel & 0x03ff;
    uint16_t vcm_rdiv = 0x0605;
    uint8_t vcm_rih = 0;
    uint8_t vcm_ib = 0x07;

    uint8_t vcm_ctrl_0 = ((vcm_target & 0x0f) << 4) | (slew_rate & 0x0f);
    uint8_t vcm_ctrl_1 = ((pd & 0x01) << 7) | ((vcm_target >> 4) & 0x3f);
    uint8_t vcm_ctrl_2 = vcm_rdiv & 0xff;
    uint8_t vcm_ctrl_3 = ((vcm_rih & 0x01) << 4) | ((vcm_rdiv >> 8) & 0x0f);
    uint8_t vcm_ctrl_4 = (vcm_ib & 0x07);

    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_4, 0xff, vcm_ctrl_4);
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_3, 0xff, vcm_ctrl_3);
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_2, 0xff, vcm_ctrl_2);
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_0, 0xff, vcm_ctrl_0);
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_1, 0xff, vcm_ctrl_1);

    // Wait duration depends on slew rate. Our coil supply could be 2.2V, so
    // we add another 100ms to make up for the lower voltage.
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return 0;
}

uint8_t ov5640_manual_focus_release(sensor_t *sensor)
{
    uint8_t rc = 0;

    // Set focus back to infinity and wait for it to physically move before
    // turning off the power to prevent clicking sound caused by the focus lens
    // snapping to the permanent magnet when power is removed.
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_1, 0x3f, 0);
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_0, 0xf0, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    rc = sensor->set_reg(sensor, OV5640_VCM_CTRL_1, 0x80, 1 << 7);

    return 0;
}

void ov5640_print_vcm_registers(sensor_t *sensor)
{
    uint8_t vcm_ctrl_0 = sensor->get_reg(sensor, OV5640_VCM_CTRL_0, 0xff);
    uint8_t vcm_ctrl_1 = sensor->get_reg(sensor, OV5640_VCM_CTRL_1, 0xff);
    uint8_t vcm_ctrl_2 = sensor->get_reg(sensor, OV5640_VCM_CTRL_2, 0xff);
    uint8_t vcm_ctrl_3 = sensor->get_reg(sensor, OV5640_VCM_CTRL_3, 0xff);
    uint8_t vcm_ctrl_4 = sensor->get_reg(sensor, OV5640_VCM_CTRL_4, 0xff);

    uint8_t pwr_down = vcm_ctrl_1 >> 7;
    uint16_t vcm_target = ((uint16_t)(vcm_ctrl_1 & 0x3f) << 4) | ((vcm_ctrl_0 & 0xf0) >> 4);
    uint8_t slew_rate = vcm_ctrl_0 & 0x0f;
    uint16_t vcm_rdiv = ((uint16_t)(vcm_ctrl_3 & 0x0f) << 8) | (vcm_ctrl_2 & 0xff);
    uint8_t vcm_rih = (vcm_ctrl_3 & 0x10) >> 4;
    uint8_t vcm_ib = vcm_ctrl_4 & 0x07;

    ESP_LOGI(TAG, "VCM: pd %x, target 0x%04x, slew 0x%02x, rdiv 0x%04x, rih %x, ib 0x%02x", pwr_down, vcm_target, slew_rate, vcm_rdiv, vcm_rih, vcm_ib);
}
