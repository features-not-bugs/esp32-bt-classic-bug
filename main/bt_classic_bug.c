#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_hidh.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#define TAG "APP"
#define PIN_PAIR_BTN 0
#define MAX_PENDING_DEVICES 10
#define MAX_CONNECTED_DEVICES 2
static esp_bd_addr_t pending_addrs[MAX_PENDING_DEVICES] = {};
static bool pending_flags[MAX_PENDING_DEVICES] = {};
static esp_hidh_dev_t *connected_handles[MAX_CONNECTED_DEVICES] = {};
static bool connected_flags[MAX_CONNECTED_DEVICES] = {};
static uint8_t s_btn_prev = 0;
static bool s_discovering = false;

esp_hidh_dev_t *find_by_addr(const esp_bd_addr_t *bda)
{
    for (int i = 0; i < MAX_CONNECTED_DEVICES; i++)
    {
        if (!connected_flags[i] || !connected_handles[i])
            continue;

        const uint8_t *dev_bda = esp_hidh_dev_bda_get(connected_handles[i]);
        if (dev_bda && memcmp(dev_bda, bda, 6) == 0)
            return connected_handles[i];
    }
    return NULL;
}

bool is_pending_or_connected(const esp_bd_addr_t *bda)
{
    if (find_by_addr(bda))
        return true;

    for (int i = 0; i < MAX_PENDING_DEVICES; i++)
    {
        if (pending_flags[i] && memcmp(pending_addrs[i], bda, 6) == 0)
            return true;
    }
    return false;
}

void set_pending(const esp_bd_addr_t *bda)
{
    for (int i = 0; i < MAX_PENDING_DEVICES; i++)
    {
        if (!pending_flags[i])
        {
            memcpy(pending_addrs[i], bda, 6);
            pending_flags[i] = true;
            return;
        }
    }
}

void clear_pending(const esp_bd_addr_t *bda)
{
    for (int i = 0; i < MAX_PENDING_DEVICES; i++)
    {
        if (pending_flags[i] && memcmp(pending_addrs[i], bda, 6) == 0)
        {
            pending_flags[i] = false;
            return;
        }
    }
}

void set_connected(const esp_hidh_dev_t *dev)
{
    for (int i = 0; i < MAX_CONNECTED_DEVICES; i++)
    {
        if (!connected_flags[i])
        {
            connected_handles[i] = dev;
            connected_flags[i] = true;
            return;
        }
    }
}

void clear_connected(const esp_hidh_dev_t *dev)
{
    for (int i = 0; i < MAX_CONNECTED_DEVICES; i++)
    {
        if (connected_flags[i] && connected_handles[i] == dev)
        {
            connected_flags[i] = false;
            return;
        }
    }
}

void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_data_t *ev = (esp_hidh_event_data_t *)event_data;
    esp_hidh_event_t event = (esp_hidh_event_t)id;

    switch (event)
    {
    case ESP_HIDH_OPEN_EVENT:
    {
        if (ev->open.status != ESP_OK)
        {
            ESP_LOGI(TAG, "HIDH open failed: 0x%04lx", (long)ev->open.status);
            break;
        }

        esp_hidh_dev_t *dev = ev->open.dev;
        set_connected(dev);
        uint16_t vid = esp_hidh_dev_vendor_id_get(dev);
        uint16_t pid = esp_hidh_dev_product_id_get(dev);
        ESP_LOGI(TAG, "HID device opened: VID=%04X PID=%04X", vid, pid);
        break;
    }

    case ESP_HIDH_CLOSE_EVENT:
    {
        esp_hidh_dev_t *dev = ev->close.dev;
        clear_connected(dev);
        ESP_LOGI(TAG, "HID device closed");
        break;
    }

    default:
        break;
    }
}

void gap_callback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BT_GAP_DISC_RES_EVT:
    {
        esp_bt_cod_t cod = {};
        for (int i = 0; i < param->disc_res.num_prop; i++)
        {
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_COD)
            {
                memcpy(&cod, param->disc_res.prop[i].val, sizeof(cod));
                break;
            }
        }

        if (cod.major == ESP_BT_COD_MAJOR_DEV_PERIPHERAL &&
            (cod.minor == ESP_BT_COD_MINOR_PERIPHERAL_GAMEPAD ||
             cod.minor == ESP_BT_COD_MINOR_PERIPHERAL_JOYSTICK))
        {
            if (!is_pending_or_connected(&param->disc_res.bda))
            {
                ESP_LOGI(TAG, "Gamepad found, opening HID connection");
                set_pending(&param->disc_res.bda);
                esp_hidh_dev_open(param->disc_res.bda, ESP_HID_TRANSPORT_BT, 0);
            }
        }
        break;
    }

    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Auth complete: %s", param->auth_cmpl.device_name);
        else
            ESP_LOGI(TAG, "Auth failed: status=%d", param->auth_cmpl.stat);
        break;
    }

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
    {
        const uint8_t *bda = param->acl_conn_cmpl_stat.bda;
        clear_pending(&param->disc_res.bda);
        ESP_LOGI(TAG, "ACL connected: %02X:%02X:%02X:%02X:%02X:%02X status=0x%02X", bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_conn_cmpl_stat.stat);
        break;
    }

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
    {
        ESP_LOGI(TAG, "ACL disconnected: reason=0x%02X", param->acl_disconn_cmpl_stat.reason);
        clear_pending(&param->disc_res.bda);
        break;
    }

    case ESP_BT_GAP_CFM_REQ_EVT:
    {
        ESP_LOGI(TAG, "SSP confirm request (auto-confirming)");
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    }

    case ESP_BT_GAP_PIN_REQ_EVT:
    {
        ESP_LOGI(TAG, "PIN request (replying 0000)");
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }

    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
    {
        ESP_LOGI(TAG, "Discovery state: %s", param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED ? "started" : "stopped");
        break;
    }

    default:
        break;
    }
}

static void periodic_timer_cb(void *arg)
{
    (void)arg;
    bool btn_now = gpio_get_level((gpio_num_t)PIN_PAIR_BTN);

    /* Detect falling edge (HIGH -> LOW = pressed, active LOW) */
    if (s_btn_prev && !btn_now)
    {
        if (!s_discovering)
        {
            ESP_LOGI(TAG, "Pairing button pressed -- starting discovery\n");
            esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);
            s_discovering = true;
        }
        else
        {
            ESP_LOGI(TAG, "Pairing button pressed -- stopping discovery\n");
            esp_bt_gap_cancel_discovery();
            s_discovering = false;
        }
    }

    s_btn_prev = btn_now;
}

void app_main(void)
{
    /* NVS — required for Bluedroid bonding storage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Release BLE memory (Classic only) */
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);

    /* BT controller */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);

    /* Bluedroid */
    esp_bluedroid_init();
    esp_bluedroid_enable();

    /* Raise log level for BT subsystems we need to trace */
    esp_log_level_set("BT_BTC", ESP_LOG_DEBUG);
    esp_log_level_set("BT_HIDH", ESP_LOG_DEBUG);
    esp_log_level_set("BT_APPL", ESP_LOG_DEBUG);

    /* Device name */
    esp_bt_gap_set_device_name("BT Classic Bug Demo");

    /* GAP: register callback + SSP config */
    esp_bt_gap_register_callback(gap_callback);

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(iocap));

    /* HIDH init */
    esp_hidh_config_t hidh_cfg = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
    };
    esp_hidh_init(&hidh_cfg);

    /* Connectable for bonded device reconnection */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

    ESP_LOGI(TAG, "Bluedroid Classic BT initialised");

    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << PIN_PAIR_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&btn_cfg);

    esp_timer_handle_t periodic_timer;
    esp_timer_create_args_t timer_args = {
        .callback = periodic_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "periodic",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, 10000); /* 10ms */

    ESP_LOGI(TAG, "Ready");
}
