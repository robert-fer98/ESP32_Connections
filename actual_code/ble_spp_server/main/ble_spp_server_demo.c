/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// FREERTOS IMPORTS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"
#include <inttypes.h>

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "ble_spp_server_demo.h"

// INPUT IMPORTS
#include "driver/gpio.h"
#include "button.h"

// TIMER IMPORTS
#include "esp_timer.h"

#define INPUT_PIN 32
#define LED_PIN 33

static const char *LED_TAG = "LED";
static const char *TIMER_TAG = "TIMER";

#define GATTS_TABLE_TAG "BLE_SERVER"

#define SPP_PROFILE_NUM 1
#define SPP_PROFILE_APP_IDX 0
#define ESP_SPP_APP_ID 0x56
#define SAMPLE_DEVICE_NAME "ESP_SERVER" // The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID 0

#define BLE_SYNC_NO_ADJUST 1
#define BLE_SYNC_STATIC_ADJUST 1
// #define BLE_SYNC_DYNAMIC_ADJUST 1
// #define BLE_SYNC_RESYNC_ADJUST 1
// #define BLE_SYNC_ENABLE_BUTTON 1


/// SPP Service
static const uint16_t spp_service_uuid = 0xABF0;
/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE 0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY 0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE 0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY 0xABF4

static const uint8_t spp_adv_data[23] = {
    /* Flags */
    0x02, 0x01, 0x06,
    /* Complete List of 16-bit Service Class UUIDs */
    0x03, 0x03, 0xF0, 0xAB,
    /* Complete Local Name in advertising */
    0x0F, 0x09, 'E', 'S', 'P', '_', 'S', 'P', 'P', '_', 'S', 'E', 'R', 'V', 'E', 'R'};

static uint16_t spp_mtu_size = 23;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;
QueueHandle_t spp_uart_queue = NULL;
static QueueHandle_t cmd_cmd_queue = NULL;

static bool enable_data_ntf = false;
static bool is_connected = false;
static esp_bd_addr_t spp_remote_bda = {
    0x0,
};

static uint16_t spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t spp_adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

struct function_durations {
    int64_t printf_duration;
    int64_t uart_duration;
    int64_t loge_duration;
    int64_t led_control_duration;
} out_function_durations;

struct server_send_message {
    int64_t s_time_send_signal;
    int64_t s_time_send_message;
    int64_t s_time_received_response;
    int64_t s_time_last_message_travel_time;
} server_message;

struct client_send_message {
    int64_t c_time_received_message;
    int64_t c_time_send_reply;
    int64_t c_time_total_response;
} client_message;

struct server_basic_message {
    char * server_message;
    uint8_t server_message_length;
} server_basic_message;

struct keyboard_button_handles {
    uint8_t *uart_compare_w;
    uint8_t *uart_compare_a;
    uint8_t *uart_compare_s;
    uint8_t *uart_compare_d;
} keyboard_buttons;

bool server_initiated_message = false;

typedef struct spp_receive_data_node
{
    int32_t len;
    uint8_t *node_buff;
    struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;

static spp_receive_data_node_t *temp_spp_recv_data_node_p1 = NULL;
static spp_receive_data_node_t *temp_spp_recv_data_node_p2 = NULL;

typedef struct spp_receive_data_buff
{
    int32_t node_num;
    int32_t buff_size;
    spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;

static spp_receive_data_buff_t SppRecvDataBuff = {
    .node_num = 0,
    .buff_size = 0,
    .first_node = NULL};

button_event_t ev;
QueueHandle_t button_events;

int64_t server_client_clock_difference;

void button_setup()
{
    #ifdef BLE_SYNC_ENABLE_BUTTON
    /*esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT_OUTPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);*/
    printf("AAAAAAAAAA");
    #endif
    // xTaskCreate(button_task, "uTask", 2048, (void*)UART_NUM_0, 8, NULL);
}

void keyboard_button_handles_setup(){
    keyboard_buttons.uart_compare_w = (uint8_t *)malloc(sizeof(uint8_t) * 1);
    keyboard_buttons.uart_compare_a = (uint8_t *)malloc(sizeof(uint8_t) * 1);
    keyboard_buttons.uart_compare_s = (uint8_t *)malloc(sizeof(uint8_t) * 1);
    keyboard_buttons.uart_compare_d = (uint8_t *)malloc(sizeof(uint8_t) * 1);
    memset(keyboard_buttons.uart_compare_w, 'w', 1);
    memset(keyboard_buttons.uart_compare_a, 'a', 1);
    memset(keyboard_buttons.uart_compare_s, 's', 1);
    memset(keyboard_buttons.uart_compare_d, 'd', 1);
}

void led_setup()
{
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT); // has to be INPUT_OUTPUT to be able to read it
    gpio_set_level(LED_PIN, 0);
    // ESP_LOGI(LED_TAG, "Turn the LED on");
}

char *response_extract_clock(char* response) {
    strtok_r(response, "|", &response);
    return strtok(response, "|");
}

char *response_extract_duration(char* response) {
    return strtok(response, "|");
}

void server_message_setup() {
    server_basic_message.server_message = NULL;
    server_basic_message.server_message_length = 6;
    server_basic_message.server_message = (char *)malloc(sizeof(char) * server_basic_message.server_message_length);
    char *str = "s_reply";
    memset(server_basic_message.server_message, 0x0, server_basic_message.server_message_length);
    strcpy(server_basic_message.server_message, str);
}

// this function will take the timer value from the incoming client message and add the value of the server response time to it
// this value should then be used to send back as a response to the client so it can determine the time duration of the response

int64_t calculate_server_client_clock_difference(int64_t clientClock, int64_t clientDuration){
    int64_t tripTime = (server_message.s_time_last_message_travel_time - clientDuration);
    return server_message.s_time_received_response - (tripTime/2 + clientClock);
}

// function duration = TODO
int64_t calculate_server_response_time(int64_t incomingTimer, int64_t serverStart)
{                                                                // this
    return incomingTimer + (esp_timer_get_time() - serverStart); // esp_timer_get_time() is around 1 us so it doesn't really matter
}

int64_t calculate_printf_duration(){
    int64_t currentTime01 = esp_timer_get_time(); 
    printf("PRINTF %lld\n", currentTime01); // this calculation is done for printing 7 characters + timer
    return esp_timer_get_time() - currentTime01;
}

int64_t calculate_loge_duration(){
    int64_t currentTime01 = esp_timer_get_time(); 
    ESP_LOGE("LOGE", "LOGE %lld", currentTime01); // this calculation is done for printing 9 characters + timer
    return esp_timer_get_time() - currentTime01;
}

int64_t calculate_uart_write_bytes_duration(){
    int64_t currentTime01 = esp_timer_get_time(); 
    uart_write_bytes(UART_NUM_0, "uart_write\n", 11); // this calculation is done for printing 10 characters + timer
    return esp_timer_get_time() - currentTime01;
}

void LED_Control_Task(void *ledPin)
{ // parameters can be empty
    int led_state = gpio_get_level(LED_PIN);
    if (led_state == 0)
    {
        gpio_set_level(LED_PIN, 1);
        // ESP_LOGI(TIME_TAG, "TIME After LED turn on - %lu", (unsigned long) (esp_timer_get_time() / 1000ULL));
        //ESP_LOGI(LED_TAG, "Turn the LED on");
    }
    else
    {
        gpio_set_level(LED_PIN, 0);
        //ESP_LOGI(LED_TAG, "Turn the LED off");
    }
    // vTaskDelete(NULL);
}

int64_t calculate_led_control_duration(){
    int64_t currentTime01 = esp_timer_get_time(); 
    LED_Control_Task((void*) LED_PIN);
    return esp_timer_get_time() - currentTime01;
}

// time of conversion from timer to this is around 3.3-4ms (3300-4000 us) = we can add this value to the measurement - 3800
char *timer_value_to_char_array(int64_t currentTime, bool addFunctionTime)
{
    if (addFunctionTime)
        currentTime += 3800;
    char currentTimeCharArray[12];
    itoa(currentTime, currentTimeCharArray, 10);
    char *temp_my = NULL;
    temp_my = (char *)malloc(12);
    // uint8_t sizeOfMy = 12*sizeof(char);
    strcpy(temp_my, currentTimeCharArray);
    return temp_my;
}

// time of conversion is around 10 us
int64_t char_array_to_timer_value(char *arrayValue)
{
    return strtoll(arrayValue, NULL, 10);
}

void timer_callback(void *param) {}

void timer_setup()
{
    // enable CONFIG_ESP_TIMER_PROFILING in sdkconfig for more details on timers
    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "Timer client"};
    esp_timer_handle_t timer_handler;
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, 50)); // play with this - try to get better precision
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
    [SPP_PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

/// SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid = ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[20] = {0x00};

/// SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid = ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[20] = {0x00};
static const uint8_t spp_data_notify_ccc[2] = {0x00, 0x00};

/// SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid = ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t spp_command_val[10] = {0x00};

/// SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid = ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t spp_status_val[10] = {0x00};
static const uint8_t spp_status_ccc[2] = {0x00, 0x00};

/// Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t spp_gatt_db[SPP_IDX_NB] =
    {
        // SPP -  Service Declaration
        [SPP_IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t *)&spp_service_uuid}},

        // SPP -  data receive characteristic Declaration
        [SPP_IDX_SPP_DATA_RECV_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // SPP -  data receive characteristic Value
        [SPP_IDX_SPP_DATA_RECV_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t *)spp_data_receive_val}},

        // SPP -  data notify characteristic Declaration
        [SPP_IDX_SPP_DATA_NOTIFY_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  data notify characteristic Value
        [SPP_IDX_SPP_DATA_NTY_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_data_notify_uuid, ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t *)spp_data_notify_val}},

        // SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_DATA_NTF_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t *)spp_data_notify_ccc}},

        // SPP -  command characteristic Declaration
        [SPP_IDX_SPP_COMMAND_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write}},

        // SPP -  command characteristic Value
        [SPP_IDX_SPP_COMMAND_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t *)spp_command_val}},

        // SPP -  status characteristic Declaration
        [SPP_IDX_SPP_STATUS_CHAR] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

        // SPP -  status characteristic Value
        [SPP_IDX_SPP_STATUS_VAL] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&spp_status_uuid, ESP_GATT_PERM_READ, SPP_STATUS_MAX_LEN, sizeof(spp_status_val), (uint8_t *)spp_status_val}},

        // SPP -  status characteristic - Client Characteristic Configuration Descriptor
        [SPP_IDX_SPP_STATUS_CFG] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_status_ccc), (uint8_t *)spp_status_ccc}},

};

static uint8_t find_char_and_desr_index(uint16_t handle)
{
    uint8_t error = 0xff;

    for (int i = 0; i < SPP_IDX_NB; i++)
    {
        if (handle == spp_handle_table[i])
        {
            return i;
        }
    }

    return error;
}

static bool store_wr_buffer(esp_ble_gatts_cb_param_t *p_data)
{
    temp_spp_recv_data_node_p1 = (spp_receive_data_node_t *)malloc(sizeof(spp_receive_data_node_t));

    if (temp_spp_recv_data_node_p1 == NULL)
    {
        ESP_LOGI(GATTS_TABLE_TAG, "malloc error %s %d\n", __func__, __LINE__);
        return false;
    }
    if (temp_spp_recv_data_node_p2 != NULL)
    {
        temp_spp_recv_data_node_p2->next_node = temp_spp_recv_data_node_p1;
    }
    temp_spp_recv_data_node_p1->len = p_data->write.len;
    SppRecvDataBuff.buff_size += p_data->write.len;
    temp_spp_recv_data_node_p1->next_node = NULL;
    temp_spp_recv_data_node_p1->node_buff = (uint8_t *)malloc(p_data->write.len);
    temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1;
    memcpy(temp_spp_recv_data_node_p1->node_buff, p_data->write.value, p_data->write.len);
    if (SppRecvDataBuff.node_num == 0)
    {
        SppRecvDataBuff.first_node = temp_spp_recv_data_node_p1;
        SppRecvDataBuff.node_num++;
    }
    else
    {
        SppRecvDataBuff.node_num++;
    }

    return true;
}

static void free_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        temp_spp_recv_data_node_p2 = temp_spp_recv_data_node_p1->next_node;
        free(temp_spp_recv_data_node_p1->node_buff);
        free(temp_spp_recv_data_node_p1);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p2;
    }

    SppRecvDataBuff.node_num = 0;
    SppRecvDataBuff.buff_size = 0;
    SppRecvDataBuff.first_node = NULL;
}

static void print_write_buffer(void)
{
    temp_spp_recv_data_node_p1 = SppRecvDataBuff.first_node;

    while (temp_spp_recv_data_node_p1 != NULL)
    {
        uart_write_bytes(UART_NUM_0, (char *)(temp_spp_recv_data_node_p1->node_buff), temp_spp_recv_data_node_p1->len);
        temp_spp_recv_data_node_p1 = temp_spp_recv_data_node_p1->next_node;
    }
}

void uart_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t total_num = 0;
    uint8_t current_num = 0;

    for (;;)
    {
        // Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
        {
            server_message.s_time_send_signal = esp_timer_get_time(); // UART singal received
            server_initiated_message = true; // info that the message came from the server
            switch (event.type)
            {
            // Event of UART receving data
            case UART_DATA:
                if ((event.size) && (is_connected))
                {
                    uint8_t *temp = NULL;
                    uint8_t *ntf_value_p = NULL;

                    if (!enable_data_ntf)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "%s do not enable data Notify\n", __func__);
                        break;
                    }
                    temp = (uint8_t *)malloc(sizeof(uint8_t) * event.size);
                    if (temp == NULL)
                    {
                        ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.1 failed\n", __func__);
                        break;
                    }
                    
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0, temp, event.size, portMAX_DELAY);

                    char *responseArray = timer_value_to_char_array(esp_timer_get_time(), true); // we also add the "static" function calculate time
                    

                    if (event.size <= (spp_mtu_size - 3)) // spp_mtu_size by default is 23
                    {
                        //printf("Send here");

/* DELETE THIS IF STRUCT WORKS
                        uint8_t *uart_compare_a = NULL;
                        uart_compare_a = (uint8_t *)malloc(sizeof(uint8_t) * 1);
                        memset(uart_compare_a, 'a', 1);
*/
                        // if 'w' is pressed on the keyboard, the LED control will be delayed by x amount of ms
                        if (memcmp(temp, keyboard_buttons.uart_compare_w, 1) == 0) {
                            server_message.s_time_send_message = esp_timer_get_time(); // sets the time of the sent message
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 12, (uint8_t *)responseArray, false); 
                            //printf("Send w");
                            vTaskDelay(40 / portTICK_PERIOD_MS); // fixed value
                        } 
                        // if 'a' is pressed on the keyboard, the LED control will be delayed by the previous message travel time divided by 2
                        else if (memcmp(temp, keyboard_buttons.uart_compare_a, 1) == 0) {
                            server_message.s_time_send_message = esp_timer_get_time(); // sets the time of the sent message
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 12, (uint8_t *)responseArray, false); 
                            vTaskDelay((server_message.s_time_last_message_travel_time/(2*1000)) / portTICK_PERIOD_MS);
                            //printf("Send a");
                        } 
                        // if 's' is pressed on the keyboard, the client will reply with the timer value and response time, so the server can calculate
                        // the difference between timers
                        else if (memcmp(temp, keyboard_buttons.uart_compare_s, 1) == 0) {
                            server_message.s_time_send_message = esp_timer_get_time(); // sets the time of the sent message
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 12, (uint8_t *)responseArray, false); 
                            printf("Send s");
                        } 
                        else if (memcmp(temp, keyboard_buttons.uart_compare_d, 1) == 0) {
                            server_message.s_time_send_message = esp_timer_get_time(); // sets the time of the sent message
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], 12, (uint8_t *)responseArray, false); 
                            printf("Send d");
                        } 
                        // if anything else is pressed on the keyboard, that character will be sent
                        else {
                            server_message.s_time_send_message = esp_timer_get_time(); // sets the time of the sent message
                            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], event.size, temp, false);
                            printf("Send 2");
                        }
                        /* this has been transferred to a specific button press*/ /*
                        #ifdef BLE_SYNC_STATIC_ADJUST // umjesto ovoga možemo i stavit neku varijablu koja se aktivira pritiskom na neku tipku
                        vTaskDelay(40 / portTICK_PERIOD_MS); // we delay the LED turning on - we can do this as a separate task, not here, but for now it's okay
                        #endif
                        */
                        LED_Control_Task((void *)LED_PIN); 
                    
                    }
                    else if (event.size > (spp_mtu_size - 3))
                    {
                        //printf("Send here 2");
                        if ((event.size % (spp_mtu_size - 7)) == 0)
                        {
                            total_num = event.size / (spp_mtu_size - 7);
                        }
                        else
                        {
                            total_num = event.size / (spp_mtu_size - 7) + 1;
                        }
                        current_num = 1;
                        ntf_value_p = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                        if (ntf_value_p == NULL)
                        {
                            ESP_LOGE(GATTS_TABLE_TAG, "%s malloc.2 failed\n", __func__);
                            free(temp);
                            break;
                        }
                        while (current_num <= total_num)
                        {
                            if (current_num < total_num)
                            {
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4, temp + (current_num - 1) * (spp_mtu_size - 7), (spp_mtu_size - 7));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], (spp_mtu_size - 3), ntf_value_p, false);
                                printf("Send 3");
                            }
                            else if (current_num == total_num)
                            {
                                ntf_value_p[0] = '#';
                                ntf_value_p[1] = '#';
                                ntf_value_p[2] = total_num;
                                ntf_value_p[3] = current_num;
                                memcpy(ntf_value_p + 4, temp + (current_num - 1) * (spp_mtu_size - 7), (event.size - (current_num - 1) * (spp_mtu_size - 7)));
                                esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], (event.size - (current_num - 1) * (spp_mtu_size - 7) + 4), ntf_value_p, false);
                                printf("Send 4");
                            }
                            vTaskDelay(20 / portTICK_PERIOD_MS);
                            current_num++;
                        }
                        free(ntf_value_p);
                    }
                    free(temp);
                }
                break;
            default:
                break;
            }
        }

        #ifdef BLE_SYNC_ENABLE_BUTTON
        else if (xQueueReceive(button_events, &ev, 1000 / portTICK_PERIOD_MS))
        {
            if ((ev.pin == 32) && (ev.event == BUTTON_DOWN))
            {
                ESP_LOGI("BUTT", "BUTTON");
            }
        }
        #endif
    }
    vTaskDelete(NULL);
}

static void spp_uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_RTS,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    // Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    // Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 2048, (void *)UART_NUM_0, 8, NULL);
}

void spp_cmd_task(void *arg)
{
    uint8_t *cmd_id;

    for (;;)
    {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        if (xQueueReceive(cmd_cmd_queue, &cmd_id, portMAX_DELAY))
        {
            esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(cmd_id), strlen((char *)cmd_id));
            free(cmd_id);
        }
    }
    vTaskDelete(NULL);
}

static void spp_task_init(void)
{
    spp_uart_init();

    cmd_cmd_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_cmd_task, "spp_cmd_task", 2048, NULL, 10, NULL);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;
    ESP_LOGE(GATTS_TABLE_TAG, "GAP_EVT, event %d\n", event);

    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising start failed: %s\n", esp_err_to_name(err));
        }
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t *)param;
    uint8_t res = 0xff;

    ESP_LOGI(GATTS_TABLE_TAG, "event = %x\n", event);
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gap_config_adv_data_raw((uint8_t *)spp_adv_data, sizeof(spp_adv_data));

        ESP_LOGI(GATTS_TABLE_TAG, "%s %d\n", __func__, __LINE__);
        esp_ble_gatts_create_attr_tab(spp_gatt_db, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
        break;
    case ESP_GATTS_READ_EVT:
        res = find_char_and_desr_index(p_data->read.handle);
        if (res == SPP_IDX_SPP_STATUS_VAL)
        {
            // TODO:client read the status characteristic
        }
        break;
    case ESP_GATTS_WRITE_EVT:
    {
        if (server_initiated_message == true) {
            server_message.s_time_received_response = esp_timer_get_time(); // getting time when the event came
            server_message.s_time_last_message_travel_time = server_message.s_time_received_response - server_message.s_time_send_message;
        }
        else {
            client_message.c_time_received_message = esp_timer_get_time();
        }
        res = find_char_and_desr_index(p_data->write.handle);
        //ESP_LOGE(TIMER_TAG, "Event received %lld", incomingEventTime);
        if (p_data->write.is_prep == false)
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_WRITE_EVT : handle = %d\n", res);
            if (!server_initiated_message) {
                LED_Control_Task((void *)LED_PIN);
            }
            /*esp_timer_dump(stdout);
            int64_t currentTime = esp_timer_get_time(); // this function is almost instantaneous - around 1us
            int64_t currentTime2 = esp_timer_get_time();
            int64_t currentTime3 = esp_timer_get_time();
            ESP_LOGE(TIMER_TAG, "Timer 1 %lld",currentTime); // returns time in microseconds since the start of the board (active timer)
            ESP_LOGE(TIMER_TAG, "Timer 2 %lld",currentTime2);
            ESP_LOGE(TIMER_TAG, "Timer 3 %lld",currentTime3);
            char currentTimeCharArray[12];
            itoa(currentTime, currentTimeCharArray, 10); // integer to array
         ESP_LOGE(TIMER_TAG, "%.10s",currentTimeCharArray);*/
         if (server_initiated_message == true) {
            printf("MESSAGE TIMER - Send signal received %lld\n", server_message.s_time_send_signal);
            printf("MESSAGE TIMER - Send message %lld\n", server_message.s_time_send_message);
            printf("MESSAGE TIMER - Received response %lld\n", server_message.s_time_received_response);
            printf("MESSAGE TIMER - Total message travel time %lld us\n", server_message.s_time_last_message_travel_time);
         }


            if (res == SPP_IDX_SPP_COMMAND_VAL)
            {
                uint8_t *spp_cmd_buff = NULL;
                spp_cmd_buff = (uint8_t *)malloc((spp_mtu_size - 3) * sizeof(uint8_t));
                if (spp_cmd_buff == NULL)
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "%s malloc failed\n", __func__);
                    break;
                }
                memset(spp_cmd_buff, 0x0, (spp_mtu_size - 3));
                memcpy(spp_cmd_buff, p_data->write.value, p_data->write.len);
                xQueueSend(cmd_cmd_queue, &spp_cmd_buff, 10 / portTICK_PERIOD_MS);
            }
            else if (res == SPP_IDX_SPP_DATA_NTF_CFG)
            {
                if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x01) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = true;
                }
                else if ((p_data->write.len == 2) && (p_data->write.value[0] == 0x00) && (p_data->write.value[1] == 0x00))
                {
                    enable_data_ntf = false;
                }
            }
            else if (res == SPP_IDX_SPP_DATA_RECV_VAL)
            {
#ifdef SPP_DEBUG_MODE
                esp_log_buffer_char(GATTS_TABLE_TAG, (char *)(p_data->write.value), p_data->write.len);
#else
                uart_write_bytes(UART_NUM_0, "\n", 1);
                uart_write_bytes(UART_NUM_0, (char *)(p_data->write.value), p_data->write.len); // this is where the received input is written out to uart
                char *responseDuration = response_extract_duration((char *) p_data->write.value);
                char *responseClock = response_extract_clock((char *) p_data->write.value);
                printf("Duration string %s\n", responseDuration);
                printf("Clock string %s\n", responseClock);
                
                server_client_clock_difference = calculate_server_client_clock_difference(char_array_to_timer_value(responseClock), char_array_to_timer_value(responseDuration));
                printf("SERVER/CLIENT CLOCK DIFFERENCE = %lld us\n", server_client_clock_difference);
                char *eptr;
                char *chararr = "aa";
                long long result;
                char value[12];
                // strcpy(value, (char *)(p_data->write.value));
                // result = strtoll(value, &eptr, 10);
                result = char_array_to_timer_value((char *)(p_data->write.value));
                //ESP_LOGE(TIMER_TAG, "Client timer is: %lld", result);
                // int64_t currentTime = esp_timer_get_time();
                // int64_t value = calculate_server_response_time(currentTime)
                // printf("Client timer is: \n%lld\n", result); // this will overflow if the esp stays on for too long
                // uint8_t * tempa = NULL;
                // tempa = (uint8_t *)malloc(sizeof(uint8_t)*3);
                // memset(tempa,0x0,3);
                // uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);
                //int64_t serverResponseTime = calculate_server_response_time(result, incomingEventTime);
                int64_t currentTime01 = esp_timer_get_time(); // getting time when the event came

                currentTime01 = esp_timer_get_time();
                
                int64_t currentTime02 = esp_timer_get_time();
                //ESP_LOGE("SERVER RESPONSE TIME", "SERVER TIME %lld", serverResponseTime); // tu mi nije dobra računica
                int64_t currentTime = esp_timer_get_time(); // getting time when the event came
               
                //char *responseArray = timer_value_to_char_array(serverResponseTime, true);
                if (server_initiated_message == false) {
                    client_message.c_time_send_reply = esp_timer_get_time();
                    esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_SPP_DATA_NTY_VAL], server_basic_message.server_message_length, (uint8_t*) server_basic_message.server_message, false);
                    client_message.c_time_total_response = client_message.c_time_send_reply - client_message.c_time_received_message;
                    printf("Send 5");
                    printf("MESSAGE TIMER - Received message %lld\n", client_message.c_time_received_message);
                    printf("MESSAGE TIMER - Send response %lld\n", client_message.c_time_send_reply);
                    printf("MESSAGE TIMER - Total response time %lld us\n", client_message.c_time_total_response);
                }


                server_initiated_message = false;
                // tu znaci mozemo vratiti odmah vrijednost timera i vidjeti koliko "otprilike" treba da se posalje poruka
                uart_write_bytes(UART_NUM_0, "\n", 1);
#endif
            }
            else
            {
                // TODO:
            }
        }
        else if ((p_data->write.is_prep == true) && (res == SPP_IDX_SPP_DATA_RECV_VAL))
        {
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d\n", res);
            store_wr_buffer(p_data);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT\n");
        if (p_data->exec_write.exec_write_flag)
        {
            print_write_buffer();
            free_write_buffer();
        }
        break;
    }
    case ESP_GATTS_MTU_EVT:
        spp_mtu_size = p_data->mtu.mtu;
        break;
    case ESP_GATTS_CONF_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
        spp_conn_id = p_data->connect.conn_id;
        spp_gatts_if = gatts_if;
        is_connected = true;
        memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        is_connected = false;
        enable_data_ntf = false;
        esp_ble_gap_start_advertising(&spp_adv_params);
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "The number handle =%x\n", param->add_attr_tab.num_handle);
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != SPP_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
        }
        else
        {
            memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
            esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
        }
        break;
    }
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "EVT %d, gatts if %d\n", event, gatts_if);

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }

    do
    {
        int idx;
        for (idx = 0; idx < SPP_PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == spp_profile_tab[idx].gatts_if)
            {
                if (spp_profile_tab[idx].gatts_cb)
                {
                    spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void peripheral_setup(){
    led_setup();
    timer_setup();
    out_function_durations.loge_duration = calculate_loge_duration();
    out_function_durations.printf_duration = calculate_printf_duration();
    out_function_durations.uart_duration = calculate_uart_write_bytes_duration();
    server_message_setup();
    keyboard_button_handles_setup();
    
    // button_setup();
    // button_init(32);
    //button_events = button_setup(PIN_BIT(32));
    button_setup(); // finish button setup and callback properly
    // button_init may be an important function
}

void app_main(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTS_TABLE_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);
    esp_ble_gatts_app_register(ESP_SPP_APP_ID);

    spp_task_init();
    peripheral_setup();

    /*
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(INPUT_PIN);
    gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);
    gpio_pulldown_en(INPUT_PIN);
    gpio_pullup_dis(INPUT_PIN);
    gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);

    interputQueue = xQueueCreate(10, sizeof(int));
    // arguments - task, task name, stack size, parameter, priority of the task, handle of the task (pointer)
    xTaskCreate(LED_Control_Task, "LED_Control_Task", 2048, NULL, 2, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);
    */

    return;
}
