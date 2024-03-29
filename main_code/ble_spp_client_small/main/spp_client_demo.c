/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// Kod baziran na primjeru Espressif-a
// Modifikacije: Robert Medvedec

/****************************************************************************
*
* This file is for ble spp client demo.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>

#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"

// FREERTOS IMPORTS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// INPUT IMPORTS
#include "driver/uart.h"
#include "driver/gpio.h"


// TIMER IMPORTS
#include "esp_timer.h"

#define INPUT_PIN 32
#define LED_PIN 33

//static const char *LED_TAG = "LED";
static const char *TIMER_TAG = "TIMER";

void LED_control_task(void *ledPin);
void timer_callback(void *param);

#define GATTC_TAG                   "BLE_CLIENT"
#define PROFILE_NUM                 1
#define PROFILE_APP_ID              0
#define BT_BD_ADDR_STR              "%02x:%02x:%02x:%02x:%02x:%02x"
#define BT_BD_ADDR_HEX(addr)        addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]
#define ESP_GATT_SPP_SERVICE_UUID   0xABF0
#define SCAN_ALL_THE_TIME           0


///////////////////////////////////////////
/////// STRUKTURE ZA SINKRONIZACIJU ///////
/////// SYNCHORNIZATION STRUCTURES ////////
///////////////////////////////////////////

// [ENG] structure that holds the ata of the message times of the messages that client sends to the server
// [HRV] struktura u koju se zapisuju podatci o vremenima poruka koje se šalju s klijenta na server
struct client_send_message {
    int64_t c_time_send_signal;
    int64_t c_time_send_message;
    int64_t c_time_received_response;
    int64_t c_time_last_message_travel_time;
} client_message;

// [ENG] structure that holds the ata of the message times of the messages that server sends to the client
// [HRV] struktura u koju se zapisuju podatci o vremenima poruka koje se šalju sa servera prema klijentu
struct server_send_message {
    int64_t s_time_received_message;
    int64_t s_time_send_reply;
    int64_t s_time_total_response;
} server_message;

// [ENG] structure that holds the content and the message size of the client to server message
// [HRV] struktura u koju se zapisuje sadržaj i duljina sadržaja poruke koja se šalje s klijenta prema serveru
struct client_basic_message {
    char * client_message;
    uint8_t client_message_length;
} client_basic_message;

// [ENG] structure that holds the data for keyboard input comparisons
// [HRV] struktura u kojoj se nalaze znakovi za usporedbu unosa znakova s tipkovnice
struct keyboard_button_handles {
    uint8_t *uart_compare_o;
} keyboard_buttons;

///////////////////////////////////////////
///////////////////////////////////////////

// [ENG] variables used for time synchonization
// [HRV] varijable korištene za vremensku sinrkonizaciju
bool client_initiated_message = false;
esp_timer_handle_t timer_handler;
int64_t last_client_timer_local_time = 0;
int64_t last_client_LED_local_time = 0;

///////////////////////////////////////////
////////// FUNKCIJE POSTAVLJANJA //////////
///////////// SETUP FUNCTIONS /////////////
///////////////////////////////////////////

void led_setup() {
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT); // has to be INPUT_OUTPUT to be able to read it
    gpio_set_level(LED_PIN, 0);
    //ESP_LOGI(LED_TAG, "Turn the LED on");
}

void client_message_setup() {
    client_basic_message.client_message = NULL;
    client_basic_message.client_message_length = 6;
    client_basic_message.client_message = (char *)malloc(sizeof(char) * client_basic_message.client_message_length);
    char *str = "c_reply";
    memset(client_basic_message.client_message, 0x0, client_basic_message.client_message_length);
    strcpy(client_basic_message.client_message, str);
}

void timer_setup() {
    esp_timer_early_init();
    const esp_timer_create_args_t my_timer_args = {
        .callback = &timer_callback,
        .name = "Timer client"};
    ESP_ERROR_CHECK(esp_timer_create(&my_timer_args, &timer_handler));
}

void keyboard_button_handles_setup(){
    keyboard_buttons.uart_compare_o = (uint8_t *)malloc(sizeof(uint8_t) * 1);
    memset(keyboard_buttons.uart_compare_o, 'o', 1);

}

///////////////////////////////////////////
///////////////////////////////////////////

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;
};

enum{
    SPP_IDX_SVC,

    SPP_IDX_SPP_DATA_RECV_VAL,

    SPP_IDX_SPP_DATA_NTY_VAL,
    SPP_IDX_SPP_DATA_NTF_CFG,
    //SPP_IDX_SPP_DATA_NOTIFY_CHAR,

    SPP_IDX_SPP_COMMAND_VAL,

    SPP_IDX_SPP_STATUS_VAL,
    SPP_IDX_SPP_STATUS_CFG,

    SPP_IDX_NB
};

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

static const char device_name[] = "ESP_SPP_SERVER";
static bool is_connect = false;
static uint16_t spp_conn_id = 0;
static uint16_t spp_mtu_size = 23;
static uint16_t cmd = 0;
static uint16_t spp_srv_start_handle = 0;
static uint16_t spp_srv_end_handle = 0;
static uint16_t spp_gattc_if = 0xff;
static char * notify_value_p = NULL;
static int notify_value_offset = 0;
static int notify_value_count = 0;
static uint16_t count = SPP_IDX_NB;
static esp_gattc_db_elem_t *db = NULL;
static esp_ble_gap_cb_param_t scan_rst;
static QueueHandle_t cmd_reg_queue = NULL;
QueueHandle_t spp_uart_queue = NULL;
bool console_logging = true;
int64_t temp_value;
int64_t temp11;
int64_t temp22;

static esp_bt_uuid_t spp_service_uuid = {
    .len  = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_SPP_SERVICE_UUID,},
};

///////////////////////////////////////////
///////////// POMOĆNE FUNKCIJE ////////////
/////////// AUXILIARY FUNCTIONS ///////////
///////////////////////////////////////////

// [ENG] callback function used for timer task
// [HRV] funkcija koja se koristi kod isteka brojača
void timer_callback(void *param) {
    LED_control_task((void *)LED_PIN);
    last_client_timer_local_time = esp_timer_get_time();
    ESP_LOGE(TIMER_TAG, "\nLast local time %lld\n", last_client_timer_local_time);
    ESP_LOGE(TIMER_TAG, "\nLast LED local time %lld\n", last_client_LED_local_time);
}

// [ENG] start the local timer
// [HRV] pokretanje lokalnog brojača
void timer_start() {
    // enable CONFIG_ESP_TIMER_PROFILING in sdkconfig for more details on timers
    uint64_t timePeriod = 10000000; // in microseconds;
    if (esp_timer_is_active(timer_handler)) {
        ESP_LOGE(TIMER_TAG, "\nUntil next timer event = %lld ms\n", (esp_timer_get_next_alarm() - esp_timer_get_time())/1000);
    }
    else {
        ESP_LOGE(TIMER_TAG, "\nTimer is being activated - period time = %llu ms", timePeriod/1000);
        //LED_control_task((void *)LED_PIN);  
        ESP_ERROR_CHECK(esp_timer_start_periodic(timer_handler, timePeriod)); // period time in microseconds
    }
}

// [ENG] stop the local timer
// [HRV] zaustavljanje lokalnog brojača
void timer_stop() {
    if (esp_timer_is_active(timer_handler)) {
        esp_timer_stop(timer_handler);
    }
}

// [ENG] converts timer value (char array) to int64_t
// time of conversion is around 10 us (irrelevant)
// [HRV] konvertira vrijeme brojača (char array) u int64_t
// vrijeme konverzije je oko 10 us (nebitno)
int64_t char_array_to_timer_value(char *arrayValue)
{
    return strtoll(arrayValue, NULL, 10);
}

// [ENG] converts timer value (int64_t) to char array
// time of conversion from timer to this is around 200 us - we can add this value to the measurement
// [HRV] konvertira vrijeme brojača (int64_t) u char array
// vrijeme konverzije je između 200 us - tu vrijednost možemo dodati na brojač ako je potrebno
char *timer_value_to_char_array(int64_t currentTime, bool addFunctionTime)
{
    temp11 = esp_timer_get_time();
    if (addFunctionTime)
        currentTime += 400;
    char currentTimeCharArray[12];
    itoa(currentTime, currentTimeCharArray, 10);
    char *temp_my = NULL;
    temp_my = (char *)malloc(12);
    // uint8_t sizeOfMy = 12*sizeof(char);
    strcpy(temp_my, currentTimeCharArray);
    temp22 = esp_timer_get_time();
    return temp_my;
}

// [ENG] calculates response time from the client 
// [HRV] računa vrijeme odgovora klijenta
int64_t calculate_client_response_time(){
    return server_message.s_time_total_response = server_message.s_time_send_reply - server_message.s_time_received_message;
}

// [ENG] concatenets local timer value and the response time value ready to be sent to the server
// [HRV] pripreme za slanje tekst koji sadrži lokalno vrijeme brojača i vrijeme odgovora prema serveru
char *timer_and_response_message_reply() {
    char *temp_my = NULL;
    temp_my = (char *)malloc(38);
    server_message.s_time_send_reply = esp_timer_get_time();
    strcpy(temp_my, timer_value_to_char_array(calculate_client_response_time(), false));
    strcat(temp_my, "|");
    strcat(temp_my, timer_value_to_char_array(server_message.s_time_send_reply, false)); // get current time - for now add nothing
    strcat(temp_my, "|");
    strcat(temp_my, timer_value_to_char_array(last_client_LED_local_time,false));
    temp_value = esp_timer_get_time();
    return temp_my;
}


///////////////////////////////////////////
///////////////////////////////////////////

///////////////////////////////////////////
///////////// FUNKCIJE ZADATAKA ///////////
////////////// TASK FUNCTIONS /////////////
///////////////////////////////////////////

// [ENG] function that turns the LED on/off
// [HRV] funkcija paljenja/gašenja LED-ice
void LED_control_task(void *ledPin){ // parameters can be empty
    int led_state = gpio_get_level(LED_PIN);
        if (led_state == 0) {
                gpio_set_level(LED_PIN, 1);
                //ESP_LOGI(TIME_TAG, "TIME After LED turn on - %lu", (unsigned long) (esp_timer_get_time() / 1000ULL));
                //ESP_LOGI(LED_TAG, "Turn the LED on");
            
        }
        else {
                gpio_set_level(LED_PIN, 0);
                //ESP_LOGI(LED_TAG, "Turn the LED off");
        }
    last_client_LED_local_time = esp_timer_get_time();
    //vTaskDelete(NULL);
}


// [ENG] function of receiving the keyboard input data and doing different actions based on different inputs
// [HRV] funkcija zadatka primanja i prepoznavanja pristisnutih tipki s tipkovnice te odrađivanja različitih akcija na temelju istih
void uart_task(void *pvParameters)
{
    uart_event_t event;
    for (;;) {
        //Waiting for UART event.
        if (xQueueReceive(spp_uart_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            client_message.c_time_send_signal = esp_timer_get_time(); // UART singal received
            switch (event.type) {
            //Event of UART receving data
            case UART_DATA:
                if (event.size && (is_connect == true) && (db != NULL) && ((db+SPP_IDX_SPP_DATA_RECV_VAL)->properties & (ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE))) {
                    uint8_t * temp = NULL;
                    temp = (uint8_t *)malloc(sizeof(uint8_t)*event.size);
                    if(temp == NULL){
                        ESP_LOGE(GATTC_TAG, "malloc failed,%s L#%d\n", __func__, __LINE__);
                        break;
                    }
                    
                    memset(temp, 0x0, event.size);
                    uart_read_bytes(UART_NUM_0,temp,event.size,portMAX_DELAY);

                    if (memcmp(temp, keyboard_buttons.uart_compare_o, 1) == 0) { // paljenje/gašenje LOGGING funkcija za vrijeme odgovora poruke - ubrzava odgovor
                        console_logging = !console_logging;
                    }

                    else {
                        LED_control_task((void*) LED_PIN);
                        client_initiated_message = true; // info that the message came from the client
                        int64_t currentTime = esp_timer_get_time(); // get this time and use it - izmjeri koliko je izmedu tog i ovog write charr
                        char *temp_my = timer_value_to_char_array(currentTime, false);
                        client_message.c_time_send_message = esp_timer_get_time();
                        esp_ble_gattc_write_char( spp_gattc_if,
                                              spp_conn_id,
                                              (db+SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                              12*sizeof(char), // size of char array temp my
                                              (uint8_t*) temp_my,
                                              //ESP_GATT_WRITE_TYPE_NO_RSP, // this should shorten the time
                                              ESP_GATT_WRITE_TYPE_RSP,
                                              ESP_GATT_AUTH_REQ_NONE);

                    free(temp_my);
                    }
                    free(temp);

                }
                break;
            default:
                break;
            }
        }
    }
    vTaskDelete(NULL);
}

///////////////////////////////////////////
///////////////////////////////////////////

// [ENG] this function is called when the message is received from the server
// the response message with the timer values is set up
// [HRV] kod primanja poruke od servera poziva se ova funkcija
// ako je server inicirao poruku, zapisat će se vrijeme primanja odgovora na poruku od klijenta 
// ako je poruku inicirao klijent, zapisat će se vrijeme primanja poruke od klijenta i vrijeme trajanja putovanja poruke
static void notify_event_handler(esp_ble_gattc_cb_param_t * p_data)
{
    uint8_t handle = 0;
    if (client_initiated_message == true) {
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Send signal received %lld\n", client_message.c_time_send_signal);
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Send message %lld\n", client_message.c_time_send_message);
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Received response %lld\n", client_message.c_time_received_response);
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Total message travel time %lld us\n", client_message.c_time_last_message_travel_time);
         }

    if (console_logging) { // ovaj dio koda usporava odgovor - može se isključiti postavljanjem varijable logging na 'false' - pritisak tipke O na tipkovnici
        if(p_data->notify.is_notify == true){
            ESP_LOGI(GATTC_TAG,"+NOTIFY:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
        }else{
            ESP_LOGI(GATTC_TAG,"+INDICATE:handle = %d,length = %d ", p_data->notify.handle, p_data->notify.value_len);
        }
    }
    handle = p_data->notify.handle;
    if(db == NULL) {
        ESP_LOGE(GATTC_TAG, " %s db is NULL\n", __func__);
        return;
    }
    if(handle == db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle){
#ifdef SPP_DEBUG_MODE
        esp_log_buffer_char(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
#else
        if((p_data->notify.value[0] == '#')&&(p_data->notify.value[1] == '#')){
            if((++notify_value_count) != p_data->notify.value[3]){
                if(notify_value_p != NULL){
                    free(notify_value_p);
                }
                notify_value_count = 0;
                notify_value_p = NULL;
                notify_value_offset = 0;
                ESP_LOGE(GATTC_TAG,"notify value count is not continuous,%s\n",__func__);
                return;
            }
            if(p_data->notify.value[3] == 1){
                notify_value_p = (char *)malloc(((spp_mtu_size-7)*(p_data->notify.value[2]))*sizeof(char));
                if(notify_value_p == NULL){
                    ESP_LOGE(GATTC_TAG, "malloc failed,%s L#%d\n",__func__,__LINE__);
                    notify_value_count = 0;
                    return;
                }
                memcpy((notify_value_p + notify_value_offset),(p_data->notify.value + 4),(p_data->notify.value_len - 4));
                if(p_data->notify.value[2] == p_data->notify.value[3]){
                    uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    free(notify_value_p);
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
            }else if(p_data->notify.value[3] <= p_data->notify.value[2]){
                memcpy((notify_value_p + notify_value_offset),(p_data->notify.value + 4),(p_data->notify.value_len - 4));
                if(p_data->notify.value[3] == p_data->notify.value[2]){
                    uart_write_bytes(UART_NUM_0, (char *)(notify_value_p), (p_data->notify.value_len - 4 + notify_value_offset));
                    free(notify_value_p);
                    notify_value_count = 0;
                    notify_value_p = NULL;
                    notify_value_offset = 0;
                    return;
                }
                notify_value_offset += (p_data->notify.value_len - 4);
                return;
            }

        } 
        // [HRV] resyncing the timer
        // [HRV] resinkrnonizacija brojača
        else if(p_data->notify.value[0] == 'c') { 
            timer_stop();
            timer_start();
        }
        // [ENG] stopping the timer if 't' is received from the server
        // [HRV] zaustavljanje brojača ukoliko je 't' primljen od servera
        else if(p_data->notify.value[0] == 'u') { // stopping timer on signal
            timer_stop();
        }
        // [ENG] starting the timer if 't' is received from the server
        // [HRV] pokretanje brojača ukoliko je 't' primljen od servera
         else if(p_data->notify.value[0] == 't') {
            //timer_stop();
            //LED_control_task(LED_PIN);
            //last_client_LED_local_time = 0;
            timer_start();
        }
        
        uart_write_bytes(UART_NUM_0, (char *)(p_data->notify.value), p_data->notify.value_len);
        if (client_initiated_message == false) {
            server_message.s_time_send_reply = esp_timer_get_time();
            calculate_client_response_time();

            esp_ble_gattc_write_char( spp_gattc_if,
                                              spp_conn_id,
                                              (db+SPP_IDX_SPP_DATA_RECV_VAL)->attribute_handle,
                                              37, // length of this reply
                                              (uint8_t*) timer_and_response_message_reply(),
                                              //client_basic_message.client_message_length,
                                              //(uint8_t*) client_basic_message.client_message,
                                              ESP_GATT_WRITE_TYPE_NO_RSP, // this should shorten the time
                                              //ESP_GATT_WRITE_TYPE_RSP,
                                              ESP_GATT_AUTH_REQ_NONE);
            ESP_LOGE(TIMER_TAG,"MESSAGE TIMER - Received message %lld\n", server_message.s_time_received_message);
            ESP_LOGE(TIMER_TAG,"MESSAGE TIMER - Send response %lld\n", server_message.s_time_send_reply);
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Total response time %lld us\n", server_message.s_time_total_response);
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Response setup function time %lld us\n", temp_value - server_message.s_time_send_reply);
            ESP_LOGE(TIMER_TAG, "MESSAGE TIMER - Conversion time %lld us\n", temp22 - temp11);
        }
            
        
#endif
    }else if(handle == ((db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle)){
        esp_log_buffer_char(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
    else{
        esp_log_buffer_char(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
}


static void free_gattc_srv_db(void)
{
    is_connect = false;
    spp_gattc_if = 0xff;
    spp_conn_id = 0;
    spp_mtu_size = 23;
    cmd = 0;
    spp_srv_start_handle = 0;
    spp_srv_end_handle = 0;
    notify_value_p = NULL;
    notify_value_offset = 0;
    notify_value_count = 0;
    if(db){
        free(db);
        db = NULL;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    esp_err_t err;

    switch(event){
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        if((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scan param set failed: %s", esp_err_to_name(err));
            break;
        }
        //the unit of the duration is second
        uint32_t duration = 0xFFFF;
        ESP_LOGI(GATTC_TAG, "Enable Ble Scan:during time %04" PRIx32 " minutes.",duration);
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scan start failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scan start successed");
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scan stop failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scan stop successed");
        if (is_connect == false) {
            ESP_LOGI(GATTC_TAG, "Connect to the remote device.");
            esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, scan_rst.scan_rst.bda, scan_rst.scan_rst.ble_addr_type, true);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "Searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
            ESP_LOGI(GATTC_TAG, "\n");
            if (adv_name != NULL) {
                if ( strncmp((char *)adv_name, device_name, adv_name_len) == 0) {
                    memcpy(&(scan_rst), scan_result, sizeof(esp_ble_gap_cb_param_t));
                    esp_ble_gap_stop_scanning();
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }else {
            ESP_LOGI(GATTC_TAG, "Stop adv successfully");
        }
        break;
    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    if (console_logging) {
        ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);
    }

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG EVT, set scan params");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT: conn_id=%d, gatt_if = %d", spp_conn_id, gattc_if);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "disconnect");
        free_gattc_srv_db();
        esp_ble_gap_start_scanning(SCAN_ALL_THE_TIME);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT: start_handle = %d, end_handle = %d, UUID:0x%04x",p_data->search_res.start_handle,p_data->search_res.end_handle,p_data->search_res.srvc_id.uuid.uuid.uuid16);
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "SEARCH_CMPL: conn_id = %x, status %d", spp_conn_id, p_data->search_cmpl.status);
        esp_ble_gattc_send_mtu_req(gattc_if, spp_conn_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG,"Index = %d,status = %d,handle = %d\n",cmd, p_data->reg_for_notify.status, p_data->reg_for_notify.handle);
        if(p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT, status = %d", p_data->reg_for_notify.status);
            break;
        }
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(
                spp_gattc_if,
                spp_conn_id,
                (db+cmd+1)->attribute_handle,
                sizeof(notify_en),
                (uint8_t *)&notify_en,
                ESP_GATT_WRITE_TYPE_NO_RSP,
                //ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);

        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        if (console_logging) {
            ESP_LOGI(GATTC_TAG,"ESP_GATTC_NOTIFY_EVT\n");
        }   
        if (client_initiated_message) {
            client_message.c_time_received_response = esp_timer_get_time(); // getting time when the event came
            client_message.c_time_last_message_travel_time = client_message.c_time_received_response - client_message.c_time_send_message;
        }
        else {
            server_message.s_time_received_message = esp_timer_get_time();
            if ((p_data->notify.value[0] == 'l')) {
                last_client_LED_local_time = last_client_timer_local_time;
            }
            else {
                LED_control_task(LED_PIN);
            }
        } // we use this only when the message comes from a server and it's not a "sync info" message

        notify_event_handler(p_data);
        client_initiated_message = false;
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGI(GATTC_TAG,"ESP_GATTC_READ_CHAR_EVT\n");
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (console_logging) {
            ESP_LOGI(GATTC_TAG,"ESP_GATTC_WRITE_CHAR_EVT:status = %d,handle = %d", param->write.status, param->write.handle);
            if(param->write.status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_CHAR_EVT, error status = %d", p_data->write.status);
                break;
            }
        }
        
        //LED_Control_Task((void*) LED_PIN);
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        break;
    case ESP_GATTC_EXEC_EVT:
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(GATTC_TAG,"ESP_GATTC_WRITE_DESCR_EVT: status =%d,handle = %d \n", p_data->write.status, p_data->write.handle);
        if(p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_DESCR_EVT, error status = %d", p_data->write.status);
            break;
        }
        switch(cmd){
        case SPP_IDX_SPP_DATA_NTY_VAL:
            cmd = SPP_IDX_SPP_STATUS_VAL;
            xQueueSend(cmd_reg_queue, &cmd,10/portTICK_PERIOD_MS);
            break;
        default:
            break;
        };
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if(p_data->cfg_mtu.status != ESP_OK){
            break;
        }
        ESP_LOGI(GATTC_TAG,"+MTU:%d\n", p_data->cfg_mtu.mtu);
        spp_mtu_size = p_data->cfg_mtu.mtu;

        db = (esp_gattc_db_elem_t *)malloc(count*sizeof(esp_gattc_db_elem_t));
        if(db == NULL){
            ESP_LOGE(GATTC_TAG,"%s:malloc db falied\n",__func__);
            break;
        }
        if(esp_ble_gattc_get_db(spp_gattc_if, spp_conn_id, spp_srv_start_handle, spp_srv_end_handle, db, &count) != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"%s:get db falied\n",__func__);
            break;
        }
        if(count != SPP_IDX_NB){
            ESP_LOGE(GATTC_TAG,"%s:get db count != SPP_IDX_NB, count = %d, SPP_IDX_NB = %d\n",__func__,count,SPP_IDX_NB);
            break;
        }
        for(int i = 0;i < SPP_IDX_NB;i++){
            switch((db+i)->type){
            case ESP_GATT_DB_PRIMARY_SERVICE:
                ESP_LOGI(GATTC_TAG,"attr_type = PRIMARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_SECONDARY_SERVICE:
                ESP_LOGI(GATTC_TAG,"attr_type = SECONDARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_CHARACTERISTIC:
                ESP_LOGI(GATTC_TAG,"attr_type = CHARACTERISTIC,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_DESCRIPTOR:
                ESP_LOGI(GATTC_TAG,"attr_type = DESCRIPTOR,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_INCLUDED_SERVICE:
                ESP_LOGI(GATTC_TAG,"attr_type = INCLUDED_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_ALL:
                ESP_LOGI(GATTC_TAG,"attr_type = ESP_GATT_DB_ALL,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            default:
                break;
            }
        }
        cmd = SPP_IDX_SPP_DATA_NTY_VAL;
        xQueueSend(cmd_reg_queue, &cmd, 10/portTICK_PERIOD_MS);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        break;
    default:
        break;
    }
}

void spp_client_reg_task(void* arg)
{
    uint16_t cmd_id;
    for(;;) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY)) {
            if(db != NULL) {
                if(cmd_id == SPP_IDX_SPP_DATA_NTY_VAL){
                    ESP_LOGI(GATTC_TAG,"Index = %d,UUID = 0x%04x, handle = %d \n", cmd_id, (db+SPP_IDX_SPP_DATA_NTY_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                }else if(cmd_id == SPP_IDX_SPP_STATUS_VAL){
                    ESP_LOGI(GATTC_TAG,"Index = %d,UUID = 0x%04x, handle = %d \n", cmd_id, (db+SPP_IDX_SPP_STATUS_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
            }
        }
    }
}

void ble_client_appRegister(void)
{
    esp_err_t status;
    char err_msg[20];

    ESP_LOGI(GATTC_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gap register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    //register the callback function to the gattc module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gattc register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    esp_ble_gattc_app_register(PROFILE_APP_ID);

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(200);
    if (local_mtu_ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed: %s", esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
    }

    cmd_reg_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);

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

    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, 4096, 8192, 10, &spp_uart_queue, 0);
    //Set UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    //Set UART pins
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    xTaskCreate(uart_task, "uTask", 4096, (void*)UART_NUM_0, 8, NULL);
    //xTaskCreate(LED_Control_Task, "ledTask", 2048, (void*)LED_PIN, 1, NULL);
}

void peripheral_setup(){
    led_setup();
    timer_setup();
    client_message_setup();
    keyboard_button_handles_setup();
}

void app_main(void)
{
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTC_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ble_client_appRegister();
    //ESP_LOGE
    spp_uart_init();
    peripheral_setup();

}
