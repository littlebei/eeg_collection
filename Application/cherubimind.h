#include "app_uart.h"
#include "ble_nus.h"
#include "bsp.h"
#include "nrf_saadc.h"

// Macro define used by RTC
#define APP_TIMER_PRESCALER           15     /**< Value of the RTC1 PRESCALER register. (327--99.9Hz, 10ms per tick)*/
#define APP_TIMER_OP_QUEUE_SIZE       16      /**< Size of timer operation queues. */

// Macro define used by RTC interval
#define BATTERY_LEVEL_MEAS_INTERVAL   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */
#define HEART_RATE_MEAS_INTERVAL			APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_200MS      		APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)				// Unit: ms
#define TIMER_INTERVAL_100MS      		APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_120MS      		APP_TIMER_TICKS(120, APP_TIMER_PRESCALER)	
#define TIMER_INTERVAL_60MS      			APP_TIMER_TICKS(60, APP_TIMER_PRESCALER)	
#define TIMER_INTERVAL_MODE_TEST_STATE1      		APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_TEST_STATE2      		APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_TEST_STATE3      		APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_TEST_STATE4      		APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_18MIN_STATE1      	APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_18MIN_STATE2      	APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_18MIN_STATE3     		APP_TIMER_TICKS(630000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_18MIN_STATE4     		APP_TIMER_TICKS(90000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_20MIN_STATE_INTRO		APP_TIMER_TICKS(135000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_20MIN_STATE1      	APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_20MIN_STATE2      	APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_20MIN_STATE3     		APP_TIMER_TICKS(628000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_20MIN_STATE4     		APP_TIMER_TICKS(92000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_25MIN_STATE1     		APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_25MIN_STATE2     		APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_25MIN_STATE3     		APP_TIMER_TICKS(1050000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_25MIN_STATE4     		APP_TIMER_TICKS(90000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_27MIN_STATE_INTRO		APP_TIMER_TICKS(135000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_27MIN_STATE1      	APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_27MIN_STATE2      	APP_TIMER_TICKS(180000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_27MIN_STATE3     		APP_TIMER_TICKS(1048000, APP_TIMER_PRESCALER)
#define TIMER_INTERVAL_MODE_27MIN_STATE4     		APP_TIMER_TICKS(92000, APP_TIMER_PRESCALER)

// Macro define used by UART
#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

// Macro define used by uart_event_handler
typedef enum
{
		EEG_PARSER_STATE_FIRST_AA,
		EEG_PARSER_STATE_SECOND_AA,
		EEG_PARSER_STATE_PAYLOAD_LENGTH,
		EEG_PARSER_STATE_PAYLOAD,
		EEG_PARSER_STATE_DATA_LENGTH,
		EEG_PARSER_STATE_DATA_HIGH_BYTE,
		EEG_PARSER_STATE_DATA_LOW_BYTE,
		EEG_PARSER_STATE_CHECKSUM
} eeg_parser_state_t;

#define SMALL_PACKAGE_LATENCY 		7		// eeg display rate = 512Hz/(1+SMALL_PACKAGE_LATENCY)
#define MAX_EEG_DATA_IN_BLE_MTU 	8
#define EEG_DATA_ARRAY_START_INDEX 	4
#define EEG_DATA_ARRAY_LENGTH 		2*MAX_EEG_DATA_IN_BLE_MTU+EEG_DATA_ARRAY_START_INDEX
#define EEG_DATA_SYNC_BYTE 		0xAA
//#define EEG_DATA_POOR_SIGNAL 	0x02
#define EEG_DATA_RAW_DATA_T 	0x80
//#define EEG_DATA_EEG_POWER_T 	0x83
//#define EEG_DATA_ATTENTION_T 	0x04
//#define EEG_DATA_MEDITATION_T 	0x05
#define EEG_DATA_RAW_DATA_BYTE_LENGTH 				2
#define EEG_DATA_SMALL_PACKAGE_PAYLOAD_LENGTH 	4
#define EEG_DATA_BIG_PACKAGE_PAYLOAD_LENGTH 		32

// Macro define used by nus_data_handler
#define COMMAND_SYNC_BYTE 0xAA
#define COMMAND_TYPE_INTERVENTION_MODE 0x11
#define COMMAND_TYPE_INTERVENTION_CONTROL 0x10
#define COMMAND_VALUE_INTERVENTION_CONTROL_START 0x00
#define COMMAND_VALUE_INTERVENTION_CONTROL_PAUSE 0x01
#define COMMAND_VALUE_INTERVENTION_CONTROL_RESUME 0x02
#define COMMAND_VALUE_INTERVENTION_CONTROL_STOP 0x03

// Macro define used by SAADC
#define SAADC_INTERVAL 2		// sample interval of the pulse signal, saadc_sample_rate = 1000 / SAADC_INTERVAL
#define SAMPLES_IN_BUFFER 8

// Macro define used by pulse
#define MAX_PULSE_DATA_IN_BLE_MTU 8
#define PULSE_DATA_ARRAY_START_INDEX 	4
#define PULSE_DATA_ARRAY_LENGTH 		2*MAX_PULSE_DATA_IN_BLE_MTU+PULSE_DATA_ARRAY_START_INDEX
#define  MAX_VALUE      (4096)   // 12bit resolution, max value is 4096
#define  MAX_RATE_ID     6       // should biger than 5, number of the beats used to calculate the average HR
#define PULSE_LATENCY 1		// pulse_display_rate = saadc_sample_rate / SAMPLES_IN_BUFFER / (1+PULSE_LATENCY)

// Macro define used by intervention
typedef enum
{
		STATE_INTRO,
		STATE0,
		STATE1,
		STATE2,
    STATE3,
		STATE4
} intervention_state_t;
typedef enum
{
		INTERVENTION_MODE_TEST,
		INTERVENTION_MODE_18MIN,
		INTERVENTION_MODE_20MIN,
		INTERVENTION_MODE_25MIN,
		INTERVENTION_MODE_27MIN
} intervention_mode_t;

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void);

/**@brief Function for starting timers.
 */
void application_timers_start(void);
void intervention_state1_start(void);
void intervention_state_intro_start(void);
void battery_measurement_timeout_handler(void * p_context);
void battery_level_update(void);
void uart_init(void);
void uart_event_handle(app_uart_evt_t * p_event);
void nus_data_handler (ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
uint32_t data_send_by_nus(uint8_t * p_string, uint16_t length);
void battery_level_update(void);
static void heart_rate_meas_timeout_handler(void * p_context);
void hrm_update(int heart_rate);
void rri_update(int IBI);
void saadc_sampling_event_init(void);
void saadc_sampling_event_enable(void);
void saadc_init(void);
void led_on(uint32_t led_idx);
void led_off(uint32_t led_idx);
void intervention_init(void);
void stop_current_state_timer(void);
void start_current_state_timer(void);
void bsp_evt_handler(bsp_event_t evt);
static void timer_200ms_timeout_handler(void * p_context);
static void timer_100ms_timeout_handler(void * p_context);
static void timer_120ms_timeout_handler(void * p_context);
static void timer_60ms_timeout_handler(void * p_context);
static void timer_state1_timeout_handler(void * p_context);
static void timer_state2_timeout_handler(void * p_context);
static void timer_state3_timeout_handler(void * p_context);
static void timer_state4_timeout_handler(void * p_context);
static void timer_state_intro_timeout_handler(void * p_context);
void bsp_configuration(void);
void led_init(void);
static void PulseSenosrCal(nrf_saadc_value_t data);
