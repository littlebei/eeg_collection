#include "app_uart.h"
#include "ble_nus.h"

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         10                                           /**< Size of timer operation queues. */

#define BATTERY_LEVEL_MEAS_INTERVAL      APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Battery level measurement interval (ticks). */

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

#define SMALL_PACKAGE_LATENCY 		7
#define MAX_EEG_DATA_IN_BLE_MTU 	8
#define EEG_DATA_ARRAY_LENGTH 		2*MAX_EEG_DATA_IN_BLE_MTU+4
#define EEG_DATA_ARRAY_START_INDEX 	4
#define EEG_DATA_SYNC_BYTE 		0xAA
//#define EEG_DATA_POOR_SIGNAL 	0x02
#define EEG_DATA_RAW_DATA_T 	0x80
//#define EEG_DATA_EEG_POWER_T 	0x83
//#define EEG_DATA_ATTENTION_T 	0x04
//#define EEG_DATA_MEDITATION_T 	0x05
#define EEG_DATA_RAW_DATA_BYTE_LENGTH 				2
#define EEG_DATA_SMALL_PACKAGE_PAYLOAD_LENGTH 	4
#define EEG_DATA_BIG_PACKAGE_PAYLOAD_LENGTH 		32

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
void timers_init(void);

/**@brief Function for starting timers.
 */
void application_timers_start(void);

void battery_measurement_timeout_handler(void * p_context);
void battery_level_update(void);
void uart_init(void);
void uart_event_handle(app_uart_evt_t * p_event);
void nus_data_handler (ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
uint32_t data_send_by_nus(uint8_t * p_string, uint16_t length);
void battery_level_update(void);
