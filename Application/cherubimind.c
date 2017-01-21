#include "cherubimind.h"
#include "app_timer.h"
#include "ble_err.h"
#include "ble_gatts.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "pca10040.h"		// uart pins are defined here.
#include "SEGGER_RTT.h"
#include "nrf_timer.h"
#include "nrf_drv_timer.h"
#include "nrf_saadc.h"
#include "nrf_drv_saadc.h"
#include "nrf_ppi.h"
#include "nrf_drv_ppi.h"

APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                     /**< Heart rate measurement timer. */

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
//static uint32_t              m_adc_evt_counter;

void timers_init(void)
{

    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create battery measurement timer.
		uint32_t err_code;
		err_code = app_timer_create(&m_battery_timer_id, APP_TIMER_MODE_REPEATED, battery_measurement_timeout_handler);
		APP_ERROR_CHECK(err_code); 
	
		// Create HR measurement timer.
		err_code = app_timer_create(&m_heart_rate_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                heart_rate_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting timers.
 */
void application_timers_start(void)
{
		uint32_t err_code;
		err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code); 
	
		err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}

void battery_measurement_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);
    battery_level_update();
		//SEGGER_RTT_WriteString(0, "battery_measurement_timeout \n");
}

static void heart_rate_meas_timeout_handler(void * p_context)
{
		rri_update();
		hrm_update();
}

void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud57600
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void uart_event_handle(app_uart_evt_t * p_event)
{
		uint32_t err_code;
		static eeg_parser_state_t eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
		static int8_t small_package_latency_counter = 0;
		static int8_t data_counter = 0;
		static uint8_t eeg_data_array[EEG_DATA_ARRAY_LENGTH] = {0xAA, 0xAA, 16, 0x80, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
		static uint8_t eeg_data_index = EEG_DATA_ARRAY_START_INDEX;
		static uint8_t send_buffer[BLE_NUS_MAX_DATA_LEN];
		static uint8_t send_buffer_index = 0;
		static uint8_t current_data= 0xff;
		static bool is_big_package = false;
		static bool is_small_package = false;
		static uint8_t payload_length = 0;
		switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
						UNUSED_VARIABLE(app_uart_get(&current_data));
						switch (eeg_parser_state)
						{
							case EEG_PARSER_STATE_FIRST_AA:
									if ( current_data == EEG_DATA_SYNC_BYTE )
											eeg_parser_state = EEG_PARSER_STATE_SECOND_AA;
									break;
							case EEG_PARSER_STATE_SECOND_AA:
									if ( current_data == EEG_DATA_SYNC_BYTE )
											eeg_parser_state = EEG_PARSER_STATE_PAYLOAD_LENGTH;
									else
											eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
									break;
							case EEG_PARSER_STATE_PAYLOAD_LENGTH:
									//if ( current_data == EEG_DATA_SYNC_BYTE )
									//		eeg_parser_state = EEG_PARSER_STATE_PAYLOAD_LENGTH;
									//else if ( current_data > EEG_DATA_SYNC_BYTE )
									//		eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
									//else
									{
											eeg_parser_state = EEG_PARSER_STATE_PAYLOAD;
											payload_length = current_data;
											if (payload_length == EEG_DATA_SMALL_PACKAGE_PAYLOAD_LENGTH)
											{
													is_small_package = true;
													//SEGGER_RTT_WriteString(0, "small package received\n");
											}
											else if (payload_length == EEG_DATA_BIG_PACKAGE_PAYLOAD_LENGTH)
											{
													is_big_package = true;
													//SEGGER_RTT_WriteString(0, "big package received\n");
													send_buffer[send_buffer_index++] = EEG_DATA_SYNC_BYTE;
													send_buffer[send_buffer_index++] = EEG_DATA_SYNC_BYTE;
													send_buffer[send_buffer_index++] = EEG_DATA_BIG_PACKAGE_PAYLOAD_LENGTH;
											}
											else
													eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
									}
									break;
							case EEG_PARSER_STATE_PAYLOAD:
									if ( is_big_package && payload_length >0 )
									{
											send_buffer[send_buffer_index++] = current_data;
											payload_length--;
											if (payload_length == 0)
													eeg_parser_state = EEG_PARSER_STATE_CHECKSUM;
											if ((send_buffer_index >= (BLE_NUS_MAX_DATA_LEN)))
											{
													err_code = data_send_by_nus(send_buffer, send_buffer_index);
													//SEGGER_RTT_WriteString(0, "ble_nus_string_send (big_package, buffer is full) \n");
													if (err_code != NRF_ERROR_INVALID_STATE)
													{
															APP_ERROR_CHECK(err_code);
													}
													send_buffer_index = 0;
											}
									}
									else if ( is_small_package )
									{
											if (( small_package_latency_counter < SMALL_PACKAGE_LATENCY ) ) 	// || current_data != EEG_DATA_RAW_DATA_T)
											{
													small_package_latency_counter++;
													is_small_package = false;
													eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
											}
											else
											{
													small_package_latency_counter = 0;
													eeg_parser_state = EEG_PARSER_STATE_DATA_LENGTH;
											}
									}
									break;
							case EEG_PARSER_STATE_DATA_LENGTH:
									if ( current_data == EEG_DATA_RAW_DATA_BYTE_LENGTH )
											eeg_parser_state = EEG_PARSER_STATE_DATA_HIGH_BYTE;
									else
									{
											eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
											is_small_package = false;
									}
									break;
							case EEG_PARSER_STATE_DATA_HIGH_BYTE:
									eeg_data_array[eeg_data_index++] = current_data;
									eeg_parser_state = EEG_PARSER_STATE_DATA_LOW_BYTE;
									break;
							case EEG_PARSER_STATE_DATA_LOW_BYTE:
									eeg_data_array[eeg_data_index++] = current_data;
									data_counter++;
									if ( data_counter == MAX_EEG_DATA_IN_BLE_MTU )
									{
											// calculate checksum 
											//for ( eeg_data_index = EEG_DATA_ARRAY_START_INDEX; eeg_data_index < EEG_DATA_ARRAY_LENGTH-1; eeg_data_index++ )
											//		current_data += eeg_data_array[eeg_data_index++];
											//eeg_data_array[EEG_DATA_ARRAY_LENGTH-1] = current_data ^ 0xFFFFFFFF & 0xFF;
											err_code = data_send_by_nus(eeg_data_array, EEG_DATA_ARRAY_LENGTH);
											//SEGGER_RTT_WriteString(0, "ble_nus_string_send (eeg raw data) \n");
											if (err_code != NRF_ERROR_INVALID_STATE)
											{
													APP_ERROR_CHECK(err_code);
											}
											eeg_data_index = EEG_DATA_ARRAY_START_INDEX;
											data_counter = 0;
									}
									eeg_parser_state = EEG_PARSER_STATE_CHECKSUM;
									break;
							case EEG_PARSER_STATE_CHECKSUM:
									if ( is_big_package )
									{
											send_buffer[send_buffer_index++] = current_data;
											is_big_package = false;
											err_code = data_send_by_nus(send_buffer, send_buffer_index);
											//SEGGER_RTT_WriteString(0, "ble_nus_string_send (big_package is over) \n");
											if (err_code != NRF_ERROR_INVALID_STATE)
											{
													APP_ERROR_CHECK(err_code);
											}
											send_buffer_index = 0;
									}
									else if ( is_small_package )
											is_small_package = false;
									eeg_parser_state = EEG_PARSER_STATE_FIRST_AA;
						}
						
            break;

        case APP_UART_COMMUNICATION_ERROR:
						SEGGER_RTT_WriteString(0, "APP_UART_COMMUNICATION_ERROR!\n");
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
						SEGGER_RTT_WriteString(0, "APP_UART_FIFO_ERROR!\n");
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
	
}

void nus_data_handler (ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
		// do nothing... handling the data received from BLE_NUS tx_char
}

static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{

}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

		// PPI initiation
    err_code = nrf_drv_ppi_init();
		//SEGGER_RTT_printf(0, "PPI initiation completed. err_code: %#04x\n" ,err_code);
		APP_ERROR_CHECK(err_code);
	
		// Timer initiation
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);
		//SEGGER_RTT_printf(0, "Timer initiation completed. err_code: %#04x\n" ,err_code);
		
    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, SAADC_INTERVAL);
    nrf_drv_timer_extended_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
    nrf_drv_timer_enable(&m_timer);
		
		// Set PPI channal
    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer, NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);
		//SEGGER_RTT_printf(0, "PPI channal allocated. err_code: %#04x\n" ,err_code);
    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, timer_compare_event_addr, saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
		//SEGGER_RTT_printf(0, "PPI channal set. err_code: %#04x\n" ,err_code);
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
		//SEGGER_RTT_printf(0, "SAADC sampling event enabled. err_code: %#04x\n" ,err_code);
		APP_ERROR_CHECK(err_code);
}


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    static uint8_t pulse_data_array[PULSE_DATA_ARRAY_LENGTH] = {0xAA, 0xAA, 16, 0x64, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
		static uint8_t pulse_data_index = PULSE_DATA_ARRAY_START_INDEX;
		if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

				//SEGGER_RTT_printf(0, "ADC event number: %#04x\n" ,m_adc_evt_counter++);

        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
						//SEGGER_RTT_printf(0, "%#04x\n" ,p_event->data.done.p_buffer[i]);
						pulse_data_array[pulse_data_index++] = (uint8_t) ((p_event->data.done.p_buffer[i] & 0xFF00) >> 8);
						pulse_data_array[pulse_data_index++] = (uint8_t) ((p_event->data.done.p_buffer[i] & 0x00FF) >> 0);
        }
				data_send_by_nus(pulse_data_array, pulse_data_index);
				pulse_data_index = PULSE_DATA_ARRAY_START_INDEX;
    }
}


void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
		NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
		//SEGGER_RTT_printf(0, "SAADC drver initiation completed. err_code: %#04x\n" ,err_code);
		APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
		//SEGGER_RTT_printf(0, "SAADC channel initiation completed. err_code: %#04x\n" ,err_code);
		APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
		//SEGGER_RTT_printf(0, "SAADC buffer1 initiation completed. err_code: %#04x\n" ,err_code);
		APP_ERROR_CHECK(err_code);
	
    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
		//SEGGER_RTT_printf(0, "SAADC buffer2 initiation completed. err_code: %#04x\n" ,err_code);
		APP_ERROR_CHECK(err_code);
	
		//SEGGER_RTT_WriteString(0, "SAADC initiation completed.\n");
}
