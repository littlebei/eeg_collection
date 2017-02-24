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
#include "boards.h"
#include "bsp.h"

// APP_TIMER
APP_TIMER_DEF(m_battery_timer_id);                        /**< Battery timer. */
APP_TIMER_DEF(m_heart_rate_timer_id);                     /**< Heart rate measurement timer. */
APP_TIMER_DEF(m_timer_200ms_id); 
APP_TIMER_DEF(m_timer_100ms_id); 
APP_TIMER_DEF(m_timer_120ms_id); 
APP_TIMER_DEF(m_timer_60ms_id); 
APP_TIMER_DEF(m_timer_state1_id); 
APP_TIMER_DEF(m_timer_state2_id); 
APP_TIMER_DEF(m_timer_state3_id); 
APP_TIMER_DEF(m_timer_state4_id);

// SAADC
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
//static uint32_t              m_adc_evt_counter;

// intervention
static intervention_state_t intervention_state = STATE0;
static intervention_mode_t intervention_mode = INTERVENTION_MODE_TEST;
static uint32_t timer_interval_state1 = TIMER_INTERVAL_MODE_TEST_STATE1;
static uint32_t timer_interval_state2 = TIMER_INTERVAL_MODE_TEST_STATE2;
static uint32_t timer_interval_state3 = TIMER_INTERVAL_MODE_TEST_STATE3;
static uint32_t timer_interval_state4 = TIMER_INTERVAL_MODE_TEST_STATE4;

// pulse detection
static int BPM;                   // used to hold the pulse rate
static int Signal;                // holds the incoming raw data
static int IBI = 500;             // holds the time between beats, the Inter-Beat Interval
static bool Pulse = false;     // true when pulse wave is high, false when it's low
//static bool QS = false;        // becomes true when finds a beat.

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
	
		// Create intervention timer.
		err_code = app_timer_create(&m_timer_200ms_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_200ms_timeout_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_timer_100ms_id,
                                APP_TIMER_MODE_REPEATED,
                                timer_100ms_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_timer_120ms_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_120ms_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_create(&m_timer_60ms_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_60ms_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_timer_state1_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_state1_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_timer_state2_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_state2_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_timer_state3_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_state3_timeout_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_timer_state4_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_state4_timeout_handler);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for starting timers.
 */
void application_timers_start(void)
{
		uint32_t err_code;
		
		// battery service timer start
		err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code); 
		
		// heart rate service timer start
		err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);

}

void intervention_timers_start(void)
{
		uint32_t err_code;
		// intervention timer start
		intervention_state = STATE1;
		led_off(0);
		led_off(1);
		SEGGER_RTT_WriteString(0, "Intervention state1...\n");
		err_code = app_timer_start(m_timer_100ms_id, TIMER_INTERVAL_100MS, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_timer_state1_id, timer_interval_state1, NULL);
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
		hrm_update(BPM);
}

static void timer_200ms_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		led_off(0);
		uint32_t err_code;
		err_code = app_timer_start(m_timer_120ms_id, TIMER_INTERVAL_120MS, NULL);
		APP_ERROR_CHECK(err_code);
}
static void timer_100ms_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		switch (intervention_state)
		{
			case STATE1:
				led_off(0);
				break;
			case STATE4:
				led_off(1);
				break;
			default:
				break;
		} 
		
		uint32_t err_code;
		err_code = app_timer_start(m_timer_60ms_id, TIMER_INTERVAL_60MS, NULL); 
		APP_ERROR_CHECK(err_code);
}
static void timer_120ms_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    led_on(0);
}
static void timer_60ms_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		switch (intervention_state)
		{
			case STATE1:
				led_on(0);
				break;
			case STATE4:
				led_on(1);
				break;
			default:
				break;
		}
}

static void timer_state1_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		intervention_state = STATE2;
    uint32_t err_code;
		
		SEGGER_RTT_WriteString(0, "Intervention state2...\n");
	
		err_code = app_timer_stop(m_timer_100ms_id);
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_timer_state2_id, timer_interval_state2, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_timer_200ms_id, TIMER_INTERVAL_200MS, NULL);
    APP_ERROR_CHECK(err_code);
		led_off(0);
		led_off(1);
}
static void timer_state2_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		intervention_state = STATE3;
    uint32_t err_code;
		
		SEGGER_RTT_WriteString(0, "Intervention state3...\n");
	
		err_code = app_timer_stop(m_timer_200ms_id);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_stop(m_timer_120ms_id);
		APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_timer_state3_id, timer_interval_state3, NULL);
    APP_ERROR_CHECK(err_code);
		led_off(0);
		led_off(1);
}
static void timer_state3_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
		intervention_state = STATE4;
    uint32_t err_code;
		
		SEGGER_RTT_WriteString(0, "Intervention state4...\n");
	
		err_code = app_timer_start(m_timer_100ms_id, TIMER_INTERVAL_100MS, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_timer_state4_id, timer_interval_state4, NULL);
    APP_ERROR_CHECK(err_code);
		led_off(0);
		led_off(1);
}
static void timer_state4_timeout_handler(void * p_context)
{
		UNUSED_PARAMETER(p_context);
		intervention_state = STATE0;
		uint32_t err_code;
		err_code = app_timer_stop(m_timer_100ms_id);
		APP_ERROR_CHECK(err_code);
		err_code = app_timer_stop(m_timer_60ms_id);
		APP_ERROR_CHECK(err_code);
    
		led_off(0);
		led_off(1);
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
		SEGGER_RTT_WriteString(0, "Received data:\n");
		for (int i=0; i<length; i++)
			SEGGER_RTT_printf(0, "0x%#04x\n", p_data[i]);
		if ( p_data[0] == 0xAA && p_data[1] == 0xAA)
		{
				uint8_t command_length = p_data[2];
				uint8_t command_type = p_data[3];
				if ( command_type == 0x11 && command_length == 0x01)
				{
						switch (p_data[4])
						{
								case 18:
										intervention_mode = INTERVENTION_MODE_18MIN;
										break;
								case 25:
										intervention_mode = INTERVENTION_MODE_25MIN;
										break;
								default:
										intervention_mode = INTERVENTION_MODE_TEST;
						}							
						switch (intervention_mode)
						{
								case INTERVENTION_MODE_18MIN:
										timer_interval_state1 = TIMER_INTERVAL_MODE_18MIN_STATE1;
										timer_interval_state2 = TIMER_INTERVAL_MODE_18MIN_STATE2;
										timer_interval_state3 = TIMER_INTERVAL_MODE_18MIN_STATE3;
										timer_interval_state4 = TIMER_INTERVAL_MODE_18MIN_STATE4;
										break;
								case INTERVENTION_MODE_25MIN:
										timer_interval_state1 = TIMER_INTERVAL_MODE_25MIN_STATE1;
										timer_interval_state2 = TIMER_INTERVAL_MODE_25MIN_STATE2;
										timer_interval_state3 = TIMER_INTERVAL_MODE_25MIN_STATE3;
										timer_interval_state4 = TIMER_INTERVAL_MODE_25MIN_STATE4;
										break;
								default:
										timer_interval_state1 = TIMER_INTERVAL_MODE_TEST_STATE1;
										timer_interval_state2 = TIMER_INTERVAL_MODE_TEST_STATE2;
										timer_interval_state3 = TIMER_INTERVAL_MODE_TEST_STATE3;
										timer_interval_state4 = TIMER_INTERVAL_MODE_TEST_STATE4;
						}
						stop_current_state_timer();
						intervention_timers_start();
				}
		}
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
		
    /* setup m_timer for compare event  */
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
		static uint8_t pulse_latency_counter = 0;
		if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
				//SEGGER_RTT_printf(0, "ADC event number: %#04x\n" ,m_adc_evt_counter++);

        for (int i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
						//SEGGER_RTT_printf(0, "%#04x\n" ,p_event->data.done.p_buffer[i]);
						PulseSenosrCal(p_event->data.done.p_buffer[i]);
						//pulse_data_array[pulse_data_index++] = (uint8_t) ((p_event->data.done.p_buffer[i] & 0xFF00) >> 8);
						//pulse_data_array[pulse_data_index++] = (uint8_t) ((p_event->data.done.p_buffer[i] & 0x00FF) >> 0);
        }
				if (pulse_latency_counter == PULSE_LATENCY)
				{
						pulse_data_array[pulse_data_index++] = (uint8_t) ((p_event->data.done.p_buffer[0] & 0xFF00) >> 8);
						pulse_data_array[pulse_data_index++] = (uint8_t) ((p_event->data.done.p_buffer[0] & 0x00FF) >> 0);
						pulse_latency_counter = 0;
				}
				else
						pulse_latency_counter++;
				if (pulse_data_index == 20)
				{
						data_send_by_nus(pulse_data_array, pulse_data_index);
						pulse_data_index = PULSE_DATA_ARRAY_START_INDEX;
				}
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

void led_on(uint32_t led_idx)
{
		bsp_board_led_off(led_idx);
}

void led_off(uint32_t led_idx)
{
		bsp_board_led_on(led_idx);
}

void intervention_init(void)
{
		SEGGER_RTT_WriteString(0, "Intervention initializing...\n");
		intervention_state = STATE0;
		led_off(0);
		led_off(1);
		//application_timers_start();
}	

void stop_current_state_timer(void)
{
		led_off(0);
		led_off(1);
		switch (intervention_state)
		{
				case STATE1:
						app_timer_stop(m_timer_state1_id);
						app_timer_stop(m_timer_100ms_id);
						app_timer_stop(m_timer_60ms_id);
						break;
				case STATE2:
						app_timer_stop(m_timer_state2_id);
						app_timer_stop(m_timer_200ms_id);
						app_timer_stop(m_timer_120ms_id);
						break;
				case STATE3:
						app_timer_stop(m_timer_state3_id);
						break;
				case STATE4:
						app_timer_stop(m_timer_state4_id);
						app_timer_stop(m_timer_100ms_id);
						app_timer_stop(m_timer_60ms_id);
						break;
				default:
						break;
		}
}

void start_current_state_timer(void)
{
		void *p;
		switch (intervention_state)
		{
				case STATE0:
						led_off(0);
						led_off(1);
						break;
				case STATE1:
						intervention_timers_start();
						break;
				case STATE2:
						timer_state1_timeout_handler(p);
						break;
				case STATE3:
						timer_state2_timeout_handler(p);
						break;
				case STATE4:
						timer_state3_timeout_handler(p);
						break;
				default:
						break;
		}
}

void bsp_evt_handler(bsp_event_t evt)
{
		stop_current_state_timer();
    switch (evt)
    {
        case BSP_EVENT_KEY_2:
						SEGGER_RTT_WriteString(0, "Starting next intervention_state...\n");
            if (intervention_state == STATE0)
                intervention_state = STATE1;
						else if(intervention_state == STATE1)
                intervention_state = STATE2;
            else if (intervention_state == STATE2)
                intervention_state = STATE3;
						else if (intervention_state == STATE3)
                intervention_state = STATE4;
						else if (intervention_state == STATE4)
                intervention_state = STATE0;
            break;

        case BSP_EVENT_KEY_3:
						SEGGER_RTT_WriteString(0, "Going back to last intervention_state...\n");
            if (intervention_state == STATE0)
                intervention_state = STATE4;
						else if (intervention_state == STATE1)
                intervention_state = STATE0;
            else if (intervention_state == STATE2)
                intervention_state = STATE1;
						else if (intervention_state == STATE3)
                intervention_state = STATE2;
						else if (intervention_state == STATE4)
                intervention_state = STATE3;
            break;

        default:
            return; // no implementation needed
    }
    start_current_state_timer();
}

void bsp_configuration()
{
    uint32_t err_code;

    err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                        APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                        bsp_evt_handler);
    APP_ERROR_CHECK(err_code);
		led_off(0);
		led_off(1);
}

void led_init()
{
		led_off(0);
		led_off(1);
}

static void PulseSenosrCal(nrf_saadc_value_t data)
{
		// pulse detection
		static int rate[MAX_RATE_ID];                    // used to hold last ten IBI values
		static unsigned long sampleCounter = 0;          // used to determine pulse timing
		static unsigned long lastBeatTime = 0;           // used to find the inter beat interval
		static int Px = 0;                      // used to find peak in pulse wave
		static int T = MAX_VALUE;                     // used to find trough in pulse wave
		static int thresh = MAX_VALUE / 2;                // used to find instant moment of heart beat
		static int amp = 0;                   // used to hold amplitude of pulse waveform
		static bool firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
		static bool secondBeat = true;       // used to seed rate array so we startup with reasonable BPM

		Signal = data;
    //NPI_PrintValue("", Signal, 10);

    sampleCounter += SAADC_INTERVAL;                         // keep track of the time in mS with this variable
    int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

//  find the peak and trough of the pulse wave
    if(Signal < thresh && N > (IBI/5)*3)       // avoid dichrotic noise by waiting 3/5 of last IBI
        if (Signal < T)                        // T is the trough
            T = Signal;                         // keep track of lowest point in pulse wave 
    if(Signal > thresh && Signal > Px)          // thresh condition helps avoid noise
        Px = Signal;                             // P is the peak
                                               // keep track of highest point in pulse wave
    
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
    if (N > 250)                                   // avoid high frequency noise
    {  
				if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) )        
        {
						Pulse = true;                               // set the Pulse flag when we think there is a pulse
						IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
						lastBeatTime = sampleCounter;               // keep track of time for next pulse
             
						if(firstBeat)                         // if it's the first time we found a beat, if firstBeat == TRUE
						{
								firstBeat = false;                 // clear firstBeat flag
                return;                            // IBI value is unreliable so discard it
						}   
            if(secondBeat)                        // if this is the second beat, if secondBeat == TRUE
            {
								secondBeat = false;                 // clear secondBeat flag
								for(int i=0; i<MAX_RATE_ID; i++)         // seed the running total to get a realisitic BPM at startup
										rate[i] = IBI;                      
						}
              
						// keep a running total of the last 10 IBI values
						int runningTotal = 0;                   // clear the runningTotal variable    

						for(int i=0; i<=MAX_RATE_ID-2; i++)                // shift data in the rate array
						{      
								rate[i] = rate[i+1];              // and drop the oldest IBI value 
								runningTotal += rate[i];          // add up the 9 oldest IBI values
						}
								
						rate[MAX_RATE_ID-1] = IBI;                          // add the latest IBI to the rate array
						runningTotal += rate[MAX_RATE_ID-1];                // add the latest IBI to runningTotal
						runningTotal /= MAX_RATE_ID;                     // average the last 10 IBI values 
						BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
						//QS = true;                              // set Quantified Self flag 

						if(BPM>55 && BPM<135)
						{
								rri_update(IBI);
						}
						else
						{
								BPM = 0;
						}
						// QS FLAG IS NOT CLEARED INSIDE THIS ISR
				}                       
    }

		if (Signal < thresh && Pulse == true)     // when the values are going down, the beat is over
    {  
				Pulse = false;                         // reset the Pulse flag so we can do it again
				amp = Px - T;                           // get amplitude of the pulse wave
				thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
				Px = thresh;                            // reset these for next time
				T = thresh;
    }
  
		if (N > 2500)                             // if 2.5 seconds go by without a beat
    {  
				BPM = 0;
				thresh = MAX_VALUE / 2;                          // set thresh default
				Px = 0;                               // set P default
				T = MAX_VALUE;                               // set T default
				
				lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
				firstBeat = true;                      // set these to avoid noise
				secondBeat = true;                     // when we get the heartbeat back
		}
}
