/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "nordic_common.h"
#include "softdevice_handler.h"
#include "boards.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "app_button.h"
#include "app_timer_appsh.h"

//#define RTT                                                               //RTT logging.

#define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           0                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                 /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define APP_CFG_NON_CONN_ADV_TIMEOUT    0                                 /**< Time for which the device must be advertising in non-connectable mode (in seconds). 0 disables timeout. */
#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS) /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                              /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                              /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                              /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                              /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                            /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0xC2, 0x00                        /**< Major value used to identify Beacons. Area + Floor No*/ 
#define APP_MINOR_VALUE                 0x00, 0x00                        /**< Minor value used to identify Beacons. Room No + Function*/ 
#define APP_BEACON_UUID                 0x4E, 0x4B, 0x46, 0x55, 0x53, 0x54, \
																				0x4D, 0x49,	0x53,										\
                                        0x50, 0x49, 0x50, 0x45,	 						\
                                        0xcd, 0xde, 0xef       			      /**< NKFUST..Door status..temperature. */
																				
#define APP_BEACON_UUID1                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, \
																				0x11, 0xFF, 												\
                                        0x11, 0xFF, 0xFF, 0xFF, 						\
                                        0x11, 0xFF, 0xFF, 0xFF            /**< NKFUST..Door status..temperature. */

#define DEAD_BEEF                       0xDEADBEEF                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_TIMER_PRESCALER             0                                 /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         10                                 /**< Size of timer operation queues. */

#define DOOR_STATUS_INTERVAL						APP_TIMER_TICKS(60000, APP_TIMER_PRESCALER)

#define	BUTTON_PIN											17
#define	LED_PIN													19

//Scheduler settings
#define SCHED_MAX_EVENT_DATA_SIZE				MAX(APP_TIMER_SCHED_EVT_SIZE, sizeof(nrf_drv_gpiote_pin_t))
#define SCHED_QUEUE_SIZE								10

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)

APP_TIMER_DEF(m_status_timer_id);

static bool 				        door_status = 0;						//0 is close, 1 is open.
static uint8_t 		          temp_uuid;
static ble_gap_adv_params_t m_adv_params;                                 /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
                            {
                              APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this 
                                                   // implementation. 
                              APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the 
                                                   // manufacturer specific data in this implementation.
                              APP_BEACON_UUID,     // 128 bit UUID value. 
                              APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons. 
                              APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons. 
                              APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in 
                                                   // this implementation. 
                            };

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
	
		m_beacon_info[20] = nrf_gpio_pin_read(BUTTON_PIN);
		m_beacon_info[21]	= temp_uuid;
    #ifdef RTT
        NRF_LOG_PRINTF("BUTTON VOL: %d\nTemp is %d\n", m_beacon_info[20], m_beacon_info[21]);
    #endif

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_NONCONN_IND;
    m_adv_params.p_peer_addr = NULL;                             // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval    = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.timeout     = APP_CFG_NON_CONN_ADV_TIMEOUT;
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

		err_code = sd_ble_gap_tx_power_set(-12);
		APP_ERROR_CHECK(err_code);
	
    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
}

static void status_timer_handler(void* p_context)
{
		uint32_t	err_code;
		int32_t 	temp;
		uint8_t		temp_quo;
	
		temp = SD_TEMP_GET/4;
		temp_quo = temp/10;
		temp_uuid = (uint8_t)temp + (uint8_t)temp_quo*6;
    #ifdef RTT
        NRF_LOG_PRINTF("Temperature is %d(int) %x(hex)...\n", temp, temp);
    #endif
			
		err_code = sd_ble_gap_adv_stop();
		APP_ERROR_CHECK(err_code);
		for(uint8_t tmr=1; tmr<6; tmr++)
		{
        #ifdef RTT
          NRF_LOG_PRINTF("tmr: %d\n", tmr);
        #endif
				nrf_delay_ms(1000);
		}
		advertising_init();
		err_code = sd_ble_gap_adv_start(&m_adv_params);
		APP_ERROR_CHECK(err_code);
	
}

//Timer initialize.
static void timer_init()
{
		uint32_t err_code;

		APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);
	
		err_code = app_timer_create(&m_status_timer_id,
																 APP_TIMER_MODE_REPEATED,
																 status_timer_handler);
	
		APP_ERROR_CHECK(err_code);
}

//Timer Start.
static void timer_start()
{
		uint32_t err_code;
		
		err_code = app_timer_start(m_status_timer_id, DOOR_STATUS_INTERVAL, NULL);
		APP_ERROR_CHECK(err_code);
}

void btn_handler(nrf_drv_gpiote_pin_t pin)
{
		uint32_t err_code;
		bool 		 door_st;
		
		door_st = nrf_gpio_pin_read(BUTTON_PIN);
	
		switch(pin)
		{
			case BUTTON_PIN:
				nrf_delay_ms(5000);
				
				if(door_st==nrf_gpio_pin_read(BUTTON_PIN))
				{
						if(!nrf_gpio_pin_read(BUTTON_PIN))
						{
								if(!door_status)
								{
										door_status = 1;
                    #ifdef RTT
                        	NRF_LOG_PRINTF("OPEN DOOR~\n");
                    #endif
								
										//LED is blink.
										nrf_drv_gpiote_out_set(LED_PIN);
							
										//change advertising data.
										err_code = sd_ble_gap_adv_stop();
										APP_ERROR_CHECK(err_code);
										for(uint8_t tmr=1; tmr<6; tmr++)
										{
                        #ifdef RTT
                            NRF_LOG_PRINTF("tmr: %d\n", tmr);
                        #endif
												nrf_delay_ms(1000);
										}
										advertising_init();
										advertising_start();
								}
						}else
						{
								if(door_status)
								{
										door_status = 0;
                    #ifdef RTT
                        NRF_LOG_PRINTF("CLOSE DOOR~\n");
                    #endif									
										nrf_drv_gpiote_out_clear(LED_PIN);
									
										err_code = sd_ble_gap_adv_stop();
										APP_ERROR_CHECK(err_code);
										for(uint8_t tmr=1; tmr<6; tmr++)
										{
                        #ifdef RTT
                            NRF_LOG_PRINTF("tmr: %d\n", tmr);
                        #endif
												nrf_delay_ms(1000);
										}
										advertising_init();
										advertising_start();
								}
						}
				}
				break;
			default:
				break;
		}
}

//btn_sched_event.
void btn_sched_evt_handler(void* p_event_data, uint16_t event_size)
{
		btn_handler(*((nrf_drv_gpiote_pin_t*)p_event_data));
}

static void gpiote_evt_handle(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		app_sched_event_put(&pin, sizeof(pin), btn_sched_evt_handler);
}

//GPIOTE configuration.
void gpiote_init()
{
		uint32_t err_code;
	
		//GPIOTE driver initialize.
		err_code = nrf_drv_gpiote_init();
		APP_ERROR_CHECK(err_code);
	
		//GPIOTE output initialize.
		nrf_drv_gpiote_out_config_t	out_config = GPIOTE_CONFIG_OUT_TASK_LOW;
		err_code = nrf_drv_gpiote_out_init(LED_PIN, &out_config);
		APP_ERROR_CHECK(err_code);
	
		nrf_drv_gpiote_out_clear(LED_PIN);
		
	
		//GPIOTE input initialize.
		nrf_drv_gpiote_in_config_t	in_config	= GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
		err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &in_config, gpiote_evt_handle);
		APP_ERROR_CHECK(err_code);
	
		nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
		
}

//Schedular initialize.
static void sched_init()
{
		APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for doing power management.
 */
static void power_manage(void)
{	
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
		// Initialize.
		gpiote_init();
		sched_init();
	
    ble_stack_init();
    advertising_init();
		timer_init();
		timer_start();

    // Start execution.
    advertising_start();		

    // Enter main loop.
    for (;;)
    {
				app_sched_execute();
        power_manage();
    }
}
