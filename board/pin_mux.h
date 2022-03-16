/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
    kPIN_MUX_DirectionInput = 0U,        /* Input direction */
    kPIN_MUX_DirectionOutput = 1U,       /* Output direction */
    kPIN_MUX_DirectionInputOrOutput = 2U /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*! @name PTD1 (number 1), Zero_X_Comp
  @{ */
#define BOARD_INITPINS_Zero_X_Comp_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_Zero_X_Comp_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_Zero_X_Comp_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_Zero_X_Comp_GPIO_PIN 25U                       /*!<@brief PORTA pin index: 25 */
#define BOARD_INITPINS_Zero_X_Comp_PORT PORTD                         /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_Zero_X_Comp_PIN 1U                             /*!<@brief PORTD pin index: 1 */
#define BOARD_INITPINS_Zero_X_Comp_CHANNEL 25                         /*!<@brief GPIOA GPIO channel: 25 */
#define BOARD_INITPINS_Zero_X_Comp_PIN_NAME PTD1                      /*!<@brief Pin name */
#define BOARD_INITPINS_Zero_X_Comp_LABEL "Zero_X_Comp"                /*!<@brief Label */
#define BOARD_INITPINS_Zero_X_Comp_NAME "Zero_X_Comp"                 /*!<@brief Identifier name */
#define BOARD_INITPINS_Zero_X_Comp_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                      /* @} */

/*! @name FTM2_CH2 (number 2), U_Z_Cross
  @{ */
#define BOARD_INITPINS_U_Z_Cross_PERIPHERAL FTM2                   /*!<@brief Device name: FTM2 */
#define BOARD_INITPINS_U_Z_Cross_SIGNAL CH                         /*!<@brief FTM2 signal: CH */
#define BOARD_INITPINS_U_Z_Cross_CHANNEL 2                         /*!<@brief FTM2 channel: 2 */
#define BOARD_INITPINS_U_Z_Cross_PIN_NAME FTM2_CH2                 /*!<@brief Pin name */
#define BOARD_INITPINS_U_Z_Cross_LABEL "U_Z_Cross"                 /*!<@brief Label */
#define BOARD_INITPINS_U_Z_Cross_NAME "U_Z_Cross"                  /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Z_Cross_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                   /* @} */

/*! @name PTE7 (number 3), U_Led_Debug_2
  @{ */
#define BOARD_INITPINS_U_Led_Debug_2_PERIPHERAL GPIOB                   /*!<@brief Device name: GPIOB */
#define BOARD_INITPINS_U_Led_Debug_2_SIGNAL GPIO                        /*!<@brief GPIOB signal: GPIO */
#define BOARD_INITPINS_U_Led_Debug_2_GPIO GPIOB                         /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITPINS_U_Led_Debug_2_GPIO_PIN 7U                        /*!<@brief PORTB pin index: 7 */
#define BOARD_INITPINS_U_Led_Debug_2_PORT PORTE                         /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_U_Led_Debug_2_PIN 7U                             /*!<@brief PORTE pin index: 7 */
#define BOARD_INITPINS_U_Led_Debug_2_CHANNEL 7                          /*!<@brief GPIOB GPIO channel: 7 */
#define BOARD_INITPINS_U_Led_Debug_2_PIN_NAME PTE7                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_Led_Debug_2_LABEL "U_Led_Debug_2"              /*!<@brief Label */
#define BOARD_INITPINS_U_Led_Debug_2_NAME "U_Led_Debug_2"               /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Led_Debug_2_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                        /* @} */

/*! @name PTB7 (number 9), U_Water_Sensor
  @{ */
#define BOARD_INITPINS_U_Water_Sensor_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Water_Sensor_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Water_Sensor_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Water_Sensor_GPIO_PIN 15U                      /*!<@brief PORTA pin index: 15 */
#define BOARD_INITPINS_U_Water_Sensor_PORT PORTB                        /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_U_Water_Sensor_PIN 7U                            /*!<@brief PORTB pin index: 7 */
#define BOARD_INITPINS_U_Water_Sensor_CHANNEL 15                        /*!<@brief GPIOA GPIO channel: 15 */
#define BOARD_INITPINS_U_Water_Sensor_PIN_NAME PTB7                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_Water_Sensor_LABEL "U_Water_Sensor"            /*!<@brief Label */
#define BOARD_INITPINS_U_Water_Sensor_NAME "U_Water_Sensor"             /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Water_Sensor_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                        /* @} */

/*! @name PTB6 (number 10), U_Heater
  @{ */
#define BOARD_INITPINS_U_Heater_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Heater_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Heater_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Heater_GPIO_PIN 14U                       /*!<@brief PORTA pin index: 14 */
#define BOARD_INITPINS_U_Heater_PORT PORTB                         /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_U_Heater_PIN 6U                             /*!<@brief PORTB pin index: 6 */
#define BOARD_INITPINS_U_Heater_CHANNEL 14                         /*!<@brief GPIOA GPIO channel: 14 */
#define BOARD_INITPINS_U_Heater_PIN_NAME PTB6                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_Heater_LABEL "U_Heater"                   /*!<@brief Label */
#define BOARD_INITPINS_U_Heater_NAME "U_Heater"                    /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Heater_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                   /* @} */

/*! @name PTB5 (number 12), U_Voltage_Config
  @{ */
#define BOARD_INITPINS_U_Voltage_Config_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Voltage_Config_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Voltage_Config_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Voltage_Config_GPIO_PIN 13U                      /*!<@brief PORTA pin index: 13 */
#define BOARD_INITPINS_U_Voltage_Config_PORT PORTB                        /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_U_Voltage_Config_PIN 5U                            /*!<@brief PORTB pin index: 5 */
#define BOARD_INITPINS_U_Voltage_Config_CHANNEL 13                        /*!<@brief GPIOA GPIO channel: 13 */
#define BOARD_INITPINS_U_Voltage_Config_PIN_NAME PTB5                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_Voltage_Config_LABEL "U_Voltage_Config"          /*!<@brief Label */
#define BOARD_INITPINS_U_Voltage_Config_NAME "U_Voltage_Config"           /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Voltage_Config_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                          /* @} */

/*! @name PTB4 (number 13), U_Pump
  @{ */
#define BOARD_INITPINS_U_Pump_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Pump_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Pump_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Pump_GPIO_PIN 12U                       /*!<@brief PORTA pin index: 12 */
#define BOARD_INITPINS_U_Pump_PORT PORTB                         /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_U_Pump_PIN 4U                             /*!<@brief PORTB pin index: 4 */
#define BOARD_INITPINS_U_Pump_CHANNEL 12                         /*!<@brief GPIOA GPIO channel: 12 */
#define BOARD_INITPINS_U_Pump_PIN_NAME PTB4                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_Pump_LABEL "U_Pump"                     /*!<@brief Label */
#define BOARD_INITPINS_U_Pump_NAME "U_Pump"                      /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Pump_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                 /* @} */

/*! @name PTC3 (number 14), U_MS_BU_Present
  @{ */
#define BOARD_INITPINS_U_MS_BU_Present_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_MS_BU_Present_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_MS_BU_Present_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_MS_BU_Present_GPIO_PIN 19U                      /*!<@brief PORTA pin index: 19 */
#define BOARD_INITPINS_U_MS_BU_Present_PORT PORTC                        /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_U_MS_BU_Present_PIN 3U                            /*!<@brief PORTC pin index: 3 */
#define BOARD_INITPINS_U_MS_BU_Present_CHANNEL 19                        /*!<@brief GPIOA GPIO channel: 19 */
#define BOARD_INITPINS_U_MS_BU_Present_PIN_NAME PTC3                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_MS_BU_Present_LABEL "U_MS_BU_Present"           /*!<@brief Label */
#define BOARD_INITPINS_U_MS_BU_Present_NAME "U_MS_BU_Present"            /*!<@brief Identifier name */
#define BOARD_INITPINS_U_MS_BU_Present_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                         /* @} */

/*! @name PTC2 (number 15), U_BU_Home_Work
  @{ */
#define BOARD_INITPINS_U_BU_Home_Work_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_BU_Home_Work_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_BU_Home_Work_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_BU_Home_Work_GPIO_PIN 18U                      /*!<@brief PORTA pin index: 18 */
#define BOARD_INITPINS_U_BU_Home_Work_PORT PORTC                        /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_U_BU_Home_Work_PIN 2U                            /*!<@brief PORTC pin index: 2 */
#define BOARD_INITPINS_U_BU_Home_Work_CHANNEL 18                        /*!<@brief GPIOA GPIO channel: 18 */
#define BOARD_INITPINS_U_BU_Home_Work_PIN_NAME PTC2                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_BU_Home_Work_LABEL "U_BU_Home_Work"            /*!<@brief Label */
#define BOARD_INITPINS_U_BU_Home_Work_NAME "U_BU_Home_Work"             /*!<@brief Identifier name */
#define BOARD_INITPINS_U_BU_Home_Work_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                        /* @} */

/*! @name PTD7 (number 16), U_Led_Debug
  @{ */
#define BOARD_INITPINS_U_Led_Debug_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Led_Debug_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Led_Debug_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Led_Debug_GPIO_PIN 31U                       /*!<@brief PORTA pin index: 31 */
#define BOARD_INITPINS_U_Led_Debug_PORT PORTD                         /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_U_Led_Debug_PIN 7U                             /*!<@brief PORTD pin index: 7 */
#define BOARD_INITPINS_U_Led_Debug_CHANNEL 31                         /*!<@brief GPIOA GPIO channel: 31 */
#define BOARD_INITPINS_U_Led_Debug_PIN_NAME PTD7                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_Led_Debug_LABEL "U_Led_Debug"                /*!<@brief Label */
#define BOARD_INITPINS_U_Led_Debug_NAME "U_Led_Debug"                 /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Led_Debug_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                      /* @} */

/*! @name PTD6 (number 17), U_BU_Fault
  @{ */
#define BOARD_INITPINS_U_BU_Fault_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_BU_Fault_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_BU_Fault_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_BU_Fault_GPIO_PIN 30U                      /*!<@brief PORTA pin index: 30 */
#define BOARD_INITPINS_U_BU_Fault_PORT PORTD                        /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_U_BU_Fault_PIN 6U                            /*!<@brief PORTD pin index: 6 */
#define BOARD_INITPINS_U_BU_Fault_CHANNEL 30                        /*!<@brief GPIOA GPIO channel: 30 */
#define BOARD_INITPINS_U_BU_Fault_PIN_NAME PTD6                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_BU_Fault_LABEL "U_BU_Fault"                /*!<@brief Label */
#define BOARD_INITPINS_U_BU_Fault_NAME "U_BU_Fault"                 /*!<@brief Identifier name */
#define BOARD_INITPINS_U_BU_Fault_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                    /* @} */

/*! @name KBI1_P5 (number 18), U_Flowmeter_Pulse
  @{ */
#define BOARD_INITPINS_U_Flowmeter_Pulse_PERIPHERAL KBI1                   /*!<@brief Device name: KBI1 */
#define BOARD_INITPINS_U_Flowmeter_Pulse_SIGNAL P                          /*!<@brief KBI1 signal: P */
#define BOARD_INITPINS_U_Flowmeter_Pulse_CHANNEL 5                         /*!<@brief KBI1 P channel: 5 */
#define BOARD_INITPINS_U_Flowmeter_Pulse_PIN_NAME KBI1_P5                  /*!<@brief Pin name */
#define BOARD_INITPINS_U_Flowmeter_Pulse_LABEL "U_Flowmeter_Pulse"         /*!<@brief Label */
#define BOARD_INITPINS_U_Flowmeter_Pulse_NAME "U_Flowmeter_Pulse"          /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Flowmeter_Pulse_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                           /* @} */

/*! @name PTC1 (number 19), U_BU_Rev
  @{ */
#define BOARD_INITPINS_U_BU_Rev_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_BU_Rev_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_BU_Rev_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_BU_Rev_GPIO_PIN 17U                       /*!<@brief PORTA pin index: 17 */
#define BOARD_INITPINS_U_BU_Rev_PORT PORTC                         /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_U_BU_Rev_PIN 1U                             /*!<@brief PORTC pin index: 1 */
#define BOARD_INITPINS_U_BU_Rev_CHANNEL 17                         /*!<@brief GPIOA GPIO channel: 17 */
#define BOARD_INITPINS_U_BU_Rev_PIN_NAME PTC1                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_BU_Rev_LABEL "U_BU_Rev"                   /*!<@brief Label */
#define BOARD_INITPINS_U_BU_Rev_NAME "U_BU_Rev"                    /*!<@brief Identifier name */
#define BOARD_INITPINS_U_BU_Rev_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                   /* @} */

/*! @name PTC0 (number 20), U_BU_Fwd
  @{ */
#define BOARD_INITPINS_U_BU_Fwd_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_BU_Fwd_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_BU_Fwd_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_BU_Fwd_GPIO_PIN 16U                       /*!<@brief PORTA pin index: 16 */
#define BOARD_INITPINS_U_BU_Fwd_PORT PORTC                         /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_U_BU_Fwd_PIN 0U                             /*!<@brief PORTC pin index: 0 */
#define BOARD_INITPINS_U_BU_Fwd_CHANNEL 16                         /*!<@brief GPIOA GPIO channel: 16 */
#define BOARD_INITPINS_U_BU_Fwd_PIN_NAME PTC0                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_BU_Fwd_LABEL "U_BU_Fwd"                   /*!<@brief Label */
#define BOARD_INITPINS_U_BU_Fwd_NAME "U_BU_Fwd"                    /*!<@brief Identifier name */
#define BOARD_INITPINS_U_BU_Fwd_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                   /* @} */

/*! @name PTB3 (number 21), Tp1
  @{ */
#define BOARD_INITPINS_Tp1_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_Tp1_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_Tp1_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_Tp1_GPIO_PIN 11U                       /*!<@brief PORTA pin index: 11 */
#define BOARD_INITPINS_Tp1_PORT PORTB                         /*!<@brief PORT device name: PORTB */
#define BOARD_INITPINS_Tp1_PIN 3U                             /*!<@brief PORTB pin index: 3 */
#define BOARD_INITPINS_Tp1_CHANNEL 11                         /*!<@brief GPIOA GPIO channel: 11 */
#define BOARD_INITPINS_Tp1_PIN_NAME PTB3                      /*!<@brief Pin name */
#define BOARD_INITPINS_Tp1_LABEL "Tp1"                        /*!<@brief Label */
#define BOARD_INITPINS_Tp1_NAME "Tp1"                         /*!<@brief Identifier name */
#define BOARD_INITPINS_Tp1_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                              /* @} */

/*! @name FTM0_CH0 (number 22), U_Valve_1
  @{ */
#define BOARD_INITPINS_U_Valve_1_PERIPHERAL FTM0                    /*!<@brief Device name: FTM0 */
#define BOARD_INITPINS_U_Valve_1_SIGNAL CH                          /*!<@brief FTM0 signal: CH */
#define BOARD_INITPINS_U_Valve_1_CHANNEL 0                          /*!<@brief FTM0 channel: 0 */
#define BOARD_INITPINS_U_Valve_1_PIN_NAME FTM0_CH0                  /*!<@brief Pin name */
#define BOARD_INITPINS_U_Valve_1_LABEL "U_Valve_1"                  /*!<@brief Label */
#define BOARD_INITPINS_U_Valve_1_NAME "U_Valve_1"                   /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Valve_1_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                    /* @} */

/*! @name ADC0_SE5 (number 23), U_BU_Current
  @{ */
#define BOARD_INITPINS_U_BU_Current_PERIPHERAL ADC       /*!<@brief Device name: ADC */
#define BOARD_INITPINS_U_BU_Current_SIGNAL SE            /*!<@brief ADC signal: SE */
#define BOARD_INITPINS_U_BU_Current_CHANNEL 5            /*!<@brief ADC SE channel: 5 */
#define BOARD_INITPINS_U_BU_Current_PIN_NAME ADC0_SE5    /*!<@brief Pin name */
#define BOARD_INITPINS_U_BU_Current_LABEL "U_BU_Current" /*!<@brief Label */
#define BOARD_INITPINS_U_BU_Current_NAME "U_BU_Current"  /*!<@brief Identifier name */
                                                         /* @} */

/*! @name ADC0_SE4 (number 24), U_NTC_Boiler
  @{ */
#define BOARD_INITPINS_U_NTC_Boiler_PERIPHERAL ADC       /*!<@brief Device name: ADC */
#define BOARD_INITPINS_U_NTC_Boiler_SIGNAL SE            /*!<@brief ADC signal: SE */
#define BOARD_INITPINS_U_NTC_Boiler_CHANNEL 4            /*!<@brief ADC SE channel: 4 */
#define BOARD_INITPINS_U_NTC_Boiler_PIN_NAME ADC0_SE4    /*!<@brief Pin name */
#define BOARD_INITPINS_U_NTC_Boiler_LABEL "U_NTC_Boiler" /*!<@brief Label */
#define BOARD_INITPINS_U_NTC_Boiler_NAME "U_NTC_Boiler"  /*!<@brief Identifier name */
                                                         /* @} */

/*! @name PTA7 (number 25), U_Power_Cut
  @{ */
#define BOARD_INITPINS_U_Power_Cut_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Power_Cut_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Power_Cut_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Power_Cut_GPIO_PIN 7U                        /*!<@brief PORTA pin index: 7 */
#define BOARD_INITPINS_U_Power_Cut_PORT PORTA                         /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_U_Power_Cut_PIN 7U                             /*!<@brief PORTA pin index: 7 */
#define BOARD_INITPINS_U_Power_Cut_CHANNEL 7                          /*!<@brief GPIOA GPIO channel: 7 */
#define BOARD_INITPINS_U_Power_Cut_PIN_NAME PTA7                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_Power_Cut_LABEL "U_Power_Cut"                /*!<@brief Label */
#define BOARD_INITPINS_U_Power_Cut_NAME "U_Power_Cut"                 /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Power_Cut_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                      /* @} */

/*! @name PTA6 (number 26), U_BU_Dreg_Door
  @{ */
#define BOARD_INITPINS_U_BU_Dreg_Door_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_BU_Dreg_Door_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_BU_Dreg_Door_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_BU_Dreg_Door_GPIO_PIN 6U                       /*!<@brief PORTA pin index: 6 */
#define BOARD_INITPINS_U_BU_Dreg_Door_PORT PORTA                        /*!<@brief PORT device name: PORTA */
#define BOARD_INITPINS_U_BU_Dreg_Door_PIN 6U                            /*!<@brief PORTA pin index: 6 */
#define BOARD_INITPINS_U_BU_Dreg_Door_CHANNEL 6                         /*!<@brief GPIOA GPIO channel: 6 */
#define BOARD_INITPINS_U_BU_Dreg_Door_PIN_NAME PTA6                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_BU_Dreg_Door_LABEL "U_BU_Dreg_Door"            /*!<@brief Label */
#define BOARD_INITPINS_U_BU_Dreg_Door_NAME "U_BU_Dreg_Door"             /*!<@brief Identifier name */
#define BOARD_INITPINS_U_BU_Dreg_Door_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                        /* @} */

/*! @name PTD4 (number 29), TP2
  @{ */
#define BOARD_INITPINS_TP2_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_TP2_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_TP2_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_TP2_GPIO_PIN 28U                       /*!<@brief PORTA pin index: 28 */
#define BOARD_INITPINS_TP2_PORT PORTD                         /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_TP2_PIN 4U                             /*!<@brief PORTD pin index: 4 */
#define BOARD_INITPINS_TP2_CHANNEL 28                         /*!<@brief GPIOA GPIO channel: 28 */
#define BOARD_INITPINS_TP2_PIN_NAME PTD4                      /*!<@brief Pin name */
#define BOARD_INITPINS_TP2_LABEL "TP2"                        /*!<@brief Label */
#define BOARD_INITPINS_TP2_NAME "TP2"                         /*!<@brief Identifier name */
#define BOARD_INITPINS_TP2_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                              /* @} */

/*! @name PTD3 (number 30), TP3
  @{ */
#define BOARD_INITPINS_TP3_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_TP3_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_TP3_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_TP3_GPIO_PIN 27U                       /*!<@brief PORTA pin index: 27 */
#define BOARD_INITPINS_TP3_PORT PORTD                         /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_TP3_PIN 3U                             /*!<@brief PORTD pin index: 3 */
#define BOARD_INITPINS_TP3_CHANNEL 27                         /*!<@brief GPIOA GPIO channel: 27 */
#define BOARD_INITPINS_TP3_PIN_NAME PTD3                      /*!<@brief Pin name */
#define BOARD_INITPINS_TP3_LABEL "TP3"                        /*!<@brief Label */
#define BOARD_INITPINS_TP3_NAME "TP3"                         /*!<@brief Identifier name */
#define BOARD_INITPINS_TP3_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                              /* @} */

/*! @name PTD2 (number 31), U_Config_1
  @{ */
#define BOARD_INITPINS_U_Config_1_PERIPHERAL GPIOA                  /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_U_Config_1_SIGNAL GPIO                       /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_U_Config_1_GPIO GPIOA                        /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_U_Config_1_GPIO_PIN 26U                      /*!<@brief PORTA pin index: 26 */
#define BOARD_INITPINS_U_Config_1_PORT PORTD                        /*!<@brief PORT device name: PORTD */
#define BOARD_INITPINS_U_Config_1_PIN 2U                            /*!<@brief PORTD pin index: 2 */
#define BOARD_INITPINS_U_Config_1_CHANNEL 26                        /*!<@brief GPIOA GPIO channel: 26 */
#define BOARD_INITPINS_U_Config_1_PIN_NAME PTD2                     /*!<@brief Pin name */
#define BOARD_INITPINS_U_Config_1_LABEL "U_Config_1"                /*!<@brief Label */
#define BOARD_INITPINS_U_Config_1_NAME "U_Config_1"                 /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Config_1_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                    /* @} */

/*! @name UART0_TX (number 32), TX_Uart_Lin
  @{ */
#define BOARD_INITPINS_TX_Uart_Lin_PERIPHERAL UART0    /*!<@brief Device name: UART0 */
#define BOARD_INITPINS_TX_Uart_Lin_SIGNAL TX           /*!<@brief UART0 signal: TX */
#define BOARD_INITPINS_TX_Uart_Lin_PIN_NAME UART0_TX   /*!<@brief Pin name */
#define BOARD_INITPINS_TX_Uart_Lin_LABEL "TX_Uart_Lin" /*!<@brief Label */
#define BOARD_INITPINS_TX_Uart_Lin_NAME "TX_Uart_Lin"  /*!<@brief Identifier name */
                                                       /* @} */

/*! @name UART0_RX (number 33), RX_Uart_Lin
  @{ */
#define BOARD_INITPINS_RX_Uart_Lin_PERIPHERAL UART0    /*!<@brief Device name: UART0 */
#define BOARD_INITPINS_RX_Uart_Lin_SIGNAL RX           /*!<@brief UART0 signal: RX */
#define BOARD_INITPINS_RX_Uart_Lin_PIN_NAME UART0_RX   /*!<@brief Pin name */
#define BOARD_INITPINS_RX_Uart_Lin_LABEL "RX_Uart_Lin" /*!<@brief Label */
#define BOARD_INITPINS_RX_Uart_Lin_NAME "RX_Uart_Lin"  /*!<@brief Identifier name */
                                                       /* @} */

/*! @name UART1_TX (number 36), TX_Uart_Aux
  @{ */
#define BOARD_INITPINS_TX_Uart_Aux_PERIPHERAL UART1    /*!<@brief Device name: UART1 */
#define BOARD_INITPINS_TX_Uart_Aux_SIGNAL TX           /*!<@brief UART1 signal: TX */
#define BOARD_INITPINS_TX_Uart_Aux_PIN_NAME UART1_TX   /*!<@brief Pin name */
#define BOARD_INITPINS_TX_Uart_Aux_LABEL "TX_Uart_Aux" /*!<@brief Label */
#define BOARD_INITPINS_TX_Uart_Aux_NAME "TX_Uart_Aux"  /*!<@brief Identifier name */
                                                       /* @} */

/*! @name UART1_RX (number 37), RX_Uart_Aux
  @{ */
#define BOARD_INITPINS_RX_Uart_Aux_PERIPHERAL UART1    /*!<@brief Device name: UART1 */
#define BOARD_INITPINS_RX_Uart_Aux_SIGNAL RX           /*!<@brief UART1 signal: RX */
#define BOARD_INITPINS_RX_Uart_Aux_PIN_NAME UART1_RX   /*!<@brief Pin name */
#define BOARD_INITPINS_RX_Uart_Aux_LABEL "RX_Uart_Aux" /*!<@brief Label */
#define BOARD_INITPINS_RX_Uart_Aux_NAME "RX_Uart_Aux"  /*!<@brief Identifier name */
                                                       /* @} */

/*! @name PTE2 (number 38), UI_Busy
  @{ */
#define BOARD_INITPINS_UI_Busy_PERIPHERAL GPIOB                  /*!<@brief Device name: GPIOB */
#define BOARD_INITPINS_UI_Busy_SIGNAL GPIO                       /*!<@brief GPIOB signal: GPIO */
#define BOARD_INITPINS_UI_Busy_GPIO GPIOB                        /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITPINS_UI_Busy_GPIO_PIN 2U                       /*!<@brief PORTB pin index: 2 */
#define BOARD_INITPINS_UI_Busy_PORT PORTE                        /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_UI_Busy_PIN 2U                            /*!<@brief PORTE pin index: 2 */
#define BOARD_INITPINS_UI_Busy_CHANNEL 2                         /*!<@brief GPIOB GPIO channel: 2 */
#define BOARD_INITPINS_UI_Busy_PIN_NAME PTE2                     /*!<@brief Pin name */
#define BOARD_INITPINS_UI_Busy_LABEL "UI_Busy"                   /*!<@brief Label */
#define BOARD_INITPINS_UI_Busy_NAME "UI_Busy"                    /*!<@brief Identifier name */
#define BOARD_INITPINS_UI_Busy_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                 /* @} */

/*! @name PTE1 (number 39), Tp4
  @{ */
#define BOARD_INITPINS_Tp4_PERIPHERAL GPIOB                   /*!<@brief Device name: GPIOB */
#define BOARD_INITPINS_Tp4_SIGNAL GPIO                        /*!<@brief GPIOB signal: GPIO */
#define BOARD_INITPINS_Tp4_GPIO GPIOB                         /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITPINS_Tp4_GPIO_PIN 1U                        /*!<@brief PORTB pin index: 1 */
#define BOARD_INITPINS_Tp4_PORT PORTE                         /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_Tp4_PIN 1U                             /*!<@brief PORTE pin index: 1 */
#define BOARD_INITPINS_Tp4_CHANNEL 1                          /*!<@brief GPIOB GPIO channel: 1 */
#define BOARD_INITPINS_Tp4_PIN_NAME PTE1                      /*!<@brief Pin name */
#define BOARD_INITPINS_Tp4_LABEL "Tp4"                        /*!<@brief Label */
#define BOARD_INITPINS_Tp4_NAME "Tp4"                         /*!<@brief Identifier name */
#define BOARD_INITPINS_Tp4_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                              /* @} */

/*! @name PTE0 (number 40), U_Grinder
  @{ */
#define BOARD_INITPINS_U_Grinder_PERIPHERAL GPIOB                   /*!<@brief Device name: GPIOB */
#define BOARD_INITPINS_U_Grinder_SIGNAL GPIO                        /*!<@brief GPIOB signal: GPIO */
#define BOARD_INITPINS_U_Grinder_GPIO GPIOB                         /*!<@brief GPIO device name: GPIOB */
#define BOARD_INITPINS_U_Grinder_GPIO_PIN 0U                        /*!<@brief PORTB pin index: 0 */
#define BOARD_INITPINS_U_Grinder_PORT PORTE                         /*!<@brief PORT device name: PORTE */
#define BOARD_INITPINS_U_Grinder_PIN 0U                             /*!<@brief PORTE pin index: 0 */
#define BOARD_INITPINS_U_Grinder_CHANNEL 0                          /*!<@brief GPIOB GPIO channel: 0 */
#define BOARD_INITPINS_U_Grinder_PIN_NAME PTE0                      /*!<@brief Pin name */
#define BOARD_INITPINS_U_Grinder_LABEL "U_Grinder"                  /*!<@brief Label */
#define BOARD_INITPINS_U_Grinder_NAME "U_Grinder"                   /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Grinder_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                    /* @} */

/*! @name PTC5 (number 41), Tp5
  @{ */
#define BOARD_INITPINS_Tp5_PERIPHERAL GPIOA                   /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_Tp5_SIGNAL GPIO                        /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_Tp5_GPIO GPIOA                         /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_Tp5_GPIO_PIN 21U                       /*!<@brief PORTA pin index: 21 */
#define BOARD_INITPINS_Tp5_PORT PORTC                         /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_Tp5_PIN 5U                             /*!<@brief PORTC pin index: 5 */
#define BOARD_INITPINS_Tp5_CHANNEL 21                         /*!<@brief GPIOA GPIO channel: 21 */
#define BOARD_INITPINS_Tp5_PIN_NAME PTC5                      /*!<@brief Pin name */
#define BOARD_INITPINS_Tp5_LABEL "Tp5"                        /*!<@brief Label */
#define BOARD_INITPINS_Tp5_NAME "Tp5"                         /*!<@brief Identifier name */
#define BOARD_INITPINS_Tp5_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                              /* @} */

/*! @name RESET (number 43), Mcu_Ext_Reset
  @{ */
#define BOARD_INITPINS_Mcu_Ext_Reset_PERIPHERAL SystemControl          /*!<@brief Device name: SystemControl */
#define BOARD_INITPINS_Mcu_Ext_Reset_SIGNAL RESET                      /*!<@brief SystemControl signal: RESET */
#define BOARD_INITPINS_Mcu_Ext_Reset_PIN_NAME RESET                    /*!<@brief Pin name */
#define BOARD_INITPINS_Mcu_Ext_Reset_LABEL "Mcu_Ext_Reset"             /*!<@brief Label */
#define BOARD_INITPINS_Mcu_Ext_Reset_NAME "Mcu_Ext_Reset"              /*!<@brief Identifier name */
#define BOARD_INITPINS_Mcu_Ext_Reset_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                       /* @} */

/*! @name FTM1_CH0 (number 4), U_Buzzer
  @{ */
#define BOARD_INITPINS_U_Buzzer_PERIPHERAL FTM1                    /*!<@brief Device name: FTM1 */
#define BOARD_INITPINS_U_Buzzer_SIGNAL CH                          /*!<@brief FTM1 signal: CH */
#define BOARD_INITPINS_U_Buzzer_CHANNEL 0                          /*!<@brief FTM1 channel: 0 */
#define BOARD_INITPINS_U_Buzzer_PIN_NAME FTM1_CH0                  /*!<@brief Pin name */
#define BOARD_INITPINS_U_Buzzer_LABEL "U_Buzzer"                   /*!<@brief Label */
#define BOARD_INITPINS_U_Buzzer_NAME "U_Buzzer"                    /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Buzzer_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                                   /* @} */

/*! @name PTC4 (number 42), SWD_CLK
  @{ */
#define BOARD_INITPINS_SWD_CLK_PERIPHERAL GPIOA /*!<@brief Device name: GPIOA */
#define BOARD_INITPINS_SWD_CLK_SIGNAL GPIO      /*!<@brief GPIOA signal: GPIO */
#define BOARD_INITPINS_SWD_CLK_GPIO GPIOA       /*!<@brief GPIO device name: GPIOA */
#define BOARD_INITPINS_SWD_CLK_GPIO_PIN 20U     /*!<@brief PORTA pin index: 20 */
#define BOARD_INITPINS_SWD_CLK_PORT PORTC       /*!<@brief PORT device name: PORTC */
#define BOARD_INITPINS_SWD_CLK_PIN 4U           /*!<@brief PORTC pin index: 4 */
#define BOARD_INITPINS_SWD_CLK_CHANNEL 20       /*!<@brief GPIOA GPIO channel: 20 */
#define BOARD_INITPINS_SWD_CLK_PIN_NAME PTC4    /*!<@brief Pin name */
#define BOARD_INITPINS_SWD_CLK_LABEL "SWD_CLK"  /*!<@brief Label */
#define BOARD_INITPINS_SWD_CLK_NAME "SWD_CLK"   /*!<@brief Identifier name */
                                                /* @} */

/*! @name ADC0_SE0 (number 35), U_Grinder_Analog
  @{ */
#define BOARD_INITPINS_U_Grinder_Analog_PERIPHERAL ADC           /*!<@brief Device name: ADC */
#define BOARD_INITPINS_U_Grinder_Analog_SIGNAL SE                /*!<@brief ADC signal: SE */
#define BOARD_INITPINS_U_Grinder_Analog_CHANNEL 0                /*!<@brief ADC SE channel: 0 */
#define BOARD_INITPINS_U_Grinder_Analog_PIN_NAME ADC0_SE0        /*!<@brief Pin name */
#define BOARD_INITPINS_U_Grinder_Analog_LABEL "U_Grinder_Analog" /*!<@brief Label */
#define BOARD_INITPINS_U_Grinder_Analog_NAME "U_Grinder_Analog"  /*!<@brief Identifier name */
                                                                 /* @} */

/*! @name KBI0_P1 (number 34), U_Grinder_Sensor
  @{ */
#define BOARD_INITPINS_U_Grinder_Sensor_PERIPHERAL KBI0                   /*!<@brief Device name: KBI0 */
#define BOARD_INITPINS_U_Grinder_Sensor_SIGNAL P                          /*!<@brief KBI0 signal: P */
#define BOARD_INITPINS_U_Grinder_Sensor_CHANNEL 1                         /*!<@brief KBI0 P channel: 1 */
#define BOARD_INITPINS_U_Grinder_Sensor_PIN_NAME KBI0_P1                  /*!<@brief Pin name */
#define BOARD_INITPINS_U_Grinder_Sensor_LABEL "U_Grinder_Sensor"          /*!<@brief Label */
#define BOARD_INITPINS_U_Grinder_Sensor_NAME "U_Grinder_Sensor"           /*!<@brief Identifier name */
#define BOARD_INITPINS_U_Grinder_Sensor_DIRECTION kPIN_MUX_DirectionInput /*!<@brief Direction */
                                                                          /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/