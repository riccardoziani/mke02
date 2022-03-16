/*******************************************************************************
Filename: isr.c
Author: Riccardo Ziani
Created: 23/01/2018
Description: Contain interrupts routines used in the board

*******************************************************************************/
// include files

#include "hal.h"	 
#include "nvm.h"
#include "timer.h"

// private definitions *********************************************************
#define Z_Cross_FTM2_IRQHandler FTM2_IRQHandler

// private typedefs ************************************************************
typedef enum {
    ADC_AIN7_BU_CURRENT,  	//  1
    ADC_DUMMY_VCC_HALF,   	//  2
	ADC_NTC_BOILER,       	//  3
    ADC_DUMMY_VCC1V,      	//  4
}Type_ADC_status;         	


typedef enum  {
	ZX_PHASE_0_CAPTURE_RISING_EDGE,
	ZX_PHASE_1_OUTPUT_COMPARE_RISING_EDGE,
	ZX_PHASE_2_CAPTURE_FALLING_EDGE,
	ZX_PHASE_3_OUTPUT_COMPARE_FALLING_EDGE,
	ZX_PHASE_4_OUTPUT_COMPARE_FIRING_GRINDER,
	ZX_PHASE_5_OUTPUT_COMPARE_FIRING_PUMP,
}zx_phase_t;

// global data *****************************************************************

// private data ****************************************************************
static zx_phase_t 	zx_phase;

static volatile uint32_t    delay_msec_cntr;
static volatile uint32_t    delay_usec_cntr;
static volatile uint32_t    msec_cntr;
static volatile uint32_t    systick_tick;

static volatile uint32_t    cntr_edge;
static volatile uint32_t    cntr_r_edge;
static volatile uint32_t    cntr_f_edge;
static volatile uint32_t    cntr_out;
static volatile uint32_t    cntr_out_r_edge;
static volatile uint32_t    cntr_out_f_edge;
static volatile uint32_t    cntr_pump;
static volatile uint32_t    cntr_grinder;

                                                                 
static volatile uint32_t    pit_ch1_time;
static volatile uint32_t    pit_ch1_time_old;
static volatile uint32_t    rising_edge_grinder_sensor;
static volatile uint32_t    falling_edge_grinder_sensor;
static volatile uint32_t    time_grinder_sensor;
static volatile uint32_t    g_keypress;

static volatile uint16_t    captureVal;
static volatile uint16_t    captureVal_old;
static volatile uint16_t    net_semiperiod_run_time;

static volatile uint8_t     fire_H1;					/** Sync heater driver with Zero-crossing routine*/
static volatile uint16_t    Pump_DelayTimeTarget;		/** Delay to turn-on pump from ZC*/
static volatile uint16_t    Grinder_DelayTimeTarget;	/** Delay to turn-on grinder from ZC*/
static          uint32_t    wdog_primask;
                            

#pragma location = ".noinit"
static volatile uint8_t counter_WDOG;
#pragma location = "DEFAULT"

//******************************************************************************
// Function: delay_usec
// Created: 18/01/2018
// Parameters:   None  
// Returns:      None
// Description:  < place here any information concerning the function purpose >
// 
//******************************************************************************
void_t delay_usec (uint32_t usec)
{
    delay_usec_cntr = usec;
    while (delay_usec_cntr);
} // end of function delay_usec

              
//******************************************************************************
// Function: delay_msec
// Created: 18/01/2018
// Parameters:   None  
// Returns:      None
// Description:  < place here any information concerning the function purpose >
// 
//******************************************************************************
void_t delay_msec (uint32_t msec)
{
    delay_msec_cntr = msec;
    while (delay_msec_cntr);
} // end of function delay_msec



//******************************************************************************
// Implementation  interrupt service routine
//******************************************************************************

// private functions ***********************************************************

//******************************************************************************
// Function: HardFault_Handler
// Created: 13/01/2018
// Parameters:   None  
// Returns:      None
// Description:  < place here any information concerning the function purpose >
// 
//******************************************************************************
void_t HardFault_Handler (void_t)
{
    __asm("NOP");
} // end of function HardFault_Handler

//******************************************************************************
// Function: NMI_Handler
// Created: 20/02/2018
// Parameters:   None  
// Returns:      None
// Description:  < place here any information concerning the function purpose >
// 
//******************************************************************************
void_t NMI_Handler (void_t)
{
    __asm("NOP");
    SIM->SOPT &= ~(SIM_SOPT_NMIE(1));

} // end of function NMI_Handler



/**
 * \brief
 * Watchdog Early interrupt time request
 * It occurs when watchdog counter is above 15,625ms
 * 
 * \author Riccardo Ziani
 * \date   16/03/2016
 * 
 * \return void_t 
 */                             
void_t WDOG_IRQHandler (void_t)
{
    wdog_primask = DisableGlobalIRQ();
    WDOG8_Refresh(WDOG);
    EnableGlobalIRQ(wdog_primask);
    WDOG8_ClearStatusFlags(WDOG, kWDOG8_InterruptFlag);
    counter_WDOG++;
    __asm("NOP");
    __asm("BKPT #0");
    
    return;
} // end of function WDOG_IRQHandler



/**
 * \brief
 * IRQ handler for SysTick_Handler.
 * This interrupt is called every 1ms in normal mode.
 * \todo Take care that the duration in under 1ms 
 * \todo verify routine in stby 
 * 
 * @author Riccardo Ziani
 * \date   23/01/2018
 * 
 * @return void_t 
 */
void_t SysTick_Handler (void_t)
{
    systick_tick++;
    msec_cntr++;
    delay_msec_cntr--;
    delay_usec_cntr--;
    led_msec_delay++;
    TimeBase_Refresh();

	return;
} // end of function ISR_RTC_Handler


/**
 * \brief
 * IRQ handler for PIT_CH0_IRQHandler.
 * This interrupt is called every 1ms in normal mode.
 * 
 * @author Riccardo Ziani
 * \date   15/01/2018
 * 
 * @return void_t 
 */
void_t PIT_CH0_IRQHandler (void_t)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
} // end of function PIT_CH0_IRQHandler


/**
 * \brief
 * Zero crossing interrupt event, and timer associated.
 * Used to turn-on AC loads syncronized with the net.
 * Channel 2 -> Used for zero-crossing input capture
 * Channel 3 -> Used for Grinder control partialization
 * Channel 4 -> Used for Pump control partialization
 * Channel 5 -> Used for output compare, sync when to turn-off loads, or turn-on AC loads @ full power
 * 
 * @author Riccardo Ziani
 * \date   15/01/2018
 * 
 * @return void_t 
 */
void_t Z_Cross_FTM2_IRQHandler (void_t)
{
    uint32_t temporary;

    __asm("NOP");


    if ((FTM_GetStatusFlags(FTM2) & kFTM_Chnl2Flag) == kFTM_Chnl2Flag)
    {
        // switch on triacs...
        GPIO_PortClear(kGPIO_PORTD, 1u << 5);	// clear PORTD5

        // HEATER CONTROL -------------------		
        if(fire_H1)
        {	   
            //HAL_TRIAC_HEATER_ON();
        }
        //fire_H1 = UpdateHeater_driver();	/** Update wave driver */

        if ((FTM2->CONTROLS[kFTM_Chnl_2].CnSC & FTM_CnSC_ELSA(1)) == kFTM_RisingEdge)
        {
            zx_phase = ZX_PHASE_0_CAPTURE_RISING_EDGE;
            // ...
            cntr_r_edge++;
        }
        else
        {
            zx_phase = ZX_PHASE_2_CAPTURE_FALLING_EDGE;
            // ...
            cntr_f_edge++;
        }
        cntr_edge++;

		//ZC_pulses_Update(isr_net_period_run_time);

        temporary = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1) >> 4;    // bus clock 16MHz >> 4, 1 usec timer... 
        pit_ch1_time = pit_ch1_time_old - temporary;
        pit_ch1_time_old = temporary;

        captureVal = FTM2->CONTROLS[kFTM_Chnl_2].CnV;
        net_semiperiod_run_time = captureVal - captureVal_old; 
        captureVal_old = captureVal;


        if (Grinder_DelayTimeTarget)
        {
            /* Setup the FTM2 channel 3(GRINDER) in output compare mode  */
            FTM_SetupOutputCompare(FTM2, kFTM_Chnl_3, kFTM_NoOutputSignal, (captureVal + 1000));

            /* Enable channel interrupt */
            FTM_EnableInterrupts(FTM2, kFTM_Chnl3InterruptEnable);
            FTM_ClearStatusFlags(FTM2, kFTM_Chnl3Flag); // Clear interrupt flag.
        }

        if (Pump_DelayTimeTarget)
        {
            /* Setup the FTM2 channel 4(PUMP) in output compare mode  */
            FTM_SetupOutputCompare(FTM2, kFTM_Chnl_4, kFTM_NoOutputSignal, (captureVal + 2000));

            /* Enable channel interrupt */
            FTM_EnableInterrupts(FTM2, kFTM_Chnl4InterruptEnable);
            FTM_ClearStatusFlags(FTM2, kFTM_Chnl4Flag); // Clear interrupt flag.
        }


        /* Setup the FTM2 channel 5 in output compare mode  */
		FTM_SetupOutputCompare(FTM2, kFTM_Chnl_5, kFTM_NoOutputSignal, (captureVal + 9000));



        /* Enable channel interrupt */
		FTM_EnableInterrupts(FTM2, kFTM_Chnl5InterruptEnable);
		FTM_ClearStatusFlags(FTM2, kFTM_Chnl5Flag); // Clear interrupt flag.

		FTM_DisableInterrupts(FTM2, kFTM_Chnl2InterruptEnable);	// Disable input capture kFTM_Chnl_2
        FTM_ClearStatusFlags(FTM2, kFTM_Chnl2Flag); // Clear interrupt flag.

        //temporary = FTM_CnSC_ELSA(1);
        //temporary1 = FTM2->CONTROLS[kFTM_Chnl_2].CnSC&FTM_CnSC_ELSA(1);
	}

    if (Grinder_DelayTimeTarget)
    {   // GRINDER duty cycle phase...
        if ((FTM_GetStatusFlags(FTM2) & kFTM_Chnl3Flag) == kFTM_Chnl3Flag)
        {
            // GRINDER in duty cycle...
            // ....
            GPIO_PortSet(kGPIO_PORTB, 1u << 0);	// set PORTB0
            GPIO_PortClear(kGPIO_PORTB, 1u << 0);	// clear PORTB0
            FTM_DisableInterrupts(FTM2, kFTM_Chnl3InterruptEnable);	// Disable input capture kFTM_Chnl_4
            FTM_ClearStatusFlags(FTM2, kFTM_Chnl3Flag); // Clear interrupt flag.
            cntr_pump++;
        }
    }

    if (Pump_DelayTimeTarget)
    {   // PUMP duty cycle phase...
        if ((FTM_GetStatusFlags(FTM2) & kFTM_Chnl4Flag) == kFTM_Chnl4Flag)
        {
            // PUMP in duty cycle...
            // ....
            GPIO_PortSet(kGPIO_PORTA, 1u << 0);	// set PORTA0
            GPIO_PortClear(kGPIO_PORTA, 1u << 0);	// clear PORTA0
            FTM_DisableInterrupts(FTM2, kFTM_Chnl4InterruptEnable);	// Disable input capture kFTM_Chnl_3
            FTM_ClearStatusFlags(FTM2, kFTM_Chnl4Flag); // Clear interrupt flag.
            cntr_grinder++;
        }
    }

    // AC loads switch-off phase...
    if ((FTM_GetStatusFlags(FTM2) & kFTM_Chnl5Flag) == kFTM_Chnl5Flag)
    {
        // switch off triacs...
        if (zx_phase == ZX_PHASE_0_CAPTURE_RISING_EDGE)
        {
            zx_phase = ZX_PHASE_1_OUTPUT_COMPARE_RISING_EDGE;

            /* Setup input capture on a FTM2 channel */
            FTM_SetupInputCapture(FTM2, kFTM_Chnl_2, kFTM_FallingEdge, 15);
            cntr_out_r_edge++;
        }
        else
        {
            zx_phase = ZX_PHASE_3_OUTPUT_COMPARE_FALLING_EDGE;

            /* Setup input capture on a FTM2 channel */
            FTM_SetupInputCapture(FTM2, kFTM_Chnl_2, kFTM_RisingEdge, 15);
            cntr_out_f_edge++;
        }
        cntr_out++;
        GPIO_PortSet(kGPIO_PORTD, 1u << 5);	// set PORTD5
        Nvm_Load_Network_Cntr();            //	Re-enable e2prom write 

		/* Enable channel 2 input capture interrupt */
		FTM_EnableInterrupts(FTM2, kFTM_Chnl2InterruptEnable);
        FTM_ClearStatusFlags(FTM2, kFTM_Chnl2Flag); // Clear interrupt flag.

		/* Disale channel 5 output compare interrupt */
        FTM_DisableInterrupts(FTM2, kFTM_Chnl5InterruptEnable);	// Disable input capture kFTM_Chnl_5
	}


	return;
}


/**
 * \brief
 * Handler for KBI0 interrupt on pin.
 * Used for Grinder sensor pin; every time the uC detect a front on grinder sensor pin
 * 
 * 
 * @author Riccardo Ziani
 * \date   15/01/2018
 * 
 * \return void_t 
 */
void_t KBI0_IRQHandler (void_t)
{
    if (KBI_IsInterruptRequestDetected(KBI0))
    {
        if (KBI0->ES == KBI_ES_KBEDG(GR_SENSOR_PIN))
        {   // Rising-edge...Set Falling-edge
            rising_edge_grinder_sensor  = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1);
            KBI0->ES &= ~KBI_ES_KBEDG(GR_SENSOR_PIN);

            time_grinder_sensor = (falling_edge_grinder_sensor - rising_edge_grinder_sensor) >> 4; // useconds..
            //Set_GR_Pulse_time((uint16_t)time_grinder_sensor);
        }
        else
        {   // Falling-edge...Set Rising-edge
            falling_edge_grinder_sensor  = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1);
            KBI0->ES |= KBI_ES_KBEDG(GR_SENSOR_PIN);
        }

        /* Clear status. */
        KBI_ClearInterruptFlag(KBI0);
        g_keypress++;
    }
	return;
} // end of function ISR_EIC_Handler
			       
			       
			       
			       
			       
			       
			       
			       
			       
//void_t ISR_SERCOM4_Handler (void)
//{
//    LIN_ERRORinterrupt();
//    LIN_RXCinterrupt();
//    LIN_TXCinterrupt();
//    return;
//}

///**
// * \brief
// * Interrupt handler for ADC conversion.
// * State machine init with BU_current conversion, goes to sample Ntc channel and finally
// * end with a dummy sample on BU_current channel.
// *
// * Check with LECROY
// * Interrupt routine time -> 3.92 us - 1,3 us- 6.4 us - 1.7 us
// * Sample acquisition -> 741 us every step
// * Total time conversion  (about 2,981ms)
// *
// * \author Emanuel Urgese
// * \date  14/03/2016
// *
// * \return None
// */
//void_t ISR_ADC_Handler (void_t)
//{
//    static  Type_ADC_status volatile    isr_adc_status;
//    static  uint16_t        volatile    adc_pin_AIN7_Bu_Current_res;
//    static  uint16_t        volatile    adc_pin_AIN6_NTC_res;
//
//    uint16_t temporary_value_1;
//
//    switch(isr_adc_status)
//    {
//        case ADC_AIN7_BU_CURRENT:
//            /** Bu current signal acquisition */
//            adc_pin_AIN7_Bu_Current_res = adc->RESULT.reg;
//            Set_ADC_BUCurrent_value(adc_pin_AIN7_Bu_Current_res);
//            /** Set register to acquire NTC signal */
//            adc->AVGCTRL.reg =      ADC_AVGCTRL_ADJRES(6) |                                     /** Division factor 64 result 10 bit    */
//                                    ADC_AVGCTRL_SAMPLENUM(ADC_AVGCTRL_SAMPLENUM_128_Val);       /** 128 sample                          */
//
//            adc->REFCTRL.reg =      ADC_REFCTRL_REFCOMP |                                       /** Enable                              */
//                                    ADC_REFCTRL_REFSEL(ADC_REFCTRL_REFSEL_INTVCC1_Val);         /** 1/2 VDDANA voltage reference        */
//
//            adc->INPUTCTRL.reg =    ADC_INPUTCTRL_GAIN(ADC_INPUTCTRL_GAIN_DIV2_Val)         |   /** Gain x 1/2                          */
//                                    ADC_INPUTCTRL_INPUTOFFSET(0)                            |   /** No offset                           */
//                                    ADC_INPUTCTRL_INPUTSCAN(0)                              |   /** No scan                             */
//                                    ADC_INPUTCTRL_MUXNEG(ADC_INPUTCTRL_MUXNEG_IOGND_Val)    |   /** IO Ground                           */
//                                    ADC_INPUTCTRL_MUXPOS(ADC_INPUTCTRL_MUXPOS_PIN6_Val);        /** ADC AIN6 NTC boiler                 */
//
//            isr_adc_status = ADC_DUMMY_VCC_HALF;
//            break;
//
//        case ADC_DUMMY_VCC_HALF:
//            /** First conversion after change of reference - discard this
//             *  value */
//            temporary_value_1 = adc->RESULT.reg;
//            (void_t)(temporary_value_1);
//            isr_adc_status = ADC_NTC_BOILER;
//            break;
//
//
//        case ADC_NTC_BOILER:
//            /** NTC signal acquisition */
//            adc_pin_AIN6_NTC_res = adc->RESULT.reg;
//            Set_ADC_NTC_value(adc_pin_AIN6_NTC_res);
//
//            /** Set register to acquire BU current signal */
//            adc->AVGCTRL.reg =      ADC_AVGCTRL_ADJRES(4)                                   |   /** Division factor 16 result 12 bit    */
//                                    ADC_AVGCTRL_SAMPLENUM(ADC_AVGCTRL_SAMPLENUM_128_Val);       /** 128 sample                          */
//
//            adc->REFCTRL.reg =      ADC_REFCTRL_REFCOMP |                                       /** Enable                              */
//                                    ADC_REFCTRL_REFSEL(ADC_REFCTRL_REFSEL_INT1V_Val);           /** 1V internal reference               */
//
//
//            adc->INPUTCTRL.reg =    ADC_INPUTCTRL_GAIN(ADC_INPUTCTRL_GAIN_1X_Val)           |   /** Gain x1                             */
//                                    ADC_INPUTCTRL_INPUTOFFSET(0)                            |   /** No offset                           */
//                                    ADC_INPUTCTRL_INPUTSCAN(0)                              |   /** No scan                             */
//                                    ADC_INPUTCTRL_MUXNEG(ADC_INPUTCTRL_MUXNEG_IOGND_Val)    |   /** IO Ground                           */
//                                    ADC_INPUTCTRL_MUXPOS(ADC_INPUTCTRL_MUXPOS_PIN7_Val);        /** ADC AIN7 Bu current                 */
//
//            isr_adc_status = ADC_DUMMY_VCC1V;
//            break;
//
//
//        case ADC_DUMMY_VCC1V:
//            /** First conversion after change of reference - discard this
//             *  value */
//            temporary_value_1 = adc->RESULT.reg;
//            (void_t)(temporary_value_1);
//            /** Stop ADC activity */
//            adc->INTENCLR.reg = ADC_INTENCLR_MASK;
//            Set_ADC_Complete();
//            isr_adc_status = ADC_AIN7_BU_CURRENT;
//            break;
//
//        default:
//            break;
//    }
//    adc->INTFLAG.reg =  ADC_INTFLAG_MASK;   /** Clear all adc interrupt flags   */
//    adc->SWTRIG.bit.START = true;           /** Start next conversion           */
//    return;
//} // end of function ISR_ADC_Handler
