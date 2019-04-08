/*
 * ECE 3849 Lab2 starter project
 *
 * Gene Bogdanov    9/13/2017
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <stdint.h>
#include <stdbool.h>


#include <string.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c1294ncpdt.h"
#include "driverlib/fpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "Crystalfontz128x128_ST7735.h"
#include "buttons.h"
#include "driverlib/comp.h"

#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/udma.h"


#define PWM_PERIOD 258  // PWM period = 2^8 + 2 system clock cycles


#define SYSTEM_CLOCK_MHZ 120            // [MHz] system clock frequency

// event and handler definitions
#define EVENT0_PERIOD           5000    // [us] event0 period

// timer periods in clock cycles
#define TIMER0_PERIOD (SYSTEM_CLOCK_MHZ * EVENT0_PERIOD)

// Kiss FFT package
#include "kiss_fft.h"
#include "_kiss_fft_guts.h"
#define PI 3.14159265358979f
#define NFFT 1024 // FFT lenght
#define KISS_FFT_CFG_SIZE (sizeof(struct kiss_fft_state) + sizeof(kiss_fft_cpx) * (NFFT - 1))


#define HALF_SCREEN LCD_HORIZONTAL_MAX / 2
#define SEARCH_REGION ADC_BUFFER_SIZE / 2
#define VIN_RANGE 3.3
#define ADC_OFFSET 2047
#define PIXELS_PER_DIV 20
#define ADC_BITS 12

uint32_t gSystemClock = 120000000; // [Hz] system clock frequency

// Shared data
float fVoltsPerDiv[] = {0.1, 0.2, 0.5, 1};  // constant voltage for changing voltage scale
const char* gVoltageScaleStr[] = {"100 mV", "200 mV", "500 mV", "1 V"};
int currentVoltSel;
bool risingTrig, FFTMode;
tContext sContext;

int displayBuffer[LCD_HORIZONTAL_MAX];
uint16_t waveformBuffer[LCD_HORIZONTAL_MAX];
uint16_t FFTBuffer[NFFT];

unsigned long button_task_latency = 0, button_task_missed = 0, button_task_response = 0, t = 0;

uint32_t start, finish, duration, max_duration;

///////////////////////////////////////////////////////////////////////////////////////////////////
#pragma DATA_ALIGN(gDMAControlTable, 1024) // address alignment required
tDMAControlTable gDMAControlTable[64];     // uDMA control table (global)


uint32_t last_count, curr_count;
uint32_t accu_period, period_count;
float freq_data;


// challenge #3
uint32_t gPhase = 0;
uint32_t gPhaseIncrement = 184650357;

#define PWM_WAVEFORM_INDEX_BITS 10
#define PWM_WAVEFORM_TABLE_SIZE (1 << PWM_WAVEFORM_INDEX_BITS)
uint8_t gPWMWaveformTable[PWM_WAVEFORM_TABLE_SIZE] = {0};
///////////////////////////////////////////////////////////////////////////////////////////////////
uint32_t count_unloaded, count_loaded;
float cpu_load;
uint32_t cpu_load_count(void)
{
    uint32_t i = 0;
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER3_BASE, TIMER_A); // start one-shot timer
    while (!(TimerIntStatus(TIMER3_BASE, false) & TIMER_TIMA_TIMEOUT))
        i++;
    return i;
}
/*
 *  ======== main ========
 */
int main(void)
{
    IntMasterDisable();

    // Lab 2 stuff goes here
    // initialize values
    currentVoltSel = 3;
    risingTrig = true;
    FFTMode = false;

    // initialize timer 3 in one-shot mode for polled timing
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    TimerDisable(TIMER3_BASE, TIMER_BOTH);
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerLoadSet(TIMER3_BASE, TIMER_A, gSystemClock / 100 - 1); // 1 sec interval -> 10 ms


    // hardware initialization goes here
    /* From lab2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, TIMER0_PERIOD - 1);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE,  TIMER_TIMA_TIMEOUT);
    */

    // Enable the Floating Point Unit, and permit ISRs to use it
    FPUEnable();
    FPULazyStackingEnable();

    // Initialize the system clock to 120 MHz
    gSystemClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, 120000000);

    Crystalfontz128x128_Init(); // Initialize the LCD display driver
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP); // set screen orientation

    GrContextInit(&sContext, &g_sCrystalfontz128x128); // Initialize the grlib graphics context
    GrContextFontSet(&sContext, &g_sFontFixed6x8); // select font

    // initialize SysTick timer for the purpose of profiling
    SysTickPeriodSet(1 << 24); // full 24-bit counter
    SysTickEnable();


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(gDMAControlTable);

    uDMAChannelAssign(UDMA_CH24_ADC1_0); // assign DMA channel 24 to ADC1 sequence 0
    uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_ADC10, UDMA_ATTR_ALL);

    // primary DMA channel = first half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_PRI_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[0], ADC_BUFFER_SIZE/2);

    // alternate DMA channel = second half of the ADC buffer
    uDMAChannelControlSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_SIZE_16 | UDMA_SRC_INC_NONE | UDMA_DST_INC_16 | UDMA_ARB_4);
    uDMAChannelTransferSet(UDMA_SEC_CHANNEL_ADC10 | UDMA_ALT_SELECT,
        UDMA_MODE_PINGPONG, (void*)&ADC1_SSFIFO0_R,
        (void*)&gADCBuffer[ADC_BUFFER_SIZE/2], ADC_BUFFER_SIZE/2);

    uDMAChannelEnable(UDMA_SEC_CHANNEL_ADC10);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // Challenge #2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_COMP0);
    ComparatorRefSet(COMP_BASE, COMP_REF_1_65V);
    ComparatorConfigure(COMP_BASE, 1, COMP_TRIG_NONE  | COMP_INT_RISE | COMP_ASRCP_REF | COMP_OUTPUT_NORMAL);

    // configure GPIO for comparator input C1- at BoosterPack Connector #1 pin 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinTypeComparator(GPIO_PORTC_BASE, GPIO_PIN_4);

    // configure GPIO for comparator output C1o at BoosterPack Connector #1 pin 15
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeComparatorOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_C1O);



    // Timer in Capture Mode

    // configure GPIO PD0 as timer input T0CCP0 at BoosterPack Connector #1 pin 14
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeTimer(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_T0CCP0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerDisable(TIMER0_BASE, TIMER_BOTH);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_CAP_TIME_UP);
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);
    TimerLoadSet(TIMER0_BASE, TIMER_A, 0xffff);   // use maximum load value
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0xff); // use maximum prescale value
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Challenge #3
    // use M0PWM1 at GPIO PF1 which is BossterPack Connector #1 Pin 40
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    // configure the PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_1);             // use system clock
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_PERIOD);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, PWM_PERIOD/2); // initial 50% duty cycle
    PWMOutputInvert(PWM0_BASE, PWM_OUT_1_BIT, true);      // invert PWM output
    PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);       // enable PWM output
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);                   // enable PWM generator

    // enable PWM interrupt in the PWM peripheral
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_0, PWM_INT_CNT_ZERO);
    PWMIntEnable(PWM0_BASE, PWM_INT_GEN_0);

    int i = 0;
    for(i = 0; i < PWM_WAVEFORM_TABLE_SIZE; i++)
        gPWMWaveformTable[i] = (uint8_t)(sinf(2 * PI * i / PWM_WAVEFORM_TABLE_SIZE) * 255.0 / 2 + 255.0 / 2);
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ButtonInit();


    /* Start BIOS */
    BIOS_start();

    return (0);
}

void ISR_Event0(UArg arg0)
{
    TIMER0_ICR_R = TIMER_ICR_TATOCINT; // clear interrupt flag
    Semaphore_post(button_time_sem);
}

void capture_ISR(UArg arg0)
{
    TIMER0_ICR_R = TIMER_ICR_CAECINT; // clear interrupt flag
    curr_count = TimerValueGet(TIMER0_BASE, TIMER_A);
    uint32_t period = (curr_count - last_count) & 0xFFFFFF;
    last_count = curr_count;

    accu_period += period;
    period_count++;

}


void PWM_ISR(void){

    PWM0_0_ISC_R = PWM_ISC_INTPWM0;

    gPhase += gPhaseIncrement;

    PWM0_0_CMPB_R = 1 + gPWMWaveformTable[gPhase >> (32 - PWM_WAVEFORM_INDEX_BITS)];
}

// highest priority -> 5 -> 6
// may need to be fixed for priority inversion
void ButtonTask_func(UArg arg1, UArg arg2)
{
//    count_unloaded = cpu_load_count();
//    count_loaded = 0;
//    cpu_load = 0.0f;


    while (true) {
        // wait for the interrupt to occur
        //Semaphore_pend(button_time_sem, BIOS_WAIT_FOREVER);

        // wait for the signal
        Semaphore_pend(button_sem, BIOS_WAIT_FOREVER);

        /*
        // measuring latency
        t = TIMER0_PERIOD - TIMER0_TAR_R;
        if (t > button_task_latency) button_task_latency = t; // measure latency
        */

        // after getting signaled
        uint32_t presses = button_func();
        char msg;

        if(presses & VOLT_DEC) { // left side
            msg = '6';
            Mailbox_post(button_mailbox, &msg, BIOS_WAIT_FOREVER);
        }
        if(presses & VOLT_INC) {  // right side
            msg = '5';
            Mailbox_post(button_mailbox, &msg, BIOS_WAIT_FOREVER);
        }
        if(presses & TRIG_CHANGE) { // user switch 1
            msg = '2';
            Mailbox_post(button_mailbox, &msg, BIOS_WAIT_FOREVER);
        }
        if(presses & MODE_CHANGE) { // user switch 2
            msg = '3';
            Mailbox_post(button_mailbox, &msg, BIOS_WAIT_FOREVER);
        }

        /* From lab2
        // measuring response time and missing deadline
        if (Semaphore_getCount(button_time_sem)) { // next event occurred
            button_task_missed++;
            t = 2 * TIMER0_PERIOD; // timer overflowed since last event
        }
        else t = TIMER0_PERIOD;
        t -= TIMER0_TAR_R;
        if (t > button_task_response) button_task_response = t; // measure response time
        */


        //count_loaded = cpu_load_count();
        //cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load

    }
}

// high priority -> 4 -> 5
void WaveformTask_func(UArg arg1, UArg arg2)
{
    count_unloaded = cpu_load_count();
    count_loaded = 0;
    cpu_load = 0.0f;
    IntMasterEnable();
    bool curr_trig;
    int32_t i, search_count;
    int32_t trig_idx;   // trigger index
    volatile int32_t curr_idx;   // current index of ADC when loaded

    while (true) {
        // wait for the signal
        Semaphore_pend(trigger_sem, BIOS_WAIT_FOREVER);

        start = SysTickValueGet(); // read SysTick timer value

        if(FFTMode) { // ====== FFT MODE ====== //
            // copy new data to FFTBuffer
            Semaphore_pend(fft_data_sem, BIOS_WAIT_FOREVER);
            for(search_count = 0, i = gADCBufferIndex; search_count < NFFT; search_count++, i = ADC_BUFFER_WRAP(i - 1))
                FFTBuffer[search_count] = gADCBuffer[i];
            Semaphore_post(fft_data_sem);

        } else { // ====== NORMAL MODE ====== //
            // Create buffer to create oscilloscope
            curr_idx = getADCBufferIndex();
            trig_idx = ADC_BUFFER_WRAP(curr_idx - HALF_SCREEN);
            curr_trig = risingTrig;
            search_count = 0;
            if(gADCBuffer[trig_idx] != ADC_OFFSET){
                if(curr_trig)
                    while(!((gADCBuffer[ADC_BUFFER_WRAP(trig_idx + 1)] >= ADC_OFFSET) && (gADCBuffer[trig_idx] < ADC_OFFSET)) && (++search_count < SEARCH_REGION))
                        trig_idx = ADC_BUFFER_WRAP(trig_idx - 1);
                else
                    while(!((gADCBuffer[ADC_BUFFER_WRAP(trig_idx + 1)] <= ADC_OFFSET) && (gADCBuffer[trig_idx] > ADC_OFFSET)) && (++search_count < SEARCH_REGION))
                        trig_idx = ADC_BUFFER_WRAP(trig_idx - 1);

                if(search_count >= SEARCH_REGION)
                    trig_idx = ADC_BUFFER_WRAP(curr_idx - HALF_SCREEN);
            }

            // copy the the buffer to global variable
            Semaphore_pend(waveform_data_sem, BIOS_WAIT_FOREVER);
            for(i = 0; i < HALF_SCREEN * 2; i++)
                waveformBuffer[i] = gADCBuffer[ADC_BUFFER_WRAP(trig_idx + i - HALF_SCREEN + 1)];
            Semaphore_post(waveform_data_sem);

        }

        finish = SysTickValueGet(); // read SysTick timer value
        duration = (start - finish) & 0xffffff;
        if(max_duration < duration) max_duration = duration;

        // signal process to process data
        Semaphore_post(process_sem);
    }
}

// new low priority -> _ -> 4
void FrequencyTask_func(UArg arg1, UArg arg2){

    uint32_t period, count;
    while(true){

        Semaphore_pend(freq_sem, BIOS_WAIT_FOREVER);

        // retrieve global data
        IArg key = GateHwi_enter(gateHwi0);

        period = accu_period;
        count = period_count;
        accu_period = 0;
        period_count = 0;

        GateHwi_leave(gateHwi0, key);

        // record the data to shared data
        freq_data = (float)gSystemClock / (float)period * (float)count;
    }
}

// medium priority -> 3 -> 3
void UserInputTask_func(UArg arg1, UArg arg2)
{

    while (true) {
        // read the value from mailbox
        char msg;
        Mailbox_pend(button_mailbox, &msg, BIOS_WAIT_FOREVER);

        // modified shared data for display
        switch(msg){
           case '6': // decrease value of voltage
               currentVoltSel = currentVoltSel > 0? currentVoltSel - 1 : 0;
               break;
           case '5': // increase value of voltage
               currentVoltSel = currentVoltSel < 3? currentVoltSel + 1 : 3;
               break;
           case '2':
               risingTrig = !risingTrig;
               break;
           case '3':
               FFTMode = !FFTMode;
               break;
       }

       // signal display function
       Semaphore_post(display_sem);


    }
}

// low priority -> 2 -> 2
void DisplayTask_func(UArg arg1, UArg arg2){



    int buffer[LCD_HORIZONTAL_MAX], i;
    bool curr_trig;
    int curr_volt_stage;
    tRectangle rectFullScreen = {0, 0, GrContextDpyWidthGet(&sContext)-1, GrContextDpyHeightGet(&sContext)-1};
    bool fft;
    int x;
    char strbuf[50];

    while (true) {
        // wait for signal
        Semaphore_pend(display_sem, BIOS_WAIT_FOREVER);

        fft = FFTMode;

        // read the data; if the semaphore is held -> ignore and use previous data
        if(Semaphore_pend(display_data_sem, BIOS_NO_WAIT)){
            curr_trig = risingTrig;
            curr_volt_stage = currentVoltSel;
            for(i = 0; i < LCD_HORIZONTAL_MAX; i++)
                buffer[i] = displayBuffer[i];
            Semaphore_post(display_data_sem);
        }
        // start drawing
        GrContextForegroundSet(&sContext, ClrBlack);
        GrRectFill(&sContext, &rectFullScreen); // fill screen with black

        GrContextForegroundSet(&sContext, ClrBlue); // blue
        if(fft){
            // draw vertical line
            for (x = 0; x <= 124; x = x + 20)
                GrLineDraw(&sContext, x, 0, x, LCD_VERTICAL_MAX);
            for (x = 0; x <= 124; x = x + 20)
                GrLineDraw(&sContext, 0, x, LCD_HORIZONTAL_MAX, x);
        } else {
            // draw vertical line
            for (x = 4; x <= 124; x = x + 20)
                GrLineDraw(&sContext, x, 0, x, LCD_VERTICAL_MAX);
            GrLineDraw(&sContext, 63, 0, 63, LCD_VERTICAL_MAX);
            for (x = 4; x <= 124; x = x + 20)
                GrLineDraw(&sContext, 0, x, LCD_HORIZONTAL_MAX, x);
            GrLineDraw(&sContext, 0, 63, LCD_HORIZONTAL_MAX, 63);
        }

        // draw waveform
        GrContextForegroundSet(&sContext, ClrYellow); // yellow
        // change buffer to position on LCD
        for(i = 1; i < LCD_HORIZONTAL_MAX; i++)
            GrLineDraw(&sContext, i - 1, buffer[i - 1], i, buffer[i]);

        GrContextForegroundSet(&sContext, ClrWhite); // white


        // draw string
        sprintf(strbuf, "f = %.3f Hz", freq_data);
        GrStringDraw(&sContext, strbuf, -1, 5, 120, false);
        if(fft){
            GrStringDraw(&sContext, "20 KHz", /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);
            GrStringDraw(&sContext, "20 dB", /*length*/ -1, /*x*/ 40, /*y*/ 0, /*opaque*/ false);

        } else {
            // printf context on screen
            GrStringDraw(&sContext, "20us", /*length*/ -1, /*x*/ 0, /*y*/ 0, /*opaque*/ false);
            GrStringDraw(&sContext, gVoltageScaleStr[curr_volt_stage], /*length*/ -1, /*x*/ 40, /*y*/ 0, /*opaque*/ false);
            if(curr_trig){
                GrLineDraw(&sContext, 85, 7, 90, 7);
                GrLineDraw(&sContext, 90, 2, 95, 2);
                GrLineDraw(&sContext, 90, 2, 90, 7);
            }
            else{
                GrLineDraw(&sContext, 90, 7, 95, 7);
                GrLineDraw(&sContext, 85, 2, 90, 2);
                GrLineDraw(&sContext, 90, 2, 90, 7);
            }
        }

        GrFlush(&sContext); // flush the frame buffer to the LCD

        count_loaded = cpu_load_count();
        cpu_load = 1.0f - (float)count_loaded/count_unloaded; // compute CPU load


    }
}



// lowest priority -> 1 -> 1
void ProcessingTask_func(UArg arg1, UArg arg2)
{

    TimerEnable(TIMER0_BASE, TIMER_A);

    int curr_volt_stage;
    float fScale;
    int i, temp;

    // FFT
    static char kiss_fft_cfg_buffer[KISS_FFT_CFG_SIZE];
    size_t buffer_size = KISS_FFT_CFG_SIZE;
    kiss_fft_cfg cfg = kiss_fft_alloc(NFFT, 0, kiss_fft_cfg_buffer, &buffer_size);
    static kiss_fft_cpx in[NFFT], out[NFFT];


    // buffer
    uint16_t inputBuffer[LCD_HORIZONTAL_MAX];
    int outputBuffer[LCD_HORIZONTAL_MAX], minOutput;
    while (true) {
        // wait for signal
        Semaphore_pend(process_sem, BIOS_WAIT_FOREVER);

        // read shared data
        curr_volt_stage = currentVoltSel;

        fScale = (VIN_RANGE * PIXELS_PER_DIV) / ((1 << ADC_BITS) * fVoltsPerDiv[curr_volt_stage]);

        if(FFTMode){ // ====== FFT Mode ====== //
            // copy the waveform data to local variable if available
            if(Semaphore_pend(fft_data_sem, BIOS_NO_WAIT)){
                for(i = 0; i < NFFT; i++){
                    in[i].r = (float)(FFTBuffer[i] - ADC_OFFSET) * VIN_RANGE / (1 << ADC_BITS);
                    //in[i].r = sinf(20 * PI * i / NFFT);
                    in[i].i = 0;
                }
                Semaphore_post(fft_data_sem);
            }
            // compute FFT
            kiss_fft(cfg, in, out);

            // use only 128 lowest bin
            minOutput = 10000;
            for(i = 0; i < LCD_HORIZONTAL_MAX; i++){
                // 20 * log10(abs) = 20 * 1/2 * log10(abs^2)
                outputBuffer[i] = (int)(10.0f * log10f(out[i].r * out[i].r + out[i].i * out[i].i));
                minOutput = minOutput <= outputBuffer[i] ? minOutput : outputBuffer[i];
            }

            for(i = 0; i < LCD_HORIZONTAL_MAX; i++)
                outputBuffer[i] = /*LCD_VERTICAL_MAX*/124 - (outputBuffer[i] - minOutput);


        } else { // ====== Normal Mode ====== //
            // copy the waveform data to local variable if available
            if(Semaphore_pend(waveform_data_sem, BIOS_NO_WAIT)){
                for(i = 0; i < LCD_HORIZONTAL_MAX; i++)
                    inputBuffer[i] = waveformBuffer[i];
                Semaphore_post(waveform_data_sem);
            }

            // update data to local buffer
            for(i = 0; i < LCD_HORIZONTAL_MAX; i++){
                temp = (int)roundf(fScale * ((int)inputBuffer[i] - ADC_OFFSET));
                if(temp > LCD_VERTICAL_MAX / 2)
                    outputBuffer[i] = -1;
                else if(temp < - LCD_VERTICAL_MAX / 2)
                    outputBuffer[i] = LCD_VERTICAL_MAX;
                else
                    outputBuffer[i] = LCD_VERTICAL_MAX / 2 - temp;
            }
        }

        // update data to global buffer
        Semaphore_pend(display_data_sem, BIOS_WAIT_FOREVER);
        for(i = 0; i < LCD_HORIZONTAL_MAX; i++)
            displayBuffer[i] = outputBuffer[i];
        Semaphore_post(display_data_sem);



        // signal Display and then Waveform
        Semaphore_post(display_sem);
        Semaphore_post(trigger_sem);


    }
}

void Frequency_tick_func(){
    Semaphore_post(freq_sem);
}

void Clock_tick_func(){
    Semaphore_post(button_sem);
}



