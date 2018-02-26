#include "mcc_generated_files/mcc.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "mcc_generated_files/EXAMPLE_BLE2.h"

// Size of (circular)buffer for acquired emg samples
#define SB_DATA_WINDOW 50

// Sampling frequency of signal. 50Hz = 20ms (Timer interrupt configured to 20ms)
#define SAMPLE_FREQUENCY 50
#define MIN_FREQUENCY 25
#define MAX_FREQUENCY 150

// The minimum distance in data points that two registered peaks can be
#define MIN_PK_GAP ((int)(SAMPLE_FREQUENCY / MAX_FREQUENCY))
// The maximum distance two consecutive peaks can be without the calculated neutral-point shifting significantly
#define MAX_PK_GAP ((int)(SAMPLE_FREQUENCY / MIN_FREQUENCY))
// Size of (circular) buffer for peak filter. Twice the maximum gap
#define PK_DATA_WINDOW (2*MAX_PK_GAP+1)

// Size of (circular) buffer for moving average filter. Considers last 0.5 seconds of data for moving average 
#define MA_DATA_WINDOW ((int)(SAMPLE_FREQUENCY * 0.5)+1)

// Actual buffer for emg data and indices to keep track of the circular buffer
uint16_t sb_data[SB_DATA_WINDOW];
int8_t sb_front = -1;
int8_t sb_rear = -1;

// Actual buffer for peak filter and indices to keep track of the circular buffer
uint16_t pk_data[PK_DATA_WINDOW];
int8_t pk_front = -1;
int8_t pk_rear = -1;

// Actual buffer for moving average filter and indices to keep track of the circular buffer
uint16_t ma_data[MA_DATA_WINDOW];
int8_t ma_front = -1;
int8_t ma_rear = -1;
// Sum of all elements in a moving average window
uint16_t ma_window_sum = 0;

// Flag that starts data acquisition
uint8_t start_flag = 0;
uint8_t i;

uint8_t sent_1 =0;
uint8_t sent_0 =0;

uint16_t pushup_count = 0;

// Returns true if circular buffer is full
bool sbuf_isfull() {
    if ((sb_front == sb_rear + 1) || (sb_front == 0 && sb_rear == SB_DATA_WINDOW - 1))
        return true;
    else
        return false;
}

// Returns true if circular buffer is empty
bool sbuf_isempty() {
    if (sb_front == -1)
        return true;
    else
        return false;
}

// Inserts element to the circular buffer
bool sbuf_insert(uint16_t element) {
    if (sbuf_isfull()) {
        // Can't insert data because buffer is full
        return false;
    } else {
        if (sb_front == -1)
            sb_front = 0;
        sb_rear = (sb_rear + 1) % SB_DATA_WINDOW;
        sb_data[sb_rear] = element;
        return true;
    }
}

// Removes element from the circular buffer
bool sbuf_remove() {
    uint16_t element;
    if (sbuf_isempty()) {
        return false;
    } else {
        element = sb_data[sb_front];
        if (sb_front == sb_rear) {
            sb_front = -1;
            sb_rear = -1;
        } else {
            sb_front = (sb_front + 1) % SB_DATA_WINDOW;
        }
        return true;
    }
}

// Returns the first inserted element (FIFO) from the circular buffer
uint16_t sbuf_peek(){
  return sb_data[sb_front];
}

bool pkdata_isfull() {
    if ((pk_front == pk_rear + 1) || (pk_front == 0 && pk_rear == PK_DATA_WINDOW - 1))
        return true;
    else
        return false;
}

// Similar circular buffer functions for peak filter

bool pkdata_isempty() {
    if (pk_front == -1)
        return true;
    else
        return false;
}

bool pkdata_insert(uint16_t element) {
    if (pkdata_isfull()) {
        // Can't insert data because buffer is full
        return false;
    } else {
        if (pk_front == -1)
            pk_front = 0;
        pk_rear = (pk_rear + 1) % PK_DATA_WINDOW;
        pk_data[pk_rear] = element;
        return true;
    }
}

bool pkdata_remove() {
    uint16_t element;
    if (pkdata_isempty()) {
        return false;
    } else {
        element = pk_data[pk_front];
        if (pk_front == pk_rear) {
            pk_front = -1;
            pk_rear = -1;
        } else {
            pk_front = (pk_front + 1) % PK_DATA_WINDOW;
        }
        return true;
    }
}

// Similar circular buffer functions for moving average filter

bool madata_isfull() {
    if ((ma_front == ma_rear + 1) || (ma_front == 0 && ma_rear == MA_DATA_WINDOW - 1))
        return true;
    else
        return false;
}

bool madata_isempty() {
    if (ma_front == -1)
        return true;
    else
        return false;
}

bool madata_insert(uint16_t element) {
    if (madata_isfull()) {
        // Can't insert data because buffer is full
        return false;
    } else {
        if (ma_front == -1)
            ma_front = 0;
        ma_rear = (ma_rear + 1) % MA_DATA_WINDOW;
        ma_data[ma_rear] = element;
        return true;
    }
}

bool madata_remove() {
    uint16_t element;
    if (madata_isempty()) {
        return false;
    } else {
        element = ma_data[ma_front];
        if (ma_front == ma_rear) {
            ma_front = -1;
            ma_rear = -1;
        } else {
            ma_front = (ma_front + 1) % MA_DATA_WINDOW;
        }
        return true;
    }
}

// Actual Peak Filter. returns the neutral_datapoint calculated using (highest_peak+lowest_peak)/2. 
// Highest peak is the maximum among the elements present in the "peak filter buffer" (i.e pk_data which is of size PK_DATA_WINDOW)
// Highest peak is the minimum among the elements present in the "peak filter buffer" (i.e pk_data which is of size PK_DATA_WINDOW)
uint16_t get_neutral_peaktopeak(uint16_t datapoint) {
    // Insert every new data into the peak buffer
    pkdata_insert(datapoint);

    // If buffer is full, remove the first inserted data (FIFO)
    if (pkdata_isfull()) {
        pkdata_remove();
    }

    // Start off with first datapoint for highest
    uint16_t highest_peak = pk_data[pk_front];
    uint16_t lowest_peak = pk_data[pk_front];
    uint16_t neutral;
    uint8_t i;
    // Running through only valid elements of the array
    for (i = pk_front; i != pk_rear; i = (i + 1) % PK_DATA_WINDOW) {
        if (pk_data[i] > highest_peak) {
            // Reassign variable highest_peak if a new highest is found among elements of "peak filter buffer"
            highest_peak = pk_data[i];
        }
        if (pk_data[i] < lowest_peak) {
            // Reassign variable lowest_peak if a new lowest is found among elements of "peak filter buffer"
            lowest_peak = pk_data[i];
        }
    }

    // Calculate neutral and return
    neutral = (highest_peak + lowest_peak) / 2;
    return neutral;
}

// Moving Average Filter. Returns the average of datapoints present in the "moving average buffer"
// Doesn't actually loop through each element in buffer to calculate average. The sum in updated for every new datapoint added(and removed)
float get_moving_average(uint16_t datapoint) {
    madata_insert(datapoint);
    // ma_window_sum starts at 0. Every new data point is added to this sum.
    ma_window_sum += datapoint;

    if (madata_isfull()) {
        // When the buffer is full, the last element is subtracted from the sum and then removed from the buffer
        ma_window_sum -= ma_data[ma_front];
        madata_remove();
    }

    // MA_DATA_WINDOW is 1 more than intended Moving Average Window
    return ma_window_sum / (MA_DATA_WINDOW-1);
}

// Interrupt handler that is called every 20ms, thus sampling the EMG signal at 50Hz
void TMR6_EMG_InterruptHandler(void)
{
    if (start_flag == 1) {
        //ADCC_StartConversion(POT_RA0);
        ADCC_StartConversion(EMG_RA2);
        adc_result_t adval = ADCC_GetConversionResult();
        // Every new sample point is added to the "Signal Circular Buffer"
        // Data is "buffered" (until the limit of 50 datapoints) so the main loop can get the first inserted element (FIFO)
        // whenever it is free and process it.
        sbuf_insert(adval/100);
    }
}


void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    // ^-^

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    // Initialize custom interrupt handler which acquires data from ADC
    TMR6_SetInterruptHandler(TMR6_EMG_InterruptHandler);
    TMR6_Start();
    
    EXAMPLE_setupBLE2("Dora");
    
    char msg[30];
    
    int count=0;
    uint16_t neutral_datapoint, result, datapoint;
    double time_elapsed;
    uint8_t flex_flag;
    flex_flag = 0;

    LED_RA7_SetLow();
    while (1)
    {            
        if (START_RC5_GetValue() == 0 && start_flag == 0) {
            //printf("START\r\n");
            start_flag = 1;
            __delay_ms(700);
            break;
        }
    }
    
    while (1)
    {
        if(start_flag == 1)
        {
            // Count the number of datapoints present in signal buffer
            // todo... is count required anymore ?
            for (i = sb_front; i != sb_rear; i = (i + 1) % SB_DATA_WINDOW) {
                count++;
            }

            if(count>0)
            {
                // Get the first inserted datapoint (in FIFO order)
                datapoint = sbuf_peek();
                // Calculates the neutral peak by supplying the datapoint to the peak filter
                neutral_datapoint = get_neutral_peaktopeak(datapoint);
                // Uses the neutral point to subtract it from the current datapoint.Rectifies (by taking absolute) the signal
                result = get_moving_average(abs(datapoint - neutral_datapoint));
                
                // Turn/Turn back motor when flexed.
                if(result>= 25 && flex_flag == 0)
                {
                    flex_flag = 1;
                    pushup_count += 1;
                    sprintf(msg, "Push up count - %d ", pushup_count);
                    EXAMPLE_sendMessageOverBLE2(msg);              
                    //printf(" You have Pushed up %d times. You can do better :D \r\n",pushup_count);
                }
                else if(result<25 && flex_flag == 1)
                {
                    flex_flag = 0;
                }
                // todo...Reject/ donot consider result until atleast 2*MIN_PK_GAP is calculated ????
                
                // Remove the first element (in FIFO order) since it has already been processed
                sbuf_remove();
                time_elapsed += 5.0;
            }

            count = 0;
        }
    }
}
/**
 End of File
*/
