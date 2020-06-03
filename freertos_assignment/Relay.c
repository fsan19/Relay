// Part of the file provided by Terasics
#include "system.h"
// Standard includes
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <io.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

// Scheduler includes
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

// Altera Peripherals
#include <altera_avalon_pio_regs.h>
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_avalon_sysid_qsys.h"
#include "altera_avalon_sysid_qsys_regs.h"
#include "alt_types.h"
#include <sys/alt_alarm.h>

// Definition of Task Stacks
#define TASK_STACKSIZE       2048

// Definition of Task Priorities
#define VGA_TASK_PRIORITY 1
#define KEYBOARD_TASK_PRIORITY 4
#define FREQUENCY_UPDATER_TASK_PRIORITY 5
#define LOAD_MANAGER_TASK_PRIORITY 2
//Timer Vars

// 500ms for Stability Observation
#define TIMER_PERIOD (500)/portTICK_PERIOD_MS
// FreeRTOS Timer
TimerHandle_t timer500ms;

//Declaration of Mutexes
SemaphoreHandle_t thresholdSemaphore;
// LCD macros
#define ESC 27
#define CLEAR_LCD_STRING "[2J"
/*#################################################################
#######################HW PERIPHERALS #############################
################################################################### */
// NOTE: VGA Task only
alt_up_pixel_buffer_dma_dev *pixel_buf;
alt_up_pixel_buffer_dma_dev *char_buf;
// NOTE: Keyboard Task only
FILE *lcd;

/*#################################################################
#####################GLOBAL VAR (MUTEX+NON-MUTEX) #################
################################################################### */
uint8_t wasStable = 1;
/*#################### Frequency and RoC Data ##################### */
float frequencyData[50];
float rocData[50];
int runningDataIndex;

/*#################### Time Reaction Data ########################## */
int reactionTimes[5] ={0,0,0,0,0};
unsigned int reactionTimeIndex = 0;
int avgReactionTime,totalTime = 0;
// 10^6 = 1000 seconds (Arbitrary Large Value)
int minReactionTime = 1000000;
int maxReactionTime = 0;

/*#################### Load Status ################################# */

#define ALLOFF 0
#define ALLON  31

#define LOAD4  16
#define LOAD3  8
#define LOAD2  4
#define LOAD1  2
#define LOAD0  1

uint8_t loads[5] = {1,2,4,8,16};
uint8_t loadStatus = ALLON;

/*#################### Frequency & RoC Thresholds ################## */
// 30.0HZ
float frequencyThreshold = 30.0;
// 300 = 30.0 Hz/Second
int rocThreshold = 300;

/*#################### Timer Expiry Flag ########################### */
uint8_t timerExpiryFlag = 0;
/*#################### Maintainence Mode Flag ###################### */
uint8_t maintainenceModeEn = 0;

/*#################################################################
############################### Queues ############################
################################################################### */
xQueueHandle frequencyQ;
xQueueHandle ps2KeyQ;
xQueueHandle freqRocDataQ;

/*#################################################################
####################### COMPOUND TYPES ############################
################################################################### */

struct freqRocQMsg
 {
	float freqData;
    float rocData;
    int timestamp;

 };

struct freqQMsg
 {
	float frequency;
    int timestamp;

 } freqISRMsg;

/*#################################################################
############################### PROTOTYPES ########################
################################################################### */

/*####################### Init Function Prototypes ############### */
void initOSDataStructs(void);
void initCreateTasks(void);
void initPeripheralsAndIsrs(void);
/*####################### Init ISR Prototypes ################## */
void setupKeyboardISR(void);
void setupButtonsISR(void);
void setupVGA(void);
/*####################### ISR Prototypes ########################## */
void frequencyAnalyserISR(void* context, alt_u32 id);
void ps2ISR (void* context, alt_u32 id);
void buttonISR (void* context, alt_u32 id);
/*####################### Timer Callback ######################### */
void vTimer500MSCallback(xTimerHandle t_timer);
/*####################### Tasks Prototypes ######################### */
void vgaTask(void *pvParameters);
void keyboardManagerTask(void *pvParameters);
void loadManagerTask(void *pvParameters);
void frequencyUpdaterTask(void *pvParameters);
/*####################### Helper Prototypes ######################### */
void stopFreeRTOSTimer(void);
void restartFreeRTOSTimer(void);
uint8_t checkTrippingConditions(struct freqRocQMsg freqRocMsg, float freqThresholdLocal, int rocThresholdLocal);
void updateSwitches(uint8_t SWITCHES[]);
int loadUpdater(int reqTime, uint8_t SWITCHES[]);
uint8_t reconnectLoad(uint8_t SWITCHES[]);
uint8_t shedLoad(uint8_t SWITCHES[]);
void computeReactionTimeStats(int currentTime,struct freqRocQMsg freqRocMsg);
void updateRunningData(struct freqRocQMsg freqRocMsg);
void manualCheckAndSwitchOffLoads(uint8_t SWITCHES[]);
/*####################### Test Prototypes ######################### */
void testLoadSheddingAndReconnecting();
void testComputeReactionTimeStats();
void testUpdateRunningData();
void testManualSwitchOffLoad1();
void testManualSwitchOffLoad2();
/*##################################################################
############################### ISR CODE ###########################
#################################################################### */

/*Maintenance Mode Enable/Disable*/
void buttonISR (void* context, alt_u32 id){

	  // need to cast the context first before using it
	  uint8_t buttonValue;
	  buttonValue = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	  //Key3
	  maintainenceModeEn = !maintainenceModeEn;
	  // clears the edge capture register
	  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x4);

}

/*ADC ISR Values for frequency and timestamp generation*/
void frequencyAnalyserISR(void* context, alt_u32 id){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	freqISRMsg.frequency = 16000/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);
	freqISRMsg.timestamp = xTaskGetTickCountFromISR();

	xQueueSendToBackFromISR(frequencyQ, &freqISRMsg, &xHigherPriorityTaskWoken);

	return;
}

/*Keyboard Event Detection and Handling*/
void ps2ISR (void* context, alt_u32 id)
{

  char ascii;
  int status = 0;
  unsigned char key = 0;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii);

  if ( status == 0  ) //success
  {

    switch ( decode_mode )
    {
      case KB_ASCII_MAKE_CODE :

    	//Write to Queue
		xQueueSendToBackFromISR(ps2KeyQ, &key, &xHigherPriorityTaskWoken);

        break ;
      case KB_LONG_BINARY_MAKE_CODE :
        // do nothing
		break;
      case KB_BINARY_MAKE_CODE :
    	//Write to Queue
    	xQueueSendToBackFromISR(ps2KeyQ, &key, &xHigherPriorityTaskWoken);
        break ;
      case KB_BREAK_CODE :
        // do nothing

		break;
      default :
    	// do nothing
        break ;
    }
    // Display key value of Seven Seg Display (To check keyboard connectivity)
    IOWR(SEVEN_SEG_BASE,0 ,key);

  }

}

/*Init VGA*/
void setupVGA(){

	// init pixel buffer

	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);

	if(pixel_buf == NULL){
		printf("Cannot find pixel buffer device\n");
	}else{
		alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);
		printf("Initialised pixel buffer\n");

	}
	// initialize character buffer

	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");

	if(char_buf == NULL){
		printf("Cannot find char buffer device\n");
	}else{
		alt_up_char_buffer_clear(char_buf);
		printf("Initialised char buffer\n");

	}
}

/*Init PS2 keyboard and register ISR*/
void setupKeyboardISR(){
  alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

  if(ps2_device == NULL){
    printf("Can't find PS/2 Keyboard\n");
  }else{
    printf("Connected to PS/2 Keyboard\n");
  }

  alt_up_ps2_clear_fifo (ps2_device) ;

  alt_irq_register(PS2_IRQ, ps2_device, ps2ISR);
  // register the PS/2 interrupt
  IOWR_8DIRECT(PS2_BASE,4,1);

  printf("Registered PS2 ISR!\n");

}

/*Init button ISR. Key 3 only*/
void setupButtonsISR(){
  //@detail Initialises buttons and registers for ISR

   // clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x4);

   // enable interrupts for all buttons
  IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x4);

  // register the ISR
  alt_irq_register(PUSH_BUTTON_IRQ,0, buttonISR);

  printf("Registered Button ISR!\n");

}

/*##################################################################
################### INIT FUNCTIONS  ################################
########### (TASKS + PERIPHERALS + ISRS + DATA STRUCTS)#############
#################################################################### */

void initCreateTasks()
{
	/*INIT TASKS*/
	xTaskCreate (vgaTask,"vgaTask", TASK_STACKSIZE, NULL, VGA_TASK_PRIORITY, NULL);
	xTaskCreate(keyboardManagerTask, "keyboardManagerTask", TASK_STACKSIZE, NULL, KEYBOARD_TASK_PRIORITY, NULL);
	xTaskCreate(frequencyUpdaterTask, "frequencyUpdaterTask", TASK_STACKSIZE,NULL,FREQUENCY_UPDATER_TASK_PRIORITY,NULL);
	xTaskCreate(loadManagerTask, "loadManagerTask", TASK_STACKSIZE,NULL,LOAD_MANAGER_TASK_PRIORITY,NULL);

	return;
}

void initPeripheralsAndIsrs(){
	setupKeyboardISR();
	setupButtonsISR();
	setupVGA();

	// setup freq isr
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, frequencyAnalyserISR);
    // create Timer for Stability Observation
	timer500ms = xTimerCreate("Timer", TIMER_PERIOD, pdFALSE, NULL, vTimer500MSCallback);

	// Init LEDS
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, ALLON);
	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ALLOFF);
}




/*This function creates communication data structures for Tasks and ISRs*/
void initOSDataStructs()
{
	/*INIT Q's*/
	ps2KeyQ = xQueueCreate(100, sizeof(uint32_t));
	frequencyQ = xQueueCreate(100, sizeof(struct freqQMsg));
	freqRocDataQ = xQueueCreate(50, sizeof( struct freqRocQMsg));

	/*INIT Mutexes*/
	thresholdSemaphore = xSemaphoreCreateMutex();

	return;
}

/*##################################################################
############################### Timer Callback #####################
#################################################################### */
void vTimer500MSCallback(xTimerHandle t_timer){

	// Set Timer Exp Flag
	timerExpiryFlag = 1;
	printf("\n\n################Timer Expired!##############\n\n");
}



/*##################################################################
############################### MAIN() ##############################
#################################################################### */
int main(int argc, char* argv[], char* envp[])
{
	initPeripheralsAndIsrs();
	initOSDataStructs();
	printf("Struct initialised!\n");
	initCreateTasks();
	printf("Tasks initialised!\n");

	vTaskStartScheduler();

	/*Code is not executed after Scheduler Starts  */

	for (;;);
	return 0;
}

/*##################################################################
############################### TASKS ##############################
#################################################################### */

/*Renders:
 * Running data(Frequency and Roc)
 * Response Time Parameters
 * Thresholds
 * System Time
 * System Stability
 * */
void vgaTask(void *pvParameters){


	#define green 0x3ff
	#define blue  0x3ff<<20
	#define red   0x3ff<<10
	#define baseROC 0
	#define perPixelROC 4.8
	#define baseFreq 45
	#define perPixelFreq 25.6

	/*Variables for plotting (Used by Frequency and RoC */
	unsigned int p1Y,p2Y,p3Y,p4Y,p5Y;
	unsigned int startX  ;
	unsigned int dpWidth ;
	unsigned long currentSystemTime = 0;
	char str[10] = "str";



	while(1){
		// Clear the screen
		alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);
		alt_up_char_buffer_clear(char_buf);
		// Freq
		alt_up_char_buffer_string(char_buf, "Lower Threshold:", 10, 50);
		alt_up_char_buffer_string(char_buf, "Hz", 35, 50);
		// RoC
		alt_up_char_buffer_string(char_buf, "RoC Threshold:", 10, 52);
		alt_up_char_buffer_string(char_buf, "Hz/Sec", 30,52);
		// System wasStable
		alt_up_char_buffer_string(char_buf, "System Status", 40, 50);
		// Reaction Time
		alt_up_char_buffer_string(char_buf, "Reaction Time(ms)", 55, 50);
		// Total Runtime
		alt_up_char_buffer_string(char_buf, "Total Run Time", 10, 55);
		/*Graph Below */

		// Y Axis Line
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 150, 20, 350, blue, 0);
		// Y Label 1
		alt_up_char_buffer_string(char_buf, "Frequency(Hz)",2, 2);
		// Y Label 2
		alt_up_char_buffer_string(char_buf, "dF/dt(Hz/Sec)",2, 24);
		// Y Grid 1
		alt_up_char_buffer_string(char_buf, "50",16, 3);
		// Y Grid 2
		alt_up_char_buffer_string(char_buf, "47.5",14, 11);
		// Y Grid 2
		alt_up_char_buffer_string(char_buf, "45",16, 19);
		//Roc Baseline
		alt_up_char_buffer_string(char_buf, "0",16, 42);
		alt_up_char_buffer_string(char_buf, "10",16, 36);
		alt_up_char_buffer_string(char_buf, "20",16, 30);
		alt_up_char_buffer_string(char_buf, "30",16, 24);
		// X Axis Frequency Plot
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 135, 550, 180,blue, 0);
		// X Axis RoC Plot
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 135, 550, 346,blue, 0);
		//X label 1
		alt_up_char_buffer_string(char_buf, "Time",70, 24);
		//X label 2
		alt_up_char_buffer_string(char_buf, "Time",70, 43);

		/*Calculations : Frequency */

		p1Y = (int)(152 - (int)( (frequencyData[0]-baseFreq) * perPixelFreq) );
		p2Y = (int)(152 - (int)( (frequencyData[1]-baseFreq) * perPixelFreq) );
		p3Y= (int)(152 - (int)( (frequencyData[2]-baseFreq)  * perPixelFreq) );
		p4Y = (int)(152 - (int)( (frequencyData[3]-baseFreq) * perPixelFreq) );
		p5Y = (int)(152 - (int)( (frequencyData[4]-baseFreq) * perPixelFreq) );
		/*Plot Settings */
		startX = 150;
		dpWidth = 80;
		/* Freq Plot */
		// Point 1
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX, startX+1*dpWidth, p1Y,red, 0);
		// Vert Line 1-2
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+1*dpWidth, p1Y, p2Y, red, 0);
		// Point 2
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+1*dpWidth, startX+2*dpWidth, p2Y,red, 0);
		// Vert Line 2-3
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+2*dpWidth, p2Y, p3Y, red, 0);
		// Point 3
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+2*dpWidth, startX+3*dpWidth, p3Y,red, 0);
		// Vert Line 3-4
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+3*dpWidth, p3Y, p4Y, red, 0);
		// Point 4
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+3*dpWidth, startX+4*dpWidth, p4Y,red, 0);
		// Vert Line 4-5
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+4*dpWidth, p4Y, p5Y, red, 0);
		// Point 5
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+4*dpWidth, startX+5*dpWidth, p5Y,red, 0);
		/*RoC Plot*/
		/*Calculations : RoC */
		p1Y = (int)(336 - (int)( (fabs(rocData[0])-baseROC)  * perPixelROC) );
		p2Y = (int)(336 - (int)( (fabs(rocData[1])-baseROC)  * perPixelROC) );
		p3Y= (int)(336 - (int)( (fabs(rocData[2])-baseROC)   * perPixelROC) );
		p4Y = (int)(336 - (int)( (fabs(rocData[3])-baseROC)  * perPixelROC) );
		p5Y = (int)(336 - (int)( (fabs(rocData[4])-baseROC)  * perPixelROC) );
		/*Plot Settings */
		startX = 150;
		dpWidth = 80;
		/* Freq Plot */
		// Point 1
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX, startX+1*dpWidth, p1Y,red, 0);
		// Vert Line 1-2
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+1*dpWidth, p1Y, p2Y, red, 0);
		// Point 2
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+1*dpWidth, startX+2*dpWidth, p2Y,red, 0);
		// Vert Line 2-3
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+2*dpWidth, p2Y, p3Y, red, 0);
		// Point 3
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+2*dpWidth, startX+3*dpWidth, p3Y,red, 0);
		// Vert Line 3-4
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+3*dpWidth, p3Y, p4Y, red, 0);
		// Point 4
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+3*dpWidth, startX+4*dpWidth, p4Y,red, 0);
		// Vert Line 4-5
		alt_up_pixel_buffer_dma_draw_vline(pixel_buf, startX+4*dpWidth, p4Y, p5Y, red, 0);
		// Point 5
		alt_up_pixel_buffer_dma_draw_hline(pixel_buf, startX+4*dpWidth, startX+5*dpWidth, p5Y,red, 0);

		// Frequency & RoC Threshold
		sprintf(str, "%.1f", frequencyThreshold);
		alt_up_char_buffer_string(char_buf, str,30, 50);

		sprintf(str, "%.1f", (float)rocThreshold/10);
		alt_up_char_buffer_string(char_buf, str,25, 52);
		// Min,Max, Avg
		sprintf(str, "Avg:%d", avgReactionTime);
		alt_up_char_buffer_string(char_buf, str, 55, 52);

		sprintf(str, "Min:%d", minReactionTime);
		alt_up_char_buffer_string(char_buf, str, 55, 54);

		sprintf(str, "Max:%d", maxReactionTime);
		alt_up_char_buffer_string(char_buf, str, 55, 56);
		// Total Running Times
		currentSystemTime = xTaskGetTickCount();
		sprintf(str, "%lu",currentSystemTime );
		alt_up_char_buffer_string(char_buf, str, 25, 55);
		// Print System Stability on VGA
		if(wasStable){
			alt_up_char_buffer_string(char_buf,"Stable", 40, 52);

		}else{
			alt_up_char_buffer_string(char_buf, "Unstable", 40, 52);

		}

		// Refresh Rate(As quick as possible rather than a specified rate)
	}

}


/*
 * Processes input from Keyboard on Rising Edge
 * Updates Frequency and RoC Thresholds
 * Displays User Prompts on LCD
 * */
void keyboardManagerTask(void *pvParameters){

	/*#################### Keyboard Manager State ###################### */
	#define IDLE 0
	#define FREQUENCY_UPDATE 1
	#define ROC_UPDATE 2
	uint8_t keyboardManagerState = IDLE;

	/*#################### Rising Edge PS2 Flag ###################### */
	uint8_t ps2RisingEdgeFlag = 0;

	/*keyboard Decimal Values*/
	uint8_t NUM_ZERO = 112;
	uint8_t NUM_ONE = 105;
	uint8_t NUM_TWO = 114;
	uint8_t NUM_THREE = 122;
	uint8_t NUM_FOUR = 107;
	uint8_t NUM_FIVE = 115;
	uint8_t NUM_SIX = 116;
	uint8_t NUM_SEVEN = 108;
	uint8_t NUM_EIGHT = 117;
	uint8_t NUM_NINE = 125;
	uint8_t NUM_ENTER = 90;

	uint8_t NUM_KEYS[10];
	NUM_KEYS[0] = NUM_ZERO;
	NUM_KEYS[1] = NUM_ONE;
	NUM_KEYS[2] = NUM_TWO;
	NUM_KEYS[3] = NUM_THREE;
	NUM_KEYS[4] = NUM_FOUR;
	NUM_KEYS[5] = NUM_FIVE;
	NUM_KEYS[6] = NUM_SIX;
	NUM_KEYS[7] = NUM_SEVEN;
	NUM_KEYS[8] = NUM_EIGHT;
	NUM_KEYS[9] = NUM_NINE;


	int rocBuffer[3];
	int rocBufferItems;
	int freqBuffer[2];
	int freqBufferItems;

	lcd = fopen(CHARACTER_LCD_NAME, "w");
	fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
	fprintf(lcd, "ENTER 1 FOR Freq \r\n");
	fprintf(lcd, "ENTER 2 FOR RoC \r\n");
	fclose(lcd);

	unsigned char input = 0;

	while(1)
	{
		if(xQueueReceive(ps2KeyQ, &input, 0) == pdPASS){
		  if (ps2RisingEdgeFlag == 0){
			  ps2RisingEdgeFlag =1;
		  }else{
			  ps2RisingEdgeFlag =0;

			  switch(keyboardManagerState){
				case IDLE:
					// input 1
					lcd = fopen(CHARACTER_LCD_NAME, "w");
					fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

					if(input == NUM_ONE ){
						keyboardManagerState = FREQUENCY_UPDATE;
						freqBufferItems = 0;
						fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
						fprintf(lcd, "FREQUENCY VAL: \n");
					//input 2
					}else if(input == NUM_TWO){
						rocBufferItems =0;
						keyboardManagerState = ROC_UPDATE;
						fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
						fprintf(lcd, "Roc: \n");
					}else{
						lcd = fopen(CHARACTER_LCD_NAME, "w");
						fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
						fprintf(lcd, "ENTER 1 FOR Freq \r\n");
						fprintf(lcd, "ENTER 2 FOR RoC");
					}
					fclose(lcd);

					break;
				case FREQUENCY_UPDATE:

					lcd = fopen(CHARACTER_LCD_NAME, "w");
					fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

					// if buffer is already full
					if(freqBufferItems == 2 ){
						// if ENTR KEY PRESSED
						if(input == NUM_ENTER){

							xSemaphoreTake(thresholdSemaphore, 0);

							frequencyThreshold = freqBuffer[0] * 10 + freqBuffer[1];
							// To notify user on console
							printf("New FreqThres: %.1f Hz\n", frequencyThreshold);

							xSemaphoreGive(thresholdSemaphore);

						}else{
							fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
							fprintf(lcd, "TOO MANY VALUES!\n");
							// To keep LCD on screen for read
							vTaskDelay(2000);
						}

						fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
						fprintf(lcd, "ENTER 1 FOR Freq \r\n");
						fprintf(lcd, "ENTER 2 FOR RoC");

						keyboardManagerState = IDLE;

					}else{

						uint8_t i;
						uint8_t foundValidKey = 0;
						// Check for 0-9
						for (i=0;i<10;i++){
							if (input == NUM_KEYS[i]){
								//Add number to buffer
								freqBuffer[freqBufferItems] = i;

								freqBufferItems++;
								foundValidKey = 1;
								break;

							}
						}


						if (!(foundValidKey)){
							keyboardManagerState = IDLE;
							fprintf(lcd, "INVALID INPUT!\n");
							vTaskDelay(2000);
							fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
							fprintf(lcd, "ENTER 1 FOR Freq \r\n");
							fprintf(lcd, "ENTER 2 FOR RoC");
						}else{

							if(freqBufferItems == 1){

								fprintf(lcd, "Freq: %d\n", freqBuffer[0]);

							}else if(freqBufferItems == 2){
								fprintf(lcd, "Freq: %d%d\n", freqBuffer[0], freqBuffer[1]);
								fprintf(lcd, "Press Enter\n");
							}


						}

					}

					fclose(lcd);

					break;

				case ROC_UPDATE:

					lcd = fopen(CHARACTER_LCD_NAME, "w");
					fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);

					// If buffer is already full
					if(rocBufferItems == 3){
						// If ENTR KEY PRESSED
						if(input == NUM_ENTER){


							xSemaphoreTake(thresholdSemaphore, 0);

							rocThreshold = rocBuffer[0] * 100 + rocBuffer[1] * 10 + rocBuffer[2];
							// To notify user on console
							printf("New RocThres: %.1f Hz/Sec\n", (float)rocThreshold/10);
							xSemaphoreGive(thresholdSemaphore);

						}else{
							fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
							fprintf(lcd, "TOO MANY VALUES!\n");
							// To keep LCD on screen for read
							vTaskDelay(2000);

						}

						fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
						fprintf(lcd, "ENTER 1 FOR Freq \r\n");
						fprintf(lcd, "ENTER 2 FOR RoC");

						keyboardManagerState = IDLE;

					}else{

						uint8_t i;
						uint8_t foundValidKey = 0;
						// Check for 0-9
						for (i=0;i<10;i++){
							if (input == NUM_KEYS[i]){
								// Add number to buffer
								rocBuffer[rocBufferItems] = i;

								rocBufferItems++;
								foundValidKey = 1;

								break;

							}
						}

						if (!(foundValidKey)){
							keyboardManagerState = IDLE;
							fprintf(lcd, "INVALID INPUT!\n");
							vTaskDelay(2000);
							fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
							fprintf(lcd, "ENTER 1 FOR Freq \r\n");
							fprintf(lcd, "ENTER 2 FOR RoC");
						}else{

							if(rocBufferItems == 1){
								fprintf(lcd, "Roc: %d.\n", rocBuffer[0]);

							}else if(rocBufferItems == 2){
								fprintf(lcd, "Roc: %d%d.\n", rocBuffer[0], rocBuffer[1]);

							}else if (rocBufferItems == 3){
								fprintf(lcd, "Roc: %d%d.%d\n", rocBuffer[0], rocBuffer[1],rocBuffer[2]);
								fprintf(lcd, "Press Enter\n");

							}

						}

					}

					fclose(lcd);
					break;
				}

			  }

			// To prevent registering incorrect values from fast typing
			vTaskDelay(200);
		}
		// This restricts checking of Keyboard Q at 50ms intervals
		// (I dont know why someone would want to type that fast)
		vTaskDelay(50);
	}
}

/*
 * Control Machine of Relay
 * Receives input messages from frequency updater Q
 * Updates Running Data (Frequency and RoC)
 * Modes:
 * Maintenance
 * -> Checks Switches for manual input
 * Normal
 * -> Checks for tripping condition to switch to Manage (and computes Reaction time params for that condition)
 * -> Checks Maintenance Mode En Flag to switch mode
 * Load Manage
 * -> Shed/Reconnect Loads,
 * -> Stability Observation
 * -> Manually Switch Off Loads
 * -> Compute Reaction Time Params
 * -> Check Maintenance Mode En Flag to switch mode
 * */

void loadManagerTask(void *pvParameters){

	//Load Manager State Machine
	#define NORMAL 0
	#define LOAD_MANAGE 1
	#define MAINTENANCE 2

	uint8_t loadManagerState = NORMAL;
	uint8_t isTripCond = 0;

	float freqThresholdLocal =  0.0;
	int rocThresholdLocal =  0;
	uint8_t SWITCHES[5] = {1,1,1,1,1};

	struct freqRocQMsg freqRocMsg;

	int timeTaken, reqTime = 0;

	while(1){

		switch(loadManagerState){
			case NORMAL:

				if(xQueueReceive(freqRocDataQ, &freqRocMsg, 0)){
					updateRunningData(freqRocMsg);
					// Check Switches
					updateSwitches(SWITCHES);
					// Manually switch on/off in Normal mode
					loadUpdater(freqRocMsg.timestamp, SWITCHES);
					// Copy over thresholds
					xSemaphoreTake(thresholdSemaphore, 0);

					rocThresholdLocal= rocThreshold;
					freqThresholdLocal = frequencyThreshold;

					xSemaphoreGive(thresholdSemaphore);

					isTripCond = checkTrippingConditions(freqRocMsg, freqThresholdLocal, rocThresholdLocal);

 	 	 	 	 	if(isTripCond){
 	 	 	 	 		// Sent Request to Trip Load 0
 	 	 	 	 		shedLoad(SWITCHES);
 	 	 	 	 		computeReactionTimeStats(xTaskGetTickCount(), freqRocMsg);

 	 	 	 	 		// Connection is Unstable
 	 	 	 	 		wasStable = 0;
 	 	 	 	 		loadManagerState = LOAD_MANAGE;
 	 	 	 	 		restartFreeRTOSTimer();

 	 	 	 	 		printf("\n################LOAD MANAGER MODE##########################\n");
 	 	 	 	 	}

				}

				if (maintainenceModeEn){

					// If switches are not all turned on do not let the mode be executed
					while(!(SWITCHES[0]&& SWITCHES[1]&& SWITCHES[2] && SWITCHES[3]&&SWITCHES[4])){
						printf("\n\nTO BEGIN MAINTENANCE MODE PLEASE SWITCH ALL LOADS ON\n\n");
						updateSwitches(SWITCHES);
						vTaskDelay(1000);

					}

					loadStatus = ALLON;
					loadManagerState = MAINTENANCE;

					printf("\n################MAINTENANCE MODE##########################\n");
				}

				break;

			case LOAD_MANAGE:

				// Check if manually disconnected
				manualCheckAndSwitchOffLoads(SWITCHES);
				// Handles timer expiry if an input has not arrived yet
				if (timerExpiryFlag){
					printf("#######TIMER EXPIRY BEFORE NEW INPUT RECEIVED#####\n");
					if(wasStable){
						// Reconnect load (if it returns 1 all loads connected)
						if(reconnectLoad(SWITCHES) == 1){
								stopFreeRTOSTimer();
								loadManagerState = NORMAL;
								printf("\n\n############ NORMAL MODE ########!\n\n");
						}else{
							// If not then need to start stability observation again
							restartFreeRTOSTimer();
						}

					}else{
						// Disconnect
						shedLoad(SWITCHES);

						restartFreeRTOSTimer();
					}

					break;
				}
				// Receive a new Frequency/RoC Value
				if(xQueueReceive(freqRocDataQ, &freqRocMsg, 0)){
					 updateRunningData(freqRocMsg);

					xSemaphoreTake(thresholdSemaphore, 0);
					// Copy over thresholds
					rocThresholdLocal = rocThreshold;
					freqThresholdLocal = frequencyThreshold;

					xSemaphoreGive(thresholdSemaphore);

					isTripCond = checkTrippingConditions(freqRocMsg, freqThresholdLocal, rocThresholdLocal);

					if(isTripCond && wasStable){
						wasStable = 0;
						// Restart Timer as it is now Tripping Condition
						restartFreeRTOSTimer();

					}else if(isTripCond && !wasStable){

						wasStable = 0;
						// If unstable for 500ms disconnect load
						if(timerExpiryFlag){

							//shed the next load
							shedLoad(SWITCHES);
							// Restart Timer for next 500ms observation
							restartFreeRTOSTimer();
						}


					}else if(!isTripCond && wasStable){

						wasStable = 1;
						// If Stability observed for 500ms reconnect load
						if(timerExpiryFlag){
							if(reconnectLoad(SWITCHES) == 1){
								// If all loads are connected back to normal state
								stopFreeRTOSTimer();
								loadManagerState = NORMAL;
								printf("\n\n############ NORMAL MODE ########!\n\n");
							}else{
								restartFreeRTOSTimer();
							}

						}



					}else if(!isTripCond && !wasStable){

						// Loads are currently stable
						wasStable = 1;
						// Load Manager needs to start Timer for Stability Observation
						restartFreeRTOSTimer();


					}

				}

				break;

			case MAINTENANCE:

				if(xQueueReceive(freqRocDataQ, &freqRocMsg, 0)){
					updateRunningData(freqRocMsg);
				}

				// Store the time before reading switch
				reqTime= xTaskGetTickCount();
				// Check Switches
				updateSwitches(SWITCHES);

				timeTaken = loadUpdater(reqTime, SWITCHES);

				// Notify user of Response Time for this mode if more than 0 ms
				// N.B. Does not show response time in VGA for maintenance
				if(timeTaken != 0){
					printf("Maintenance Mode Response Time: %u\n", timeTaken);
				}

				// Go back to Normal Mode on Button press
				if (!maintainenceModeEn){

					loadManagerState = NORMAL;
					printf("\n\n############ NORMAL MODE ########!\n\n");
				}

				break;
		}
		// Delay for 10 ms (is 2 Times speed of ADC so should never miss an input)
		vTaskDelay(10);
	}
}


/*
 * Receives Message from frequency analyser ISR (Contains frequency and timestamp)
 * Compute ROC and send params to Load Manager
 */
void frequencyUpdaterTask(void *pvParameters){

	float freqValNew = 0;
	float freqValOld = 0;
	float roc = 0;

	uint8_t isFirstIteration = 1;

	struct freqRocQMsg freqRocMsg;
	struct freqQMsg receiveIsrMsg;

		while(1)
		{
			if(xQueueReceive(frequencyQ, &receiveIsrMsg, 0) == pdPASS){
				freqValNew = receiveIsrMsg.frequency;

				if(isFirstIteration ){
					isFirstIteration = 0;
					// update the old value
					freqValOld = freqValNew;
					continue;
				}
				roc = ((freqValNew - freqValOld) * 2) / ((1/freqValNew) + (1/freqValOld));
				freqValOld = freqValNew;
				freqRocMsg.freqData = freqValNew;
				freqRocMsg.rocData = roc;
				freqRocMsg.timestamp = receiveIsrMsg.timestamp;
				xQueueSendToBack(freqRocDataQ, &freqRocMsg, 0);

			}
			// To run at same speed as inputs are received
			// Task execution takes under 1 tick so same rate should be fine.
			vTaskDelay(20);
		}
}


/*##################################################################
############################### HELPER FUNCTIONS ###################
#################################################################### */

/*
 * Reads switches to and updates Load Status and Leds
 * in NORMAL and MAINTENANCE state
 * */
int loadUpdater(int reqTime, uint8_t SWITCHES[]){

	// Check Load Status against Switch Value (from highest-> lowest priority)
	// If discrepancy turn on/off load to new Switch Value
	int  reactTime, timeTaken;
	int8_t i;

	for(i=4;i>=0;--i){
		// If switch value has changed update loadStatus
		if(SWITCHES[i] != (loadStatus & loads[i])>>i){

			printf("Load ID: %d\n isTurnON: %d\n", loads[i], SWITCHES[i]);

			if (SWITCHES[i]){
				//Set the bit corresponding to load
				loadStatus |= loads[i];
			}else{
				//Clear the bit corresponding to load
				loadStatus &= ~(loads[i]);
			}
			// Update Red LEDS
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadStatus);

		}

	}
	reactTime = xTaskGetTickCount();

	timeTaken = reactTime - reqTime;

	// Return time taken to switch off load
	return timeTaken;
}

/*
 * Reads and stores switch value
 * */
void updateSwitches(uint8_t SWITCHES[]){

	SWITCHES[0] =(IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)&LOAD0);
	SWITCHES[1] =(IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)&LOAD1)>>1;
	SWITCHES[2] =(IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)&LOAD2)>>2;
	SWITCHES[3] =(IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)&LOAD3)>>3;
	SWITCHES[4] =(IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE)&LOAD4)>>4;

}

/*
 * Restarts FreeRTOS timer and clears global Timer expiry Flag
 * */
void restartFreeRTOSTimer(){
	if (xTimerIsTimerActive(timer500ms) == pdTRUE){

		while (xTimerStop(timer500ms, 0) ==  pdFAIL){
			// Restart Timer
			printf("Cannot stop timer\n\n");

		}
	}

	timerExpiryFlag = 0;

	while (xTimerStart(timer500ms, 0) == pdFAIL){
		printf("Cannot start timer\n\n");
	}

}

/*
 * Stops FreeRTOS timer and clears global Timer expiry Flag
 * */
void stopFreeRTOSTimer(){
	if (xTimerIsTimerActive(timer500ms) == pdTRUE){

		while (xTimerStop(timer500ms, 0) ==  pdFAIL){
			// Restart Timer
			printf("Cannot stop timer\n\n");

		}
	}

	timerExpiryFlag = 0;


}

/*
 * Checks Tripping Conditions
 * Returns 1 if tripping 0 if not
 * */
uint8_t checkTrippingConditions(struct freqRocQMsg freqRocMsg, float freqThresholdLocal, int rocThresholdLocal){

	uint8_t isTripCond;

	if (freqRocMsg.freqData < freqThresholdLocal){

		isTripCond = 1;

	}else if ((freqRocMsg.rocData * 10) > rocThresholdLocal ){
		isTripCond = 1;


	}else if((freqRocMsg.rocData * 10) < -(rocThresholdLocal)){
		isTripCond = 1;


	}else{
		isTripCond = 0;

	}
	return isTripCond;
}


/*
 * Sheds a load in Load Manage State and Normal State
 * Returns 1 for success 0 for failure
 * */
uint8_t shedLoad(uint8_t SWITCHES[]){

	// DO NOT CHANGE i TO UINT
	int8_t i;
	// Default value
	int8_t loadToShed = -1;
	// Check how many loads are connected
	for(i=0;i<5;i++){
		// Check if the load is connected and not manually switched off
		if(((loadStatus & loads[i])>> i) && (SWITCHES[i] == 1) )  {
			loadToShed = i;
			break;
		}

	}

	if (loadToShed != -1){
		// Disconnect load
		// Clear the bit corresponding to load
		loadStatus &= ~(loads[loadToShed]);

		// TURN OFF RED LED
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadStatus);

		// turn bit on for green led
		int greenLed = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);

		greenLed |= (1 << loadToShed);
		// TURN ON GREEN LED
		IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLed);
		printf("Shed Load: %d\n", loadToShed);

		// Return success
		return 1;
	}else{
		printf("\n ############ ATTEMPT TO DISCONNECT LOAD ERROR: ALL LOADS ARE ALREADY DISCONNECTED##### \n");
		return 0;
	}
}

/*
 * Reconnects a load in Load Manage State
 * Returns 1 if all loads connected, 0 if not,-1 for error
 * */
uint8_t reconnectLoad(uint8_t SWITCHES[]){
	// DO NOT CHANGE TO UINT
	int8_t i;
	// Default value
	int8_t loadToReconnect = -1;
	// Check how many loads are connected (Reconnect Highest Priority First)

	for(i=4;i>=0;--i){

		// Check if the load is not connected and if load is not switched off
		if(!((loadStatus & loads[i])>> i) && (SWITCHES[i]== 1) ){
			loadToReconnect= i;
			break;
		}

	}
	if (loadToReconnect != -1){

			// Reconnect Load
			loadStatus |= loads[loadToReconnect];

			// Read RED LED STATUS
			int currentRedLed = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
			currentRedLed |= loads[loadToReconnect];
			// SET RED LED BIT
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, currentRedLed);

			int greenLED = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
			// CLEAR GREEN LED BIT
			greenLED &= ~(1UL << loadToReconnect) ;
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLED);
			printf("Reconnected Load: %d\n", loadToReconnect);
			// Return success
			if (loadStatus == ALLON){
				return 1;
			}else{
				return 0;
			}

	}else{
		printf("\n ############ ATTEMPT TO RECONNECT LOAD ERROR: ALL LOAD CONNECTED OR LOADS MANUALLY SWITCHED OFF##### \n");
		return -1;
	}
}

/*
 * Check the Switches to manually switch off loads in LOAD MANAGE State
 * */
void manualCheckAndSwitchOffLoads(uint8_t SWITCHES[]){

	uint8_t newSWITCHES[5];
	uint8_t isLoadOn = 0;

	updateSwitches(newSWITCHES);

	int8_t i;
	for (i=4;i>=0;--i){
		// Check that switch was pulled down to turn off a load
		if(!(newSWITCHES[i]) && (SWITCHES[i]) ){

			// Check Red LED value to see if load is on
			int currentRedLed = IORD_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE);
			isLoadOn = (currentRedLed >> i) & 1U;

			if (isLoadOn){
				// Set corresponding bit to 0 in loadStatus
				loadStatus &= ~(loads[i]);
				//TURN OFF RED LED
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, loadStatus);

				printf("Manually Turned Off Load %d!\n",i);

			}else{

				// Switch Green Led off as load cannot be managed (manual switch off).
				int greenLED = IORD_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE);
				// CLEAR GREEN LED BIT
				greenLED &= ~(1UL << i) ;
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, greenLED);
				// If Load is already Shed
				printf("Load %d already turned off by Relay/Switch !\n",i);

			}
		}
	}
	// Update the Switches to new value
	updateSwitches(SWITCHES);
}


/*
 * Computes Reaction Time when load is shed
 * */
void computeReactionTimeStats(int currentTime,struct freqRocQMsg freqRocMsg){
	// DONOT CHANGE TO UINT
	int8_t i;
	int reactionTimeLocal = currentTime -freqRocMsg.timestamp;
	// Add the current reactiontime  if its already full take the first item out
	if(reactionTimeIndex<4){
		reactionTimes[reactionTimeIndex] = reactionTimeLocal;
		reactionTimeIndex++;

	}else{
		// Shift the values from 0-3 forward by 1
		for(i=3;i>=0;--i){
			reactionTimes[i+1] = reactionTimes[i];

		}
		// Write new value to index 0
		reactionTimes[0] = reactionTimeLocal;


	}
	int sum = 0;
	// Compute average reaction time using for loop and dividing by 5
	for (i=0;i<5;i++){
		sum+=reactionTimes[i];

	}
	avgReactionTime= sum/5;

	// Total Time System has been running for
	totalTime = currentTime;

	// Update max/min reaction time
	if(reactionTimeLocal > maxReactionTime){
		maxReactionTime = reactionTimeLocal;
	}
	// If currentReaction time < existing lowest update the min Time
	if(reactionTimeLocal < minReactionTime){
		minReactionTime = reactionTimeLocal;
	}
}

/*
 * Reads Message and updates running time data of frequency and roc
 * */
void updateRunningData(struct freqRocQMsg freqRocMsg){
	int8_t i;
	float freqDataLocal = freqRocMsg.freqData;
	float rocDataLocal = freqRocMsg.rocData;

	// Add the current Freq & RoC Data.If its already full take the first item out
	if(runningDataIndex < 49){
		frequencyData[runningDataIndex] = freqDataLocal;
		rocData[runningDataIndex]= rocDataLocal;

		runningDataIndex++;

	}else{
		// Shift the values from 0-48 forward by 1
		for(i=48;i>=0;--i){
			frequencyData[i+1] = frequencyData[i];
			rocData[i+1] = rocData[i];

		}
		// Write new value to index 0
		frequencyData[0] = freqDataLocal;
		rocData[0] = rocDataLocal;


	}
}

/*##################################################################
#################### TEST FUNCTIONS (Dev Use Only) #################
####################################################################*/
void testLoadSheddingAndReconnecting(){

	uint8_t SWITCHES[5] = {1, 1, 1, 1, 1};

	IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, ALLOFF);
	IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, ALLON);

	printf("LOAD STATUS %d\n",loadStatus);
	shedLoad(SWITCHES);
	printf("LOAD STATUS AFTER SHED:  %d\n",loadStatus);

	shedLoad(SWITCHES);
	usleep(1000000);
	shedLoad(SWITCHES);
	usleep(1000000);
	shedLoad(SWITCHES);
	usleep(100000);
	shedLoad(SWITCHES);
	usleep(100000);
	printf("LOAD STATUS AFTER SHED:  %d\n",loadStatus);

	reconnectLoad(SWITCHES);
	usleep(1000000);
	printf("LOAD STATUS AFTER CONNECT: %d\n",loadStatus);
	reconnectLoad(SWITCHES);
	usleep(1000000);
	reconnectLoad(SWITCHES);
	usleep(1000000);
	reconnectLoad(SWITCHES);
	usleep(1000000);
	reconnectLoad(SWITCHES);
	usleep(1000000);
	printf("LOAD STATUS AFTER CONNECT: %d\n",loadStatus);

}

void testComputeReactionTimeStats(){
	// Create Dummy Message
	struct freqRocQMsg freqRocMsg;
	// Assign a sample timestamp
	freqRocMsg.timestamp = 5000;

	computeReactionTimeStats(5005, freqRocMsg);
	computeReactionTimeStats(5010, freqRocMsg);
	computeReactionTimeStats(5007, freqRocMsg);
	computeReactionTimeStats(5006, freqRocMsg);
	computeReactionTimeStats(5002, freqRocMsg);
	computeReactionTimeStats(5003, freqRocMsg);
	// Avg is calculated from the latest 5 values
	printf("Avg %d, Min %d, Max %d\n", avgReactionTime, minReactionTime, maxReactionTime);
}

void testUpdateRunningData(){
	struct freqRocQMsg freqRocMsg;
	int i;

	freqRocMsg.freqData = 50;
	freqRocMsg.rocData = 7.0;

	updateRunningData(freqRocMsg);
	printf("ADD A VALUE\n");
	printf("Roc Value:%f\n",rocData[0]);
	printf("Freq Value:%f\n",frequencyData[0]);

	freqRocMsg.freqData = 51;
	freqRocMsg.rocData = 8.0;
	updateRunningData(freqRocMsg);
	printf("ADD A VALUE\n");

	for (i=0;i<runningDataIndex;i++){
			printf("Roc Value:%f\n",rocData[i]);
			printf("Freq Value:%f\n",frequencyData[i]);
	}

	freqRocMsg.freqData = 52;
	freqRocMsg.rocData = 9.0;
	updateRunningData(freqRocMsg);
	printf("ADD A VALUE\n");

	for (i=0;i<runningDataIndex;i++){
		printf("Roc Value:%f\n",rocData[i]);
		printf("Freq Value:%f\n",frequencyData[i]);
	}
}

void testManualSwitchOffLoad1(){
	// Can add delays with usleep to see changes
	uint8_t SWITCHES[5] = {1, 1, 1, 1, 1};
	usleep(3000000);
	manualCheckAndSwitchOffLoads(SWITCHES);
	usleep(3000000);
	manualCheckAndSwitchOffLoads(SWITCHES);
	usleep(3000000);
	manualCheckAndSwitchOffLoads(SWITCHES);
}

void testManualSwitchOffLoad2(){
	// Can add delays with usleep to see changes
	uint8_t SWITCHES[5] = {1,1,1,1,1};
	shedLoad(SWITCHES);
	shedLoad(SWITCHES);
	manualCheckAndSwitchOffLoads(SWITCHES);
	printf("Test Load Status after User: %d\n", loadStatus);
	printf("Test Load Status before 2nd shedLoad : %d\n", loadStatus);
	shedLoad(SWITCHES);
	reconnectLoad(SWITCHES);
	reconnectLoad(SWITCHES);
	reconnectLoad(SWITCHES);
}
