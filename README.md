## OS and Tools for Assignment
Windows 10, Quartus 13.0 64 bit, Nios II Software Build Tools for Eclipse and DE2-115 FPGA Board.

### Note:- No spaces should be present in path names for the sections below
## FPGA Setup
Some instructions are taken from [COMPSYS723 2020 FreeRTOS Extra.pdf, Sec. 1] - Intro to FreeRTOS].
1) Open Quartus 13.0 64 bit and select Tools/Programmer.

2) Make sure USB-Blaster is selected from hardware setup.

3) Auto detect the FPGA and select COMPSYS723 2020 FreeRTOS Resources\SOPC files\freq_relay_controller.sof from canvas.

4) Program the FPGA in JTAG mode.

5) Connect PS2 keyboard and VGA cable to the ports provided on the board. A VGA Monitor is needed for visualising graph. 

## Running the Program on Nios II Software Build Tools for Eclipse:-
Some instructions are taken from [COMPSYS723 2020 FreeRTOS Lab.pdf, Part3 - Intro to FreeRTOS].
1) Create a new workspace.

2) For SOPC Information File Name, copy nios2.sopcinfo from COMPSYS723 2020 FreeRTOS Resources\SOPC files\ on canvas into the workspace. 
   Make sure no spaces are present in the path.

3) Make a new project by clicking File/New/Nios II application and BSP from Template.

3) Type freertos_assignment1 for project name, provide the nios2.sopcinfo file, select Blank Project under template and click finish.

4) Copy FreeRTOS folder from "COMPSYS723 2020 FreeRTOS Resources\FreeRTOS ver.8.2.0 Kernel for NIOS II" on canvas into freertos_assignment1.

5) Make sure #define ALT_LEGACY_INTERRUPT_API_PRESENT(as opposed to ALT_ENHANCED_INTERRUPT_API_PRESENT) is present in system.h

6) Update #define configTIMER_TASK_PRIORITY 10 in FreeRTOSConfig.h from #define configTIMER_TASK_PRIORITY 3 under #define configUSE_TIMERS 1

7) Copy Relay.c in COMPSYS723_Group17 to freertos_assignment1.

8) Refresh the whole project to detect the changes and then build the whole project.

9) In Run/Run Configurations... double click Nios II hardware to create a new run configuration.

10) In project tab make sure freertos_assignment1 is specified and elf file is detected.

11) In target connection make sure FPGA is detected and both checkboxes in System ID checks are ticked.

12) Run the program.

## User Interface:-
1) Maintenance Mode can be activated/deactivated using Key3 button on the FPGA

2) Lower(Frequency Hz) and ROC (Rate of Change Hz/Sec) can be updated using Numpad on a PS2 keyboard. 1 for frequency and 2 for roc thresholds respectively. 
   2 digits for frequency and 3 digit limits for roc threshold. LCD displays information as keys are pressed.

3) Switches[0-4] on the FPGA are connected to loads. Red leds and green LEDS correspond to load status. 
   Red LED OFF for switched off/shed load and ON for turned on/reconnected load. Green LED ON for load shed and OFF indicates it has been reconnected by relay.

4) VGA Monitor displays reaction time stats for first load shed, total run-time of system, thresholds and graph for frequency and roc against time.

5) Nios II Console prints information like load manager state, loads, thresholds and timer expiry

