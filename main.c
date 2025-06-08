/***************************************************************************************
 * @file        main.c
 * @brief       Initializes system components and starts the RTOS scheduler.
 *
 * @project     Final Project: Aircraft Display System
 *              Real-time display and management of aircraft data on a radar screen.
 *
 * @author      Cannon Spencer
 * @date        November 10, 2024
 * @university  University of Florida
 *
 * @version     1.0.0
 *
 * @details
 * This file is part of the Final Project: Aircraft Display System. The goal of this project
 * is to implement a radar system that displays aircraft positions and allows user interaction
 * using a joystick and buttons. This module initializes the hardware, threads, and
 * semaphore resources before starting the G8RTOS scheduler.
 *
***************************************************************************************\

/************************************Includes***************************************/

#include "G8RTOS/G8RTOS.h"
#include "./MultimodDrivers/multimod.h"

#include "./threads.h"
#include "driverlib/interrupt.h"

/************************************Includes***************************************/

/************************************MAIN*******************************************/

int main(void){
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    multimod_init();
    G8RTOS_Init();

    // Initialize semaphores
    G8RTOS_InitSemaphore(&sem_DATA_READY, 0);
    G8RTOS_InitSemaphore(&sem_BURST_COMPLETE, 0);
    G8RTOS_InitSemaphore(&sem_CURRENT_AIRCRAFTS, 1);
    G8RTOS_InitSemaphore(&sem_STAGING_AIRCRAFTS, 1);

    G8RTOS_InitSemaphore(&sem_MAIN_DISPLAY, 1);
    G8RTOS_InitSemaphore(&sem_INFO_DISPLAY, 1);

    G8RTOS_InitSemaphore(&sem_I2CA, 1);
    G8RTOS_InitSemaphore(&sem_SPIA, 1);
    G8RTOS_InitSemaphore(&sem_UART, 1);
    G8RTOS_InitSemaphore(&sem_PCA9555_Debounce, 0);
    G8RTOS_InitSemaphore(&sem_Joystick_Debounce, 0);

    // Add threads
    G8RTOS_AddThread(Idle_Thread, 255, "Idle");
    G8RTOS_AddThread(Process_New_Aircraft_Thread, 1, "Process_New_Aircraft_Thread");
    G8RTOS_AddThread(Update_Current_Aircrafts_Thread, 2, "Update_Current_Aircrafts_Thread");
    G8RTOS_AddThread(Update_Search_Range, 12, "Update_Search_Range");
    G8RTOS_AddThread(Display_Aircraft_Info_Thread, 13, "Display_Aircraft_Info_Thread");
    G8RTOS_AddThread(Display_Aircrafts_Thread, 11, "Display_Aircrafts_Thread");
    G8RTOS_AddThread(Select_Aircraft_Thread, 10, "Select_Aircraft_Thread");


    // Add FIFOs
    G8RTOS_InitFIFO(DATA_FIFO);

    // Add aperiodic threads
    G8RTOS_Add_APeriodicEvent(UART4_Handler, 1, INT_UART4);
    G8RTOS_Add_APeriodicEvent(Button_Handler, 2, BUTTON_INTERRUPT);
    G8RTOS_Add_APeriodicEvent(Joystick_Button_Handler, 3, JOYSTICK_GPIOD_INT);

    // Launch RTOS
    G8RTOS_Launch();

    // Shouldn't ever make it here
    while(1);
}

/************************************MAIN*******************************************/
