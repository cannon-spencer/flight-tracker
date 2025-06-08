#ifndef THREADS_H_
#define THREADS_H_

/************************************Includes***************************************/

#include "./G8RTOS/G8RTOS.h"

/************************************Includes***************************************/

/*************************************Defines***************************************/

#define DATA_FIFO           0
#define BUTTONS_FIFO        1
#define JOYSTICK_FIFO       2
#define Aircrafts_FIFO      4

#define MESSAGE_SIZE        28   // total bytes for each aircraft message
#define FLOAT_BUFF_SIZE     20   // for the buffer size of casting floats to strings
#define INT_BUFF_SIZE       12
#define MAX_AIRCRAFTS       200  // max number of allowed aircrafts

#define M_PI                3.14159265358979323846
#define RAD_TO_DEG          (180.0f / M_PI)
#define MIDLINE             70




/*************************************Defines***************************************/

/***********************************Semaphores**************************************/


semaphore_t sem_I2CA;
semaphore_t sem_SPIA;
semaphore_t sem_UART;
semaphore_t sem_PCA9555_Debounce;
semaphore_t sem_Joystick_Debounce;

semaphore_t sem_CURRENT_AIRCRAFTS;
semaphore_t sem_STAGING_AIRCRAFTS;
semaphore_t sem_DATA_READY;
semaphore_t sem_BURST_COMPLETE;

semaphore_t sem_MAIN_DISPLAY;
semaphore_t sem_INFO_DISPLAY;




/***********************************Semaphores**************************************/


/***********************************Structures**************************************/

typedef struct {
    char callsign[8];
    float longitude;
    float latitude;
    float altitude;
    float velocity;
    float heading;
    int16_t screen_x;
    int16_t screen_y;
    int8_t on_screen;
} AircraftData_t;


/***********************************Structures**************************************/

/*******************************Background Threads**********************************/

void Idle_Thread(void);

void Process_New_Aircraft_Thread(void);
void Update_Current_Aircrafts_Thread(void);

void Update_Search_Range(void);

void Select_Aircraft_Thread(void);

void Display_Aircraft_Info_Thread(void);
void Display_Aircrafts_Thread(void);

/*******************************Background Threads**********************************/

/********************************Periodic Threads***********************************/

/********************************Periodic Threads***********************************/

/*******************************Aperiodic Threads***********************************/

void UART4_Handler(void);

void Button_Handler(void);

void Joystick_Button_Handler(void);

/*******************************Aperiodic Threads***********************************/


#endif /* THREADS_H_ */

