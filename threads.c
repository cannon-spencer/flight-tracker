/***************************************************************************************
 * @file        threads.c
 * @brief       Implementation of threads for managing aircraft display and interactions.
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
 * using a joystick and buttons. This module contains thread implementations for display,
 * aircraft selection, and data processing.
 *
***************************************************************************************/

/************************************Includes***************************************/
#include "./threads.h"
#include "./MultimodDrivers/multimod.h"
#include "./MultimodDrivers/font.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <float.h>

/*********************************Global Variables**********************************/

uint16_t display_range_km = 50;

uint16_t display_track = true;
uint16_t display_callsign = true;

// Staging array and counter
AircraftData_t stagingAircrafts[MAX_AIRCRAFTS];
int stagingAircraftCount = 0;

// Live array and counter
AircraftData_t currentAircrafts[MAX_AIRCRAFTS];
int currentAircraftCount = 0;

// Index to "Selected" Aircraft
int16_t selectedAircraft = -1;

const float CENTER_LATITUDE = 29.6465;
const float CENTER_LONGITUDE = -82.3533;


/********************************Public Functions***********************************/

void float_to_string(float value, char *buffer, int decimal_places) {
    int32_t int_part = (int32_t)value;
    int32_t decimal_part = abs((int32_t)((value - int_part) * pow(10, decimal_places)));

    snprintf(buffer, FLOAT_BUFF_SIZE, "%d.%0*d", int_part, decimal_places, decimal_part);
}

void int_to_string(int32_t value, char *buffer) {
    snprintf(buffer, INT_BUFF_SIZE, "%d", value);
}


/**
 * @brief Recalculates the screen positions of aircraft based on their real-world coordinates.
 *
 * This function uses the aircraft's latitude and longitude to calculate their position on the
 * display screen. Aircraft outside the display range are marked as off-screen. The display range
 * and the center of the map are taken into account.
 *
 * The function locks the currentAircrafts array with a semaphore to ensure thread-safe access.
 */
void recalculate_screen_positions(void) {
    const int16_t radar_top = 70;
    const int16_t radar_bottom = 279;
    const int16_t radar_height = radar_bottom - radar_top + 1;
    const int16_t radar_center_x = X_MAX / 2;
    const int16_t radar_center_y = radar_top + (radar_height / 2);
    float max_pixel_radius = 100.0f;

    G8RTOS_WaitSemaphore(&sem_CURRENT_AIRCRAFTS);
    for (int i = 0; i < currentAircraftCount; i++) {
        AircraftData_t *aircraft = &currentAircrafts[i];

        // Calculate Differences
        float delta_latitude = aircraft->latitude - CENTER_LATITUDE;
        float delta_longitude = aircraft->longitude - CENTER_LONGITUDE;

        // Convert to Distances
        float latitude_in_radians = CENTER_LATITUDE * (M_PI / 180.0f);
        float km_per_degree_lat = 111.32f;
        float km_per_degree_lon = 111.32f * cosf(latitude_in_radians);

        float distance_latitude_km = delta_latitude * km_per_degree_lat;
        float distance_longitude_km = delta_longitude * km_per_degree_lon;

        // Calculate Distance and Angle
        float distance_from_center = sqrtf(distance_latitude_km * distance_latitude_km +
                                           distance_longitude_km * distance_longitude_km);

        float angle = atan2f(distance_latitude_km, distance_longitude_km);

        // Check Display Range
        if (distance_from_center > display_range_km) {
            aircraft->on_screen = false;
            if (i == selectedAircraft) {
                selectedAircraft = -1;
                G8RTOS_SignalSemaphore(&sem_INFO_DISPLAY);
            }
            continue;
        } else {
            aircraft->on_screen = true;
        }

        // Map to Screen Coordinates
        float scale = max_pixel_radius / display_range_km;
        float pixel_distance = distance_from_center * scale;

        int16_t screen_x = radar_center_x + (int16_t)(pixel_distance * cosf(angle));
        int16_t screen_y = radar_center_y - (int16_t)(pixel_distance * sinf(angle));

        aircraft->screen_x = abs(screen_x);
        aircraft->screen_y = abs(screen_y);

        /*UARTprintf("CallSign: %s\t", aircraft->callsign);
        UARTprintf("X: %d\tY: %d\t", screen_x, screen_y);

        char float_buffer[FLOAT_BUFF_SIZE];
        float_to_string(distance_from_center, float_buffer, 2);
        UARTprintf("Distance from center: %s km\t", float_buffer);
        UARTprintf("Display Range: %d km\t", display_range_km);

        float_to_string(pixel_distance, float_buffer, 2);
        UARTprintf("Pixel Distance: %s\t", float_buffer);

        float_to_string(delta_longitude, float_buffer, 2);
        UARTprintf("Delta Lon: %s\t", float_buffer);

        float_to_string(delta_latitude, float_buffer, 2);
        UARTprintf("Delta Lat: %s\t", float_buffer);

        float_to_string(aircraft->latitude, float_buffer, 2);
        UARTprintf("Lat: %s\t", float_buffer);

        float_to_string(aircraft->longitude, float_buffer, 2);
        UARTprintf("Lon: %s\n", float_buffer);*/
    }

    // Relinquish control of array
    G8RTOS_SignalSemaphore(&sem_CURRENT_AIRCRAFTS);
}



/**
 * @brief Finds the index of the closest aircraft, prioritizing direction but always selecting an on-screen aircraft.
 *
 * @param joystick_dx Joystick X position.
 * @param joystick_dy Joystick Y position.
 * @return int16_t The index of the closest aircraft to the joystick's direction,
 *                 or -1 if no valid aircraft exists on-screen.
 */
int16_t closest_aircraft_by_angle(int32_t joystick_dx, int32_t joystick_dy) {
    const int16_t MIDPOINT = 4096 / 2;

    // Ensure there's a selected aircraft
    if (selectedAircraft == -1) {
        return -1;
    }

    // Compute joystick deltas from midpoint
    int32_t dx = joystick_dx - MIDPOINT;
    int32_t dy = joystick_dy - MIDPOINT;

    float refLatitude = currentAircrafts[selectedAircraft].latitude;
    float refLongitude = currentAircrafts[selectedAircraft].longitude;

    // Determine direction
    int8_t goingWest = (abs(dx) > abs(dy)) && (dx > 0);
    int8_t goingEast = (abs(dx) > abs(dy)) && (dx < 0);
    int8_t goingSouth = (abs(dy) > abs(dx)) && (dy > 0);
    int8_t goingNorth = (abs(dy) > abs(dx)) && (dy < 0);

    int16_t closestIndex = -1;
    float minDifference = FLT_MAX;

    // For the selected direction, determine closest aircraft
    for (int i = 0; i < currentAircraftCount; i++) {
        if (i == selectedAircraft) continue;

        AircraftData_t *aircraft = &currentAircrafts[i];
        if (!aircraft->on_screen) continue;

        float deltaLon = aircraft->longitude - refLongitude;
        float deltaLat = aircraft->latitude - refLatitude;

        if (goingEast && deltaLon > 0.0f) {
            if (deltaLon < minDifference) {
                minDifference = deltaLon;
                closestIndex = i;
            }
        }
        else if (goingWest && deltaLon < 0.0f) {
            float diff = fabsf(deltaLon);
            if (diff < minDifference) {
                minDifference = diff;
                closestIndex = i;
            }
        }
        else if (goingNorth && deltaLat > 0.0f) {
            if (deltaLat < minDifference) {
                minDifference = deltaLat;
                closestIndex = i;
            }
        }
        else if (goingSouth && deltaLat < 0.0f) {
            float diff = fabsf(deltaLat);
            if (diff < minDifference) {
                minDifference = diff;
                closestIndex = i;
            }
        }
    }

    // Leave previous selection if there's nothing in that direction
    if(closestIndex == -1)
        return selectedAircraft;

    return closestIndex;
}




/*************************************Threads***************************************/

void Idle_Thread(void) { while(1); }




/**
 * @brief Displays aircraft positions and information on the screen.
 *
 * This thread is responsible for rendering the map display, including range circles and
 * the aircraft positions. It reads the aircraft data from `currentAircrafts` and displays
 * their positions on the screen.
 *
 * Aircraft positions are calculated based on their real-world coordinates and the display range.
 */
void Display_Aircrafts_Thread(void) {
    const int32_t MAX_SIZE = (Y_MAX - MIDLINE) / 2 - 1;
    const int32_t MIN_SIZE = MAX_SIZE / 2;
    const int32_t CENTER_X = X_MAX / 2;
    const int32_t CENTER_Y = (Y_MAX + MIDLINE) / 2;

    while(1){

        // Wait for an update event to be signaled
        G8RTOS_WaitSemaphore(&sem_MAIN_DISPLAY);

        // Main background
        ST7789_DrawRectangle(0, MIDLINE, X_MAX, Y_MAX, ST7789_BLACK);

        // Draw major/minor radius
        ST7789_DrawCircle(CENTER_X, CENTER_Y, MAX_SIZE, ST7789_LIGHTORANGE);
        ST7789_DrawCircle(CENTER_X, CENTER_Y, MIN_SIZE, ST7789_LIGHTORANGE);
        ST7789_FillCircle(CENTER_X, CENTER_Y,        5, ST7789_LIGHTORANGE);

        // Append current display ranges to the radar circles
        char buffer[INT_BUFF_SIZE];
        int_to_string(display_range_km, buffer);
        char *str= strcat(buffer, " km");
        ST7789_DrawString((X_MAX - (strlen(str) * (FONT_WIDTH + 1))) / 2, Y_MAX - 15, str, ST7789_LIGHTORANGE, ST7789_BLACK);

        int_to_string(display_range_km / 2, buffer);
        str= strcat(buffer, " km");
        ST7789_DrawString((X_MAX - (strlen(str) * (FONT_WIDTH + 1))) / 2, Y_MAX - 68, str, ST7789_LIGHTORANGE, ST7789_BLACK);

        // Loop through all current aircrafts, display it on the screen if needed
        G8RTOS_WaitSemaphore(&sem_CURRENT_AIRCRAFTS);
        for (int i = 0; i < currentAircraftCount; i++) {
            AircraftData_t *aircraft = &currentAircrafts[i];
            if (aircraft->on_screen) {

                // Calculations for the endpoints of a line representing the heading of the aircraft
                int16_t d = 30;
                int16_t x1 = aircraft->screen_x;
                int16_t y1 = aircraft->screen_y;
                int16_t dx = x1 + d * cos((90 - aircraft->heading) * 0.0174533);
                int16_t dy = y1 + d * sin((90 - aircraft->heading) * 0.0174533);

                //UARTprintf("Printing:: Callsign: %s\tX: %d\tY: %d\n", aircraft->callsign, aircraft->screen_x, aircraft->screen_y);


                // Use a different color if the aircraft is selected
                if(i == selectedAircraft){
                    // Draw aircraft symbol
                    ST7789_FillCircle(aircraft->screen_x, aircraft->screen_y, 5, ST7789_MAGENTA);

                    // Draw callsign next to the aircraft
                    if(display_callsign)
                        ST7789_DrawString(aircraft->screen_x + 7, aircraft->screen_y - 5, aircraft->callsign, ST7789_WHITE, ST7789_BLACK);

                    // Draw the heading line
                    if(display_track)
                        ST7789_DrawDottedLine(x1, y1, dx, dy, ST7789_MAGENTA, 3);

                } else {
                    // Draw aircraft symbol
                    ST7789_FillCircle(aircraft->screen_x, aircraft->screen_y, 3, ST7789_BLUE);

                    // Draw callsign next to the aircraft
                    if(display_callsign)
                        ST7789_DrawString(aircraft->screen_x + 5, aircraft->screen_y - 5, aircraft->callsign, ST7789_WHITE, ST7789_BLACK);

                    // Draw the heading line
                    if(display_track)
                        ST7789_DrawDottedLine(x1, y1, dx, dy, ST7789_BLUE, 3);
                }

            }
        }
        G8RTOS_SignalSemaphore(&sem_CURRENT_AIRCRAFTS);

        sleep(100);
    }
}



/**
 * @brief Displays detailed information about a selected aircraft.
 *
 * This thread updates the bottom portion of the screen to show detailed information such as
 * the selected aircraft's call sign, latitude, longitude, altitude, velocity, and heading.
 *
 * It waits for a semaphore to signal when an update is needed.
 */
void Display_Aircraft_Info_Thread(void) {

    while (1) {

        // Wait for some update to refresh this
        G8RTOS_WaitSemaphore(&sem_INFO_DISPLAY);

        char *CallSign = "N/A";
        char Longitude[FLOAT_BUFF_SIZE] = "N/A";
        char Latitude[FLOAT_BUFF_SIZE] = "N/A";
        char Altitude[FLOAT_BUFF_SIZE] = "N/A";
        char Velocity[FLOAT_BUFF_SIZE] = "N/A";
        char TrueTrack[FLOAT_BUFF_SIZE] = "N/A";

        // if there is a selected aircraft, populate that data
        if (selectedAircraft != -1 && selectedAircraft < MAX_AIRCRAFTS) {
            AircraftData_t *aircraft = &currentAircrafts[selectedAircraft];

            CallSign = aircraft->callsign;

            // Cast floats to strings for printing
            float_to_string(aircraft->longitude, Longitude, 4);
            float_to_string(aircraft->latitude, Latitude, 4);
            float_to_string(aircraft->altitude, Altitude, 4);
            float_to_string(aircraft->velocity, Velocity, 4);
            float_to_string(aircraft->heading, TrueTrack, 4);
        }

        // Draw background
        ST7789_DrawRectangle(0, 0, X_MAX, MIDLINE, ST7789_LGRAY);

        // Populate Information
        ST7789_DrawString(10, MIDLINE - 15, "CALL SIGN", ST7789_BLACK, ST7789_LGRAY);
        ST7789_DrawString(10, MIDLINE - 25, CallSign, ST7789_BLACK, ST7789_LGRAY);

        ST7789_DrawString(95, MIDLINE - 15, "LONGITUDE", ST7789_BLACK, ST7789_LGRAY);
        ST7789_DrawString(95, MIDLINE - 25, Longitude, ST7789_BLACK, ST7789_LGRAY);

        ST7789_DrawString(175, MIDLINE - 15, "LATITUDE", ST7789_BLACK, ST7789_LGRAY);
        ST7789_DrawString(175, MIDLINE - 25, Latitude, ST7789_BLACK, ST7789_LGRAY);

        ST7789_DrawString(10, MIDLINE - 43, "ALTITUDE", ST7789_BLACK, ST7789_LGRAY);
        ST7789_DrawString(10, MIDLINE - 53, Altitude, ST7789_BLACK, ST7789_LGRAY);

        ST7789_DrawString(95, MIDLINE - 43, "TRUE TRACK", ST7789_BLACK, ST7789_LGRAY);
        ST7789_DrawString(95, MIDLINE - 53, TrueTrack, ST7789_BLACK, ST7789_LGRAY);

        ST7789_DrawString(175, MIDLINE - 43, "VELOCITY", ST7789_BLACK, ST7789_LGRAY);
        ST7789_DrawString(175, MIDLINE - 53, Velocity, ST7789_BLACK, ST7789_LGRAY);

        sleep(100);
    }
}



/**
 * @brief Allows users to select an aircraft using the joystick.
 *
 * This thread reads joystick inputs to navigate through the list of on-screen aircraft.
 * It identifies the nearest aircraft in the direction of the joystick movement and
 * selects it for display.
 *
 * The selection is signaled to update the screen with new aircraft details.
 */
void Select_Aircraft_Thread(void){
    int32_t joystick_dx, joystick_dy, joystick_dxy;
    int8_t joystick_debounce = true;

    while(1){

        // with no selected aircraft, pick the most centered one!
        if(selectedAircraft == -1){

            // wait for a button press to signal
            G8RTOS_WaitSemaphore(&sem_Joystick_Debounce);
            sleep(5);

            int32_t press_status = JOYSTICK_GetPress();
            UARTprintf("press_status: %d \n", press_status);

            // Iterate through the visible aircrafts and select the one closest to the center
            if(press_status){

                int32_t min_distance = INT32_MAX;


                G8RTOS_WaitSemaphore(&sem_CURRENT_AIRCRAFTS);
                for (int i = 0; i < currentAircraftCount; i++) {
                    AircraftData_t *aircraft = &currentAircrafts[i];
                    if (aircraft->on_screen) {
                        int32_t dx = aircraft->screen_x - (X_MAX / 2);
                        int32_t dy = aircraft->screen_y - ((Y_MAX + ((Y_MAX - 70) / 2)) / 2);
                        int32_t distance = dx * dx + dy * dy;

                        if (distance < min_distance) {
                            min_distance = distance;
                            selectedAircraft = i;
                        }
                    }
                }
                G8RTOS_SignalSemaphore(&sem_CURRENT_AIRCRAFTS);

                // Signal the display to refresh with the new selection
                G8RTOS_SignalSemaphore(&sem_INFO_DISPLAY);
                G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);

            }


            // Clear the interrupt
            GPIOIntClear(JOYSTICK_INT_GPIO_BASE, JOYSTICK_INT_PIN);

            // Re-enable interrupt for the joystick
            GPIOIntEnable(GPIO_PORTD_BASE, JOYSTICK_INT_PIN);

            sleep (1000);
        }

        // check for any change requested in selected aircraft
        else {

            // Read the joystick data from the FIFO (packed as 32 bits: upper 16 bits = X, lower 16 bits = Y)
            joystick_dxy = JOYSTICK_GetXY();

            // Unpack the X and Y values
            joystick_dx = joystick_dxy & 0xFFFF;          // Lower 16 bits for X
            joystick_dy = (joystick_dxy >> 16) & 0xFFFF;  // Upper 16 bits for Y

            // Ignore joystick movement in the deadzone
            const int32_t DEADZONE = 900;
            const int32_t MIDPOINT = 4096 / 2;

            // Check if the joystick is in a neutral state
            int8_t isNeutral = (joystick_dx > (MIDPOINT - DEADZONE) && joystick_dx < (MIDPOINT + DEADZONE) &&
                                joystick_dy > (MIDPOINT - DEADZONE) && joystick_dy < (MIDPOINT + DEADZONE));

            if (!isNeutral) {

                // Debounce check
                if(!joystick_debounce){
                    sleep(100);
                    continue;
                }

                UARTprintf("X pos: %d \t", joystick_dx);
                UARTprintf("Y pos: %d \n", joystick_dy);
                joystick_debounce = false;

                // Update the currently selected aircraft & update the display to reflect that
                selectedAircraft = closest_aircraft_by_angle(joystick_dx, joystick_dy);
                G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);
                G8RTOS_SignalSemaphore(&sem_INFO_DISPLAY);


            } else {
                joystick_debounce = true;
                sleep(50);
                continue;
            }
        }

        sleep(100);
    }
}


/**
 * @brief Updates the display range based on button inputs.
 *
 * This thread listens for button presses to increase or decrease the display range. If
 * the range changes, it recalculates the aircraft positions and signals the main display
 * to refresh.
 *
 * The range is constrained between `MIN_RANGE` and `MAX_RANGE`.
 */
void Update_Search_Range(void){

    const int16_t MAX_RANGE = 200;
    const int16_t MIN_RANGE = 20;

    uint8_t button_status = 0;

    while(1){
        // wait for button semaphore
        G8RTOS_WaitSemaphore(&sem_PCA9555_Debounce);

        // debounce buttons
        sleep(5);

        // Get buttons
        G8RTOS_WaitSemaphore(&sem_I2CA);
        button_status = MultimodButtons_Get();
        G8RTOS_SignalSemaphore(&sem_I2CA);

        // clear button interrupt
        GPIOIntClear(BUTTONS_INT_GPIO_BASE, BUTTONS_INT_PIN);


        // check which buttons are pressed -- Increment or Decrement display range
        if (!(button_status & SW1)) {
            UARTprintf("SW1: +10km Search Range\n");
            display_range_km = (display_range_km + 10 <= MAX_RANGE) ? (display_range_km + 10) : MAX_RANGE;
            recalculate_screen_positions();
            G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);
        }

        else if (!(button_status & SW2)) {
            UARTprintf("SW2: -10km Search Range\n");
            display_range_km = (display_range_km - 10 >= MIN_RANGE) ? (display_range_km - 10) : MIN_RANGE;
            recalculate_screen_positions();
            G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);
        }

        else if (!(button_status & SW3)) {
            UARTprintf("SW3: Toggle True Track\n");
            display_track = !display_track;
            G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);
        }

        else if (!(button_status & SW4)) {
            UARTprintf("SW2: Toggle CallSign\n");
            display_callsign = !display_callsign;
            G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);
        }

        // Re-enable interrupt for the buttons
        GPIOIntEnable(GPIO_PORTE_BASE, BUTTONS_INT_PIN);

        sleep(50);
    }
}



/**
 * @brief Processes incoming aircraft data and updates the staging array.
 *
 * This thread reads aircraft data from the FIFO, parses it, and populates the `stagingAircrafts`
 * array. It converts raw integer data into meaningful float values for display.
 *
 * If the staging array is full, new data is ignored, and a warning is printed.
 */
void Process_New_Aircraft_Thread(void) {

    while (1) {
        // Wait for a complete message to be available
        G8RTOS_WaitSemaphore(&sem_DATA_READY);

        // Special case for call-sign
        int32_t low_byte = G8RTOS_ReadFIFO(DATA_FIFO);
        int32_t high_byte = G8RTOS_ReadFIFO(DATA_FIFO);
        int64_t callsign_raw = ((int64_t)high_byte << 32) | low_byte;

        // Send string to array (7 ASCII chars and a null terminator)
        char callsign[8];
        for (int32_t i = 0; i < 7; i++) {
            callsign[i] = (char)((callsign_raw >> (i * 8)) & 0xFF);
        }

        // Check for empty or invalid callsign
        if (callsign[0] == ' ') {
            strncpy(callsign, "N/A    ", sizeof(callsign));
        }

        UARTprintf("Call Sign: %s\t", callsign);


        // Cast the scaled integer into floats
        int32_t longitude_int = G8RTOS_ReadFIFO(DATA_FIFO);
        int32_t latitude_int = G8RTOS_ReadFIFO(DATA_FIFO);
        int32_t altitude_int = G8RTOS_ReadFIFO(DATA_FIFO);
        int32_t velocity_int = G8RTOS_ReadFIFO(DATA_FIFO);
        int32_t heading_int = G8RTOS_ReadFIFO(DATA_FIFO);

        float longitude = (float)longitude_int / 10000.0f;
        float latitude = (float)latitude_int / 10000.0f;
        float altitude = (float)altitude_int / 10000.0f;
        float velocity = (float)velocity_int / 10000.0f;
        float heading = (float)heading_int / 10000.0f;


        // cast floats to strings and print the strings
        char buffer[FLOAT_BUFF_SIZE];

        float_to_string(longitude, buffer, 4);
        UARTprintf("Longitude: %s\t", buffer);

        float_to_string(latitude, buffer, 4);
        UARTprintf("Latitude: %s\t", buffer);

        float_to_string(altitude, buffer, 4);
        UARTprintf("Altitude: %s\t", buffer);

        float_to_string(velocity, buffer, 4);
        UARTprintf("Velocity: %s\t", buffer);

        float_to_string(heading, buffer, 4);
        UARTprintf("True Track: %s\n", buffer);



        // Pack collected information into struct

        AircraftData_t aircraftData;
        strncpy(aircraftData.callsign, callsign, 8);

        // Assign the float values
        aircraftData.longitude = longitude;
        aircraftData.latitude = latitude;
        aircraftData.altitude = altitude;
        aircraftData.velocity = velocity;
        aircraftData.heading = heading;

        // Append new aircraft to staging array
        if (stagingAircraftCount < MAX_AIRCRAFTS)
            stagingAircrafts[stagingAircraftCount++] = aircraftData;
        else
            UARTprintf("Staging array overflow!\n");

    }
}



/**
 * @brief Transfers data from the staging array to the main aircraft array and updates screen positions.
 *
 * This thread synchronizes the `stagingAircrafts` array with the `currentAircrafts` array when a
 * burst of new data is received. It recalculates the screen positions for all updated aircraft
 * and signals the main display to refresh.
 *
 * Thread-safe access to both arrays is ensured with semaphores.
 */
void Update_Current_Aircrafts_Thread(void) {

    while (1) {
        G8RTOS_WaitSemaphore(&sem_BURST_COMPLETE);
        UARTprintf("BURST SEND COMPLETE!\n");

        // Synchronize access to staging array and currentAircrafts
        G8RTOS_WaitSemaphore(&sem_CURRENT_AIRCRAFTS);
        G8RTOS_WaitSemaphore(&sem_STAGING_AIRCRAFTS);

        // Quick check to preserve the selected aircraft
        char *SelectedCallSign = "N/A";
        if(selectedAircraft != -1){
            for (int32_t i = 0; i < currentAircraftCount; i++) {
                if(selectedAircraft == i)
                    SelectedCallSign = currentAircrafts[i].callsign;
            }
        }


        // Replace the main array with the staging array
        for (int32_t i = 0; i < stagingAircraftCount; i++) {
            currentAircrafts[i] = stagingAircrafts[i];

            // Update selected index if necessary
            if(currentAircrafts[i].callsign == SelectedCallSign)
                selectedAircraft = i;
        }

        // Reset the staging array count for the next burst
        currentAircraftCount = stagingAircraftCount;
        stagingAircraftCount = 0;

        // Release semaphores
        G8RTOS_SignalSemaphore(&sem_CURRENT_AIRCRAFTS);
        G8RTOS_SignalSemaphore(&sem_STAGING_AIRCRAFTS);

        // Calculate where new aircrafts belong on the screen
        recalculate_screen_positions();

        // Signal refresh screen
        G8RTOS_SignalSemaphore(&sem_MAIN_DISPLAY);

        sleep(100);

    }

}




/********************************Periodic Threads***********************************/
/*******************************Aperiodic Threads***********************************/


/**
 * @brief Handles button interrupts and signals the debounce semaphore.
 *
 * This handler is triggered when a button press interrupt occurs. It disables the button
 * interrupt temporarily and signals the semaphore responsible for handling the button logic.
 */
void Button_Handler(void) {

    //UARTprintf("Button interrupt triggered!\n");

    // Disable interrupt
    GPIOIntDisable(GPIO_PORTE_BASE, BUTTONS_INT_PIN);

    // Signal semaphore to handle button press
    G8RTOS_SignalSemaphore(&sem_PCA9555_Debounce);
}



void Joystick_Button_Handler(void){

    //UARTprintf("Joystick interrupt triggered!\n");

    // Disable interrupt
    GPIOIntDisable(GPIO_PORTD_BASE, JOYSTICK_INT_PIN);

    // Signal semaphore to handle joystick press
    G8RTOS_SignalSemaphore(&sem_Joystick_Debounce);

}



/**
 * @brief Handles incoming UART data for aircraft information.
 *
 * This handler processes incoming UART data from the FIFO. It reads bytes into a buffer and
 * reconstructs 32-bit words. Completed messages trigger semaphores to signal the data-ready
 * threads. End-of-burst signals are handled separately.
 */
void UART4_Handler(void) {
    uint32_t status = UARTIntStatus(UART4_BASE, true);
    UARTIntClear(UART4_BASE, status);

    static uint8_t byte_buffer[4];          // Temporary buffer to hold 4 bytes
    static uint8_t byte_index = 0;          // Tracks how many bytes have been read
    static uint8_t message_byte_count = 0;  // Tracks total bytes in the message

    while (UARTCharsAvail(UART4_BASE)) {
        // Read one byte
        uint8_t received_byte = UARTCharGetNonBlocking(UART4_BASE);

        // Add it to the buffer
        byte_buffer[byte_index++] = received_byte;
        message_byte_count++;

        // If the buffer is full (4 bytes), write to FIFO as an int32_t
        if (byte_index == 4) {
            int32_t word = (byte_buffer[0]) |
                           (byte_buffer[1] << 8) |
                           (byte_buffer[2] << 16) |
                           (byte_buffer[3] << 24);

            // if the end burst signal is sent, signal the semaphore
            if(word == 0xFFFFFFFF){
                G8RTOS_SignalSemaphore(&sem_BURST_COMPLETE);
                message_byte_count = 0;
            } else {

                // otherwise send the data to the FIFO for processing
                G8RTOS_WriteFIFO(DATA_FIFO, word);
            }

            byte_index = 0; // Reset buffer index

        }

        // If the entire message is received, signal the thread
        if (message_byte_count >= MESSAGE_SIZE) {
            G8RTOS_SignalSemaphore(&sem_DATA_READY);
            message_byte_count = 0;
        }

    }
}







