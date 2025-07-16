#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h" // Include to get network interface
#include "lwip/ip_addr.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"
#include "hardware/uart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//GPS
#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 0
#define UART_RX_PIN 1
//GPS buffers
#define BUF_SIZE 1024
char gps_buffer[BUF_SIZE];

volatile float current_latitude = 0.0;
volatile float current_longitude = 0.0;
    // HOME COORDINATES
const float home_lat = 37.7749;
const float home_long = -122.4194;

// MOTOR DRIVER

// A
#define EN_A 14
#define IN_1 10 // forward
#define IN_2 11 // back

// B
#define EN_B 15
#define IN_3 12 // forward
#define IN_4 13 // back

// WIFI Credentials - take care if pushing to github!
const char WIFI_SSID[] = "KAFI4G";
const char WIFI_PASSWORD[] = "126065AS";

// Declare movement flags
volatile bool go_forward_flag = false;
volatile bool go_back_flag = false;
volatile bool turn_left_flag = false;
volatile bool turn_right_flag = false;
volatile bool stop_flag = false;

void init_uart() {
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
}

// Convert NMEA lat/lon to decimal
float convert_to_decimal(const char* nmea_coord, const char* direction) {
    float raw = atof(nmea_coord);
    int degrees = (int)(raw / 100);
    float minutes = raw - (degrees * 100);
    float decimal = degrees + (minutes / 60.0f);

    if (direction[0] == 'S' || direction[0] == 'W') {
        decimal *= -1;
    }

    return decimal;
}

void parse_gprmc(const char* sentence) {
    char buffer[BUF_SIZE];
    strncpy(buffer, sentence, BUF_SIZE);
    char* token = strtok(buffer, ",");

    int field = 0;
    char lat[16] = {0};
    char ns[2] = {0};
    char lon[16] = {0};
    char ew[2] = {0};
    char valid = 'V';

    while (token) {
        switch (field) {
            case 2: valid = token[0]; break;
            case 3: strncpy(lat, token, sizeof(lat)); break;
            case 4: strncpy(ns, token, sizeof(ns)); break;
            case 5: strncpy(lon, token, sizeof(lon)); break;
            case 6: strncpy(ew, token, sizeof(ew)); break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (valid == 'A') {
        current_latitude = convert_to_decimal(lat, ns);
        current_longitude = convert_to_decimal(lon, ew);
        printf("Latitude: %.6f, Longitude: %.6f\n", current_latitude, current_longitude);

    } else {
        printf("No valid GPS fix.\n");

    }
}

void read_gps_data() {
    static int idx = 0;

    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);
        if (c == '\n' || idx >= BUF_SIZE - 1) {
            gps_buffer[idx] = '\0';
            idx = 0;

            if (strstr(gps_buffer, "$GPRMC")) {
                parse_gprmc(gps_buffer);
            }
        } else {
            gps_buffer[idx++] = c;
        }
    }
}

float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
    lat1 = deg2rad(lat1);
    lat2 = deg2rad(lat2);
    float dLon = deg2rad(lon2 - lon1);

    float y = sin(dLon) * cos(lat2);
    float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    float bearing = atan2(y, x);
    return fmod((bearing * (180.0f / M_PI) + 360.0f), 360.0f); // Normalize to 0–360
}


void go_forward(void) {
    gpio_put(IN_1, 1); // forward
    gpio_put(IN_3, 1); // forward
    gpio_put(IN_2, 0);
    gpio_put(IN_4, 0);
    gpio_put(EN_A, 1);
    gpio_put(EN_B, 1);
    printf("Moving forward\n");
}

void go_back(void) {
    gpio_put(IN_2, 1);
    gpio_put(IN_4, 1);
    gpio_put(IN_1, 0);
    gpio_put(IN_3, 0);
    gpio_put(EN_A, 1);
    gpio_put(EN_B, 1);
    printf("Moving backward\n");
}

void turn_left(void) {
    gpio_put(IN_3, 1); // forward
    gpio_put(IN_4, 0);
    gpio_put(EN_B, 1);
    gpio_put(EN_A, 0);
    printf("Turning left\n");
}

void turn_right(void) {
    gpio_put(IN_1, 1); // forward
    gpio_put(IN_2, 0);
    gpio_put(EN_A, 1);
    gpio_put(EN_B, 0);
    printf("Turning right\n");
}

void stop(void) {
    gpio_put(EN_A, 0);
    gpio_put(EN_B, 0);
    printf("Stopping\n");
}

int main() {
    stdio_init_all();
    //GPS initialization
    init_uart();

 // printf("GPS Module Initialized (Parsing GPRMC)...\n");

    // MOTOR DRIVER
    gpio_init(EN_A);
    gpio_set_dir(EN_A, GPIO_OUT);

    gpio_init(IN_1);
    gpio_set_dir(IN_1, GPIO_OUT);

    gpio_init(IN_2);
    gpio_set_dir(IN_2, GPIO_OUT);

    gpio_init(EN_B);
    gpio_set_dir(EN_B, GPIO_OUT);

    gpio_init(IN_3);
    gpio_set_dir(IN_3, GPIO_OUT);

    gpio_init(IN_4);
    gpio_set_dir(IN_4, GPIO_OUT);

    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();

    // Connect to the WiFi network - loop until connected
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0) {
        printf("Attempting to connect...\n");
    }
    printf("Connected! \n");

    // Retrieve and print the assigned IP address
    printf("IP Address: %s\n", ipaddr_ntoa(&netif_default->ip_addr));

    // Initialise web server
    httpd_init();
    printf("Http server initialised\n");

    // Configure SSI and CGI handler
    ssi_init();
    printf("SSI Handler initialised\n");
    cgi_init();
    printf("CGI Handler initialised\n");


    // Infinite loop to execute actions based on flags
    while (1) {
        // Bearing
        float homeBearing = calculateBearing(current_latitude, current_longitude, home_lat, home_long);
      //printf("Bearing to home: %.2f degrees\n", homeBearing);
        read_gps_data();  // actively read and update current_latitude and current_longitude

        // Convert to integer representation (scaled by 1,000,000)
        int32_t int_lat = (int32_t)(current_latitude * 1000000);
        int32_t int_lon = (int32_t)(current_longitude * 1000000);

        int32_t target_lat = (int32_t)(home_lat * 1000000);
        int32_t target_lon = (int32_t)(home_long * 1000000);

        // Simple threshold check (~3m range)
        if (abs(int_lat - target_lat) < 300 && abs(int_lon - target_lon) < 300) {
            stop_flag = true;
            }
        
        if (go_forward_flag) {
            go_forward();
            // Remove the flag reset here to keep moving forward continuously
        } else if (go_back_flag) {
            go_back();
            // Remove the flag reset here to keep moving backward continuously
        } else if (turn_left_flag) {
            turn_left();
            // Remove the flag reset here to keep turning left continuously
        } else if (turn_right_flag) {
            turn_right();
            // Remove the flag reset here to keep turning right continuously
        } else if (stop_flag) {
            stop();
            // Do not clear stop_flag here so it keeps stopping continuously
        }

        // Optionally reset flags when a new command is triggered
        if (go_forward_flag || go_back_flag || turn_left_flag || turn_right_flag) {
            stop_flag = false; // Ensure stop is cleared when a new command is triggered
        }

    }

    return 0; // Should never reach here
}
