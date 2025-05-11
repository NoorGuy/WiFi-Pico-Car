#include "lwip/apps/httpd.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/netif.h" // Include to get network interface
#include "lwip/ip_addr.h"
#include "lwipopts.h"
#include "ssi.h"
#include "cgi.h"

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
const char WIFI_SSID[] = "PMAX";
const char WIFI_PASSWORD[] = "126065as*/-";

// Declare movement flags
volatile bool go_forward_flag = false;
volatile bool go_back_flag = false;
volatile bool turn_left_flag = false;
volatile bool turn_right_flag = false;
volatile bool stop_flag = false;

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
