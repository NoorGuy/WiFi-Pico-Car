#ifndef CGI_H
#define CGI_H

#include "lwip/apps/httpd.h"
#include "pico/cyw43_arch.h"
#include <string.h>
#include <stdbool.h>

// Declare movement flags defined in main.c
extern volatile bool go_forward_flag;
extern volatile bool go_back_flag;
extern volatile bool turn_left_flag;
extern volatile bool turn_right_flag;
extern volatile bool stop_flag;
extern volatile bool go_to_home_flag;

// CGI handler for motor control via /led.cgi
const char * cgi_led_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
    // Reset all flags first
    go_forward_flag = false;
    go_back_flag = false;
    turn_left_flag = false;
    turn_right_flag = false;
    stop_flag = false;
    go_to_home_flag = false;

    // Parse parameters safely
    for (int i = 0; i < iNumParams; i++) {
        if (strcmp(pcParam[i], "led") == 0) {
            if (strcmp(pcValue[i], "0") == 0) {
                stop_flag = true;
            } else if (strcmp(pcValue[i], "1") == 0) {
                go_forward_flag = true;
            } else if (strcmp(pcValue[i], "2") == 0) {
                go_back_flag = true;
            } else if (strcmp(pcValue[i], "3") == 0) {
                turn_right_flag = true;
            } else if (strcmp(pcValue[i], "4") == 0) {
                turn_left_flag = true;
            } else if (strcmp(pcValue[i], "5") == 0) {
                go_to_home_flag = true;
            }
        }
    }

    return "/index.shtml"; // Redirect back to main control page
}

// Link handler with route
static const tCGI cgi_handlers[] = {
    { "/led.cgi", cgi_led_handler }
};

// Register CGI handlers
void cgi_init(void) {
    http_set_cgi_handlers(cgi_handlers, sizeof(cgi_handlers) / sizeof(tCGI));
}

#endif
