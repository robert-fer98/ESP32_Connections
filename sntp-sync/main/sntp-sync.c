#include <stdio.h>
#include "esp_sntp.h"

// EXPLANATION
/*
SNTP uses Internet to connect to the server and sync clocks. That's why in the menu config, the network name and password must be defined. 


*/

/* available functions 
gettimeofday
time
asctime
clock
ctime
difftime
gmtime
localtime
mktime
strftime

*/

// functions

sntp_sync_status_t sntp_get_sync_status(void); // getting sync status
void sntp_set_sync_status(sntp_sync_status_t sync_status); // setting sync status

void app_main(void)
{

}
