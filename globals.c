
#include "constants.h"


unsigned char RX_Data_0;
unsigned char RX_Data_1;
unsigned char RX_data_received;

unsigned char RX_data_uc;
unsigned char RX_data_auc[RX_ARRAY_SIZE];
unsigned char RX_data_index_uc = 0;
unsigned char ThisIsAddress_uc = 0;

unsigned char stepDir_uc = 0;

int   steps_i;
int   speed_i;
unsigned char   direction_uc;

unsigned char commandReceived_uc;

int i, j, k; 
