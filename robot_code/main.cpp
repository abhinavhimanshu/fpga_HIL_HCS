/********************************  ROBOT CODE to get control ipnuts from FPGA *******************************************/


/*
 * email : abhinavhimanshu6@gmail.com  
 * date  : 5th,July,2017
 * 
 * Description: Robot receives control inputs from FPGA
 *              Robot pings the FPGA and Enters in a loop where it receives control inputs periodically
 *                          
 */



/* inlcude files   */
#include <khepera/khepera.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <khepera/khepera.h>
#include <signal.h>
#include <stdint.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>

static knet_dev_t* dsPic; // robot pic microcontroller access
static int quitReq = 0;   // quit variable for loop

using namespace std;

/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler(int)
{
    quitReq = 1;
    printf("exited \n");
    kh4_set_speed(0, 0, dsPic); // stop robot
    kh4_SetMode(kh4RegIdle, dsPic);
    kh4_SetRGBLeds(0, 0, 0, 0, 0, 0, 0, 0, 0, dsPic); // clear rgb leds because consumes energy
    kb_change_term_mode(0); // revert to original terminal if called
    printf("exited gracefully\n");
    exit(0);
}


int main(void)

{

    int clientSocket, nBytes; //socket to receive data from FPGA
    char buffer[1]={'p'};     // ping buffer
    float buff[2];            //buffer to receive control inputs
    struct sockaddr_in serverAddr;//Address details of FPGA socket
    socklen_t addr_size;     
    float vel_right, vel_left;  // left and right velocities of Robot
    int kp, ki, kd;             // PID control variables
    char Buffer[100],revision, version;

   /* tuned parameters */
    kp = 10;
    ki = 5;
    kd = 1;
    kh4_ConfigurePID(kp, ki, kd, dsPic); // configure P,I,D

    printf("\nClosed Loop Trajectory Tracking Program\n");

    // initiate libkhepera and robot access
    if (kh4_init(0, NULL) != 0)
    {
        printf("\nERROR: could not initiate the libkhepera!\n\n");
        return -1;
    }

    /* open robot socket and store the handle in its pointer */
    dsPic = knet_open("Khepera4:dsPic", KNET_BUS_I2C, 0, NULL);

    if (dsPic == NULL)
    {
        printf("\nERROR: could not initiate communication with Kh4 dsPic\n\n");
        return -2;
    }


    // get revision
    if (kh4_revision(Buffer, dsPic) == 0)
    {
        version = (Buffer[0] >> 4) + 'A';
        revision = Buffer[0] & 0x0F;
        printf("\r\nVersion = %c, Revision = %u\r\n", version, revision);
    }

  signal(SIGINT, ctrlc_handler); // set signal for catching ctrl-c
  kh4_SetMode(kh4RegSpeed, dsPic);

  /*Create UDP socket*/
  clientSocket = socket(PF_INET, SOCK_DGRAM, 0);
  if(clientSocket > 0)
    printf("Socket created succesfully\n");
  else{
    printf("Error Creatinf the socket\n");
     return clientSocket;
  }

  /*Details of FPGA server address */
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(7819);
  serverAddr.sin_addr.s_addr = inet_addr("192.168.0.105");
  memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  
  addr_size = sizeof serverAddr;
 
  /* Ping FPGA */  	
  sendto(clientSocket,buffer,nBytes,0,(struct sockaddr *)&serverAddr,addr_size);
  printf("Ping SENT\n");
  
 while(1){ 
    //Wait to recieve control inputs
        printf("wait\n");
        recvfrom(clientSocket,buff,sizeof(buff),0,NULL, NULL);
    /* map u and v to vel_left and vel_right as follows */
          vel_left = buff[0];
	  vel_right = buff[1];
          printf("\n%f and %f\n", buff[0], buff[1]);
          kh4_set_speed(vel_left, vel_right, dsPic);
 }

    return 0;
}
