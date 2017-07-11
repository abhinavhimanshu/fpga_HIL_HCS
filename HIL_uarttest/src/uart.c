/*
 * Placeholder PetaLinux user application.
 * author: Abhinav Himanshu
 * email:  abhinavhimanshu6@gmail.com
 * date:   5,July,2017
 *
 * Description ::  FPGA has determinized controller in the form of IP which takes state input in bits(BDD Variables).
 *               > Make sure state and input space is less than 32 bit or make changes in vivado synthesis and edit the IP core
 *                 FPGA receives x y and theta values in form of BDD variables from the Matlab simultion code through UART 
 *                 Ands sends back the Control inputs in form of a BDD variable through UART to matlab 
 *                 
 *                 Make sure that you are not logged in to the FPGA through any-other serial terminal if so, 
 *                 then logout through that terminal close it .... and then run the Matlab code
 *                 =================================================================
 *                  
 */
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "ourheader.h"


//config for UART Communication //Do not touch
int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf ("Error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                printf ("Error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

//To set the communication blocking
void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("Error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf ("Error %d setting term attributes", errno);
}

//Read line from buffer and return the echo
int readLine(int fd, char* lineBuffer, int max){

    char ch;
    int count = 0;

    read(fd, &ch, 1);
    do
    {
        while(read(fd, &ch, 1) < 1);

	if(ch != '\n' && ch != '\r'){
		lineBuffer[count] = ch;
        	count++;
	}
	if(ch == '\n'){
		write (fd, "\r\n", 2);
		usleep (4);
	}
	else{
		write (fd, &ch, 1);
		usleep (2);
	}

    }while ((ch != '\n') && (count != max-1));

    lineBuffer[count] = '\0';
    return count++;
}


char *portname = "/dev/ttyPS0"; //portname of FPGA


/****************************************************************************************************/

int main(int argc, char *argv[]){
	unsigned long exit=0,fd,state;
	ssize_t n;
	char buf[100];
	char action[100];

	fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);//open the PORT
	if (fd < 0){
		printf("Error: %d opening %s: %s", errno, portname, strerror (errno));
		return 0;
	}

	set_interface_attribs (fd, B115200, 0);  	// set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                		// set no blocking
	tcflush(fd, TCIOFLUSH);
	write (fd, "New HIL started\r\n", 17);      // send 7 character greeting
    int* w = getpointer(0);
    int* r = getpointer(4);
	do{
		n = readLine(fd, buf, sizeof buf);
		if(n > 0){
			if(strcmp(buf,"exit")==0){
				exit = 1;
			}
			else if(strcmp(buf,"ping")==0){
				write (fd, "pong\r\n", 6);
				usleep (6*2);
			}
			else{

				state = atol(buf);//string to int
		        writeme(w,state);// write at the address of Controller IP CORE INPUT
		        n = sprintf(action,"%d",readme(w));
				write (fd,action,n);//Read the input and send back
				write (fd, "\r\n", 2);//send ending characters
				usleep ((n+2) * 2);
				n = sprintf(action,"%d",readme(r));//Read result from Controller IP CORE
				write (fd,action,n); //send the result through UART
				write (fd, "\r\n",2);//send ending characters
				usleep ((n+2) * 2);

		    }
		}
	}while(!exit);

	return 0;
}
/****************************************************************************************************/
