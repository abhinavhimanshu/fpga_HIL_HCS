/*
 *
 * email: abhinavhimanshu6@gmail.com
 * date: 5,July,2017
 * Description : This header file allows to get pointer to physical address on fpga.
 *               You can pass the base address of the controller IP core in your design as  BDD_PHY_ADDRESS
 *               Offset address to read and write
 *               Map size and map mask
 *               All these parameters are available in the Address editor of your vivado file
 *            -> To see these details: Open your Vivado Project > click on Open block design > Goto Windows from drop down list select Address Editor
 *               MAP_Size = pow(2,(RANGE)/((Data width)/offset in address of each register))
 *               Look into the vivado videos for ip synthesis process
 *
 */


#ifndef OURHEADER_H_
#define OURHEADER_H_


/*include files*/
#include <stdint.h>
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stddef.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#define BDD_PHY_ADDRESS 0x43C00000  //Intial the Base Address of your IP CORE * depends on upon Vivado synthesis process
#define BDD_INP_OFFSET 0            //Offest for INPUT Feed ... slvReg0 * depends on upon IP core Vivado synthesis process
#define BDD_OUT_OFFSET 4            //Offest for OUTPUT Feed ... slvReg1 * depends on upon IP core Vivado synthesis process

// You need to be in root to run this !! * But we are always in root Petalinux zynq core
#define MAP_SIZE 65536UL		//4K range of the AXI BDD decided when making the block design in Vivado
#define MAP_MASK (MAP_SIZE-1)



//Returns pointers of any physical address
//Try  not touching this
void* get_v_addr(int phys_addr){
	void* mapped_base;
	int memfd;

	void* mapped_dev_base;
	off_t dev_base = phys_addr;

	memfd = open("/dev/mem",O_RDWR | O_SYNC);
	if(memfd == -1){
		printf("Cant open /dev/mem.\n");
		exit(0);
	}

	mapped_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, dev_base & ~MAP_MASK);
	if(mapped_base == (void*)-1){
		printf("Cant map the physial address to user-space !.\n");
		exit(0);
	}

	mapped_dev_base = mapped_base + (dev_base & MAP_MASK);
	return mapped_dev_base;
}


//Get Base Address not needed
void* getbaseaddrs()
{

	int* baseaddr = get_v_addr(BDD_PHY_ADDRESS);
	return baseaddr;

}

//Get pointer to the offset address
void* getpointer(int offset)
{

	int* baseaddr = get_v_addr(BDD_PHY_ADDRESS+offset);
    return baseaddr;

}

int getaddrsdiff(){
					int  addrsdiff  = get_v_addr((int)(BDD_PHY_ADDRESS)+1)-get_v_addr(BDD_PHY_ADDRESS);
					return addrsdiff;
}

//Reads the value at p address
int readme(int* p){

    return *p;
}

//Writes the value v at p address
void writeme(int *p, int v){
	*p = v;
}

#endif /* OURHEADER_H_ */
