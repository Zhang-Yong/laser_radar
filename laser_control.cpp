/*
 * laser_control.cpp
 *
 *  Created on: 2016年7月11日
 *      Author: Zhang Yong
 *  Revised from  Dom and Gert 's example program
 */

// // call sequencer :  setup_io() --> set_laser_on / set_laser_off

//#define BCM2708_PERI_BASE        0x20000000
#define BCM2708_PERI_BASE        0x3F000000				// change for raspi 2 & 3
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */
// add for GPIO driver current setting, Yong
#define PADS_BASE                (BCM2708_PERI_BASE + 0x100000) /* PADS   */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

int  mem_fd;
void *gpio_map;

void *gpio_drive_current_map;	 //output drive current setup, Yong

// I/O access
volatile unsigned *gpio;
//for driver current, Yong
volatile unsigned *pads;


// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock


#define PADS0   *(pads+11)			// gpio 0-27
#define PADS1   *(pads+12)			// gpio 28-45
#define PADS2   *(pads+13)			// gpio 46-53





void set_laser_on()
{

      // Set pin 7 ( physical pin 26 )
      GPIO_SET = 1 << 7;

} // set_laser_on


void set_laser_off()
{

    GPIO_CLR = 1 << 7;
} //set_laser_off

//
// Set up a memory regions to access GPIO
//
void setup_io()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
      printf("can't open /dev/mem \n");
      exit(-1);
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );

   //close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
	   // Yong, comment out temp for compile error
     // printf("mmap error %d\n", (int)gpio_map);//errno also set!
      exit(-1);
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned *)gpio_map;

   //setup GPIO drive current here , Yong
   /* mmap GPIO */
      gpio_drive_current_map = mmap(
         NULL,             //Any adddress in our space will do
         BLOCK_SIZE,       //Map length
         PROT_READ|PROT_WRITE,// Enable reading & writting to mapped memory
         MAP_SHARED,       //Shared with other processes
         mem_fd,           //File to map
		 PADS_BASE         //Offset to GPIO peripheral
      );

   if (gpio_drive_current_map == MAP_FAILED) {
  	   // Yong, comment out temp for compile error
       // printf("mmap error %d\n", (int)gpio_drive_current_map);//errno also set!
        exit(-1);
     }
     // Always use volatile pointer!
     pads = (volatile unsigned *)gpio_drive_current_map;

     // setup drvier current here, has to after mmap, Yong
       PADS0 = (0xFFFFFFF8 & PADS0) | 7 ;// Sets GPIO 0-27 to 16mA
       //PADS1 = (0xFFFFFFF8 & PADS1) | 4; // Sets GPIO 28-45 to 10mA
       //PADS2 = (0xFFFFFFF8 & PADS2);     // Sets GPIO 46-53 to 2mA

   close(mem_fd); //No need to keep mem_fd open after mmap

   // Define pin 26, GPIO 7 as output
       INP_GPIO(7); // must use INP_GPIO before we can use OUT_GPIO
       OUT_GPIO(7);

} // setup_io


