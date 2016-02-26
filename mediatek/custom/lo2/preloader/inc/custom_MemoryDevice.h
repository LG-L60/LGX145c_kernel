#ifndef __CUSTOM_MEMORYDEVICE__
#define __CUSTOM_MEMORYDEVICE__

/*
 ****************************************************************************
     Specify memory part number and clock rate
     Example :
     #define CS_PART_NUMBER[0]       KMN5U000ZM_B203
     #define EMI_CLK[0]              266M
     #define CS_PART_NUMBER[1]       H9TA4GH2GDMCPR_4GM
     #define EMI_CLK[1]              266M
 ****************************************************************************
*/

#define BOARD_ID                MT6572_EVB1
//<2014/02/26-kyle chang, CONFIG MCP
#define CS_PART_NUMBER[0]       H9TP32A4GDCCPR_KGM
//>2014/02/26-kyle chang
#define EMI_CLK[0]              266M

//<2014/05/12-davidda,37756,[5502][BSP]add second memory
#define CS_PART_NUMBER[1]       KMN5X000ZM_B209
#define EMI_CLK[1]              266M
//>2014/05/12-davidda
      
#endif /* __CUSTOM_MEMORYDEVICE__ */
