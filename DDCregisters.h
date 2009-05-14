/**********************************************************************************************

Desc: Pentek 7142 HCR DDC Registers

**********************************************************************************************/
// ADC FIFO Control Registers

#define ADC_FIFO_CTRL_1 0x8C08  //ADC #1
#define ADC_FIFO_CTRL_2 0x8C40  //ADC #2
#define ADC_FIFO_CTRL_3 0x8C78  //ADC #3
#define ADC_FIFO_CTRL_4 0x8CB0  //ADC #4

//PulseWidths for Decimation
#define _0_2us 2
#define _0_4us 4
#define _0_6us 6
#define _0_8us 8
#define _1_0us 10
#define _1_2us 12
#define _1_4us 14
#define _1_6us 16


//Kaiser Filter Registers
#define KAISER_ADDR 0x8E98
#define KAISER_DATA_MSW 0x8EA8
#define KAISER_DATA_LSW 0x8EA0
#define KAISER_READ_MSW 0x8EC0
#define KAISER_READ_LSW 0x8EB8
#define KAISER_WR       0x8EB0
//Values
#define DDC_START 0x0
#define DDC_STOP  (0x01<<12)

//Gaussian Filter Registers
#define GUASSIAN_ADDR 0x8EC8
#define GUASSIAN_DATA_MSW 0x8ED8
#define GUASSIAN_DATA_LSW 0x8ED0
#define GUASSIAN_READ_MSW 0x8EF0
#define GUASSIAN_READ_LSW 0x8EE8
#define GUASSIAN_WR       0x8EE0

/// The number of radar gates
#define RADAR_GATES         0x8EF8
/// The number of values for one coherent integrator sum
#define CI_NSUM             0x8F00
/// The fpga source code repository revision number
#define FPGA_REPO_REV       0x8F08

//Clock Decimation Registers
//  #define DEC_REG     0xA20
//  #define DEC_RST_REG 0xA24
//  #define RST_ACT     0x1
//  #define RST_CLR     0x0

//Gate Splitter Register
//  #define DPRT_REG 0xA50
//Values
//  #define DPRT_ON  0x1
//  #define DPRT_OFF 0x0

//MultiTimer Registers
#define MT_ADDR 0x8E80
#define MT_DATA 0x8E88
#define MT_WR   0x8E90

//MultTimer Sub Registers
#define WRITE_ON  0x3
#define WRITE_OFF 0x0

#define CONTROL_REG 0x8
#define DELAY_REG   0x9
#define WIDTH_REG   0xA
#define PERIOD_REG  0xB
#define PRT_REG     0xC

#define TIMER0 (0x01<<4)
#define TIMER1 (0x02<<4)
#define TIMER2 (0x04<<4)
#define TIMER3 (0x08<<4)
#define TIMER4 (0x10<<4)
#define TIMER5 (0x20<<4)
#define TIMER6 (0x40<<4)
#define TIMER7 (0x80<<4)

#define W_Ch0 (0x01<<4)
#define W_Ch1 (0x02<<4)
#define W_Ch2 (0x04<<4)
#define W_Ch3 (0x08<<4)
#define K_Ch0 (0x10<<4)
#define K_Ch1 (0x20<<4)
#define K_Ch2 (0x40<<4)
#define K_Ch3 (0x80<<4)

#define TIMER_EN  (0x1<<12)
#define ADDR_TRIG (0x1<<15)

#define TIMER_ON  (0x1<<0)
#define TIMER_OFF (0x0<<0)
#define TIMER_NEG (0x1<<1)
#define TIMER_POS (0x0<<1)
#define CLK_DIV1  (0x0<<2)
#define CLK_DIV2  (0x1<<2)
#define CLK_DIV4  (0x2<<2)
#define CLK_DIV8  (0x3<<2)

#define PRT1 (0x0<<0)
#define PRT2 (0x0<<4)
#define PRT3 (0x0<<8)
#define PRT4 (0x0<<12)

// SVN Revision Register on V4
//  #define SVN_REV_ADR 0xA5C

// Timer DCM Lock Status
//  #define FILTER_DCM_UNLOCKED (0x1<<1)
//  #define FILTER_DCM_RST (0x1<<1)

//-------------------------------
//Added by Tom 1-3-08 for timing error detection
//Bits 23-20 of status register correspond to sync error for channel A-D
#define A_SYNC_ERROR (0x1<<23)
#define B_SYNC_ERROR (0x1<<22)
#define C_SYNC_ERROR (0x1<<21)
#define D_SYNC_ERROR (0x1<<20)

