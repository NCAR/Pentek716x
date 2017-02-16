/**********************************************************************************************

Desc: Pentek 716x DDC Registers

**********************************************************************************************/
#include <stdint.h>
#ifndef DDCREGISTERS_H
#define DDCREGISTERS_H

namespace Pentek {

/// ADC FIFO Control Registers
static const uint32_t ADC_FIFO_CTRL_1   = 385;  ///< ADC #1
static const uint32_t ADC_FIFO_CTRL_2   = 392;  ///< ADC #2
static const uint32_t ADC_FIFO_CTRL_3   = 399;  ///< ADC #3
static const uint32_t ADC_FIFO_CTRL_4   = 406;  ///< ADC #4

/// The DAC FIFO Control Register
static const uint32_t DAC_FIFO_CONTROL  = 130;

/// The DDR Memory Control Register
static const uint32_t DDR_MEM_CONTROL   = 256;

/// Pentek DCM control register
static const uint32_t DCM_CONTROL       = 4;    ///< Pentek Gateflow defined register, used to reset the digital clock managers.

/// GateFlow option 223 ttl_in register
static const uint32_t TTL_IN            = 64;   ///< Partial implementation of the Pentek Gateflow Option 223 allows us to perform TTL input via this register.

/// GateFlow option 223 ttl_out1 register, 16 bits
static const uint32_t TTL_OUT1          = 65;   ///< Partial implementation of the Pentek Gateflow Option 223 allows us to perform TTL output via this register.

static const uint32_t XCVR_CNTRL        = 463;  ///< FRAP Transceiver firmware control register
static const uint32_t MT_ADDR           = 464;  ///< Multi-timer address and control
static const uint32_t MT_DATA           = 465;  ///< Multi-timer data
static const uint32_t MT_WR             = 466;  ///< Multi-timer write strobes
static const uint32_t KAISER_CTL        = 467;  ///< Kaiser filter coefficient address and control
static const uint32_t KAISER_DATA_LSW   = 468;  ///< Kaiser filter coefficient data (LSW)
static const uint32_t KAISER_DATA_MSW   = 469;  ///< Kaiser filter coefficient data (MSW)
static const uint32_t KAISER_WR         = 470;  ///< Kaiser filter write
static const uint32_t KAISER_READ_LSW   = 471;  ///< Kaiser filter coefficient read (LSW)
static const uint32_t KAISER_READ_MSW   = 472;  ///< Kaiser filter coefficient read (MSW)
static const uint32_t GAUSSIAN_CTL      = 473;  ///< Gaussian filter coefficient address and control
static const uint32_t GAUSSIAN_DATA_LSW = 474;  ///< Gaussian filter coefficient data (LSW)
static const uint32_t GAUSSIAN_DATA_MSW = 475;  ///< Gaussian filter coefficient data (MSW)
static const uint32_t GAUSSIAN_WR       = 476;  ///< Gaussian filter write
static const uint32_t GAUSSIAN_READ_LSW = 477;  ///< Gaussian filter coefficient read (LSW)
static const uint32_t GAUSSIAN_READ_MSW = 478;  ///< Gaussian filter coefficient read (MSW)
static const uint32_t RADAR_GATES       = 479;  ///< The number of radar gates
static const uint32_t CI_NSUM           = 480;  ///< The number of values for one coherent integrator sum
static const uint32_t FPGA_REPO_REV     = 481;  ///< The FPGA source code repository revision number
static const uint32_t DDC_DECIMATION1   = 482;  ///< Gatewidth decimation count chan 1 (sets gatewidth; is the number of DDC output clocks, e.g. ADC_CLK10D)
static const uint32_t DDC_DECIMATION2   = 483;  ///< Gatewidth decimation count chan 2
static const uint32_t DDC_DECIMATION3   = 484;  ///< Gatewidth decimation count chan 3
static const uint32_t SPOL_XMIT_FLAGS   = 485;  ///< SPOL transmitter flags

//Clock Decimation Registers
//  static const uint32_t DEC_REG = 0xA20;
//  static const uint32_t DEC_RST_REG = 0xA24;
//  static const uint32_t RST_ACT = 0x1;
//  static const uint32_t RST_CLR = 0x0;

//Gate Splitter Register
//  static const uint32_t DPRT_REG = 0xA50;
//Values
//  static const uint32_t DPRT_ON = 0x1;
//  static const uint32_t DPRT_OFF = 0x0;

// Transceiver control register bits
static const uint32_t XCBIT_FREE_RUN        = (1 << 0); ///< Free-run enable bit of the transceiver control register
static const uint32_t XCBIT_IGNORE_2ND_SYNC = (1 << 1); ///< Set to ignore secondary sync signal (after 1 PPS) when
                                                        ///< starting timers

// DDC Filter control bits
static const uint32_t DDC_START = 0x0; ///< Allow the filter to run. Set in the filter ADDR register.
static const uint32_t DDC_STOP = (0x01<<12); ///< Force the filter to stop. Set in the filter ADDR register.

//MultTimer Sub Registers
static const uint32_t WRITE_ON = 0x3;
static const uint32_t WRITE_OFF = 0x0;

static const uint32_t CONTROL_REG = 0x8;
static const uint32_t DELAY_REG = 0x9;
static const uint32_t WIDTH_REG = 0xA;
static const uint32_t PERIOD_REG = 0xB;
static const uint32_t PRT_REG = 0xC;

static const uint32_t W_Ch0 = (0x01<<4);
static const uint32_t W_Ch1 = (0x02<<4);
static const uint32_t W_Ch2 = (0x04<<4);
static const uint32_t W_Ch3 = (0x08<<4);
static const uint32_t K_Ch0 = (0x10<<4);
static const uint32_t K_Ch1 = (0x20<<4);
static const uint32_t K_Ch2 = (0x40<<4);
static const uint32_t K_Ch3 = (0x80<<4);

static const uint32_t GLOBAL_EN = (0x1<<12);
static const uint32_t GPS_EN = (0x1<<13);
static const uint32_t ADDR_TRIG = (0x1<<15);

static const uint32_t TIMER_ON = (0x1<<0);
static const uint32_t TIMER_OFF = (0x0<<0);
static const uint32_t TIMER_NEG = (0x1<<1);
static const uint32_t TIMER_POS = (0x0<<1);
static const uint32_t CLK_DIV1 = (0x0<<2);
static const uint32_t CLK_DIV2 = (0x1<<2);
static const uint32_t CLK_DIV4 = (0x2<<2);
static const uint32_t CLK_DIV8 = (0x3<<2);

static const uint32_t PRT1 = (0x0<<0);
static const uint32_t PRT2 = (0x0<<4);
static const uint32_t PRT3 = (0x0<<8);
static const uint32_t PRT4 = (0x0<<12);

// Timer DCM Lock Status
//  static const uint32_t FILTER_DCM_UNLOCKED = (0x1<<1);
//  static const uint32_t FILTER_DCM_RST = (0x1<<1);

//-------------------------------
//Added by Tom 1-3-08 for timing error detection
//Bits 23-20 of status register correspond to sync error for channel A-D
//static const uint32_t A_SYNC_ERROR = (0x1<<23);
//static const uint32_t B_SYNC_ERROR = (0x1<<22);
//static const uint32_t C_SYNC_ERROR = (0x1<<21);
//bstatic const uint32_t D_SYNC_ERROR = (0x1<<20);

}

#endif  // DDCREGISTERS_H
