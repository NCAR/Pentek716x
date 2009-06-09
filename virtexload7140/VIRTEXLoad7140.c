/******************************************************************************
 * VERSION DATA:        $Archive: /src/Drivers/share/drv/VIRTEXLoad7140.c $
 *                      $Revision: 7 $
 *                      $Modtime: 2/14/07 2:22p $
 * AUTHOR:
 * DESCRIPTION:         This program can be used to upload a FPGA
 * configuration file to a Xilinx Virtex II via a Pentek device driver.
 * Currently, this program only works with the 7140
 *
 *
 ******************************************************************************/


#ifdef WIN32
#define sleep(t) Sleep(t*1000)
#else
#include <fcntl.h>
#endif


#ifdef VXWORKS
#include <ioLib.h>
#define sleep(t) usleep(t*1000000)
#endif

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "ptkddr.h"

#define VIRTEX_CFG_REG  0x80
#define VIRTEX_DATA_REG 0x80

static unsigned int    peek(unsigned int addr);
static void            poke(unsigned int addr, unsigned int data);
static unsigned long   *load_mcs_file(FILE *infile, unsigned int *length);

#ifdef WIN32
static HANDLE fdc;
static DWORD bR;
static BOOL bSuccess;
#else
static int fdc;
#endif

#ifdef VXWORKS
int VIRTEXLoad7140(char *deviceName, char *mcsfile)
{
#else
int main(int argc, char **argv)
{
  char deviceName[80];
  char mcsfile[80];
#endif

  unsigned int done;
  FILE *infile;
  unsigned long *BootProgram;
  unsigned int length;
  volatile unsigned int c_word;
  unsigned int i;
  unsigned int out_byte;
  unsigned int in_byte;


#ifndef VXWORKS
  if (argc < 3)
    {
#if WIN32
      printf("Enter Board Device name: Example  \\\\.\\p7140i1 \n");
      scanf("%s",deviceName);

      printf("Enter mcs file path:\n");
      scanf("%s",mcsfile);
#else
      printf("VIRTEXLoad7140 <path to board/ctrl device> <mcs file>\n");
      return(1);
#endif
    }
  else
    {
      strcpy(mcsfile,argv[2]);
      strcpy(deviceName,argv[1]);
    }
#endif

  infile = fopen(mcsfile,"r");

  if (infile == NULL)
  {
        printf("VIRTEXLoad: failed to open input file\n");
        return(1);
  }

  BootProgram = load_mcs_file(infile, &length);

  fclose(infile);

  if (BootProgram == NULL)
    {
      printf("VIRTEXLoad: Failed to load mcs file\n");
      return(1);
    }

#ifdef WIN32
  fdc = CreateFile(deviceName,GENERIC_READ,0,NULL,OPEN_EXISTING,
                   FILE_ATTRIBUTE_NORMAL,NULL );

  if (fdc  == INVALID_HANDLE_VALUE)
    {
      printf("VIRTEXLoad: Failed to obtain device file handle: Win32 error code: %d\n",GetLastError() );
      return 1;
    }
#else
  fdc = open(deviceName,O_RDONLY,0);
  if (fdc == -1)
    {
      printf("VIRTEXLoad: Open of device failed\n");
      return(1);
    }
#endif

  /* Disable all interrupts, so we don't get hung up */

  poke(0x10,0x0);

  /* set Virtex into SelectMap mode and reset */

  poke(VIRTEX_CFG_REG, 0x00);
  poke(VIRTEX_CFG_REG, 0x01);   /* Set to upload */
  poke(VIRTEX_CFG_REG, 0x03);   /* prog_b low */
  poke(VIRTEX_CFG_REG, 0x01);   /* prog_b high */
  sleep(1);

  if ((peek(VIRTEX_CFG_REG) & 0x8) == 0)
    {
      printf("Int_B never went high\n");
      return(1);
    }

  sleep(1);

  /* Set rdwr_b low and prog_b high */

  poke(VIRTEX_CFG_REG, 0x21);

  sleep(1);

  if ((peek(VIRTEX_CFG_REG) & 0x8) == 0)
    {
      printf("Int_B never went high\n");
      return(1);
    }

  sleep(1);

  if ((peek(VIRTEX_CFG_REG) & 0x4) == 0x4)
    {
      printf("DONE never went away: %x\n",peek(VIRTEX_CFG_REG));
      return(1);
    }

  printf("Image Size = %x bytes\n",length * 4);
  printf("Loading");

  /* Load image into Xilinx */
  for (i = 0; i < length; i++)
    {
      c_word = BootProgram[i];

      out_byte = ((((c_word >> 24) /*& 0xff*/) << 8) | 0xa1);

      poke((VIRTEX_DATA_REG), out_byte);

#if 0
      in_byte = peek(VIRTEX_DATA_REG);

      if ((in_byte & 0xff00) != (out_byte & 0xff00))
        printf("Failed to readback, count = %d, expecting = %x, got = %x\n",i,
               in_byte,out_byte);
#endif

      out_byte =  ((((c_word >> 16) /*& 0xff*/) << 8) | 0xa1);

      poke((VIRTEX_DATA_REG), out_byte);

#if 0

      in_byte = peek(VIRTEX_DATA_REG);

      if ((in_byte & 0xff00) != (out_byte & 0xff00))
        printf("Failed to readback, count = %d, expecting = %x, got = %x\n",i,
               in_byte,out_byte);
#endif

      out_byte = ((((c_word >> 8) /*& 0xff*/) << 8) | 0xa1);

      poke((VIRTEX_DATA_REG), out_byte);

#if 0
      in_byte = peek(VIRTEX_DATA_REG);

      if ((in_byte & 0xff00) != (out_byte & 0xff00))
        printf("Failed to readback, count = %d, expecting = %x, got = %x\n",i,
               in_byte,out_byte);
#endif

      out_byte = (((c_word /*& 0xff*/) << 8) | 0xa1);

      poke((VIRTEX_DATA_REG), out_byte);

#if 0
      in_byte = peek(VIRTEX_DATA_REG);

      if ((in_byte & 0xff00) != (out_byte & 0xff00))
        printf("Failed to readback, count = %d, expecting = %x, got = %x\n",i,
               in_byte,out_byte);
#endif

      if(!(i&0xfff))
        {
          printf(".");
          fflush(stdout);
        }
    }

  printf("\n");

  sleep(3);


  /* check status of  done and init_n */
  /* if done is low, then Virtex thinks configuration isn't complete */
  /* if init_n is low then there was an error during configuration */

  /* Read the state of the Virtex DONE signal */
  done  = ((peek(VIRTEX_CFG_REG)) & 0x4);

  if (done == 0)
    printf("Programming failed - done = %x\n",done);
  else
    {
      printf("Programming Succeeded\n");
      printf("Driver module must be removed, and reinstalled into kernel\n");
    }

#ifdef WIN32
  CloseHandle(fdc);
#else
  close(fdc);
#endif

  free(BootProgram);

#if WIN32
  if (argc < 3)
    {
      printf("Hit<ctrl+c>to return\n");
      scanf("%s",deviceName);
    }
#endif

  return(0);
}

static void poke(unsigned int addr, unsigned int data)
{
  ARG_PEEKPOKE  pp;

  pp.page = 0;
  pp.mask = 0;
  pp.offset = addr;
  pp.value = data;

#ifdef WIN32
  bSuccess =
    DeviceIoControl(fdc, IOCTL_REGSET,
                    &pp, sizeof(pp),    // input buffer
                    &pp, sizeof(pp),
                    &bR, NULL);
  if (!bSuccess)
    {
      printf("VIRTEXLoad: Failed call to IOCTL_REGSET DeviceIoControl, error = %X\n",
             GetLastError());
    }
#else
  if (ioctl(fdc, FIOREGSET, (long)&pp) == -1)
    {
      printf("VIRTEXLoad: FIOREGSET failed\n");
    }
#endif

}

static unsigned int peek(unsigned int addr)
{
  ARG_PEEKPOKE  pp;

  pp.page = 0;
  pp.offset = addr;
  pp.mask = 0;

#ifdef WIN32
  bSuccess =
    DeviceIoControl(fdc, IOCTL_REGGET,
                    &pp, sizeof(pp),    // input buffer
                    &pp, sizeof(pp),
                    &bR, NULL);
  if (!bSuccess)
    {
      printf("VIRTEXLoad: Failed call to IOCTL_REGGET DeviceIoControl, error = %X\n",
             GetLastError());
    }
#else
  if (ioctl(fdc, FIOREGGET, (long)&pp) == -1)
    {
      printf("VIRTEXLoad: FIOREGGET failed\n");
    }
#endif

  return(pp.value);
}


/* Load an MCS file into memory */

static unsigned long *load_mcs_file(FILE *infile, unsigned int *retlength)
{
  char           string[80];
  int            done = 0;
  unsigned long  length;
  char           tempc;
  int            recordType;
  int            addrLSW;
  int            addrMSW;
  int            nxtAddrLSW;
  unsigned long  *nxtWriteAddr;
  unsigned long  recordAddr;
  char           firstRecord = 1;
  unsigned long  dataWord;
  unsigned int   numBytes;
  unsigned long  size;
  unsigned long   *dataBuf;

  /* Determine File Size */
  fseek(infile, 0L, SEEK_END);
  size = ftell(infile);

  /* Set file pointer to beginning of file */
  fseek(infile, 0L, SEEK_SET);

  dataBuf = (unsigned long *)malloc(size);
  if (dataBuf == NULL)
    {
      printf("Can't Allocate Temporary Data Buffer - Size = %lx\n",
             size);
      return(NULL);
    }

  nxtWriteAddr = dataBuf;
  while (done == 0)
    {
      /* Check for end of file */
      if (feof(infile))
        {
          printf("\nEOF1 was detected !");
          return(NULL);
        }

      /* See if this is a hex file */
      tempc = (char)fgetc(infile);
      if (tempc != ':')
        {
          printf("\nNot a HEX file !");
          return(NULL);
        }

      fscanf(infile, "%2x", &numBytes);
      fscanf(infile, "%4x", &addrLSW);
      fscanf(infile, "%2x", &recordType);
#if DEBUG
      printf("numBytes = %x\n", numBytes);
      printf("AddrLSW = %x\n", addrLSW);
      printf("recordType = %x\n", recordType);
#endif

      switch (recordType)
        {
        case 4:
          fscanf(infile, "%4x", &addrMSW);

          /* Read End of Line */
          fgets(string, 80, infile);
          length = 0;
          nxtAddrLSW = 0;
          recordAddr = ((unsigned long)addrMSW << 16) | addrLSW;
#if DEBUG
          printf("AddrMSW = %x\n", addrMSW);
          printf("recordAddr = %x\n", recordAddr);
          printf("nextWriteAddr = %x\n", nxtWriteAddr);
#endif
          break;
        case 0:
          if (nxtAddrLSW != addrLSW)
            {
              if (nxtAddrLSW == 0)
                recordAddr = ((unsigned long)addrMSW << 16) | addrLSW;
              else
                {
                  length = 0;
                  recordAddr = ((unsigned long)addrMSW << 16) | addrLSW;
                }
            }
          length += numBytes / 4;
          nxtAddrLSW = addrLSW + numBytes;
          while (numBytes)
            {
              fscanf(infile,"%8lx", &dataWord);
              *nxtWriteAddr++ = dataWord;
              numBytes -= 4;
            }
          firstRecord = 0;

          /* Read End of Line */
          fgets(string, 80, infile);

          break;
        case 1:
          done = 1;
          break;
        }  /* End Switch */
    }

  *retlength = nxtWriteAddr - dataBuf;

  return(dataBuf);
}
