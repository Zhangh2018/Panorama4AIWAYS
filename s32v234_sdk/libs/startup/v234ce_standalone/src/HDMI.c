#include "config.h"
#include "i2c.h"
#include "HDMI.h"

#ifdef OLD_EVB
  #define HDMI_I2C I2C2
#else  
  #define HDMI_I2C I2C1
#endif // ifdef OLD_EVB

#define HDMI_ADDR_LOCAL 0x39

void HDMI_configuration(void) {

  // --------------------------------------------------------------------
  // 1280*720@60Hz, HDMI Output, RGB 4:4:4 24-Bit Input
  // 20131211 Silicon Image
  // ----------------------------------------------------------------------

  // Initial Settings
  // ----------------------------------------------------------------------
  uint8_t lpData[2];
  uint8_t lByteCnt = 2;
  lpData[0] = 0xc7; lpData[1] = 0x00;  // Program HW TPI mode
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0xc7,0x00);  // Program HW TPI mode
  lpData[0] = 0x1e; lpData[1] = 0x00;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x1e,0x00);// Power up Chip

#ifdef HDMI_RES_1280X720  
  // Program 1280x720@60Hz Resolution, 74.25MHz
  // ----------------------------------------------------------------------
  lpData[0] = 0x00; lpData[1] = 0x01;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x00, 0x01);	// 74.25MHz input clock, Low byte
  lpData[0] = 0x01; lpData[1] = 0x1D;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x01, 0x1D);	// 74.25MHz input clock, High byte
  lpData[0] = 0x02; lpData[1] = 0x70;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x02, 0x70);	// 60Hz vertical timing, Low byte
  lpData[0] = 0x03; lpData[1] = 0x17;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x03, 0x17);	// 60Hz vertical timing, High byte
  lpData[0] = 0x04; lpData[1] = 0x72;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x04, 0x72);	// Total 1650 pixel per line, Low byte
  lpData[0] = 0x05; lpData[1] = 0x06;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x05, 0x06);	// Total 1650 pixels per line, High byte
  lpData[0] = 0x06; lpData[1] = 0xEE;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x06, 0xEE);	// Total 750 lines per frame, Low byte
  lpData[0] = 0x07; lpData[1] = 0x02;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x07, 0x02);	// Total 750 lines per frame , High byte
#else
#ifdef HDMI_RES_1920X1080    // #ifdef HDMI_RES_1280x720 
  // Program 1920x1080@60Hz Resolution, 148.50MHz
  // ----------------------------------------------------------------------
  lpData[0] = 0x00; lpData[1] = 0x02;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x00, 0x02);  // 148.50MHz input clock, Low byte
  lpData[0] = 0x01; lpData[1] = 0x3A;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x01, 0x3A);  // 148.50MHz input clock, High byte
  lpData[0] = 0x02; lpData[1] = 0x70;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x02, 0x70);  // 60Hz vertical timing, Low byte
  lpData[0] = 0x03; lpData[1] = 0x17;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x03, 0x17);  // 60Hz vertical timing, High byte
  lpData[0] = 0x04; lpData[1] = 0x98;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x04, 0x98);  // Total 2200 pixel per line, Low byte
  lpData[0] = 0x05; lpData[1] = 0x08;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x05, 0x08);  // Total 2200 pixel per line, Low byte
  lpData[0] = 0x06; lpData[1] = 0x65;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x06, 0x65);  // Total 1125 lines per frame, Low byte
  lpData[0] = 0x07; lpData[1] = 0x04;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x07, 0x04);  // Total 1125 lines per frame, Low byte
#else // #ifdef HDMI_RES_1920X1080
#ifdef HDMI_RES_1280X1024    
  // Program 1280X1024@60Hz VESA Resolution, 108.000MHz
  // ----------------------------------------------------------------------
  lpData[0] = 0x00; lpData[1] = 0x94;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x00, 0x94);  // 109.00MHz input clock, Low byte	0x2A94 => 10900
  lpData[0] = 0x01; lpData[1] = 0x2A;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x01, 0x2A);  // 109.00MHz input clock, High byte
  lpData[0] = 0x02; lpData[1] = 0x70;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x02, 0x70);  // 60Hz vertical timing, Low byte		0x1770 => 6000
  lpData[0] = 0x03; lpData[1] = 0x17;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x03, 0x17);  // 60Hz vertical timing, High byte
  lpData[0] = 0x04; lpData[1] = 0xB0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x04, 0xB0);  // Total 1712 pixel per line, Low byte
  lpData[0] = 0x05; lpData[1] = 0x06;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x05, 0x06);  // Total 1712 pixel per line, High byte
  lpData[0] = 0x06; lpData[1] = 0x27;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);   
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x06, 0x27);  // Total 1063 lines per frame, Low byte
  lpData[0] = 0x07; lpData[1] = 0x04;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);    
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x07, 0x04);  // Total 1063 lines per frame, High byte*/
#else
  // Program 1920x1080@60Hz Resolution, 148.50MHz
  // ----------------------------------------------------------------------
  lpData[0] = 0x00; lpData[1] = 0x02;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x00, 0x02);  // 148.50MHz input clock, Low byte
  lpData[0] = 0x01; lpData[1] = 0x3A;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x01, 0x3A);  // 148.50MHz input clock, High byte
  lpData[0] = 0x02; lpData[1] = 0x70;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt); 
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x02, 0x70);  // 60Hz vertical timing, Low byte
  lpData[0] = 0x03; lpData[1] = 0x17;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);   
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x03, 0x17);  // 60Hz vertical timing, High byte
  lpData[0] = 0x04; lpData[1] = 0x98;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);   
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x04, 0x98);  // Total 2200 pixel per line, Low byte
  lpData[0] = 0x05; lpData[1] = 0x08;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);   
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x05, 0x08);  // Total 2200 pixel per line, Low byte
  lpData[0] = 0x06; lpData[1] = 0x65;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);     
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x06, 0x65);  // Total 1125 lines per frame, Low byte
  lpData[0] = 0x07; lpData[1] = 0x04;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);    
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x07, 0x04);  // Total 1125 lines per frame, Low byte*/
#endif // else from #ifdef HDMI_RES_1280X1024
#endif // else from #ifdef HDMI_RES_1920X1080
#endif // else from #ifdef HDMI_RES_1280X720  

  // Setup input/output mode
  // -----------------------------------------------------------------------
  lpData[0] = 0x08; lpData[1] = 0x60;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x08, 0x60);	// 24-bit input, TMDS Clock = Input Clock
  lpData[0] = 0x09; lpData[1] = 0x00;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x09, 0x00);	// Input is RGB
  lpData[0] = 0x0a; lpData[1] = 0x00;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x0a, 0x00);	// Output is RGB

  // Setup AVI Infoframe & update input/output mode registers in HW
  // -----------------------------------------------------------------------
  lpData[0] = 0x0C; lpData[1] = 0x43;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x0C,0x43);// AVI_DBYTE0 = Checksum = 0x43
  lpData[0] = 0x0D; lpData[1] = 0x00;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x0D,0x00); // AVI_DBYTE1 = 0x00 -- Y=00(RGB), A0=0(act vid indicator OFF), B=00(no bar data), S=00(no scan data)
  lpData[0] = 0x0E; lpData[1] = 0x28;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x0E,0x28); // AVI_DBYTE2 = 0x28 -- C=00(nodata), M=10(16:9), R=1000(same as aspect ratio)
  lpData[0] = 0x0F; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x0F,0x0); // AVI_DBYTE3 = 0x00 -- ITC=0(nodata), EC=000(xvYCC601), Q=00(default), SC=00(no known scaling)
  lpData[0] = 0x10; lpData[1] = 0x4;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x10,0x4); // AVI_DBYTE4 = 0x04 -- VIC = Code 4, 128HDMI_I2C0p60Hz
  lpData[0] = 0x11; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);    
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x11,0x0); // AVI_DBYTE5 = 0x00 -- PR=0000(no pixel repetition)
  lpData[0] = 0x12; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x12,0x0); // AVI_DBYTE6 = 0x00
  lpData[0] = 0x13; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);  
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x13,0x0); // AVI_DBYTE7 = 0x00
  lpData[0] = 0x14; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x14,0x0); // AVI_DBYTE8 = 0x00
  lpData[0] = 0x15; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x15,0x0); // AVI_DBYTE9 = 0x00
  lpData[0] = 0x16; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x16,0x0); // AVI_DBYTE10 = 0x00
  lpData[0] = 0x17; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x17,0x0); // AVI_DBYTE11 = 0x00
  lpData[0] = 0x18; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x18,0x0); // AVI_DBYTE12 = 0x00
  lpData[0] = 0x19; lpData[1] = 0x0;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL,0x19,0x0); // AVI_DBYTE13 = 0x00

  // -------------- AUDIO DISABLED -----------------------------------
  lpData[0] = 0x26; lpData[1] = 0x00;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x26, 0x00);	// Audio disabled
  lpData[0] = 0xBF; lpData[1] = 0x02;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0xBF, 0x02);	// Disable Sending of Audio-Infoframe

  // Setup output mode & Power-on
  // -----------------------------------------------------------------------
  lpData[0] = 0x1A; lpData[1] = 0x01;
  I2CX_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, lpData, lByteCnt);
  //I2C8_DevTx(HDMI_I2C, HDMI_ADDR_LOCAL, 0x1A, 0x01); 	// Output Mode = HDMI, TMDS On
  // -----------------------------------------------------------------------
  // -----------------------------------------------------------------------
}
