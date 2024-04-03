#ifndef MAIN__H
#define MAIN__H

//-------------------------
// I2s defines
//-------------------------
#ifndef PIN_SCK
    #define PIN_SCK 0
#endif //PIN_SCK

#ifndef PIN_WS
    #define PIN_WS (PIN_SCK+1) // needs to be PIN_SCK +1
#endif //PIN_WS

#ifndef PIN_SD0
    #define PIN_SD0 (PIN_WS + 1) // Can be different
#endif //PIN_SD0

#ifndef PIN_SD1
    #define PIN_SD1 (PIN_SD0 + 1)  // needs to be PIN_SD0 +1
#endif //PIN_SD1

#ifndef PIN_SD2
    #define PIN_SD2 (PIN_SD1 + 1)  // needs to be PIN_SD1 +1
#endif //PIN_SD2

#ifndef PIN_SD3
    #define PIN_SD3 (PIN_SD2 + 1)  // needs to be PIN_SD2 +1
#endif //PIN_SD3

#ifndef BPS
    #define BPS 32 // 24 is not valid in this implementation, but INMP441 outputs 24 bits samples
#endif //BPS

#ifndef RATE
    #define RATE (16*1000)
#endif //RATE



//-------------------------

#endif //MAIN__H