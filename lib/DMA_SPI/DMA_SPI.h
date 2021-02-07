
#ifndef DMA_SPI_H_
#define DMA_SPI_H_

#include <Arduino.h>
#include "SPI.h"

class DMA_SPI: public SPIClass{

public:
    DMA_SPI(NRF_SPIM_Type *p_spi, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, uint8_t uc_pinSS):SPIClass(p_spi, uc_pinMISO,  uc_pinSCK,  uc_pinMOSI){
        _uc_pinSS = g_ADigitalPinMap[uc_pinSS];
        received = false;
    }


    void begin();
    void transfer(const void *tx_buf, void *rx_buf, size_t count);
    bool rxDone();
    static void handler(nrfx_spim_evt_t const * p_event,void *p_context);

private:
    uint8_t _uc_pinSS;
    volatile bool received;
};


#endif