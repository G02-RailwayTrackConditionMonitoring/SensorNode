
#ifndef DMA_SPI_H_
#define DMA_SPI_H_

#include <Arduino.h>
#include "SPI.h"
#include "nrfx_timer.h"

class DMA_SPI: public SPIClass{

public:
    DMA_SPI(NRF_SPIM_Type *p_spi, uint8_t uc_pinMISO, uint8_t uc_pinSCK, uint8_t uc_pinMOSI, uint8_t uc_pinSS):SPIClass(p_spi, uc_pinMISO,  uc_pinSCK,  uc_pinMOSI){
        _uc_pinSS = g_ADigitalPinMap[uc_pinSS];
        received = false;
    }

    //Initialization function. Call before using the other functions.
    void begin();

    //This will retuen immediatley. Check rxDone to know if transfer is completed.
    void transfer(const void *tx_buf, void *rx_buf, size_t count);

    //Blocking transfer. Internally it waits for rxDone to return true.
    void transferBlocking(const void * tx_buff, uint16_t tx_count, const void * rx_buff,uint16_t rx_count);
    
    //Check if a transfer is done.
    bool rxDone();
    
    //This starts a recurring transfer.
    //This will transfer "n" blocks of "m" bytes every "delay" milliseconds.
    void startReccuringTransfer(uint16_t n, uint8_t m, uint8_t delay);

    //Change the SPI clock frequency.
    void setClock(uint8_t freq_MHz);
    

private:
    uint8_t _uc_pinSS;
    volatile bool received;
    nrfx_timer_t* tim0;

    void setup_recurring_timer(uint8_t delay_ms);

    //Handler called from the "event done" interrupt. Basiccaly called when a transfer is complete.
    static void spi_handler(nrfx_spim_evt_t const * p_event,void *p_context);

    //Handler for timer 0.
    static void tim0_handler(nrf_timer_event_t event_type, void* p_context);
};


#endif