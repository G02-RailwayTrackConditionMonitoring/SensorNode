
#ifndef DMA_SPI_H_
#define DMA_SPI_H_

#include <Arduino.h>
#include "SPI.h"
#include "nrfx_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"

#define SPI_NUM_BLOCKS  80 //Fifo holds 85 samples, and the fifo size takes 1 more block.
#define SPI_BYTES_PER_BLOCK 7   //Each "block" transfers one sample, which is 6 bytes, plus the command.
#define SPI_BLOCK_DELAY_MS  15  //Fifo takes 21ms to fill, so 20ms is conservative.
#define SPI_NUM_FIFO        10   //How many FIFOs we can read before we need CPU intervention.

typedef struct {

    uint8_t buffer[SPI_BYTES_PER_BLOCK];
} ArrayList_t;

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
    
    //This initializes a recurring transfer.
    //This will transfer SPI_NUM_BLOCKS blocks of SPI_BYTES_PER_BLOCK bytes, every SPI_BLOCK_DELAY_MS milliseconds.
    void setupReccuringTransfer();

    //Starts the transfers
    void startRecuringTransfers();

    void pauseRecurringTransfers();

    
    //Change the SPI clock frequency.
    void setClock(uint8_t freq_MHz);
    
    //Returns how many times we have read fifo. 1-based index !
    uint16_t getRxBuffIndex();

    //Returns the number of microseconds until the next transfer.
    uint32_t getTimeUntilTransfer();

    //Copies the rx buffer to a user buffer, starting at "index", and copying "numCopy" fifos worth of data.
    //Also resets the spi dma buffers, so we can again have a delay without losing samples.
    void getRxData(uint8_t* buff, uint8_t index,uint8_t numCopy);

private:
    uint8_t _uc_pinSS;
    volatile bool received;
    uint8_t rx_buffer_index=0;

    //Parameters for the recurring block transfers.
    uint16_t numBlocks = SPI_NUM_BLOCKS;
    uint8_t  bytesPerBlock = SPI_BYTES_PER_BLOCK;
    uint8_t  blockDelay_ms = SPI_BLOCK_DELAY_MS;

    ArrayList_t rx_buffer[SPI_NUM_BLOCKS*SPI_NUM_FIFO+(SPI_NUM_FIFO-1)*1]; 
    ArrayList_t tx_buffer[SPI_NUM_BLOCKS*SPI_NUM_FIFO+(SPI_NUM_FIFO-1)*1];

    nrfx_timer_t tim0; //Used as a timer to initiate recurring block transfer.NOT USED ANYMORE.
    nrfx_timer_t tim3; //Used to count the "sample ready" interrupts.
    nrfx_timer_t tim1; //Used as a couter for counting number of bytes transfered in each recurring block transfer.
    nrfx_timer_t tim2; //Used to count how many fifo reads we have done, so we know if there is data to transfer and reset buffers.

    //PPI Channels for configuring control of the SPI peripheral without CPU intervention.
    nrf_ppi_channel_t timerToSpi_PPI_CHAN;
    nrf_ppi_channel_t SpiToCounter_PPI_CHAN;
    nrf_ppi_channel_t CounterToSpi_PPI_CHAN;
    nrf_ppi_channel_t spiToSpi_PPI_CHAN;
    nrf_ppi_channel_t gpioteToCounter_PPI_CHAN;
    nrf_ppi_channel_group_t ppiGroup;

    void setup_recurring_timer(uint8_t delay_ms);
    void setup_recurring_counter(uint8_t num_transfer);
    void setup_sample_counter(uint8_t numSamples);
    void setup_tracking_counter();
    void setup_pinChange_event();

    //Enable or disable PPI channels. true-> enable, false->disable.
    void enablePPI(bool enable);

    //Handler called from the "event done" interrupt. Basically called when a transfer is complete.
    static void spi_handler(nrfx_spim_evt_t const * p_event,void *p_context);

    //Handler for timer 0.
    static void tim0_handler(nrf_timer_event_t event_type, void* p_context);

    static void tim2_handler(nrf_timer_event_t event_type, void* p_context);

    //Handler for timer 1.
    static void tim1_handler(nrf_timer_event_t event_type, void* p_context);

    //Handler for timer 1.
    static void tim3_handler(nrf_timer_event_t event_type, void* p_context);

    //Handler for gpiote
    static void gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

};


#endif