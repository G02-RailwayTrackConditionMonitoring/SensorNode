#include "DMA_SPI.h"
#include "nrf52840.h"
#include <nrfx.h>
#include "nrfx_spim.h"
#include "nrfx_timer.h"
#include "hal/nrf_timer.h"

void DMA_SPI::begin(){

if (initialized) return;
  initialized = true;

  nrfx_spim_config_t cfg;

    cfg.sck_pin        = _uc_pinSCK;
    cfg.mosi_pin       = _uc_pinMosi;
    cfg.miso_pin       = _uc_pinMiso;
    cfg.ss_pin         = _uc_pinSS;
    cfg.ss_active_high = false;
    cfg.irq_priority   = 3;
    cfg.orc            = 0xFF;
    // default setting 4 Mhz, Mode 0, MSB first
    cfg.frequency      = NRF_SPIM_FREQ_1M;
    cfg.mode           = NRF_SPIM_MODE_0;
    cfg.bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    cfg.dcx_pin        =NRFX_SPIM_PIN_NOT_USED;
    cfg.rx_delay       = 0x02;
    cfg.use_hw_ss      =true;
    cfg.ss_duration    = 0x02;
  

  _dataMode = SPI_MODE0;
  _bitOrder = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  // non-blocking?
  nrfx_spim_init(&_spim, &cfg, spi_handler, this);

  // highspeed SPIM should set SCK and MOSI to high drive
  nrf_gpio_cfg(_uc_pinSCK,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_CONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1,
               NRF_GPIO_PIN_NOSENSE);

  nrf_gpio_cfg(_uc_pinMosi,
               NRF_GPIO_PIN_DIR_OUTPUT,
               NRF_GPIO_PIN_INPUT_DISCONNECT,
               NRF_GPIO_PIN_NOPULL,
               NRF_GPIO_PIN_H0H1,
               NRF_GPIO_PIN_NOSENSE);
}

void DMA_SPI::spi_handler(nrfx_spim_evt_t const * p_event,void *p_context){
  digitalWrite(PIN_A1,HIGH);
  DMA_SPI* dma_spi_obj = (DMA_SPI*)p_context;
  dma_spi_obj->received = true;
  // Serial.printf("SPI handler event: %x\n",p_event->type);
  // Serial.printf("tx amount: %d, rx amount: %d\n",p_event->xfer_desc.tx_length,p_event->xfer_desc.rx_length);
  // Serial.printf("rx buff addr: %x\n",p_event->xfer_desc.p_rx_buffer);
  digitalWrite(PIN_A1,LOW);
}

void DMA_SPI::transfer(const void *tx_buf, void *rx_buf, size_t count){

  received = false;
  SPIClass::transfer(tx_buf,rx_buf,count);
}

void DMA_SPI::transferBlocking(const void * tx_buff, uint16_t tx_count, const void * rx_buff,uint16_t rx_count){

   received = false;
   nrfx_spim_xfer_desc_t xfer_desc =
    {
      .p_tx_buffer = (uint8_t*)tx_buff,
      .tx_length   = tx_count,

      .p_rx_buffer = (uint8_t*)rx_buff,
      .rx_length   = rx_count,
    };

    nrfx_spim_xfer(&_spim, &xfer_desc, 0);

    while(!rxDone()){} // Block while waiting for transfer to complete.
}

bool DMA_SPI::rxDone(){

  return received;
}

void DMA_SPI::setClock(uint8_t freq_MHz){

  nrf_spim_frequency_t clockFreq;
  switch(freq_MHz){

    case 1: clockFreq = NRF_SPIM_FREQ_1M;
            break;
    case 8: clockFreq = NRF_SPIM_FREQ_8M;
            break;
    default:clockFreq = NRF_SPIM_FREQ_1M;
            break;
  }
  nrf_spim_frequency_set(_spim.p_reg, clockFreq);
}

void DMA_SPI::setup_recurring_timer(uint8_t delay_ms){

  //Allocate memory for the timer instance.
  tim0 = (nrfx_timer_t*)malloc(sizeof(nrfx_timer_t));

  //Boilerplate stuff, just assigning the TIMER0 struct.
  tim0->cc_channel_count = TIMER0_CC_NUM;
  tim0->instance_id = NRFX_TIMER0_INST_IDX;
  tim0->p_reg = NRF_TIMER0;

  //Configure the timer , 32bit, 8Mhz clock, timer mode.
  nrfx_timer_config_t cfg;
  cfg.frequency = NRF_TIMER_FREQ_8MHz;
  cfg.mode = NRF_TIMER_MODE_TIMER;
  cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  cfg.p_context = this;

  //First we initialize a timer which will initiate the bulk transfers.
  nrfx_err_t err = nrfx_timer_init(tim0,&cfg,tim0_handler);
  Serial.printf("Timer setup error: %x\n",err);
  Serial.flush();

  //Setup the compare channel. This will cause an event when the timer gets to the specified value.
  //Right now we are using channel 0 (arbritrary), short circuit the compare to the clear so we automatically restart the timer, interrupt disabled.
  nrfx_timer_extended_compare(tim0, NRF_TIMER_CC_CHANNEL0,8000*delay_ms,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,false);

}

void DMA_SPI::startReccuringTransfer(uint16_t n, uint8_t m, uint8_t delay){

  //Setup the timer that controls how often the block transfers happen.
  setup_recurring_timer(delay);

  //Start the timer.
  nrfx_timer_enable(tim0);

}

void DMA_SPI::tim0_handler(nrf_timer_event_t event_type, void* p_context){
  //Not really needed for the application but useful for debugging. 
  //Make sure to enable interrupt in "nrfx_timer_extended_compare" of "setup_recurring_timer".
}