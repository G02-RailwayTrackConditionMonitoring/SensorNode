#include "DMA_SPI.h"
#include "nrf52840.h"
#include "nrfx_spim.h"

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
    cfg.frequency      = NRF_SPIM_FREQ_8M;
    cfg.mode           = NRF_SPIM_MODE_0;
    cfg.bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    cfg.dcx_pin        =NRFX_SPIM_PIN_NOT_USED;
    cfg.rx_delay       = 0x02;
    cfg.use_hw_ss      =true;
    cfg.ss_duration    = 0x02;
  

  _dataMode = SPI_MODE0;
  _bitOrder = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  // non-blocking?
  nrfx_spim_init(&_spim, &cfg, handler, this);

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

void DMA_SPI::handler(nrfx_spim_evt_t const * p_event,void *p_context){
  digitalWrite(PIN_A1,HIGH);
  DMA_SPI* dma_spi_obj = (DMA_SPI*)p_context;
  dma_spi_obj->received = true;
  Serial.printf("SPI handler event: %x\n",p_event->type);
  Serial.printf("tx amount: %d, rx amount: %d\n",p_event->xfer_desc.tx_length,p_event->xfer_desc.rx_length);
  Serial.printf("rx buff addr: %x\n",p_event->xfer_desc.p_rx_buffer);
  digitalWrite(PIN_A1,LOW);
}

void DMA_SPI::transfer(const void *tx_buf, void *rx_buf, size_t count){

  received = false;
  SPIClass::transfer(tx_buf,rx_buf,count);
}

void DMA_SPI::transferBlocking(const void * tx_buff, uint16_t tx_count, const void * rx_buff,uint16_t rx_count){

   nrfx_spim_xfer_desc_t xfer_desc =
    {
      .p_tx_buffer = (uint8_t*)tx_buff,
      .tx_length   = tx_count,

      .p_rx_buffer = (uint8_t*)rx_buff,
      .rx_length   = rx_count,
    };

    nrfx_spim_xfer(&_spim, &xfer_desc, 0);

}

bool DMA_SPI::rxDone(){

  return received;
}