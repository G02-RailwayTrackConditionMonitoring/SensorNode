#include "DMA_SPI.h"
#include "nrf52840.h"
#include <nrfx.h>
#include "nrfx_spim.h"
#include "nrfx_timer.h"
#include "hal/nrf_timer.h"
#include "nrfx_ppi.h"
#include "nrfx_gpiote.h"

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
    cfg.orc            = 0x00;
    // default setting 4 Mhz, Mode 0, MSB first
    cfg.frequency      = NRF_SPIM_FREQ_8M;
    cfg.mode           = NRF_SPIM_MODE_3;
    cfg.bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST;
    cfg.dcx_pin        =NRFX_SPIM_PIN_NOT_USED;
    cfg.rx_delay       = 0x02;
    cfg.use_hw_ss      =true;
    cfg.ss_duration    = 0x04;
  

  _dataMode = SPI_MODE3;
  _bitOrder = NRF_SPIM_BIT_ORDER_MSB_FIRST;

  // non-blocking?
  nrfx_spim_init(&_spim, &cfg, NULL, this);

  _spim.p_reg->EVENTS_END = 0x01; // Enable the END event generation.
  
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

  nrf_spim_enable(_spim.p_reg);


}

void DMA_SPI::spi_handler(nrfx_spim_evt_t const * p_event,void *p_context){

  //digitalWrite(PIN_A1,HIGH);
  DMA_SPI* dma_spi_obj = (DMA_SPI*)p_context;
  dma_spi_obj->received = true;
  // Serial.printf("SPI handler event: %x\n",p_event->type);
  // Serial.printf("tx amount: %d, rx amount: %d\n",p_event->xfer_desc.tx_length,p_event->xfer_desc.rx_length);
  // Serial.printf("rx buff addr: %x\n",p_event->xfer_desc.p_rx_buffer);
  //digitalWrite(PIN_A1,LOW);

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

    //while(!rxDone()){} // Block while waiting for transfer to complete.
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
    case 16: clockFreq = NRF_SPIM_FREQ_16M;
            break;
    default:clockFreq = NRF_SPIM_FREQ_1M;
            break;
  }
  nrf_spim_frequency_set(_spim.p_reg, clockFreq);
}

void DMA_SPI::setup_recurring_timer(uint8_t delay_ms){

  //Boilerplate stuff, just assigning the TIMER0 struct.
  tim0.cc_channel_count = TIMER3_CC_NUM;
  tim0.instance_id = NRFX_TIMER3_INST_IDX;
  tim0.p_reg = NRF_TIMER3;

  //Configure the timer , 32bit, 8Mhz clock, timer mode.
  nrfx_timer_config_t cfg;
  cfg.frequency = NRF_TIMER_FREQ_8MHz;
  cfg.mode = NRF_TIMER_MODE_TIMER;
  cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  cfg.p_context = this;

  //First we initialize a timer which will initiate the bulk transfers.
  nrfx_err_t err = nrfx_timer_init(&tim0,&cfg,tim0_handler);
  Serial.printf("Timer setup error: %x\n\r",err);
  Serial.flush();

  //Setup the compare channel. This will cause an event when the timer gets to the specified value.
  //Right now we are using channel 0 (arbritrary), short circuit the compare to the clear so we automatically restart the timer, interrupt disabled.
  nrfx_timer_extended_compare(&tim0, NRF_TIMER_CC_CHANNEL0,8000*delay_ms,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,false);
  tim0.p_reg->EVENTS_COMPARE[0] = 0x1; //Should enable event generation for CC channel 0.
  
}

 void DMA_SPI::setup_recurring_counter(uint8_t num_transfer){

  //Boilerplate stuff, just assigning the TIMER0 struct.
  tim1.cc_channel_count = TIMER1_CC_NUM;
  tim1.instance_id = NRFX_TIMER1_INST_IDX;
  tim1.p_reg = NRF_TIMER1;
  
  //Configure the timer , 16bit, 16Mhz clock, counter.
  nrfx_timer_config_t cfg;
  cfg.frequency = NRF_TIMER_FREQ_16MHz;
  cfg.mode = NRF_TIMER_MODE_COUNTER;
  cfg.bit_width = NRF_TIMER_BIT_WIDTH_16; //Apparently timer 1 and 2 don't support 32 bit mode!
  cfg.p_context = this;

  //First we initialize a timer which will initiate the bulk transfers.
  nrfx_err_t err = nrfx_timer_init(&tim1,&cfg,tim1_handler);
  Serial.printf("Counter setup error: %x\n\r",err);
  Serial.flush();

   //Setup the compare channel. This will cause an event when the counter gets to the specified value.
  //Right now we are using channel 0 (arbritrary), short circuit the compare to the clear so we automatically restart the counter, interrupt disabled.
  //Add one since it seems to do one less transfer, except on the first time, so we increment manually by 1 to account for the first time.
  nrfx_timer_extended_compare(&tim1, NRF_TIMER_CC_CHANNEL0,num_transfer+1,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,false);
  

  tim1.p_reg->EVENTS_COMPARE[0] = 0x1; //Should enable event generation for CC channel 0.

}

void DMA_SPI::setup_sample_counter(uint8_t numSamples){

  //Boilerplate stuff, just assigning the TIMER0 struct.
  tim3.cc_channel_count = TIMER3_CC_NUM;
  tim3.instance_id = NRFX_TIMER3_INST_IDX;
  tim3.p_reg = NRF_TIMER3;
  
  //Configure the timer , 16bit, 16Mhz clock, counter.
  nrfx_timer_config_t cfg;
  cfg.frequency = NRF_TIMER_FREQ_16MHz;
  cfg.mode = NRF_TIMER_MODE_COUNTER;
  cfg.bit_width = NRF_TIMER_BIT_WIDTH_16; //Apparently timer 1 and 2 don't support 32 bit mode!
  cfg.p_context = this;

  //First we initialize a timer which will initiate the bulk transfers.
  nrfx_err_t err = nrfx_timer_init(&tim3,&cfg,tim3_handler);
  Serial.printf("Sample Counter setup error: %x\n\r",err);
  Serial.flush();

   //Setup the compare channel. This will cause an event when the counter gets to the specified value.
  //Right now we are using channel 0 (arbritrary), short circuit the compare to the clear so we automatically restart the counter, interrupt disabled.
  //Add one since it seems to do one less transfer, except on the first time, so we increment manually by 1 to account for the first time.
  nrfx_timer_extended_compare(&tim3, NRF_TIMER_CC_CHANNEL0,numSamples,NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,false);
  

  tim3.p_reg->EVENTS_COMPARE[0] = 0x1; //Should enable event generation for CC channel 0.

}

void DMA_SPI::setup_tracking_counter(){

   //Boilerplate stuff, just assigning the TIMER0 struct.
  tim2.cc_channel_count = TIMER2_CC_NUM;
  tim2.instance_id = NRFX_TIMER2_INST_IDX;
  tim2.p_reg = NRF_TIMER2;

  //Configure the timer , 16bit, 16Mhz clock, counter.
  nrfx_timer_config_t cfg;
  cfg.frequency = NRF_TIMER_FREQ_16MHz;
  cfg.mode = NRF_TIMER_MODE_COUNTER;
  cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
  cfg.p_context = this;

  nrfx_err_t err = nrfx_timer_init(&tim2,&cfg,tim2_handler);
  Serial.printf("Counter setup error: %x\n\r",err);
  Serial.flush();

  //This is a failsafe for if we overflow the buffer. The handler clears the buffer and resets the spi buffers.
  ///This will cause us to lose the data, but it prevents the fifo from going out of sync, which would probably need a reset to deal with.
  nrfx_timer_extended_compare(&tim2, NRF_TIMER_CC_CHANNEL1,(SPI_NUM_BLOCKS)*(SPI_NUM_FIFO-1)+(SPI_NUM_FIFO-1),NRF_TIMER_SHORT_COMPARE1_STOP_MASK,true);

}

void DMA_SPI::setup_pinChange_event(){

  // //Setup a pin config for a rising edge detection, high-acc = true
  nrfx_gpiote_in_config_t pinConfig = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  nrfx_err_t res;
  if(!nrfx_gpiote_is_init()){
    //If not already configured, initialize module.
    res = nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    Serial.printf("gpiote init err: %x\n\r",res);
  }                                

  //pin 11 corresponds to D12
  res = nrfx_gpiote_in_init(11,&pinConfig,gpiote_handler);
  Serial.printf("gpiote pin config: %x\n\r",res);
  nrfx_gpiote_in_event_enable(11, false);

}

void DMA_SPI::setupReccuringTransfer(){

  // digitalToggle(PIN_A0);
  Serial.println("Setting up block transfers...");
  Serial.flush();
  //Setup the timer that controls how often the block transfers happen.
  //setup_recurring_timer(blockDelay_ms);
  setup_pinChange_event();

  //setup up the counter to track how many samples are in the fifo.
  setup_sample_counter(numBlocks);

  //Setup the counter for controlling the number of transfers per period.
  setup_recurring_counter(numBlocks);

  //Setup the counter for counting how many blocks we have sent.
  setup_tracking_counter();


  //Setup the Programmable Peripheral Interconnect (PPI).
  //We want the timer CC event to trigger the START of the SPIM module.
  //Then the END event of the SPIM should activate the COUNT of the counter.
  //Lastly, the counter CC should activate the SPIM STOP task.

  //Allocate the PPI channels.
  nrfx_err_t err = nrfx_ppi_channel_alloc(&timerToSpi_PPI_CHAN);
  //  Serial.printf("PPI channe err: %x\n",err);
  err = nrfx_ppi_channel_alloc(&gpioteToCounter_PPI_CHAN);
  // Serial.printf("PPI channe err: %x\n\r",err);
  err = nrfx_ppi_channel_alloc(&SpiToCounter_PPI_CHAN);
  // Serial.printf("PPI channe err: %x\n",err);
  err = nrfx_ppi_channel_alloc(&CounterToSpi_PPI_CHAN);
  // Serial.printf("PPI channe err: %x\n",err);
  err = nrfx_ppi_channel_alloc(&spiToSpi_PPI_CHAN);
  // Serial.printf("spi->spi PPI channel err: %x\n",err);
  err = nrfx_ppi_group_alloc(&ppiGroup);
  // Serial.printf("group alloc err: %x\n",err);

  //Assign the ppi channel routing.

  err = nrfx_ppi_channel_assign(gpioteToCounter_PPI_CHAN,nrfx_gpiote_in_event_addr_get(11),nrf_timer_task_address_get(tim3.p_reg,NRF_TIMER_TASK_COUNT));
  //  Serial.printf("PPI assign gpiote to counter: %x\n\r",err);
  //err = nrfx_ppi_channel_assign(timerToSpi_PPI_CHAN,nrf_timer_event_address_get(tim0.p_reg,NRF_TIMER_EVENT_COMPARE0),nrf_spim_task_address_get(_spim.p_reg,NRF_SPIM_TASK_START));
  err = nrfx_ppi_channel_assign(timerToSpi_PPI_CHAN,nrf_timer_event_address_get(tim3.p_reg,NRF_TIMER_EVENT_COMPARE0),nrf_spim_task_address_get(_spim.p_reg,NRF_SPIM_TASK_START));
  // Serial.printf("PPI assign counter to spi: %x\n\r",err);
    //err = nrfx_ppi_channel_assign(timerToCounter_PPI_CHAN,nrf_timer_event_address_get(tim1.p_reg,NRF_TIMER_EVENT_COMPARE0),nrf_timer_task_address_get(tim2.p_reg,NRF_TIMER_TASK_COUNT));
  // Serial.printf("PPI assign timer to counter: %x\n\r",err);
  err = nrfx_ppi_channel_assign(SpiToCounter_PPI_CHAN,nrf_spim_event_address_get(_spim.p_reg,NRF_SPIM_EVENT_END),nrf_timer_task_address_get(tim1.p_reg,NRF_TIMER_TASK_COUNT));
  // Serial.printf("PPI assign spi to counter: %x\n\r",err);
  err = nrfx_ppi_channel_assign(CounterToSpi_PPI_CHAN,nrf_timer_event_address_get(tim1.p_reg,NRF_TIMER_EVENT_COMPARE0),nrf_spim_task_address_get(_spim.p_reg,NRF_SPIM_TASK_STOP));
  // Serial.printf("PPI assign counter to spi: %x\n\r",err);
  err = nrfx_ppi_channel_assign(spiToSpi_PPI_CHAN,nrf_spim_event_address_get(_spim.p_reg,NRF_SPIM_EVENT_END),nrf_spim_task_address_get(_spim.p_reg,NRF_SPIM_TASK_START));
  // Serial.printf("PPI assign spi to spi: %x\n\r",err);

  //Assign the spi->spi to a group so we can enable/disable it.
  err = nrfx_ppi_channel_include_in_group(spiToSpi_PPI_CHAN,ppiGroup);
  // Serial.printf("PPI assign chan to group err: %x\n\r",err);

  //Set forks for starting/stopping the spi channel.
  err = nrfx_ppi_channel_fork_assign(timerToSpi_PPI_CHAN,nrfx_ppi_task_addr_group_enable_get(ppiGroup));
  // Serial.printf("PPI assign fork to enable group: %x\n\r",err);
  err = nrfx_ppi_channel_fork_assign(CounterToSpi_PPI_CHAN,nrfx_ppi_task_addr_group_disable_get(ppiGroup));
  // Serial.printf("PPI assign fork to disable group: %x\n\r",err);
  err = nrfx_ppi_channel_fork_assign(SpiToCounter_PPI_CHAN,nrf_timer_task_address_get(tim2.p_reg,NRF_TIMER_TASK_COUNT));
  // Serial.printf("PPI assign fork to trackign counter: %x\n\r",err);

 


  // Have the end of one transmission start the next.
  //Apparently the STOP is ignored if shorts are enabled :(
  //nrf_spim_shorts_enable(_spim.p_reg, NRF_SPIM_SHORT_END_START_MASK);



  //Setup the tx buffer. Puts the "read fifo" command as the first byte of each transaction.
  for(int i=0; i<SPI_NUM_BLOCKS;i++){

      tx_buffer[i].buffer[0] = 0x74|0x80; //Read the fifo data.
  }

  //Copy for the remaining fifo buffers worth of tx buffer.
  //Add one to the location because it skips one. Otherwise we lose the first byte (i.e. the command).
  for(int i=0; i<SPI_NUM_FIFO-1;i++){

    memcpy(&tx_buffer[(SPI_NUM_BLOCKS*(i+1))+(i+1)],&tx_buffer,SPI_NUM_BLOCKS*SPI_BYTES_PER_BLOCK);
  }

  // //For debugging purpose, print the tx buff. Comment out later!
  // for(int i=0; i < SPI_NUM_BLOCKS*SPI_NUM_FIFO;i++){
  //   Serial.printf("%d: ",i);
  //   for(int j=0; j< SPI_BYTES_PER_BLOCK; j++){

  //     Serial.printf("%x, ",tx_buffer[i].buffer[j]);
  //   }
  //   Serial.printf("\n\r");
  //   Serial.flush();
  // }

}

void DMA_SPI::startRecuringTransfers(){

  // Serial.println("Starting recurring transfers...");
  // Serial.flush();
  enablePPI(true);
  //Enable the array list feature of SPI DMA.
  nrf_spim_rx_list_enable(_spim.p_reg);
  nrf_spim_tx_list_enable(_spim.p_reg);

  //Use 8MHz SPI clock speed.
  setClock(8);

  nrf_spim_rx_buffer_set(_spim.p_reg,&rx_buffer->buffer[0],SPI_BYTES_PER_BLOCK);
  nrf_spim_tx_buffer_set(_spim.p_reg,&tx_buffer->buffer[0],SPI_BYTES_PER_BLOCK);

  // Serial.println("set buffers...");
  // Serial.flush();

  //Start the timer.
 nrfx_timer_enable(&tim3);
 nrfx_timer_clear(&tim3);

  // Serial.println("set tim0...");
  // Serial.flush();
  //Start the counter.
  nrfx_timer_enable(&tim1);
  nrfx_timer_clear(&tim1);
  nrfx_timer_increment(&tim1); //Increment to deal with first time extra byte.

  // Serial.println("set tim1...");
  // Serial.flush();
  //Clear and start the tracking counter.
  nrfx_timer_enable(&tim2);
  nrfx_timer_clear(&tim2);

  // Serial.println("set tim2...");
  // Serial.flush();

}

void DMA_SPI::pauseRecurringTransfers(){

  //Start the timer.
  nrfx_timer_disable(&tim3);

  //Start the counter.
  nrfx_timer_disable(&tim1);
  //nrfx_timer_increment(&tim1); //Increment to deal with first time extra byte.

  //Clear and start the tracking counter.
  nrfx_timer_disable(&tim2);
  //nrfx_timer_clear(&tim2);

  enablePPI(false);
  //Enable the array list feature of SPI DMA.
  nrf_spim_rx_list_disable(_spim.p_reg);
  nrf_spim_tx_list_disable(_spim.p_reg);

}

void DMA_SPI::tim0_handler(nrf_timer_event_t event_type, void* p_context){
  //Not really needed for the application but useful for debugging. 
  //Make sure to enable interrupt in "nrfx_timer_extended_compare" of "setup_recurring_timer".
  digitalToggle(PIN_A0);
}

void DMA_SPI::tim1_handler(nrf_timer_event_t event_type, void* p_context){

  
  digitalToggle(PIN_A1);
  //Not really needed for the application but useful for debugging. 
}

void DMA_SPI::tim3_handler(nrf_timer_event_t event_type, void* p_context){

  
  digitalToggle(PIN_A0);
  //Not really needed for the application but useful for debugging. 
}
void DMA_SPI::tim2_handler(nrf_timer_event_t event_type, void* p_context){
  digitalWrite(PIN_A1,HIGH);
  //This is a failsafe for if we overflow the buffer. It clears the buffer and resets the spi buffers.
  ///This will cause us to lose the data, but it prevents the fifo from going out of sync, which would probably need a reset to deal with.
  
  DMA_SPI* dma = (DMA_SPI*)p_context;

  Serial.println("DMA buffer Overflow... Reseting buffer.");
  Serial.printf("tim2 count:%d, getrx:%d \n\r",nrfx_timer_capture(&(dma->tim2), NRF_TIMER_CC_CHANNEL0),dma->getRxBuffIndex());
  Serial.flush();
  nrf_spim_rx_buffer_set(dma->_spim.p_reg,&(dma->rx_buffer->buffer[0]),SPI_BYTES_PER_BLOCK);
  nrf_spim_tx_buffer_set(dma->_spim.p_reg,&(dma->tx_buffer->buffer[0]),SPI_BYTES_PER_BLOCK);

  nrfx_timer_clear(&(dma->tim2));
  nrfx_timer_enable(&(dma->tim2));
  digitalWrite(PIN_A1,LOW);
}

void DMA_SPI::gpiote_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action){

  // digitalToggle(PIN_A0);
  
}

uint16_t DMA_SPI::getRxBuffIndex(){

  if(nrfx_timer_is_enabled(&tim2)){

  return nrfx_timer_capture(&tim2, NRF_TIMER_CC_CHANNEL0);
  }
  else{
    
    return 0;
  }
}

uint32_t DMA_SPI::getTimeUntilTransfer(){

    if(nrfx_timer_is_enabled(&tim3)){

  return (SPI_NUM_BLOCKS) - nrfx_timer_capture(&tim3, NRF_TIMER_CC_CHANNEL1); 
  }
  else{
    
    return 0;
  }
}



void DMA_SPI::getRxData(uint8_t* buff, uint8_t index,uint8_t numCopy){

  
  for(int i=0; i<index;i++){

    for(int j=0; j<80; j++){
      
      //Increment buff index by bytes/block for each sample(j). Then for each frame(i) the buff index just increase by the frame size in bytes.
      //The rx buff index increses by 1, since rx_buff[x] holds bytes/block bytes. Then for each frame we have to skip one sample, which propagates forward hence the i*80+i.
      memcpy(&buff[j*SPI_BYTES_PER_BLOCK+i*80*SPI_BYTES_PER_BLOCK],&(rx_buffer[j+(i*80+i)].buffer[0]),SPI_BYTES_PER_BLOCK);
    }

    // memcpy(&buff[i*SPI_NUM_BLOCKS*SPI_BYTES_PER_BLOCK],&(rx_buffer[i*(SPI_NUM_BLOCKS+1)].buffer[0]),SPI_NUM_BLOCKS*SPI_BYTES_PER_BLOCK);
  }
  //TODO: Make sure we are not in a transfer currently!!!!
  //Reset our transfers
  nrf_spim_rx_buffer_set(_spim.p_reg,&rx_buffer->buffer[0],SPI_BYTES_PER_BLOCK);
  nrf_spim_tx_buffer_set(_spim.p_reg,&tx_buffer->buffer[0],SPI_BYTES_PER_BLOCK);

  nrfx_timer_clear(&tim2);
  nrfx_timer_enable(&tim2);
  // for(int i= 0; i<index;i++){

  //     for(int j=0;j<80;j++){

  //       for(int k=0; k<7;k++){
  //         Serial.printf("%x, ",buff[i*SPI_BYTES_PER_BLOCK*SPI_NUM_BLOCKS + j*SPI_BYTES_PER_BLOCK+k]);

  //       }
  //       Serial.println(" ");
  //     }

  //     Serial.println(" ");
  // }

}


 void DMA_SPI::enablePPI(bool enable){

   if(enable){
       //Enable the PPI channels
      nrfx_err_t err = nrfx_ppi_channel_enable(timerToSpi_PPI_CHAN);
      // Serial.printf("PPI enable timer to spi: %x\n\r",err);
      err = nrfx_ppi_channel_enable(gpioteToCounter_PPI_CHAN);
      // Serial.printf("PPI enable timer to counter: %x\n\r",err);
      err = nrfx_ppi_channel_enable(SpiToCounter_PPI_CHAN);
      // Serial.printf("PPI enable spi to counter: %x\n\r",err);
      err = nrfx_ppi_channel_enable(CounterToSpi_PPI_CHAN);
      // Serial.printf("PPI enable counter to spi: %x\n\r",err);
      err = nrfx_ppi_channel_enable(spiToSpi_PPI_CHAN);
      // Serial.printf("PPI enable spi to spi: %x\n\r",err);  
   }
   else{
       //Disable the PPI channels
      nrfx_err_t err = nrfx_ppi_channel_disable(timerToSpi_PPI_CHAN);
      // Serial.printf("PPI enable timer to spi: %x\n\r",err);
      err = nrfx_ppi_channel_disable(gpioteToCounter_PPI_CHAN);
      // Serial.printf("PPI enable timer to counter: %x\n\r",err);
      err = nrfx_ppi_channel_disable(SpiToCounter_PPI_CHAN);
      // Serial.printf("PPI enable spi to counter: %x\n\r",err);
      err = nrfx_ppi_channel_disable(CounterToSpi_PPI_CHAN);
      // Serial.printf("PPI enable counter to spi: %x\n\r",err);
      err = nrfx_ppi_channel_disable(spiToSpi_PPI_CHAN);
      // Serial.printf("PPI enable spi to spi: %x\n\r",err);  

   }
 }