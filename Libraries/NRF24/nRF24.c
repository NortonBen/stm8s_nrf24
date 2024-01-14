#include "nRF24.h"
#include "stm8s.h"
#include <cstdlib>

// NRF PIN

#define MISO	        GPIO_PIN_7
#define MOSI	        GPIO_PIN_6
#define	SCK	          GPIO_PIN_5
#define	CE	          GPIO_PIN_3	
#define	CSN		        GPIO_PIN_4	
#define IRQ		        GPIO_PIN_2




#define NRFGPIO            GPIOC
uint8_t PTX = 0;
uint8_t nrf_payload;

void csnHi() {
  GPIO_WriteHigh(NRFGPIO, CSN);
}

void csnLow() {
  GPIO_WriteLow(NRFGPIO, CSN);
}

void ceHi() {
  GPIO_WriteHigh(NRFGPIO, CE);
}

void ceLow() {
  GPIO_WriteLow(NRFGPIO, CE);
}

void powerUpRx();
void powerUpTx();
void powerDown();
void flushRx();

void spi_write(uint8_t data) {
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(data);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE)== RESET);
  SPI_ReceiveData();
}

uint8_t spi_read() {
  uint8_t result;
  //Get data
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(0x00);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  result = SPI_ReceiveData();
  return result;
}

uint8_t spi_sync(uint8_t data) {
  uint8_t result;
  //Get data
  while (SPI_GetFlagStatus(SPI_FLAG_TXE)== RESET);
  SPI_SendData(data);
  while (SPI_GetFlagStatus(SPI_FLAG_BSY)== SET);
  while (SPI_GetFlagStatus(SPI_FLAG_RXNE) == RESET);
  result = SPI_ReceiveData();
  return result;
}




void Nrf24l_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len){
    uint8_t i;
    for(i = 0;i < len;i++){
      datain[i] = spi_sync(dataout[i]);
    }
}

void Nrf24l_transmitSync(uint8_t *dataout,uint8_t len){
    uint8_t i;
    for(i = 0;i < len;i++){
      spi_write(dataout[i]);
    }
}


extern void Nrf24l_init(void) {
  
  //CSN
  GPIO_Init( NRFGPIO, CSN, GPIO_MODE_OUT_PP_HIGH_FAST);
  //CE
  GPIO_Init( NRFGPIO, CE, GPIO_MODE_OUT_PP_HIGH_FAST);
  
  // setup INIT
  ceLow();
  csnHi();

  //SPI
  SPI_Init(
      SPI_FIRSTBIT_MSB,
      SPI_BAUDRATEPRESCALER_256,
      SPI_MODE_MASTER,
      SPI_CLOCKPOLARITY_LOW,
      SPI_CLOCKPHASE_1EDGE,
      SPI_DATADIRECTION_2LINES_FULLDUPLEX,
      SPI_NSS_SOFT,
      (uint8_t)0x07
  );
  SPI_Cmd(ENABLE);
}

extern void Nrf24l_config(uint8_t channel,uint8_t payload) 
// Sets the important registers in the MiRF module and powers the module
// in receiving mode
// NB: channel and payload must be set now.
{       
    // Set RF channel
    Nrf24l_configRegister(RF_CH,channel);

    // Set length of incoming payload 
    Nrf24l_configRegister(RX_PW_P0, payload);
    Nrf24l_configRegister(RX_PW_P1, payload);
    
    nrf_payload = payload;

    // Start receiver 
    powerUpRx();
    flushRx();
}


extern void Nrf24l_setRADDR(uint8_t * adr) 
// Sets the receiving address
{
    ceLow();
    Nrf24l_writeRegister(RX_ADDR_P1,adr,nrf_ADDR_LEN);
    ceHi();
}

extern void Nrf24l_setTADDR(uint8_t * adr)
// Sets the transmitting address
{
    /*
     * RX_ADDR_P0 must be set to the sending addr for auto ack to work.
     */

    Nrf24l_writeRegister(RX_ADDR_P0,adr,nrf_ADDR_LEN);
    Nrf24l_writeRegister(TX_ADDR,adr,nrf_ADDR_LEN);
}

extern void Nrf24l_setChanel(uint8_t channel){
  Nrf24l_configRegister(RF_CH,channel);
}

extern bool Nrf24l_dataReady() 
// Checks if data is available for reading
{
    // See note in getData() function - just checking RX_DR isn't good enough
	uint8_t status = Nrf24l_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) ) return 1;
    return !Nrf24l_rxFifoEmpty();
}

extern bool Nrf24l_rxFifoEmpty(){
    uint8_t fifoStatus;

    Nrf24l_readRegister(FIFO_STATUS,&fifoStatus,sizeof(fifoStatus));
    return (fifoStatus & (1 << RX_EMPTY));
}


extern void Nrf24l_getData(uint8_t * data) 
// Reads payload bytes into data array
{
    csnLow();                               // Pull down chip select
    spi_write( R_RX_PAYLOAD );            // Send cmd to read rx payload
    Nrf24l_transferSync(data,data, nrf_payload); // Read payload
    csnHi();                               // Pull up chip select
    // NVI: per product spec, p 67, note c:
    //  "The RX_DR IRQ is asserted by a new packet arrival event. The procedure
    //  for handling this interrupt should be: 1) read payload through SPI,
    //  2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more 
    //  payloads available in RX FIFO, 4) if there are more data in RX FIFO,
    //  repeat from step 1)."
    // So if we're going to clear RX_DR here, we need to check the RX FIFO
    // in the dataReady() function
    Nrf24l_configRegister(STATUS,(1<<RX_DR));   // Reset status register
}

void Nrf24l_configRegister(uint8_t reg, uint8_t value)
// Clocks only one byte into the given MiRF register
{
    csnLow();
    spi_write(W_REGISTER | (REGISTER_MASK & reg));
    spi_write(value);
    csnHi();
}

void Nrf24l_readRegister(uint8_t reg, uint8_t * value, uint8_t len)
// Reads an array of bytes from the given start position in the MiRF registers.
{
    csnLow();
    spi_write(R_REGISTER | (REGISTER_MASK & reg));
    Nrf24l_transferSync(value,value,len);
    csnHi();
}

void Nrf24l_writeRegister(uint8_t reg, uint8_t * value, uint8_t len) 
// Writes an array of bytes into inte the MiRF registers.
{
    csnLow();
    spi_write(W_REGISTER | (REGISTER_MASK & reg));
    Nrf24l_transmitSync(value,len);
    csnHi();
}


void Nrf24l_send(uint8_t * value) 
// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
{
    uint8_t status;
    status = Nrf24l_getStatus();

    while (PTX) {
        status = Nrf24l_getStatus();

        if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
            PTX = 0;
            break;
        }
    }                  // Wait until last paket is send

    ceLow();
    
    powerUpTx();       // Set to transmitter mode , Power up
    
    csnLow();                    // Pull down chip select
    spi_write( FLUSH_TX );     // Write cmd to flush tx fifo
    csnHi();                    // Pull up chip select
    
    csnLow();                    // Pull down chip select
    spi_write( W_TX_PAYLOAD ); // Write cmd to write payload
    Nrf24l_transmitSync(value,nrf_payload);   // Write payload
    csnHi();                    // Pull up chip select

    ceHi();                     // Start transmission
}

/**
 * isSending.
 *
 * Test if chip is still sending.
 * When sending has finished return chip to listening.
 *
 */

bool Nrf24l_isSending(){
    uint8_t status;
    if(PTX){
      status = Nrf24l_getStatus();
    
      /*
       *  if sending successful (TX_DS) or max retries exceded (MAX_RT).
       */

      if((status & ((1 << TX_DS)  | (1 << MAX_RT)))){
        powerUpRx();
        return FALSE; 
      }

      return TRUE;
    }
    return FALSE;
}

uint8_t Nrf24l_getStatus(){
    uint8_t rv;
    Nrf24l_readRegister(STATUS,&rv,1);
    return rv;
}

void powerUpRx(){
    PTX = 0;
    ceLow();
    Nrf24l_configRegister(CONFIG, nrf_CONFIG | ( (1<<PWR_UP) | (1<<PRIM_RX) ) );
    ceHi();
    Nrf24l_configRegister(STATUS,(1 << TX_DS) | (1 << MAX_RT)); 
}

void flushRx(){
    csnLow();
    spi_write( FLUSH_RX );
    csnHi();
}

void powerUpTx(){
    PTX = 1;
    Nrf24l_configRegister(CONFIG, nrf_CONFIG | ( (1<<PWR_UP) | (0<<PRIM_RX) ) );
}


void powerDown(){
    ceLow();
    Nrf24l_configRegister(CONFIG, nrf_CONFIG );
}