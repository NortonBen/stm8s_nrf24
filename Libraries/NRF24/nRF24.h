
#ifndef _NRF24_H_
#define _NRF24_H_

#include "nRF24L01.h"
#include "stm8s.h"

// Nrf24l settings

#define nrf_ADDR_LEN	5
#define nrf_CONFIG ((1<<EN_CRC) | (0<<CRCO) )


extern void Nrf24l_init();
extern void Nrf24l_config(uint8_t channel,uint8_t payload);
extern void Nrf24l_send(uint8_t *value);
extern void Nrf24l_setRADDR(uint8_t * adr);
extern void Nrf24l_setTADDR(uint8_t * adr);
extern void Nrf24l_setChanel(uint8_t chanel);
extern bool Nrf24l_dataReady();
extern bool Nrf24l_isSending();
extern bool Nrf24l_rxFifoEmpty();
extern bool Nrf24l_txFifoEmpty();
extern void Nrf24l_getData(uint8_t * data);
extern uint8_t Nrf24l_getStatus();

void Nrf24l_transmitSync(uint8_t *dataout,uint8_t len);
void Nrf24l_transferSync(uint8_t *dataout,uint8_t *datain,uint8_t len);
void Nrf24l_configRegister(uint8_t reg, uint8_t value);
void Nrf24l_readRegister(uint8_t reg, uint8_t * value, uint8_t len);
void Nrf24l_writeRegister(uint8_t reg, uint8_t * value, uint8_t len);


#endif /* _NRF24_H_ */