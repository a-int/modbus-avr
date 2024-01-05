#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>				
#include <avr/interrupt.h>

#define ADC_CODE_TO_VOLTS (4.887585532747)
#define ADC_CODE_TO_TEMPERATURE (0.97751710654936)

#define MB_REC_DEV_ID_INDEX 0
#define MB_REC_FN_INDEX 1
#define MB_REC_ST_ADDRH_INDEX 2
#define MB_REC_ST_ADDRL_INDEX 3
#define MB_REC_DATAH_INDEX 4
#define MB_REC_DATAL_INDEX 5

#define MB_MEMORY_COIL_START (0)
#define MB_MEMORY_COIL_SIZE (4)
#define MB_MEMORY_INPUT_START (MB_MEMORY_COIL_START + MB_MEMORY_COIL_SIZE)
#define MB_MEMORY_INPUT_SIZE (4)
#define MB_MEMORY_HOLDING_START (MB_MEMORY_INPUT_START + MB_MEMORY_INPUT_SIZE)
#define MB_MEMORY_HOLDING_SIZE (32)
#define MB_MEMORY_SIZE ((MB_MEMORY_COIL_SIZE) + (MB_MEMORY_INPUT_SIZE) + (MB_MEMORY_HOLDING_SIZE))

#define MB_ERROR_WRONG_FUNCTION (0x01)
#define MB_ERROR_WRONG_START_ADDRESS (0x02)
#define MB_ERROR_MEMORY_MAX_EXCEED (0x02)
#define MB_MEMORY_READ_MAX (MB_MEMORY_SIZE)

void init_timer(void);
void init_PORT(void);
void init_adc(void);
void init_COM(void);
void init_modbus();
void adc_conv();

void modbus_read_fn(unsigned char function, unsigned int data);
void modbus_write_fn(unsigned char function, unsigned int data);
void modbusError(unsigned char errorCode);
void modbusAnswerReadFN(unsigned int data);
unsigned int modbus_crc16(unsigned char* buf, unsigned int len);

using ModBus = struct {
	unsigned char deviceId;
	unsigned char currentByteIndex;
	unsigned char timerLastByte;
	unsigned char isSending;
	unsigned char startAddress;
	unsigned char bytesToTransmitt;
  unsigned char receivedData[MB_MEMORY_SIZE];
  unsigned char transmittData[MB_MEMORY_SIZE];
  unsigned char memory[MB_MEMORY_SIZE];
};

ModBus MB = {.deviceId = 0x01};
const volatile unsigned short serialNumber[] = {'s', 'e', 'r', 'i', 'a', 'l'};
volatile unsigned int outcounter=0;

int main(void)
{
	asm ("cli");
	init_PORT();
	init_timer();
	init_adc();
	init_COM();
	init_modbus();
	asm ("sei");
	
	while(1)
	{
		adc_conv();
		if(MB.isSending){
			MB.isSending = 0;
			
			unsigned short crc_calculated = modbus_crc16(MB.receivedData, MB.currentByteIndex-2);
			unsigned short crc_received = MB.receivedData[MB.currentByteIndex-1]+(MB.receivedData[MB.currentByteIndex-2]<<8);
			
			if( (MB.receivedData[MB_REC_DEV_ID_INDEX] == MB.deviceId) && (crc_calculated == crc_received)){
				unsigned char function = MB.receivedData[MB_REC_FN_INDEX];
				MB.startAddress = MB.receivedData[MB_REC_ST_ADDRL_INDEX] + (MB.receivedData[MB_REC_DATAH_INDEX]<<8);
				unsigned int data = (MB.receivedData[MB_REC_DATAL_INDEX] + (MB.receivedData[MB_REC_DATAH_INDEX]<<8));
				
				MB.transmittData[1] = function;
				
				if(function <= 0x04){ // fn is read command
					modbus_read_fn(function, data);
				}
				else if(function <= 0x10){ // fn is write command
					modbus_write_fn(function, data);
				}
				else {
					modbusError(MB_ERROR_WRONG_FUNCTION);
				}
			}
		}
	}
	return 0;
}

ISR (TIMER2_COMP_vect){
	MB.timerLastByte++;
	if(MB.timerLastByte == 2){ // the time until the byte is considered as the same message is 3.5 ch.
		MB.timerLastByte = 0;
		if(MB.currentByteIndex > 0){
			MB.isSending = true;
		}
	}
}

ISR (USART_UDRE_vect) {
	if(MB.bytesToTransmitt == 0){
		MB.currentByteIndex = 0;
		//TURN ON TIMER 2
		MB.timerLastByte = 0;
		TIMSK |= (1<<OCIE2); // enable output compare match interrupt
		UCSRB&=~(1<<UDRIE);
	} else {
		UDR = MB.transmittData[outcounter - MB.bytesToTransmitt];
		MB.bytesToTransmitt--;
	}	
}

ISR (USART_RXC_vect) {
	unsigned char temp_UDR = UDR;
	
	MB.timerLastByte = 0;
	MB.receivedData[MB.currentByteIndex] = temp_UDR; // save next byte
	MB.currentByteIndex = (MB.currentByteIndex+1)%256;
}

void modbusError(unsigned char errorCode){
	MB.transmittData[1] = function|0x80;
	MB.transmittData[2] = errorCode;
	
	volatile unsigned short crc = modbus_crc16(MB.transmittData, 3);
	MB.transmittData[3] = crc / 256;
	MB.transmittData[4] = crc % 256;
	
	MB.bytesToTransmitt = 5;
	outcounter = MB.bytesToTransmitt;
	UCSRB|=(1<<UDRIE);
}

void modbusAnswerReadFN(unsigned int data){
	MB.transmittData[2] = data; // number of bytes
	for(unsigned int i = 0; i < data; ++i){
		MB.transmittData[3+i] = MB.memory[MB.startAddress + i];
	}
	volatile unsigned short crc = modbus_crc16(MB.transmittData, 3+data);
	MB.transmittData[3+data] = crc/256;
	MB.transmittData[3+data+1] = crc%256;
	MB.bytesToTransmitt = data + 5; //id + fn + N + CRC = 5 bytes
	outcounter = MB.bytesToTransmitt;
	UCSRB|=(1<<UDRIE);
}

void init_modbus(){
	// clear transmitt at startup
	for (unsigned int i = 0; i < sizeof(MB.transmittData)/sizeof(MB.transmittData[0]); ++i){
		MB.transmittData[i] = 0;
	}
	//copy the serial number into holding registers
	for(unsigned int i = 0; i < sizeof(serialNumber)/sizeof(serialNumber[0]); ++i){
		MB.memory[MB_MEMORY_HOLDING_START + i] = serialNumber[i];
	}
	MB.transmittData[0] = MB.deviceId; // assign device ID for transmitted data
}

void modbus_read_fn(unsigned char function, unsigned int data){
	if( function == 0x01){ // read coil
		if(MB.startAddress <= MB_MEMORY_COIL_SIZE){
			if((MB.startAddress+data) <= (MB_MEMORY_COIL_SIZE)){
				MB.startAddress += MB_MEMORY_COIL_START;
			} else {
				modbusError(MB_ERROR_MEMORY_MAX_EXCEED);
			}
		} else {
			modbusError(MB_ERROR_WRONG_START_ADDRESS);
		}
	} else if(function == 0x03){ //read Holding registers
		if(MB.startAddress <= MB_MEMORY_HOLDING_SIZE){
			if((MB.startAddress+data) <= MB_MEMORY_HOLDING_SIZE){
				MB.startAddress += MB_MEMORY_HOLDING_START;
			} else {
				modbusError(MB_ERROR_MEMORY_MAX_EXCEED);
			}
		} else {
			modbusError(MB_ERROR_WRONG_START_ADDRESS);
		}
	} else if (function == 0x04){ // read Input registers
		data <<= 1;
		if(MB.startAddress <= MB_MEMORY_INPUT_SIZE){
			if((MB.startAddress+data) <= MB_MEMORY_INPUT_SIZE){
				MB.startAddress += MB_MEMORY_INPUT_START;
			} else {
				modbusError(MB_ERROR_MEMORY_MAX_EXCEED);
			}
		} else {
			modbusError(MB_ERROR_WRONG_START_ADDRESS);
		}
	}
	modbusAnswerReadFN(data);
}

void modbus_write_fn(unsigned char function, unsigned int data){
	MB.transmittData[2] = MB.startAddress / 256;
	MB.transmittData[3] = MB.startAddress % 256;
	MB.transmittData[4] = data / 256;
	MB.transmittData[5] = data % 256;
	MB.bytesToTransmitt = 8; //id + fn + N + CRC = 5 bytes
	outcounter = MB.bytesToTransmitt;
	
	if( function == 0x05 ){ // write 1 coil
		if(MB.startAddress <= MB_MEMORY_COIL_SIZE){
			MB.startAddress += MB_MEMORY_COIL_START;
			if(data == 0xFF00){
				MB.memory[MB.startAddress] = true;
			} else if(data == 0x0000){
				MB.memory[MB.startAddress] = false;
			} else {
				modbusError(MB_ERROR_WRONG_FUNCTION);
			}
			UCSRB|=(1<<UDRIE);
		} else {
			modbusError(MB_ERROR_WRONG_START_ADDRESS);
		}
	} else if( function == 0x10){ // write multiple registers
		volatile unsigned short N = MB.receivedData[6]; // 5th byte is N - number of bytes(8 bit not 16) to write
		if(MB.startAddress <= MB_MEMORY_INPUT_SIZE){
			if((MB.startAddress+data) <= MB_MEMORY_INPUT_SIZE){
				MB.startAddress += MB_MEMORY_INPUT_START;
			} else {
				modbusError(MB_ERROR_MEMORY_MAX_EXCEED);
			}
		} else {
			modbusError(MB_ERROR_WRONG_START_ADDRESS);
		}
		MB.currentByteIndex -= 8; // fn + N + addr + data + crc eq to 8 bytes
		for(unsigned int i = 0; i != MB.currentByteIndex; ++i){
			MB.memory[MB.startAddress + i] = MB.receivedData[7 + i];
		}
		UCSRB|=(1<<UDRIE);
	} else {
		modbusError(MB_ERROR_WRONG_FUNCTION);
	}
}

void adc_conv(){
	if (sec_flag==1){
		sec_flag=0;
		ADCSRA = (1 << ADEN) | (1<<ADSC)| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
		while((ADCSRA&(1<<ADSC))!=0){}
		temp_code_ADC = ADC * (ADC_CODE_TO_VOLTS);
		// save data for volts
		MB.memory[MB_MEMORY_INPUT_START + 0] = temp_code_ADC / 256;
		MB.memory[MB_MEMORY_INPUT_START + 1] = temp_code_ADC % 256;
		// save data for temperature
		temp_code_ADC = ADC * (ADC_CODE_TO_TEMPERATURE);
		MB.memory[MB_MEMORY_INPUT_START + 2] = temp_code_ADC / 256;
		MB.memory[MB_MEMORY_INPUT_START + 3] = temp_code_ADC % 256;
	}
}

void init_PORT(){
	PORTA = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	
	DDRA = (1 << PA0) | (1 << PA1) | (1 << PA2) | (1 << PA3);
	DDRB = (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB3) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7);
	DDRC = (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5) | (1 << PC6) | (1 << PC7);
	DDRD = 0;
}

void init_timer(void) {
	TCNT0 = 0x00;
	OCR0 = 49; // number 50
	TCCR0 = (1 << CS02) | (1 << WGM01); // prescaler of 256

	TCNT2 = 0; // default value
	OCR2 = 7; //OCR2 is selected so the Timer 2 works at 1 us speed
	TCCR2 = (1<<CS21)|(WGM20); //prescaler of 8 and CTC mode
	
	TIFR = (1 << OCF0)|(1<<OCF2); // clear output compare match flag
	TIMSK = (1 << OCIE0)|(1<<OCIE2); // enable output compare match interrupt
}

void init_adc(void) {
	DDRA = (1 << DDA0) | (1 << DDA1) | (1 << DDA2) | (1 << DDA3);
	PORTA = 0x00;
	ADMUX = (1 << MUX0) | (1 << MUX2);
	ADCSRA = (1 << ADEN) | (1<<ADSC)| (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

ISR (TIMER0_COMP_vect) {
	temp_time++;
	// update coil state
	PORTB = (MB.memory[MB_MEMORY_COIL_START]) | (MB.memory[MB_MEMORY_COIL_START+1]<<1) | (MB.memory[MB_MEMORY_COIL_START+2]<<2) | (MB.memory[MB_MEMORY_COIL_START+3]<<3);
	if(temp_time==62){
		sec_flag = 1;
		temp_time = 0;
	}
}

void init_COM() {
	UBRRH = 0x00;
	UBRRL = 25; // 19200, 8 bit

	UCSRC = (1 << URSEL) | (1 << UCSZ1) | (1 << UCSZ0); // 8 data bits, no parity, 1 stop bit
	UCSRA = (1 << RXC) | (1 << TXC) | (1 << UDRE); // double the USART transmission speed
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN); // enable RX and TX and their interrupts
	_delay_ms(50);
}

uint16_t modbus_crc16(unsigned char* buf, unsigned int len){
	unsigned short crcOut = 0xFFFF;
	
	for(unsigned int i = 0; i != len; i++){
		crcOut^=(unsigned char) buf[i];
		for(char bit = 0; bit<8; bit++){
			if(crcOut&0x0001){
				crcOut>>=1;
				crcOut ^=0xA001;
			}
			else{
				crcOut>>=1;
			}
		}
	}
	return crcOut;
}
