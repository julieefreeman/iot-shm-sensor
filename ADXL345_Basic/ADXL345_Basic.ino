 #define LOG_OUT 1 // use the log payload function
 #define FFT_N 64 // set to 256 point fft
 
//Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>
#include <SoftwareSerial.h>
#include <FFT.h>
#include <Time.h>

//Assign the Chip Select signal to pin 10.
int CS=10;

//This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

const int X = 0;
const int Y = 1;
const int Z = 2;
const int SENSOR_MAC_SIZE = 4;
const int TIME_SIZE = 4;
const int TOTAL_PAYLOAD_SIZE = 3*FFT_N/2 + TIME_SIZE + 2 * SENSOR_MAC_SIZE;

//This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];

int xarr[FFT_N*2];
int yarr[FFT_N*2];
int zarr[FFT_N*2];
uint8_t payload[3*FFT_N/2 + 2*SENSOR_MAC_SIZE + TIME_SIZE];
uint8_t SENSOR_MAC[SENSOR_MAC_SIZE] = {0xFE, 0xFE, 0xFE, 0xFE};
uint8_t timeArr[TIME_SIZE];
time_t currentTime;

void setup(){ 
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);
  //Set up the Chip Select pin to be an payload from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  int xarr[FFT_N];
  
  for(int i = 0; i < FFT_N*2; i+=2) {
    xarr[i+1] = 0;
    yarr[i+1] = 0;
    zarr[i+1] = 0;
  }

  memcpy(payload, SENSOR_MAC, SENSOR_MAC_SIZE);
  memcpy(&payload[3*FFT_N/2 + SENSOR_MAC_SIZE + TIME_SIZE], SENSOR_MAC, SENSOR_MAC_SIZE);
  
}

void loop(){
  currentTime = now();
  timeArr[0] = currentTime & 0xFF;
  timeArr[1] = currentTime >> 8 & 0xFF;
  timeArr[2] = currentTime >> 16 & 0xFF;
  timeArr[3] = currentTime >> 24 & 0xFF;
  memcpy(&payload[SENSOR_MAC_SIZE], timeArr, TIME_SIZE);
  for(int i = 0; i < FFT_N*2; i += 2) {
  //Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  //The results of the read operation will get stored to the values[] buffer.
    readRegister(DATAX0, 6, values);
  
    //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    //The X value is stored in values[0] and values[1].
    xarr[i] = ((int)values[1]<<8)|(int)values[0];
    //The Y value is stored in values[2] and values[3].
    yarr[i] = ((int)values[3]<<8)|(int)values[2];
    //The Z value is stored in values[4] and values[5].
    zarr[i] = ((int)values[5]<<8)|(int)values[4];
  }
  memcpy(fft_input, xarr, FFT_N*2);
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_log(); // take the payload of the fft
  sei(); // turn interrupts back on
  memcpy(&payload[FFT_N*X/2 + SENSOR_MAC_SIZE + TIME_SIZE], fft_log_out, FFT_N/2); 
  
  memcpy(fft_input, yarr, FFT_N*2);
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_log(); // take the payload of the fft
  sei(); // turn interrupts back on
  memcpy(&payload[FFT_N*Y/2 + SENSOR_MAC_SIZE + TIME_SIZE], fft_log_out, FFT_N/2); 

  memcpy(fft_input, zarr, FFT_N*2);
  fft_window(); // window the data for better frequency response
  fft_reorder(); // reorder the data before doing the fft
  fft_run(); // process the data in the fft
  fft_mag_log(); // take the payload of the fft
  sei(); // turn interrupts back on
  memcpy(&payload[FFT_N*Z/2 + SENSOR_MAC_SIZE + TIME_SIZE], fft_log_out, FFT_N/2); 
  
  Serial.write(payload, TOTAL_PAYLOAD_SIZE);
  delay(1000);
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, unsigned char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}
