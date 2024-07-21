/*
 This firm ware is made just to cek ESP32 UART1 Pin
 > Sensor: Turbidity NEP-5000 on sensorPort
 > Output: Modbus-RTU Slave RS-485 on UART .?.
 > Monitor Port: ESP32-USB (UART0) as HostPort
*/

#include <Arduino.h>

const char *EoLoop = "-<EOL>-"; // End Of Loop Void
u_int16_t SensorID = 1;
float NTUval; // float (4-byte)

HardwareSerial HostPort = Serial1;    // Serial1 RXTXpin tobe Defined
HardwareSerial sensorPort = Serial2; // Serial2 RX_GPIO16 TX_GPIO17

void setup()
{
  // put your setup code here, to run once:

  //HostPort.setPins(GPIO_NUM_33, GPIO_NUM_32); //RX_GPIO33 TX_GPIO32
  //HostPort = Serial1;
  HostPort.begin(9600, SerialConfig::SERIAL_8N1, GPIO_NUM_33, GPIO_NUM_32);
  // Serial1.begin(9600, SerialConfig::SERIAL_8N1);
  sensorPort.begin(9600, SerialConfig::SERIAL_8N1);
  HostPort.printf("inisialization\n\r");
  delay(5000);
  HostPort.printf("Started...\r\n");
}

void loop()
{
  // put your main code here, to run repeatedly:

  // txtResponse = "#,1,R1,5000.30\n\r";
  // int scanFound;

  sensorPort.read();               // clear read buffer
  sensorPort.printf("1,read\r\n"); // read NTU Value from Sensor
  int ipass = 0;
  while (!sensorPort.available() && ipass < 6) // Wait for respose
  {
    HostPort.printf("Wait for Sensor response... %d\r", ipass);
    ipass++;
    delay(1000);
  }
  String txtResponse = sensorPort.readStringUntil('\n');
  const char *cResponse = txtResponse.c_str();
  HostPort.printf("txtResponse: %s\n\r", cResponse);

  int iRange;
  int scanFound = sscanf(cResponse, "#,1,R%d,%f", &iRange, &NTUval);
  if (scanFound != EOF)
  {
    // NTUval = atof("5000.51");
    uint32_t dval = *((uint32_t *)&NTUval); // typeCast to unsigned 32bit integer (4-byte).
    uint16_t Hval = dval >> 16;             // High Word 16-bit
    uint16_t LVal = dval & 0xFFFF;          // Low Word 16-bit
    HostPort.printf("Response found %d Value[s] \r\n", scanFound);
    HostPort.printf("Turbidity: %#.2f NTU \n\r Sensor Range: R%d\r\n", NTUval, iRange);
    HostPort.printf("To 32bit Register: 0x%X \r\n", dval);
    HostPort.printf("High Word : 0x%X \n\r Low Word : 0x%X \r\n", Hval, LVal);
  }
  else
  {
    HostPort.printf("There is no response found \r\n");
  }
  HostPort.println(EoLoop);
  delay(3000);
}
