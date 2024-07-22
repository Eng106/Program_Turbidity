/*
 This firm ware is made to let ESP32 act as: Modbus-RTU Slave for non Modbus Digital Sensor
> Sensor      : Turbidity NEP-5000 on sensorPort
> Output      : Modbus-RTU Slave RS-485 on UART1 (Serial1)
> Monitor Port: ESP32-USB (UART0 = Serial) as HostPort
> Wipe Button : at GPIO34 [Normally Low]
*/

#include <Arduino.h>
#include <ModbusRTU.h>

const char *EoLoop = "-<EOL>-"; // End Of Loop Void
const uint32_t DemoModeDelay = 10; //3000; // msec before Initialization, used for online simulation only

// Define UARTs Settings
#define HostPort  Serial    // Serial0 RX TX
#define ModbusPort Serial1 // RXTXpin tobe Defined RX_GPIO33 TX_GPIO32
#define sensorPort Serial2 // Serial2 RX_GPIO16 TX_GPIO17


// Sensor asObject -> Turbidity Sensor
struct SensorObject
{
  HardwareSerial port = sensorPort; // Serial2 RX_GPIO16 TX_GPIO17
  u_int16_t SensorID = 1;           // All Sensor's ID set as 1, Modbus ID are defined under.
  float NTUval = NAN;               // float (4-byte)
  enum eRange  {None = -1, Auto = 0, Low = 1, Medium = 2, Large = 3};
  eRange Range = eRange::None; // Sensor reading Range :   1 = Low; 2 = Medium; 3 = Large
  bool valValid = false;    // if Sensor reading value is Valid.
  uint32_t failCount = 0;   // to store value, how many times Sensors Fails to response command

  const char* Range_toString(eRange enumRange) throw()
  {
    switch (enumRange)
    {
    case eRange::Auto : return "Auto"; break;
    case eRange::Low : return "Low"; break;
    case eRange::Medium : return "Medium"; break;
    case eRange::Large : return "Large"; break;
    case eRange::None : return "None-Undefined"; break;
    default: throw std::invalid_argument("Invalid Range"); break;
    }
  }
  
  bool Init_Check(HardwareSerial* host);
  String read(u_int wait_t_seconds, HardwareSerial* host);
  void setRange(eRange Range, HardwareSerial* host);
  void wipe(HardwareSerial* host);
} Sensor; // Turbidity Sensor

//TMR-Converter Module
struct HWModule
{
  #define wipe_pin GPIO_NUM_34
  // Initialize TMR-Converter Device: Wipe Button
  void Init(){pinMode(wipe_pin, INPUT);} 
  //Read Wipe Button Status
  bool WipeButton_Status()
  {
    bool btnStatus = false;
    if (digitalRead(wipe_pin) > 0)
    {
      delay(100);
      if (digitalRead(wipe_pin) > 0) // recheck Avoid Bounching
      {btnStatus = true;}
    }
    return btnStatus;
  }
}TMR_Device;

//>------ModBus Def & Declaration-----------
struct ModBusRTU_Slave
{
#define SLAVE_ID 1 // Cange depen on Device Requrement.
  HardwareSerial port = ModbusPort;
  //---Modbus-RTU Loop Task Def.-----
  BaseType_t MBCore;   // [ModBus Loop Core] var: Core0?
  TaskHandle_t MBTask; // Modbus Task Handle for Core 0 

  // Modbus Registers Offsets (0-9999)
  const int NTUval_LoWord_Ireg = 0; //  2x Inpput Registers 4-bytes floating Point (Single)
  const int NTUval_HiWord_Ireg = 1; //  2x Inpput Registers 4-bytes floating Point (Single)
  const int Sensor_Range_Ireg = 2;  // Sensor Range Value: 1 = Low; 2 = Medium; 3 = Large

  const int Sensor_Wipe_Coil = 0; // Sensor Wiper Registers/Coil

  ModbusRTU mb; // ModbusSerial object

  // Values Variables
  float NTUval;
  uint32_t dval; // typeCast to unsigned 32bit integer (4-byte).  //= *((uint32_t *) &Sensor.NTUval);
  uint16_t Hval; // High Word 16-bit                               // = dval >> 16;
  uint16_t LVal; // Low Word 16-bit                                // = dval & 0xFFFF;

  /* Initialize TMR_ModBus Object: ModBusPort, SlaveID, Registers, Initial Values, etc \
  |> NTUval : Inital NTU Value, NaN as Default                                          |
  |> RangeMode: Sensor eRange (-1: None, 0: Auto, 1: Low, 2: Medium, 3: Large)          | 
  \------------------------------------------------------------------------------------*/
  void Init(float NTUval = NAN, int RangeMode = -1)
  {
    port.begin(9600, SerialConfig::SERIAL_8N1, GPIO_NUM_33, GPIO_NUM_32); // RXTXpin RX_GPIO33 TX_GPIO32
    mb.begin(&port);
    mb.slave(SLAVE_ID);
    
    // Defiine Modbus Register
    mb.addIreg(NTUval_HiWord_Ireg);
    mb.addIreg(NTUval_LoWord_Ireg);
    mb.addIreg(Sensor_Range_Ireg);
    mb.addCoil(Sensor_Wipe_Coil);

    UpdateNTUVal(NTUval, RangeMode); //Initial Reading
    mb.Coil(Sensor_Wipe_Coil,0);               //Inital WipeCoil Staus
  }

  // Update NTU Vlaue to Modbus Registers
  bool UpdateNTUVal(float fNTUVal, int RangeMode) {
     NTUval = fNTUVal;
     dval = *((uint32_t *) &NTUval); // typeCast to unsigned 32bit integer (4-byte).
     Hval = dval >> 16;             // High Word 16-bit
     LVal = dval & 0xFFFF;          // Low Word 16-bit
    bool Result = true;
    Result = mb.Ireg(NTUval_HiWord_Ireg,Hval)? Result:false;
    Result = mb.Ireg(NTUval_LoWord_Ireg,LVal)? Result:false;
    Result = mb.Ireg(Sensor_Range_Ireg,RangeMode)? Result:false;
    return Result;
  }
  
  //To Add:
  // Wipe.IsRequested
  // Wipe.Done

} TMR_Modbus;
void Aux_Loop(void *pvParameters); 
//---------ModBus Def & Dec End Here 

void setup()
{ 
  // Host Port Initialization:
  HostPort.begin(9600, SerialConfig::SERIAL_8N1);
  HostPort.printf("setup() is running on Core %d\r\n\n", xPortGetCoreID());
  HostPort.printf("initialization\r\n");
  delay(DemoModeDelay);
  TMR_Device.Init();
  delay(100);

  // Modbus Initialization
  HostPort.println("Modbus Port Inisialization");
  TMR_Modbus.Init(Sensor.NTUval, SensorObject::eRange::None);
  HostPort.printf("Modbus Slave ID = %d\r\n",SLAVE_ID);
  //Create Aux_Loop "Aux_ModBusLoop" Loop Task
  xTaskCreatePinnedToCore(Aux_Loop,"Aux_ModBusLoop",10000,NULL,1,&TMR_Modbus.MBTask,0);

  // Sensor Initialisation Start here..
  HostPort.printf("sensorPort Init...\r\n");
  Sensor.port.begin(9600, SerialConfig::SERIAL_8N1); yield();
  delay(1000);
  HostPort.printf("Sensor check..?\r\n");
  if (Sensor.Init_Check(&HostPort))
  {
    Sensor.setRange(Sensor.Auto, &HostPort);
    Sensor.wipe(&HostPort);
  } else
  {
    esp_restart();
  }

  HostPort.printf("Started...\r\n");
}

void loop()
{
  // put your main code here, to run repeatedly:
  HostPort.printf("loop void on Core %d\r\n\n", xPortGetCoreID());

  //Check Input Command
  if (TMR_Modbus.mb.Coil(TMR_Modbus.Sensor_Wipe_Coil))
  {
    Sensor.wipe(&HostPort); yield();
    TMR_Modbus.mb.Coil(TMR_Modbus.Sensor_Wipe_Coil, false);
  }
  
  Sensor.read(10, &HostPort);
  if (Sensor.valValid)
  {
    // ModBus
    TMR_Modbus.UpdateNTUVal(Sensor.NTUval, Sensor.Range);
    uint32_t dval = TMR_Modbus.dval; // typeCast to unsigned 32bit integer (4-byte).
    uint16_t Hval = TMR_Modbus.Hval;             // High Word 16-bit
    uint16_t LVal = TMR_Modbus.LVal;          // Low Word 16-bit
    // HostPort
    String sRange = Sensor.Range_toString(Sensor.Range);
    HostPort.printf("Sensor ID: %d\r\n", Sensor.SensorID);
    HostPort.printf("Turbidity: %#.2f NTU \n\r Range R%d: %s\r\n", Sensor.NTUval, Sensor.Range, &sRange);
    HostPort.printf("32bit Register: 0x%X \r\n", dval);
    HostPort.printf("High Word : 0x%X \n\r Low Word : 0x%X \r\n", Hval, LVal);   

  }
  else
  {
    HostPort.printf("There is no response found \r\n");
    HostPort.printf("Fail Count : %dx\r\n", Sensor.failCount);
  }
  HostPort.println(EoLoop);

  if (Sensor.failCount >= 3)    //If Fail Over 3x, System Restart
  {
    printf("System Restarting...\r\n"); delay(3000);
    esp_restart();
  }
  
  HostPort.printf("Modbus Slave ID = %d @Core %d\r\n",SLAVE_ID, TMR_Modbus.MBCore);
  delay(100);
}

// Additional Loop to Server Modbus Host Request at Core 0
void Aux_Loop(void * pvParameters)
{
  for (;;)
  {
    TMR_Modbus.mb.task();
    yield();
    delay(100);
    if(TMR_Device.WipeButton_Status()){
      TMR_Modbus.mb.Coil(TMR_Modbus.Sensor_Wipe_Coil, true);
    }
    TMR_Modbus.MBCore = xPortGetCoreID(); // GEt Modbus Loop Core#
  }  
}

// Sensor Initialization Check, function
bool SensorObject::Init_Check(HardwareSerial* host = &HostPort)
{
  String txtResponse = Sensor.read(10, host); // check if Sensor available, wait for (1..10 second)
  //txtResponse.trim();  
  host->printf("Sensor txtResponse: \"%s\" \r\n", &txtResponse); yield();

  host->printf("Check if Sensor Value is Valid"); yield();
  if (Sensor.valValid)
  { // if sensor available  => set RAnge to Auto {R=0} cmd = "1,range,0\n\r"
    host->printf("...Ok\r\n"); yield();
    host->printf("sensor Ready...\r\n"); yield();
    return true;
  }
  else
  { // if Sensor no response: "Sensor Check no Response"
    host->printf("...Invalid\r\nSensor Value is Invalid\r\n"); yield();
    host->printf("System Restarting."); yield();
    for (size_t i = 0; i < 3; i++)
    {
      delay(1000);
      host->printf("."); yield();
    }
    host->printf("\r\n"); host->println(EoLoop); yield();
    return false;
  }  
}

String SensorObject::read(u_int wait_t_seconds, HardwareSerial* host)
{
  String txtResponse = emptyString;
  port.read(); yield();              // clear read buffer
  port.printf("%d,read\r\n", SensorID); yield();// cmd to read NTU Value from Sensor
  int iPass = 0;
  host->printf("Wait for Sensor response."); yield();
  u_int i = 0;
  while ((!port.available()) && (i < (wait_t_seconds * 10U)))
  {
    if (iPass != ((i+5) / 10))
    {
      iPass = (i + 5) / 10;
      host->print("."); yield();
    }
    delay(100);
    i++;
  }
  host->printf("%ds w/n %dsec\r\n", iPass, wait_t_seconds); yield();
  Sensor.failCount++;   // CountUp as Fail, Reseted when success.

  if (port.available())
  {    
    txtResponse = port.readString();
    txtResponse.trim();
    int LenResp = txtResponse.length();
    const char *cResponse = txtResponse.c_str();
    int scanFound = sscanf(cResponse, "#,%d,R%d,%f", &SensorID, &Range, &NTUval);
    valValid = (scanFound = 3) ? true : false;
    if (valValid)
    {
      Sensor.failCount = 0; //when success, Reset the FailCount
    }    
  }
  else
  {
    host->printf("No Sensor Response...\r\n");
    valValid = false;
  }
  return txtResponse;
}
// Set Sensor Measuring Range Mode
void SensorObject::setRange(eRange Range, HardwareSerial *host)
{
  String strRange;
  strRange = Range_toString(Range);
  host->printf("Sensor range set to R%d: %s\r\n", Range, &strRange); yield();
  port.read(); yield();
  port.printf("%d,range,%d\n\r", SensorID, Range); yield(); yield();
  delay(1000);
}
// Invoke Sensor to Wipe
void SensorObject::wipe(HardwareSerial *host)
{
  host->printf("Wipe Sensor...!\n\r");  yield();
  port.read();  yield();
  port.printf("%d,wipe\n\r", SensorID);  yield();
  delay(1000);
  read(10, host);  yield();  // wait for sensor response

}
