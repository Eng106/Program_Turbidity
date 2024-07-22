#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_SIZE 1  // Ukuran EEPROM dalam byte
#define ID_ADDRESS 0     // Alamat EEPROM untuk menyimpan nilai id

const char *EoLoop = "-<EOL>-"; // End Of Loop Void

int SlaveID = -1;

void setup() {
  Serial.begin(9600);  // Memulai komunikasi serial
  EEPROM.begin(EEPROM_SIZE);  // Inisialisasi EEPROM dengan ukuran yang ditentukan
  
  SlaveID = EEPROM.read(ID_ADDRESS);
  Serial.printf("Modbus Slave ID = %d\r\n", SlaveID);
  Serial.println("Ready to receive commands.");
}

void loop() {
if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Membaca input serial sampai newline
    command.trim();  // Menghapus spasi putih di awal dan akhir string

    if (command.startsWith("AT+id=")) {
      if (command.length() > 6) {  // Memastikan ada nilai setelah "AT+id="
        String idValue = command.substring(6);  // Mendapatkan nilai setelah "AT+id="
        int id = idValue.toInt();  // Mengubah nilai menjadi integer

        EEPROM.write(ID_ADDRESS, id);  // Menulis nilai ke EEPROM pada alamat ID_ADDRESS
        EEPROM.commit();  // Menyimpan perubahan ke EEPROM
        Serial.print("Stored id: ");
        Serial.println(id);
      } else {
        Serial.println("Error: No value provided after AT+id=");
      }
    } else if (command.equals("AT+id")) {
      int storedId = EEPROM.read(ID_ADDRESS);  // Membaca nilai dari EEPROM pada alamat ID_ADDRESS
      Serial.print("Current id: ");
      Serial.println(storedId);
    } else {
      Serial.println("Invalid command. Use AT+id=value or AT+id.");
    }
  }
  //Serial.println(EoLoop);
  delay(100);
}