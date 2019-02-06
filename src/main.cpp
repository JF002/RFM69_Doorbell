#if 1
#include <memory>

#include <SPI.h>
#include "RFM69.h"
#include "RFM69.h"
#include <Arduino.h>
#include <az_iot/iothub_client/inc/iothub_client_ll.h>

/// TODO error quelque part dans Build Frame : Symbol 16 et 24 incorrects

using namespace Codingfield::Communication;

SPIClass spi(HSPI);
std::unique_ptr<Codingfield::Communication::RFM69> radio;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int32_t interruptCounter = 0;
bool packetReceived = false;
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;
  packetReceived = true;
  portEXIT_CRITICAL_ISR(&mux);
}

std::vector<uint8_t> outputFrame;
uint8_t currentByte = 0;
uint8_t currentNbBit = 0;
void Push(uint8_t value, uint8_t nbBits) {
  for(uint8_t i = 0; i < nbBits; i++) {

    currentByte = static_cast<uint8_t>(currentByte & 0xFE);
    currentByte = static_cast<uint8_t>(currentByte | ((value & 0x80) >> 7));


    currentNbBit ++;
    if(currentNbBit == 8) {
      Serial.println("NEW BYTE : " + String(currentByte,BIN));
      outputFrame.push_back(currentByte);
      currentByte = 0;
      currentNbBit = 0;
    }else {
      currentByte = currentByte << 1;
    }
    value = value << 1;

  }
/*
  auto curSize = static_cast<uint8_t>((nbBits < (8 - currentNbBit)) ? nbBits : (8 - currentNbBit));

  for(uint8_t i = 0; i < curSize; i++) {
    currentByte = static_cast<uint8_t>(currentByte & 0xFE);
    currentByte = static_cast<uint8_t>(currentByte | ((value & 0x80) >> 7));
    Serial.print(String(((value & 0x80) >> 7)));
    currentNbBit ++;
    if(currentNbBit < 8)
      currentByte = currentByte << 1;
    value = value << 1;
  }

  if(currentNbBit == 8) {
    outputFrame.push_back(currentByte);
    currentByte = 0;
    currentNbBit = 0;
  }

  if(curSize < nbBits) {
    curSize = nbBits - curSize;
    for(uint8_t i = 0; i < curSize; i++) {
      currentByte = static_cast<uint8_t>(currentByte & 0xFE);
      currentByte = static_cast<uint8_t>(currentByte | ((value & 0x80) >> 7));
      Serial.print(String(((value & 0x80) >> 7)));
      currentNbBit ++;
      if(currentNbBit < 8)
        currentByte = currentByte << 1;
      value = value << 1;
    }
  }*/
}

void FinishFrame() {
  if(currentNbBit != 0) {
    for(uint8_t i = 0; (i < 8-currentNbBit); i ++) {
      currentByte = currentByte << 1;
      currentByte = static_cast<uint8_t>(currentByte & (0xFE));
      outputFrame.push_back(currentByte);
    }
  }
}

std::vector<uint8_t> payload = {0x93, 0x69, 0x36, 0xD3, 0x4D, 0xA4, 0xDB, 0x69, 0x24, 0x93, 0x49, 0x24, 0x92, 0x49, 0x24, 0x92, 0x49, 0x26};
std::vector<uint8_t> checkPayload = {0x12, 0x6D, 0x26, 0xDA, 0x69, 0xB4, 0x9B, 0x6D, 0x24, 0x92, 0x69, 0x24, 0x92, 0x49, 0x24, 0x92, 0x49, 0x24, 0xDF};
void BuildData() {
  for (uint8_t i = 0; i < 50; i++) {
    Push(0x00, 3);
    for (auto b : payload) {
      Push(b, 8);
    }
    Push((0x07 << 5), 3);
  }
  Push((0x07 << 2), 6);

  for(uint8_t i = 0; i < 13; i++) {
    Push(0, 8);
  }
  FinishFrame();

  Serial.println("Payload size : " + String(outputFrame.size()));
  Serial.println("Expected size : " + String(checkPayload.size()));
  uint8_t s = static_cast<uint8_t>((outputFrame.size() < checkPayload.size()) ? outputFrame.size() : checkPayload.size());
  for(uint8_t i = 0; i < s ; i++) {
    if(outputFrame[i] != checkPayload[i]) {
      Serial.println("Error at index " + String(i) + " : " + String(outputFrame[i],BIN) + " != " + String(checkPayload[i], BIN));
    }
  }

}


void setup() {
  Serial.begin(115200);
  Serial.flush();
  Serial.println("Hello");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(100);
  digitalWrite(4, LOW);
  delay(100);

  Serial.println("");
  BuildData();
  Serial.println("");
  Serial.println("Frame Size : " + String(outputFrame.size()));

  delay(100);
  pinMode(15, OUTPUT); //HSPI SS

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, CHANGE);

  spi.begin();

  radio.reset(new RFM69(&spi, 15));

  bool isRFM69 = radio->IsRfm69();
  if(isRFM69) {
    Serial.println("RFM found!");
    auto temperature = radio->ReadTemperature();
    Serial.println("Temperature : " + String(temperature));

    radio->SetOperatingMode(RFM69::Modes::Standby);
    Serial.println("Mode -> Standby");

    Serial.println("Init RFM69...");
    radio->SetDataModulation(RFM69::ModulationShapings::NoShaping,
            RFM69::ModulationTypes::FSK,
            RFM69::DataModes::Packet);
    radio->SetBitRate(0x1430); // 6.250 kpbs
    radio->SetFrequencyDeviation(0x0333); // 50Khz
    radio->SetFrequency(0x00D913E8); // 868Mhz
    //radio->SetDio0Mapping(RFM69::DIO0MappingPacketRX::PayloadReady);
    radio->SetDio0Mapping(RFM69::DIO0MappingPacketTX::PacketSent);
//    radio->SetDio2Mapping(RFM69::DIO2MappingPacketRX::Data);
    radio->SetDio2Mapping(RFM69::DIO2MappingPacketRX::FifoNotEmpty);
    radio->ClearFifoOverrunFlag();
    radio->SetRssiThreshold(0xA0);
    radio->SetRxTimeoutStart(0);
    radio->SetTimeoutRssiThreshold(40);
    //radio->SetSyncWordConfig(0, 1, RFM69::FifoFillConditions::IfSyncAddressInterruptOccurs, true);
    radio->SetSyncWordConfig(0, 1, RFM69::FifoFillConditions::IfSyncAddressInterruptOccurs, false);
    radio->SetSyncWordValue1(0x33);
    radio->SetSyncWordValue2(0xAC);
    radio->SetPacketConfig(RFM69::AddressFilterings::None,
            false,
            false,
            RFM69::DcFreeTypes::None,
            RFM69::PacketFormats::FixedLength);
    radio->SetPayloadLength(64);
    radio->SetFifoThreshold(32, RFM69::TxStartCondition::FifoLevel);
    radio->SetPreambleSize(0);
    radio->SetOcp(false);


    Serial.println("Init RFM69 Done!");
  }
  else {
    Serial.println("RFM ERROR!");
    radio.reset(nullptr);
  }
}





int state = 0;
int count = 0;



int frameIndex = 0;
std::vector<uint8_t> currentFrame;
int currentFrameSize = 0;
void loop() {
  if(radio) {
    switch (state) {
      case 0:
        // Send to fifo

        Serial.print("Write packet to FIFO... ");
//        radio->TransmitPacket({0x33, 0xAC, 0xF0, 0x10, 0x00, 0x01});
//        radio->TransmitPacket({0x12, 0x6D, 0x26, 0xDA, 0x69, 0xB4, 0x9B, 0x6D, 0x24, 0x92,
//                               0x69, 0x24, 0x92, 0x49, 0x24, 0x92, 0x49, 0x24, 0xDC, 0x1C,
//                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//                               0x00, 0x00});
        //radio->TransmitPacket();
        currentFrameSize = 64;
        for(uint8_t i = 0; i < currentFrameSize; i++) {
          currentFrame.push_back(outputFrame[i + frameIndex]);
        }
        frameIndex += currentFrameSize;
        radio->TransmitPacket(currentFrame);
        currentFrame.clear();
        //Serial.println("OK");
        //Serial.print("Swtich to TX mode... ");
        radio->SetOcp(false);
        radio->SetOperatingMode(RFM69::Modes::Tx);
        //Serial.println("OK");
        state = 1;
        break;

      case 1:
        // Poll FIFO level
        if(frameIndex < outputFrame.size()) {
          currentFrameSize = ((outputFrame.size() - (frameIndex))  > 32) ? 32 :  (outputFrame.size() - (frameIndex));
          if(radio->IsIrqFlagSet(RFM69::IrqFlags2::FifoLevel) == false) {
            for(uint8_t i = 0; i < currentFrameSize; i++) {
              currentFrame.push_back(outputFrame[i + frameIndex]);
            }
            frameIndex += currentFrameSize;
            radio->TransmitPacket(currentFrame);
            currentFrame.clear();
          }
        }
        else {
          state = 2;
        }

        break;
      case 2:
        // Wait end of TX
        if(packetReceived) {
          Serial.println("End of TX");
          radio->SetOperatingMode(RFM69::Modes::Standby);
          state = 3;
        }

        break;
      case 3:
        break;
    }
  }
}

/*
int state = 0;
void loop() {
  if(radio) {
    switch(state) {
      case 0:
        Serial.println("Start");
        radio->SetOperatingMode(RFM69::Modes::Rx);
        Serial.print("RX READY");
        state = 1;
        break;
      case 1:
        if(radio->IsIrqFlagSet(RFM69::IrqFlags::Timeout)) {
          Serial.println("WaitTimeout");
          radio->RestartRx();
        }

        if(packetReceived) {
          Serial.println("RX");
          state = 2;
          packetReceived = false;
        }
        break;
      case 2:
        break;
      default: break;
    }
  }
  delay(10);
}

 */
#endif


#if 0

#include <Arduino.h>
#include <SPI.h>

SPIClass spi(HSPI);

void Select() {
  digitalWrite(15, LOW);
}

void Unselect() {
  digitalWrite(15, HIGH);
}

void Write(uint8_t addr, uint8_t value) {
  Select();
  spi.transfer(0x80 | addr);
  spi.transfer(value);
  Unselect();
}

uint8_t Read(uint8_t addr) {
  Select();
  spi.transfer(0x7f & addr);
  uint8_t value =  spi.transfer(0);
  Unselect();
  return value;
}

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
int32_t interruptCounter = 0;
bool packetReceived = false;
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;
  packetReceived = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  Serial.flush();
  Serial.println("Hello");
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  delay(10);
  digitalWrite(4, LOW);
  delay(10);

  delay(100);
  pinMode(15, OUTPUT); //HSPI SS

  spi.begin();

  delay(100);

  Serial.println("Start temperature measurement ...");
  // Start temperature measurement
  Write(0x4e, 0x08);

  uint8_t ready = 0;
  uint8_t temperature = 0;
  uint8_t version = 0;
  do {
    ready = Read(0x4e);
    Serial.println("READY : " + String(ready));
    ready = ready & (uint8_t)0x04;

  }while(ready != 0x4);
  Serial.println("Finished");


  temperature = Read(0x4f);

  version = Read(0x10);

  Serial.println("Temperature : " + String(temperature));
  Serial.println("Version : " + String(version));

  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, CHANGE);

  Write(0x01, 0x04); // STANDBY mode
  //Write(0x02, 0x08); // OOK
  Write(0x02, 0x00); // FSK
  //Write(0x02, 0x60); // FContinuous
  Write(0x03, 0x14); // Bitrate = 6250kbps
  Write(0x04, 0x00); // Bitrate = 6250kbps
  Write(0x05, 0x03); // Freq deviation = 50khz
  Write(0x06, 0x33); // Freq deviation = 50khz
  Write(0x07, 0xd9); // Freq = 868Mhz
  Write(0x08, 0x33); // Freq = 868Mhz
  Write(0x09, 0x3A); // Freq = 868Mhz
  Write(0x25, 0x44); // DIO 0 = Payload Ready
  Write(0x28, 0x10); // Clear fifo overrun flag
  Write(0x29, 0xA0);
  Write(0x2a, 0x00);
  Write(0x2b, 40);
  //Write(0x2e, 0x00); // No sync word
  Write(0x2e, 0x88); // Sync word 2bytes
  Write(0x2f, 0x33);
  Write(0x30, 0xac);
  //Write(0x37, 0xC0); // messages variable length + dc filtering
  Write(0x37, 0); // fix length message
  Write(0x38, 6); // message length
/*
  Write(0x01, 0x10); // RXmode

  uint8_t rxready = 0;
  while(rxready != 0x40) {
    rxready = Read(0x27);
    rxready = rxready & 0x40;
  }
  Serial.print("RX READY");
  */
}

int state = 0;
uint8_t rxready = 0;
uint8_t irqflags1 = 0;
uint8_t irqflags2 = 0;
uint8_t packetconfig2 = 0;
uint8_t length = 0;
void loop() {


  switch(state) {
    case 0:
      Serial.println("Start");
      Write(0x01, 0x10); // RXmode
      rxready = 0;
      while(rxready != 0x80) {
        rxready = Read(0x27);
        rxready = rxready & (uint8_t)0x80;
      }
      Serial.print("RX READY");
      state = 1;
      break;
    case 1:

      irqflags1 = Read(0x27);
      if((irqflags1 & 0x04) == 0x04) {
        Serial.println("WaitTimeout");
        //TIMEOUT
        packetconfig2 = Read(0x3D);
        Write(0x3D,packetconfig2 | (uint8_t)0x04);
      }

      if(packetReceived) {
        Serial.println("RX");
        state = 2;
        packetReceived = false;
      }
      //else
        //Serial.println("Wait");
      break;
    case 2:
      Write(0x01, 0x04); // Standbt

      length = Read(0x00);
      Serial.println("YEAH! " + String(length));



      state = 0;

      break;
    case 3:
      break;
  }
/*
  uint8_t irqstatus1 = Read(0x27);
  Serial.println("IRQ 1 : " + String(irqstatus1));

  uint8_t irqstatus2 = Read(0x28);
  Serial.println("IRQ 2 : " + String(irqstatus2));
*/
  delay(1);




#if 0
  // Restart RX
  Write(0x3d, 0x04);


  /*Write(0x23, 0x01);
  uint8_t rssiready = 0;
  while(rssiready != 0x02) {
    rssiready = Read(0x23);
    rssiready = rssiready & (uint8_t)0x02;
  }
*/
  uint8_t irqstatus1 = Read(0x27);
  uint8_t irqstatus2 = Read(0x28);
  //uint8_t rssi = Read(0x24);

  Serial.println("IRQ 1 : " + String(irqstatus1));
  Serial.println("IRQ 2 : " + String(irqstatus2));
  //Serial.println("RSSI : " + String(rssi, HEX));




  delay(2000);
#endif
}
#endif