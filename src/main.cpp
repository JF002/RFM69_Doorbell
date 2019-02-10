#include <memory>

#include <SPI.h>
#include "RFM69.h"
#include "RFM69.h"
#include <Arduino.h>
#include <az_iot/iothub_client/inc/iothub_client_ll.h>

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
    radio->SetDio0Mapping(RFM69::DIO0MappingPacketTX::PacketSent);
    radio->SetDio2Mapping(RFM69::DIO2MappingPacketRX::FifoNotEmpty);
    radio->ClearFifoOverrunFlag();
    radio->SetRssiThreshold(0xA0);
    radio->SetRxTimeoutStart(0);
    radio->SetTimeoutRssiThreshold(40);
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

int frameIndex = 0;
std::vector<uint8_t> currentFrame;
int currentFrameSize = 0;
void loop() {
  if(radio) {
    switch (state) {
      case 0:
        // Send to fifo

        Serial.print("Write packet to FIFO... ");
        currentFrameSize = 64;
        for(uint8_t i = 0; i < currentFrameSize; i++) {
          currentFrame.push_back(outputFrame[i + frameIndex]);
        }
        frameIndex += currentFrameSize;
        radio->TransmitPacket(currentFrame);
        currentFrame.clear();
        radio->SetOcp(false);
        radio->SetOperatingMode(RFM69::Modes::Tx);
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
