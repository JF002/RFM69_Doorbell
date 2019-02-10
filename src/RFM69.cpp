#include "RFM69.h"
#include <Arduino.h>
#include <SPI.h>

using namespace Codingfield::Communication;

RFM69::RFM69(SPIClass *spi, uint8_t selectPin) : spi{spi}, selectPin{selectPin} {

}


void RFM69::Select() {
  digitalWrite(15, LOW);
}

void RFM69::Unselect() {
  digitalWrite(15, HIGH);
}

void RFM69::WriteRegister(uint8_t addr, uint8_t value) {
  Select();
  spi->transfer(0x80 | addr);
  spi->transfer(value);
  Unselect();
}

uint8_t RFM69::ReadRegister(uint8_t addr) {
  Select();
  spi->transfer(0x7f & addr);
  uint8_t value =  spi->transfer(0);
  Unselect();
  return value;
}

void RFM69::WriteRegisterNoSelect(uint8_t addr, uint8_t value) {
  spi->transfer(0x80 | addr);
  spi->transfer(value);
}

uint8_t RFM69::ReadRegisterNoSelect(uint8_t addr) {
  spi->transfer(0x7f & addr);
  uint8_t value =  spi->transfer(0);
  return value;
}

bool RFM69::IsRfm69() {
  return ReadRegister(static_cast<uint8_t>(Registers::Version)) == 0x24;
}

uint8_t RFM69::ReadTemperature() {
  WriteRegister(static_cast<uint8_t>(Registers::Temp1),
                static_cast<uint8_t>(TemperatureMeasurementValues::TemperatureMeasurementStart));

  bool ready = false;
  do {
    ready = !IsBitSet(ReadRegister(static_cast<uint8_t>(Registers::Temp1)),
                      static_cast<uint8_t>(TemperatureMeasurementValues::TemperatureMeasurementRunning));
    delay(1);
  }while(!ready);

  return ReadRegister(static_cast<uint8_t>(Registers::Temp2));
}

bool RFM69::IsBitSet(uint8_t value, uint8_t bit) {
  return (value & bit) == bit;
}

void RFM69::SetOperatingMode(RFM69::Modes mode) {
  WriteRegister(static_cast<uint8_t>(Registers::OperatingMode), static_cast<uint8_t>(mode));

  bool ready = false;
  while(!ready) {
    ready = IsBitSet(ReadRegister(static_cast<uint8_t>(Registers::IrqFlags1)),
                     static_cast<uint8_t>(IrqFlags1Values::ModeReady));
  }
}

void
RFM69::SetDataModulation(const ModulationShapings modulationShaping, const ModulationTypes modulationType,
                         const DataModes dataMode) {
  auto value = static_cast<uint8_t>(modulationShaping) | static_cast<uint8_t>(modulationType) | static_cast<uint8_t>(dataMode);
  WriteRegister(static_cast<uint8_t>(Registers::DataModulation), value);
}

void RFM69::SetBitRate(uint16_t bitrate) {
  auto msb = static_cast<uint8_t>((bitrate >> 8) & 0x00FF);
  auto lsb = static_cast<uint8_t>(bitrate & 0x00FF);
  WriteRegister(static_cast<uint8_t>(Registers::BitRate_MSB), msb);
  WriteRegister(static_cast<uint8_t>(Registers::BitRate_LSB), lsb);
}

void RFM69::SetFrequencyDeviation(uint16_t deviation) {
  auto msb = static_cast<uint8_t>((deviation >> 8) & 0x00FF);
  auto lsb = static_cast<uint8_t>(deviation & 0x00FF);
  WriteRegister(static_cast<uint8_t>(Registers::FrequencyDeviation_MSB), msb);
  WriteRegister(static_cast<uint8_t>(Registers::FrequencyDeviation_LSB), lsb);
}

void RFM69::SetFrequency(uint32_t freq) {
  auto msb = static_cast<uint8_t>((freq >> 16) & 0x000000FF);
  auto mid = static_cast<uint8_t>((freq >> 8) & 0x000000FF);
  auto lsb = static_cast<uint8_t>(freq & 0x000000FF);
  WriteRegister(static_cast<uint8_t>(Registers::Frequency_MSB), msb);
  WriteRegister(static_cast<uint8_t>(Registers::Frequency_MID), mid);
  WriteRegister(static_cast<uint8_t>(Registers::Frequency_LSB), lsb);
}

void RFM69::SetDio0Mapping(RFM69::DIO0MappingPacketRX mapping) {
  auto regValue = ReadRegister(static_cast<uint8_t>(Registers::DIOMapping1));
  regValue &= static_cast<uint8_t>(RFM69::DIO0MappingPacketRX::Mask);
  regValue |= static_cast<uint8_t>(mapping);
  WriteRegister(static_cast<uint8_t>(Registers::DIOMapping1), regValue);
}

void RFM69::SetDio0Mapping(RFM69::DIO0MappingPacketTX mapping) {
  auto regValue = ReadRegister(static_cast<uint8_t>(Registers::DIOMapping1));
  regValue &= static_cast<uint8_t>(RFM69::DIO0MappingPacketTX::Mask);
  regValue |= static_cast<uint8_t>(mapping);
  WriteRegister(static_cast<uint8_t>(Registers::DIOMapping1), regValue);
}

void RFM69::SetDio1Mapping(RFM69::DIO1MappingContinuousRX mapping) {
  auto regValue = ReadRegister(static_cast<uint8_t>(Registers::DIOMapping1));
  regValue &= static_cast<uint8_t>(RFM69::DIO1MappingContinuousRX::Mask);
  regValue |= static_cast<uint8_t>(mapping);
  WriteRegister(static_cast<uint8_t>(Registers::DIOMapping1), regValue);
}


void RFM69::SetDio2Mapping(RFM69::DIO2MappingPacketRX mapping) {
  auto regValue = ReadRegister(static_cast<uint8_t>(Registers::DIOMapping1));
  regValue &= static_cast<uint8_t>(RFM69::DIO2MappingPacketRX::Mask);
  regValue |= static_cast<uint8_t>(mapping);
  WriteRegister(static_cast<uint8_t>(Registers::DIOMapping1), regValue);
}

void RFM69::ClearFifoOverrunFlag() {
  WriteRegister(static_cast<uint8_t>(Registers::IrqFlags2), ClearFifoOverrunFlagValue);
}

void RFM69::SetRssiThreshold(uint8_t threshold) {
  WriteRegister(static_cast<uint8_t>(Registers::RssiThreshold), threshold);
}

void RFM69::SetRxTimeoutStart(uint8_t value) {
  WriteRegister(static_cast<uint8_t>(Registers::RxTimeout1), value);
}

void RFM69::SetTimeoutRssiThreshold(uint8_t value) {
  WriteRegister(static_cast<uint8_t>(Registers::RxTimeout2), value);
}

void RFM69::SetSyncWordConfig(uint8_t bitTolerance, uint8_t syncWordSize, RFM69::FifoFillConditions condition,
                              bool enableSyncWord) {
  auto tolerance = static_cast<uint8_t>(bitTolerance & 0x03);
  auto size = static_cast<uint8_t>(((syncWordSize-1) & 0x03) << 3);
  auto enable = static_cast<uint8_t>(enableSyncWord ? (1 << 7) : 0);
  auto c = static_cast<uint8_t>(condition);

  WriteRegister(static_cast<uint8_t>(Registers::SyncConfig), tolerance | size | c | enable);
}

void RFM69::SetSyncWordValue1(uint8_t value) {
  WriteRegister(static_cast<uint8_t>(Registers::SyncValue1), value);
}

void RFM69::SetSyncWordValue2(uint8_t value) {
  WriteRegister(static_cast<uint8_t>(Registers::SyncValue2), value);
}

void RFM69::SetSyncWordValue3(uint8_t value) {
  WriteRegister(static_cast<uint8_t>(Registers::SyncValue3), value);
}

void RFM69::SetSyncWordValue4(uint8_t value) {
  WriteRegister(static_cast<uint8_t>(Registers::SyncValue4), value);
}



void RFM69::SetPacketConfig(RFM69::AddressFilterings addressFiltering, bool crcAutoClearOff, bool enableCrc,
                            RFM69::DcFreeTypes dcFreeType, RFM69::PacketFormats format) {
  auto filtering = static_cast<uint8_t>(addressFiltering);
  auto crc = static_cast<uint8_t>(crcAutoClearOff ? (1 << 3) : 0);
  auto enable = static_cast<uint8_t>(enableCrc ? (1 << 4) : 0);
  auto dcFree = static_cast<uint8_t>(dcFreeType);
  auto f = static_cast<uint8_t>(format);
  WriteRegister(static_cast<uint8_t>(Registers::PacketConfig), filtering | crc | enable | dcFree | f);
}

void RFM69::SetPayloadLength(uint8_t length) {
  WriteRegister(static_cast<uint8_t>(Registers::PayloadLength), length);
}

bool RFM69::IsIrqFlagSet(RFM69::IrqFlags1 flag) {
  return IsBitSet(ReadRegister(static_cast<uint8_t>(Registers::IrqFlags1)), static_cast<uint8_t>(flag));
}

bool RFM69::IsIrqFlagSet(RFM69::IrqFlags2 flag) {
  return IsBitSet(ReadRegister(static_cast<uint8_t>(Registers::IrqFlags2)), static_cast<uint8_t>(flag));
}


void RFM69::RestartRx() {
  auto packetconfig2 = ReadRegister(static_cast<uint8_t>(Registers::PacketConfig2));
  WriteRegister(static_cast<uint8_t>(Registers::PacketConfig2), packetconfig2 | ForceRestartRx);
}

void RFM69::TransmitPacket(std::vector<uint8_t> message) {
  Select();
  spi->transfer(0x00 | 0x80);
  for(auto b : message) {
    spi->transfer(b);
  }
  Unselect();
}

void RFM69::SetFifoThreshold(uint8_t threshold, RFM69::TxStartCondition startCondition) {
  auto t = static_cast<uint8_t>(threshold & 0x3F);
  auto condition = static_cast<uint8_t>(startCondition);
  WriteRegister(static_cast<uint8_t>(Registers::FifoThreshold), t | condition);
}

void RFM69::SetPreambleSize(uint16_t size) {
  WriteRegister(static_cast<uint8_t>(Registers::Preamble_MSB), static_cast<uint8_t>((size >> 8) & 0x00FF)),
  WriteRegister(static_cast<uint8_t>(Registers::Preamble_LSB), static_cast<uint8_t>(size& 0x00FF));
}

void RFM69::SetOcp(bool enabled) {
  WriteRegister(static_cast<uint8_t>(Registers::Ocp), static_cast<uint8_t>(enabled ? 0x1A : 0x0F));
  WriteRegister(static_cast<uint8_t>(0x11), static_cast<uint8_t>(0xff));
}



