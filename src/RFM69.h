#pragma once


#include <stdint.h>
#include <vector>
#include <Arduino.h>


class SPIClass;

namespace Codingfield {
  namespace Communication {

    class RFM69 {
      public:

      enum class Registers : uint8_t {
        OperatingMode = 0x01,
        DataModulation = 0x02,
        BitRate_MSB = 0x03,
        BitRate_LSB = 0x04,
        FrequencyDeviation_MSB = 0x05,
        FrequencyDeviation_LSB = 0x06,
        Frequency_MSB = 0x07,
        Frequency_MID = 0x08,
        Frequency_LSB = 0x09,
        Version = 0x10,
        Ocp = 0x13,
        DIOMapping1 = 0x25,
        IrqFlags1 = 0x27,
        IrqFlags2 = 0x28,
        RssiThreshold = 0x29,
        RxTimeout1 = 0x2A,
        RxTimeout2 = 0x2B,
        Preamble_MSB = 0x2C,
        Preamble_LSB = 0x2D,
        SyncConfig = 0x2E,
        SyncValue1 = 0x2F,
        SyncValue2 = 0x30,
        SyncValue3 = 0x31,
        SyncValue4 = 0x32,
        PacketConfig = 0x37,
        PayloadLength = 0x38,
        FifoThreshold = 0x3C,
        PacketConfig2 = 0x3D,
        Temp1 = 0x4E,
        Temp2 = 0x4F,
      };

      enum class Modes : uint8_t {
        Sleep = 0,
        Standby = 1 << 2,
        FrequencySynth = 2 << 2,
        Tx = 3 << 2,
        Rx = 4 << 2
      };

      enum class ModulationShapings {
        NoShaping = 0,
        FSK_GaussianFilter_1_0 = 0x01,
        FSK_GaussianFilter_0_5 = 0x02,
        FSK_GaussianFilter_0_3 = 0x03,

        OOK_FilteringCutOff_BR = 0x01,
        OOK_FilteringCutOff_2BR = 0x02,
      };

      enum class ModulationTypes {
        FSK = 0,
        OOK = 1 << 3
      };

      enum class DataModes {
        Packet = 0,
        ContinuousBitSync = 2 << 5,
        Continuous = 3 << 5
      };

      enum class DIO0MappingPacketRX {
        CrcOk = 0,
        PayloadReady = 1 << 6,
        SyncAddress = 2 << 6,
        Rssi = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO1MappingContinuousRX {
        Dclk = 0,
        RxReady = 0x01 << 4,
        SyncAddress = 0x03 << 4,
        Mask = 0x30
      };

      enum class DIO0MappingPacketTX {
        PacketSent = 0,
        TxReady = 1 << 6,
        PllLock = 3 << 6,
        Mask = 0xC0
      };

      enum class DIO2MappingPacketRX {
        FifoNotEmpty = 0,
        Data = 1 << 2,
        AutoMode = 3 << 2,
        Mask = 0x0C
      };

      enum class IrqFlags1Values {
        ModeReady = 1<<7
      };

      enum class TemperatureMeasurementValues {
        TemperatureMeasurementStart = 1 << 3,
        TemperatureMeasurementRunning = 1 << 2
      };

      enum class FifoFillConditions {
        IfSyncAddressInterruptOccurs = 0,
        AsLongAsFifoFillConditionIsSet = 1 << 6
      };

      enum class AddressFilterings {
        None = 0,
        AddressMustMatchNodeAddress = 1 << 1,
        AddressMustMatchNodeAddressAndBroadcast = 2 << 1
      };

      enum class DcFreeTypes {
        None = 0,
        Manchester = 1 << 5,
        Whitening = 2 << 5
      };

      enum class PacketFormats {
        FixedLength = 0,
        VariableLength = 1 << 7
      };

      enum class IrqFlags1 {
        SyncAddressMatch = 1,
        AutoMode = 1 << 1,
        Timeout = 1 << 2,
        Rssi = 1 << 3,
        PllLock = 1 << 4,
        TxReady = 1 << 5,
        RxReady = 1 << 6,
        ModeReady = 1 << 7
      };

      enum class IrqFlags2 {
        CrcOk = 1 << 1,
        PayloadReady = 1 << 2,
        PacketSent = 1 << 3,
        FifoOverrun = 1 << 4,
        FifoLevel = 1 << 5,
        FifoNotEmpty = 1 << 6,
        FifoFull = 1 << 7
      };


      enum class TxStartCondition {
        FifoLevel = 0x00,
        FifoNotEmpty = 1<<7
      };

      static constexpr uint8_t ClearFifoOverrunFlagValue = 1<<4;
      static constexpr uint8_t ForceRestartRx = 1<<2;

      RFM69(SPIClass *spi, uint8_t selectPin);

      void WriteRegister(uint8_t addr, uint8_t value);
      uint8_t ReadRegister(uint8_t addr);

      void WriteRegisterNoSelect(uint8_t addr, uint8_t value);
      uint8_t ReadRegisterNoSelect(uint8_t addr);

      bool IsRfm69();
      uint8_t ReadTemperature();
      void SetOperatingMode(Modes mode);
      void SetDataModulation(ModulationShapings modulationShaping, ModulationTypes modulationType, DataModes dataMode);
      void SetBitRate(uint16_t bitrate);
      void SetFrequencyDeviation(uint16_t deviation);
      void SetFrequency(uint32_t freq);
      void SetDio0Mapping(DIO0MappingPacketRX mapping);
      void SetDio0Mapping(DIO0MappingPacketTX mapping);
      void SetDio1Mapping(DIO1MappingContinuousRX mapping);
      void SetDio2Mapping(DIO2MappingPacketRX mapping);
      void ClearFifoOverrunFlag();
      void SetRssiThreshold(uint8_t threshold);
      void SetRxTimeoutStart(uint8_t value);
      void SetTimeoutRssiThreshold(uint8_t value);
      void SetSyncWordConfig(uint8_t bitTolerance, uint8_t syncWordSize, FifoFillConditions condition,
                             bool enableSyncWord);
      void SetSyncWordValue1(uint8_t value);
      void SetSyncWordValue2(uint8_t value);
      void SetSyncWordValue3(uint8_t value);
      void SetSyncWordValue4(uint8_t value);
      void SetPacketConfig(AddressFilterings addressFiltering, bool crcAutoClearOff, bool enableCrc, DcFreeTypes dcFreeType, PacketFormats format);
      void SetPayloadLength(uint8_t length);
      bool IsIrqFlagSet(IrqFlags1 flag);
      bool IsIrqFlagSet(IrqFlags2 flag);
      void RestartRx();
      void SetFifoThreshold(uint8_t threshold, TxStartCondition startCondition);
      void SetPreambleSize(uint16_t size);
      void SetOcp(bool enabled);

      void TransmitPacket(std::vector<uint8_t> message);

      private:
      SPIClass *spi = nullptr;
      uint8_t selectPin;

      void Select();
      void Unselect();
      bool IsBitSet(uint8_t value, uint8_t bit);

    };
  }
}