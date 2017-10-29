// DataProtocol.h

#ifndef _DATAPROTOCOL_h
#define _DATAPROTOCOL_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <StandardCplusplus.h>
#include <vector>

class DataProtocol {
private:
	std::vector<uint8_t> ArrayManipulator;
	Stream* SerialPort;

	const uint8_t ClinostatExperimentPackageLenght = 0x36;
	const uint8_t GreenHouseExperimentPackageLenght = 0x00;
	const uint8_t ControlCenterExperimentPackageLenght = 0x00;

	union Data16bits {
		int16_t Value;
		uint8_t ByteArray[sizeof(int16_t)];
	};

	union Data32bits {
		int32_t Value;
		uint8_t ByteArray[sizeof(int32_t)];
	};

	union Data64bits {
		int64_t Value;
		uint8_t ByteArray[sizeof(int64_t)];
	};

	union uData16bits {
		uint16_t Value;
		uint8_t ByteArray[sizeof(uint16_t)];
	};

	union uData32bits {
		uint32_t Value;
		uint8_t ByteArray[sizeof(uint32_t)];
	};

	union uData64bits {
		uint64_t Value;
		uint8_t ByteArray[sizeof(uint64_t)];
	};

public:
	struct ConfigData {
		uData16bits ExperimentID;
		uData16bits ExperimentTime;
	};

	struct ExperimentData {

	};

	struct ControlCenterConfigData {
		ConfigData GeneralDataConfig;

	}ControlCenterConfigPackage;

	struct ControlCenterExperimentData {
		ExperimentData GeneralDataExperiment;

	}ControlCenterExperimentPackage;

	struct GreenHoseConfigData {
		ConfigData GeneralDataConfig;

		uData16bits Temperature;
		uData16bits Humidity;
		uint8_t UVLed;
		uint8_t IRLed;
		uint8_t FullSpectrumLed;
		uint8_t RRgb;
		uint8_t GRgb;
		uint8_t BRgb;
		uData32bits Mq2;
		uData32bits Mq7;
	}GreenHouseConfigPackage;

	struct GreenHouseExperimentData {
		ExperimentData GeneralDataExperiment;

	}GreenHouseExperimentPackage;

	struct ClinostatConfigData {
		ConfigData GeneralDataConfig;

		uData16bits ExperimentID;
		uint8_t SupportType;
		uint8_t SampleQuantity;
		uint8_t SampleMass;
		uData16bits ExperimentTime;
		uData32bits MinGravitationalAcceleration;
		uData32bits MaxGravitationalAcceleration;
		uint8_t RpmX;
		uint8_t RpmY;
		uint8_t RpmZ;
	}ClinostatConfigPackage;

	struct ClinostatExperimentData {
		ExperimentData GeneralDataExperiment;

		uData16bits ExperimentID;
		uData32bits ExperimentTime;

		uData16bits Temperature;

		Data16bits AcelerometerX;
		Data16bits AcelerometerY;
		Data16bits AcelerometerZ;

		Data16bits GiroscopeX;
		Data16bits GiroscopeY;
		Data16bits GiroscopeZ;

		Data16bits ResultantAcceleration;
		Data16bits ResultantSpeed;

		Data16bits GravitationalAcceleration;
		Data16bits CentripetalAcceleration;
		Data16bits CentrifugalAcceleration;

		uint8_t RpmX;
		uint8_t RpmY;
		uint8_t RpmZ;

		uint8_t ResultantRotation;
	}ClinostatExperimentPackage;

	struct DataCode {
		const uint8_t ErrorData = 0x00;
		const uint8_t MessageData = 0x01;
		const uint8_t CommandData = 0x02;
		const uint8_t RequestData = 0x03;
		const uint8_t ExperimentId = 0x04;
		const uint8_t SampleSupport = 0x05;
		const uint8_t SampleQuantity = 0x06;
		const uint8_t SampleMass = 0x07;
		const uint8_t ExperimentTime = 0x08;
		const uint8_t MinGForce = 0x09;
		const uint8_t MaxGForce = 0x0A;
		const uint8_t RpmX = 0x0B;
		const uint8_t RpmY = 0x0C;
		const uint8_t RpmZ = 0x0D;
		const uint8_t TemperatureMpu6050 = 0x0E;
		const uint8_t TemperatureDht22 = 0x0F;
		const uint8_t HumidityDht22 = 0x10;
		const uint8_t UvLed = 0x11;
		const uint8_t IrLed = 0x12;
		const uint8_t FullSpectrumLed = 0x13;
		const uint8_t RgbLedR = 0x14;
		const uint8_t RgbLedG = 0x15;
		const uint8_t RgbLedB = 0x16;
		const uint8_t Mq2 = 0x17;
		const uint8_t Mq7 = 0x18;
		const uint8_t AcelerometerX = 0x19;
		const uint8_t AcelerometerY = 0x1A;
		const uint8_t AcelerometerZ = 0x1B;
		const uint8_t GiroscopeX = 0x1C;
		const uint8_t GiroscopeY = 0x1D;
		const uint8_t GiroscopeZ = 0x1E;
		const uint8_t GravitationalForce = 0x1F;
		const uint8_t Ldr1 = 0x20;
		const uint8_t Ldr2 = 0x21;
		const uint8_t Ldr3 = 0x22;
		const uint8_t CentripetalForce = 0x23;
		const uint8_t CentrifugalForce = 0x24;
		const uint8_t ResultantAcceleration = 0x25;
		const uint8_t ResultantSpeed = 0x26;
		const uint8_t ResultantRotation = 0x27;

		const uint8_t CheckSumFail = 0x80;
		const uint8_t AddressNotFound = 0x81;

		const uint8_t ConfigExperimentDataReceived = 0xA0;
		const uint8_t InitExperimentDataReceived = 0xA1;
		const uint8_t PauseExperimentDataReceived = 0xA2;
		const uint8_t CancelExperimentDataReceived = 0xA3;
		const uint8_t ConfigExperimentInformation = 0xA4;
		const uint8_t SerialAlive = 0xA5;

		const uint8_t ParityByte = 0x7F;

		const uint8_t ControlCenterID = 0x00;
		const uint8_t ClinostatID = 0x01;
		const uint8_t GreenHouseID = 0x02;
	}Codes;

	DataProtocol();
	DataProtocol(Stream &_SerialPort);

	void SendData(ControlCenterExperimentData _ControlCenterPackage);
	void SendData(ClinostatExperimentData _ClinostatPackage);
	void SendData(GreenHouseExperimentData _GreenHousePackage);

	void SendSinglePackage(uint8_t _Address, uint8_t *_Payload);

	uint8_t* SinglePayloadMaker(uint8_t _DataCode, uint8_t *_DataArray);
	uint8_t* PackageHeaderMaker(uint8_t _Address, uint8_t _Length);

	void SendWakeUp(uint8_t _Address);

	uint8_t ReceiveConfigData();
	uint8_t ReceiveExperimentData();
	uint8_t CalculeCheckSum(uint8_t *PayLoad, uint8_t PackageLenght);
};
#endif

