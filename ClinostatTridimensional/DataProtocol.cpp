// 
// 
// 

//  __________________________________________________
//	| PRB  | PRB  | PCT  | PCL  | COD  | DAC  | CHK  |
//	| 0x7F | 0x7F |	0x02 | 0x02 | 0x01 | 0xA5 |	0xA6 |
//	-------------------------------------------------  

#include "DataProtocol.h"	
#include <StandardCplusplus.h>
#include <vector>

DataProtocol::DataProtocol() {
}

DataProtocol::DataProtocol(Stream &_SerialPort) {
	SerialPort = &_SerialPort;
}

void DataProtocol::SendData(ControlCenterExperimentData _ControlCenterPackage) {

}

void DataProtocol::SendData(ClinostatExperimentData _ClinostatPackage) {
	uint8_t Count = 0;
	uint8_t *Payload;

	Payload = (uint8_t*)malloc(ClinostatExperimentPackageLenght);

#pragma region ExperimentTime	  
	Payload[Count++] = Codes.ExperimentTime;
	for (int i = (sizeof(_ClinostatPackage.ExperimentTime.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.ExperimentTime.ByteArray[i]);
#pragma endregion
#pragma region AcelerometerX   
	Payload[Count++] = Codes.AcelerometerX;
	for (int i = (sizeof(_ClinostatPackage.AcelerometerX.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.AcelerometerX.ByteArray[i]);
#pragma endregion			
#pragma region AcelerometerY 
	Payload[Count++] = Codes.AcelerometerY;
	for (int i = (sizeof(_ClinostatPackage.AcelerometerY.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.AcelerometerY.ByteArray[i]);
#pragma endregion
#pragma region AcelerometerZ   
	Payload[Count++] = Codes.AcelerometerZ;
	for (int i = (sizeof(_ClinostatPackage.AcelerometerZ.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.AcelerometerZ.ByteArray[i]);
#pragma endregion			
#pragma region GiroscopeX	 
	Payload[Count++] = Codes.GiroscopeX;
	for (int i = (sizeof(_ClinostatPackage.GiroscopeX.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.GiroscopeX.ByteArray[i]);
#pragma endregion			
#pragma region GiroscopeY	  
	Payload[Count++] = Codes.GiroscopeY;
	for (int i = (sizeof(_ClinostatPackage.GiroscopeX.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.GiroscopeY.ByteArray[i]);
#pragma endregion	
#pragma region GiroscopeZ	  
	Payload[Count++] = Codes.GiroscopeZ;
	for (int i = (sizeof(_ClinostatPackage.GiroscopeX.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.GiroscopeZ.ByteArray[i]);
#pragma endregion
#pragma region GravitationalAcceleration	   	  
	Payload[Count++] = Codes.GravitationalForce;
	for (int i = (sizeof(_ClinostatPackage.GravitationalAcceleration.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.GravitationalAcceleration.ByteArray[i]);
#pragma endregion
#pragma region CentripetalAcceleration	   	  
	Payload[Count++] = Codes.CentripetalForce;
	for (int i = (sizeof(_ClinostatPackage.CentripetalAcceleration.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.CentripetalAcceleration.ByteArray[i]);
#pragma endregion
#pragma region CentrifugalAcceleration	   	  
	Payload[Count++] = Codes.CentrifugalForce;
	for (int i = (sizeof(_ClinostatPackage.CentrifugalAcceleration.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.CentrifugalAcceleration.ByteArray[i]);
#pragma endregion				 
#pragma region ResultantAcceleration	   	  
	Payload[Count++] = Codes.ResultantAcceleration;
	for (int i = (sizeof(_ClinostatPackage.ResultantAcceleration.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.ResultantAcceleration.ByteArray[i]);
#pragma endregion	  
#pragma region ResultantSpeed	   	  
	Payload[Count++] = Codes.ResultantSpeed;
	for (int i = (sizeof(_ClinostatPackage.ResultantSpeed.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.ResultantSpeed.ByteArray[i]);
#pragma endregion
#pragma region ResultantRotation			  
	Payload[Count++] = Codes.ResultantRotation;
	Payload[Count++] = (_ClinostatPackage.ResultantRotation);
#pragma endregion	
#pragma region Temperature	  
	Payload[Count++] = Codes.TemperatureMpu6050;
	for (int i = (sizeof(_ClinostatPackage.Temperature.Value) - 1); i >= 0; i--)
		Payload[Count++] = (_ClinostatPackage.Temperature.ByteArray[i]);
#pragma endregion
#pragma region RpmX			  
	Payload[Count++] = Codes.RpmX;
	Payload[Count++] = (_ClinostatPackage.RpmX);
#pragma endregion
#pragma region RpmY		   
	Payload[Count++] = Codes.RpmY;
	Payload[Count++] = (_ClinostatPackage.RpmY);
#pragma endregion
#pragma region RpmZ		
	Payload[Count++] = Codes.RpmZ;
	Payload[Count++] = (_ClinostatPackage.RpmZ);
#pragma endregion

	SerialPort->write(PackageHeaderMaker(Codes.ClinostatID, ClinostatExperimentPackageLenght), 0x04);
	SerialPort->write(Payload, sizeof(Payload));
	SerialPort->write(CalculeCheckSum(Payload, sizeof(Payload)));
}

void DataProtocol::SendData(GreenHouseExperimentData _GreenHousePackage) {

}

uint8_t DataProtocol::ReceiveConfigData() {
	if (SerialPort->available() > 0) {
		SerialPort->println(SerialPort->readString());
	}
}

/**
Return a complete package with only one data, this data must be contains
DataCode and Data
*/
void DataProtocol::SendSinglePackage(uint8_t _Address, uint8_t *_Payload){		
	SerialPort->write(PackageHeaderMaker(_Address, sizeof(_Payload)), 0x04);
	SerialPort->write(_Payload, sizeof(_Payload));
	SerialPort->write(CalculeCheckSum(_Payload, sizeof(_Payload)));
}

/**
Returns a single data payload, this payload contains DataCode and Data
*/
uint8_t* DataProtocol::SinglePayloadMaker(uint8_t _DataCode, uint8_t *_DataArray) {
	uint8_t *Payload;

	Payload = (uint8_t*)malloc(sizeof(_DataArray) + 1);
	Payload[0] = _DataCode;

	for (int i = 1; i < sizeof(_DataArray); i++) {
		Payload[i] = _DataArray[i - 1];
	}

	return Payload;
}

uint8_t* DataProtocol::PackageHeaderMaker(uint8_t _Address, uint8_t _Length) {
	uint8_t Header[4];

	Header[0] = Codes.ParityByte;
	Header[1] = Codes.ParityByte;
	Header[2] = _Address;
	Header[3] = _Length;

	return Header;
}

/**
Return a complete message package
*/
void DataProtocol::SendWakeUp(uint8_t _Address) {
	SerialPort->write(PackageHeaderMaker(_Address, 0x02), 0x04);
	//DataCode
	SerialPort->write(Codes.MessageData);
	//Data
	SerialPort->write(Codes.SerialAlive);
	//CheckSum
	SerialPort->write(0xA6);
}

uint8_t DataProtocol::ReceiveExperimentData() {

}

uint8_t DataProtocol::CalculeCheckSum(uint8_t *PayLoad, uint8_t PackageLenght) {

}

