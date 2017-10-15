/*
 Name:		ClinostatTridimensional.ino
 Created:	9/4/2017 9:52:16 PM
 Author:	CrismanSantos
*/

#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Thread.h>
#include <ThreadController.h>	
#include <math.h>

#pragma region Atributes   
const byte MotorX1Port = 0x03;
const byte MotorX2Port = 0x05;

const byte MotorY1Port = 0x03;
const byte MotorY2Port = 0x05;

const byte MotorZ1Port = 0x03;
const byte MotorZ2Port = 0x05;

const byte pwmMotorXPort = 0x06;
const byte pwmMotorYPort = 0x06;
const byte pwmMotorZPort = 0x06;

const int MPUAddress = 0x68;

double MotorXSetPoint = 0;
double MotorXInput = 0;
double MotorXOutput = 0; 

double MotorYSetPoint = 0;
double MotorYInput = 0;
double MotorYOutput = 0;

double MotorZSetPoint = 0;
double MotorZInput = 0;
double MotorZOutput = 0;

float RadiusSampleCircunference = 0.01f;
double EarthGravitation = 9.80665f;

unsigned long TimerPID = 0;

union Data16bits {
	int16_t Value;
	byte ByteArray[sizeof(int16_t)];
};

union Data32bits {
	int32_t Value;
	byte ByteArray[sizeof(int32_t)];
};

union Data64bits {
	int64_t Value;
	byte ByteArray[sizeof(int64_t)];
};

union uData16bits {
	uint16_t Value;
	byte ByteArray[sizeof(uint16_t)];
};

union uData32bits {
	uint32_t Value;
	byte ByteArray[sizeof(uint32_t)];
};

union uData64bits {
	uint64_t Value;
	byte ByteArray[sizeof(uint64_t)];
};

struct ConfigData {
	uData16bits ExperimentID;
	byte SupportType;
	byte SampleQuantity;
	byte SampleMass;
	uData16bits ExperimentTime;
	uData32bits MinGravitationalAcceleration;
	uData32bits MaxGravitationalAcceleration;
	byte RpmX;
	byte RpmY;
	byte RpmZ;
}ConfigPackage;

struct ExperimentData {
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

	byte RpmX;
	byte RpmY;
	byte RpmZ;

	byte ResultantRotation;
}ExperimentPackage;

struct Vectors {
	double X;
	double Y;
	double Z;
	double Result;
};

struct PhysicsData {
	Vectors Acceleration;
	Vectors AngularSpeed;
	Vectors GravitationalAcceleration;
	double ResultantRotation;
	double CentripetalAcceleration;
	double CentrifugalAcceleration;
}PhysicsValues;

struct MotorSpeed {
	byte MotorX;
	byte MotorY;
	byte MotorZ;
}Speed;

struct Weight {
	double Kp;
	double Ki;
	double kd;
}TuningsMotorX, TuningsMotorY, TuningsMotorZ;

struct ByteLimits {
	byte Min;
	byte Max;
}MotorXSpeedLimits, MotorYSpeedLimits, MotorZSpeedLimits;

enum MotorDirections {
	FORWARD = 0,
	BACKWARD = 1,
	STOP = 2,
	BREAK = 3
};

enum Motor {
	MotorX = 0,
	MotorY = 1,
	MotorZ = 1
};

PID PIDMotorX(&MotorXInput, &MotorXOutput, &MotorXSetPoint, TuningsMotorX.Kp, TuningsMotorX.Ki, TuningsMotorX.kd, DIRECT);
PID PIDMotorY(&MotorYInput, &MotorYOutput, &MotorYSetPoint, TuningsMotorY.Kp, TuningsMotorY.Ki, TuningsMotorY.kd, DIRECT);
PID PIDMotorZ(&MotorZInput, &MotorZOutput, &MotorZSetPoint, TuningsMotorZ.Kp, TuningsMotorZ.Ki, TuningsMotorZ.kd, DIRECT);

Thread SerialReader;
Thread SerialWriter;
Thread SerialAlive;
Thread DebugWriter;

Thread SensorReader;

Thread MotorXWriter;
Thread MotorYWriter;
Thread MotorZWriter;

ThreadController MotorManager;
ThreadController SensorManager;
ThreadController SerialManager;
ThreadController TaskManager;
#pragma endregion

void MotorDirection(const byte Direction, const  byte PortA, const  byte PortB) {
	switch (Direction) {
	case 0: {
		digitalWrite(PortA, HIGH);
		digitalWrite(PortB, LOW);
	}	break;
	case 1: {
		digitalWrite(PortA, LOW);
		digitalWrite(PortB, HIGH);
	}	break;
	case 2: {
		digitalWrite(PortA, LOW);
		digitalWrite(PortB, LOW);
	}	break;
	case 3: {
		digitalWrite(PortA, HIGH);
		digitalWrite(PortB, HIGH);
	}	break;
	default:
		break;
	}
}

void MotorRun(const byte Motor, const  byte Direction) {
	switch (Motor) {
	case 0: {
		MotorDirection(Direction, MotorX1Port, MotorX2Port);
	}	break;
	case 1: {
		MotorDirection(Direction, MotorY1Port, MotorY2Port);
	}	break;
	case 2: {
		MotorDirection(Direction, MotorZ1Port, MotorX2Port);
	}	break;
	case 3: {

	}	break;
	default:
		break;
	}
}

byte CalculeCheckSum(byte *PayLoad, byte PackageLenght) {
	uData32bits CheckSum;

	CheckSum.Value = 0;

	for (int i = 0; i < PackageLenght; i++)
		CheckSum.Value += PayLoad[i];

	return (byte)CheckSum.ByteArray[0];
}

void DefinePIDLimits() {
	MotorXSpeedLimits.Min = 0xA0;
	MotorXSpeedLimits.Max = 0xFF;

	MotorYSpeedLimits.Min = 0xA0;
	MotorYSpeedLimits.Max = 0xFF;

	MotorZSpeedLimits.Min = 0xA0;
	MotorZSpeedLimits.Max = 0xFF;
}

void DefinePIDSetPoint() {
	MotorXSetPoint = ConfigPackage.MinGravitationalAcceleration.Value;
	MotorYSetPoint = ConfigPackage.MinGravitationalAcceleration.Value;
	MotorZSetPoint = ConfigPackage.MinGravitationalAcceleration.Value;
}

void ConfigurePID() {
	DefinePIDLimits();
	DefinePIDSetPoint();

	PIDMotorX.SetOutputLimits(MotorXSpeedLimits.Min, MotorXSpeedLimits.Max);
	PIDMotorY.SetOutputLimits(MotorYSpeedLimits.Min, MotorYSpeedLimits.Max);
	PIDMotorZ.SetOutputLimits(MotorZSpeedLimits.Min, MotorZSpeedLimits.Max);

	PIDMotorX.SetMode(AUTOMATIC);
	PIDMotorY.SetMode(AUTOMATIC);
	PIDMotorZ.SetMode(AUTOMATIC);
}

void PIDCompute(int ElapsedTime) {
	MotorXInput = PhysicsValues.GravitationalAcceleration.X;
	PIDMotorX.SetSampleTime(ElapsedTime);
	PIDMotorX.Compute();

	MotorYInput = PhysicsValues.GravitationalAcceleration.Y;
	PIDMotorY.SetSampleTime(ElapsedTime);
	PIDMotorY.Compute();

	MotorZInput = PhysicsValues.GravitationalAcceleration.Z;
	PIDMotorZ.SetSampleTime(ElapsedTime);
	PIDMotorZ.Compute();
}

void setCircunferenceRadius() {
	switch (ConfigPackage.SupportType) {
	case 0: {
		RadiusSampleCircunference = 80 / 1000.0f;
	}	break;
	case 1: {
		RadiusSampleCircunference = 80 / 1000.0f;
	}	break;
	case 2: {
		RadiusSampleCircunference = 80 / 1000.0f;
	}	break;
	default:
		break;
	}
}

#pragma region ThreadMethods 
void SensorRead() {		
	TimerPID = millis() - TimerPID;

	Wire.beginTransmission(MPUAddress);
	Wire.write(0x3B);
	Wire.endTransmission(false);

	Wire.requestFrom(MPUAddress, 14, true);

	if(Wire.available() >= 14){
		ExperimentPackage.AcelerometerX.Value = Wire.read() << 8 | Wire.read();
		ExperimentPackage.AcelerometerY.Value = Wire.read() << 8 | Wire.read();
		ExperimentPackage.AcelerometerZ.Value = Wire.read() << 8 | Wire.read();
		ExperimentPackage.Temperature.Value = Wire.read() << 8 | Wire.read();
		ExperimentPackage.GiroscopeX.Value = Wire.read() << 8 | Wire.read();
		ExperimentPackage.GiroscopeY.Value = Wire.read() << 8 | Wire.read();
		ExperimentPackage.GiroscopeZ.Value = Wire.read() << 8 | Wire.read();

		ExperimentPackage.GiroscopeX.Value += 113;
		ExperimentPackage.GiroscopeY.Value += 77;
	} else {
		Wire.endTransmission(true);
		return;
	}

	PhysicsValues.Acceleration.X = ExperimentPackage.AcelerometerX.Value * ((2 * EarthGravitation) / 32768);
	PhysicsValues.Acceleration.Y = ExperimentPackage.AcelerometerY.Value * ((2 * EarthGravitation) / 32768);
	PhysicsValues.Acceleration.Z = ExperimentPackage.AcelerometerZ.Value * ((2 * EarthGravitation) / 32768);

	PhysicsValues.Acceleration.Result = sqrt((pow(PhysicsValues.Acceleration.X, 2)) + (pow(PhysicsValues.Acceleration.Y, 2)) + (pow(PhysicsValues.Acceleration.Z, 2)));	
	ExperimentPackage.ResultantAcceleration.Value = lround(PhysicsValues.Acceleration.Result / ((2 * EarthGravitation) / 32768));

	PhysicsValues.AngularSpeed.X = ExperimentPackage.GiroscopeX.Value * ((250.0f) / 32768);
	PhysicsValues.AngularSpeed.Y = ExperimentPackage.GiroscopeY.Value * ((250.0f) / 32768);
	PhysicsValues.AngularSpeed.Z = ExperimentPackage.GiroscopeZ.Value * ((250.0f) / 32768);

	PhysicsValues.AngularSpeed.Result = sqrt(pow(PhysicsValues.AngularSpeed.X, 2) + pow(PhysicsValues.AngularSpeed.Y, 2) + pow(PhysicsValues.AngularSpeed.Z, 2));
	ExperimentPackage.ResultantSpeed.Value = lround(PhysicsValues.AngularSpeed.Result / ((250.0f) / 32768));

	PhysicsValues.CentripetalAcceleration = (ConfigPackage.SampleMass * pow(PhysicsValues.AngularSpeed.Result, 2) * RadiusSampleCircunference);
	PhysicsValues.CentrifugalAcceleration = (ConfigPackage.SampleMass * pow(PhysicsValues.AngularSpeed.Result, 2) * RadiusSampleCircunference);

	PhysicsValues.GravitationalAcceleration.X = ((pow(PhysicsValues.AngularSpeed.X, 2) * RadiusSampleCircunference) / EarthGravitation);
	PhysicsValues.GravitationalAcceleration.Y = ((pow(PhysicsValues.AngularSpeed.Y, 2) * RadiusSampleCircunference) / EarthGravitation);
	PhysicsValues.GravitationalAcceleration.Z = ((pow(PhysicsValues.AngularSpeed.Z, 2) * RadiusSampleCircunference) / EarthGravitation);
	PhysicsValues.GravitationalAcceleration.Result = ((pow(PhysicsValues.AngularSpeed.Result, 2) * RadiusSampleCircunference) / EarthGravitation);
																					
	ExperimentPackage.CentripetalAcceleration.Value = lround(PhysicsValues.CentripetalAcceleration / ((2 * EarthGravitation) / 32768));
	ExperimentPackage.CentrifugalAcceleration.Value = lround(PhysicsValues.CentrifugalAcceleration / ((2 * EarthGravitation) / 32768));	   
	ExperimentPackage.GravitationalAcceleration.Value = lround(PhysicsValues.GravitationalAcceleration.Result / ((2 * EarthGravitation) / 32768));

	PIDCompute((int)TimerPID);
}
			 
void DebugWrite(){
	/*Serial.println("\n-- Acelerometer values");
	Serial.print(" | xGAraw->");
	Serial.print(ExperimentPackage.AcelerometerX.Value);
	Serial.print(" | yGAraw->");
	Serial.print(ExperimentPackage.AcelerometerY.Value);
	Serial.print(" | zGAraw->");
	Serial.print(ExperimentPackage.AcelerometerZ.Value);
	Serial.println("");*/

	//Serial.println("\n-- Giroscope values");
	Serial.print(" | xASraw->");
	Serial.print(ExperimentPackage.GiroscopeX.Value);
	Serial.print(" | yASraw->");
	Serial.print(ExperimentPackage.GiroscopeY.Value);
	Serial.print(" | zASraw->");
	Serial.print(ExperimentPackage.GiroscopeZ.Value);
	Serial.println("");

	//Serial.println("\n-- Resultant values");
	//Serial.print(" | rGAraw->");
	//Serial.print(ExperimentPackage.ResultantAcceleration.Value);
	//Serial.print(" | rASraw->");
	//Serial.print(ExperimentPackage.ResultantSpeed.Value);
	//Serial.print(" | rGRraw->");
	//Serial.print(ExperimentPackage.GravitationalAcceleration.Value);
	//Serial.println("");

	//Serial.print(" | xGA->");
	//Serial.print(abs(PhysicsValues.Acceleration.X), 2);
	//Serial.print(" | yGA->");
	//Serial.print(abs(PhysicsValues.Acceleration.Y),2);
	//Serial.print(" | zGA->");
	//Serial.print(abs(PhysicsValues.Acceleration.Z),2);
	//Serial.print(" | rGA->");
	//Serial.print(PhysicsValues.Acceleration.Result, 6);
	//Serial.print(" || xAS->");
	//Serial.print(abs(PhysicsValues.AngularSpeed.X),2);
	//Serial.print(" | yAS->");
	//Serial.print(abs(PhysicsValues.AngularSpeed.Y),2);
	//Serial.print(" | zAS->");
	//Serial.print(abs(PhysicsValues.AngularSpeed.Z),2);
	//Serial.print(" | rAS->");
	//Serial.print(PhysicsValues.AngularSpeed.Result, 6);
	//Serial.print(" || CpA->");
	//Serial.print(PhysicsValues.CentripetalAcceleration, 6);
	//Serial.print(" || CfA->");
	//Serial.print(PhysicsValues.CentrifugalAcceleration, 6);
	//Serial.print(" || GtA->");
	//Serial.print(PhysicsValues.GravitationalAcceleration,6);
	//Serial.println("");

}

void MotorXWrite() { 
	Speed.MotorX = MotorXOutput;
	analogWrite(pwmMotorXPort, Speed.MotorX);
	ExperimentPackage.RpmX = map(Speed.MotorX, MotorXSpeedLimits.Min, MotorXSpeedLimits.Max, 0x00, 0xFF);

	ExperimentPackage.ResultantRotation =  (byte)lround(sqrt(pow(0, 2) + pow(ExperimentPackage.RpmX, 2) + pow(0, 2)));
}
void MotorYWrite() {
	Speed.MotorY = MotorYOutput;
	analogWrite(pwmMotorYPort, Speed.MotorY);
	ExperimentPackage.RpmY = map(Speed.MotorY, MotorYSpeedLimits.Min, MotorYSpeedLimits.Max, 0x00, 0xFF);

	ExperimentPackage.ResultantRotation = (byte)lround(sqrt(pow(0, 2) + pow(ExperimentPackage.RpmY, 2) + pow(0, 2)));
}
void MotorZWrite() {
	Speed.MotorZ = MotorZOutput;
	analogWrite(pwmMotorZPort, Speed.MotorZ);
	ExperimentPackage.RpmZ = map(Speed.MotorZ, MotorZSpeedLimits.Min, MotorZSpeedLimits.Max, 0x00, 0xFF);

	ExperimentPackage.ResultantRotation = (byte)lround(sqrt(pow(0, 2) + pow(ExperimentPackage.RpmZ, 2) + pow(0, 2)));
}

void SerialRead() {					  
	byte PackageType;
	byte PackageLenght;
	byte *PayLoad;

	byte Contador = 0;

	if (Serial.available() >= 4) {
		if ((Serial.read() == 0x7F) && (Serial.read() == 0x7F)) {
			PackageType = Serial.read();
			PackageLenght = Serial.read();

			/*Serial.print("PackageType -> 0x");
			Serial.println(PackageType, HEX);	   
			Serial.print("PackageLenght -> 0x");
			Serial.println(PackageLenght, HEX);*/

			while (Serial.available() < PackageLenght);

			PayLoad = (byte *)malloc(PackageLenght);

			Serial.readBytes(PayLoad, PackageLenght);	  
			
			/*Serial.print("ReceivedPayLoad -> ");
			Serial.write(PayLoad, PackageLenght);
			Serial.println("");*/


			byte CheckSum = CalculeCheckSum(PayLoad, PackageLenght);
			//Serial.print("CheckSum -> 0x");
			//Serial.write(CheckSum);

			if (Serial.read() == CheckSum) {
				//Serial.println("CheckSum -> Okay");
				if (PackageType == 0x02) {
					while (Contador < PackageLenght) {
						switch (PayLoad[Contador]) {
#pragma region IDDoExperimento	
						case 0x04: {
							Contador++;
							for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
								ConfigPackage.ExperimentID.ByteArray[i] = PayLoad[Contador++];
							}
						}	break;
#pragma endregion
#pragma region TipoDeSuporte   
						case 0x05: {
							Contador++;
							ConfigPackage.SupportType = PayLoad[Contador++];
						}	break;
#pragma endregion
#pragma region QuantidadeDeAmostras	   
						case 0x06: {
							Contador++;
							ConfigPackage.SampleQuantity = PayLoad[Contador++];
						}	break;
#pragma endregion
#pragma region MassaDaAmostra	   
						case 0x07: {
							Contador++;
							ConfigPackage.SampleMass = PayLoad[Contador++];
						}	break;
#pragma endregion
#pragma region TempoDeExperimento	
						case 0x08: {
							Contador++;
							for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
								ConfigPackage.ExperimentTime.ByteArray[i] = PayLoad[Contador++];
							}
						}	break;
#pragma endregion
#pragma region ForçaGMinima	   
						case 0x09: {
							Contador++;
							for (int i = sizeof(uint32_t) - 1; i >= 0; i--) {
								ConfigPackage.MinGravitationalAcceleration.ByteArray[i] = PayLoad[Contador++];
							}
						}	break;
#pragma endregion
#pragma region ForcaGMaxima	   
						case 0x0A: {
							Contador++;
							for (int i = sizeof(uint32_t) - 1; i >= 0; i--) {
								ConfigPackage.MaxGravitationalAcceleration.ByteArray[i] = PayLoad[Contador++];
							}
						}	break;
#pragma endregion
#pragma region RotacaoEixoX		 
						case 0x0B: {
							Contador++;
							ConfigPackage.RpmX = PayLoad[Contador++];
						}	break;
#pragma endregion
#pragma region RotacaoEixoY			 
						case 0x0C: {
							Contador++;
							ConfigPackage.RpmY = PayLoad[Contador++];
						}	break;
#pragma endregion
#pragma region RotacaoEixoZ		   
						case 0x0D: {
							Contador++;
							ConfigPackage.RpmZ = PayLoad[Contador++];
						}	break;
#pragma endregion	
#pragma region MessageData		   
						case 0x01: {
							Contador++;
							if (PayLoad[Contador++] == 0xA4) {
								Serial.write(0x7F);
								Serial.write(0x7F);
								Serial.write(0x02);
								Serial.write(0x02);
								Serial.write(0x01);
								Serial.write(0xA0);
								Serial.write(0xA1);
							}
						}	break;
#pragma endregion
#pragma region DataUnknowed
						default: {
							Contador++;
						} break;
#pragma endregion
						}
					}

					/*Serial.print("ExperimentID -> ");
					Serial.println(ConfigPackage.ExperimentID.Value);
					Serial.print("SupportType -> ");
					Serial.println(ConfigPackage.SupportType);
					Serial.print("SampleQuantity -> ");
					Serial.println(ConfigPackage.SampleQuantity);
					Serial.print("SampleMass -> ");
					Serial.println(ConfigPackage.SampleMass);
					Serial.print("ExperimentTime -> ");
					Serial.println(ConfigPackage.ExperimentTime.Value);
					Serial.print("MinGravitationalAcceleration -> ");
					Serial.println(ConfigPackage.MinGravitationalAcceleration.Value);
					Serial.print("MaxGravitationalAcceleration -> ");
					Serial.println(ConfigPackage.MaxGravitationalAcceleration.Value);
					Serial.print("RpmX -> ");
					Serial.println(ConfigPackage.RpmX);
					Serial.print("RpmY -> ");
					Serial.println(ConfigPackage.RpmY);
					Serial.print("RpmZ -> ");
					Serial.println(ConfigPackage.RpmZ);*/
				} 
				else {
					Serial.write(0x7F);
					Serial.write(0x7F);
					Serial.write(0x02);
					Serial.write(0x02);
					Serial.write((byte)0x00);
					Serial.write(0x81);
					Serial.write(0x81);
				}
			} 	
			else {
				Serial.write(0x7F);
				Serial.write(0x7F);
				Serial.write(0x02);
				Serial.write(0x02);
				Serial.write((byte)0x00);
				Serial.write(0x80);
				Serial.write(0x80);
			}		 
		}
	}
}

void SerialWrite() {
	byte PackageLenght = 0x39;
	byte Payload[0x39] = { 0 };

	byte Contador = 0;

	Serial.write(0x7F);
	Serial.write(0x7F);
	Serial.write(0x02);
	Serial.write(PackageLenght);

#pragma region ExperimentID		 
	Payload[Contador++] = (0x04);
	for (int i = (sizeof(ExperimentPackage.ExperimentID.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.ExperimentID.ByteArray[i]);
#pragma endregion
#pragma region ExperimentTime	  
	Payload[Contador++] = (0x08);
	for (int i = (sizeof(ExperimentPackage.ExperimentTime.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.ExperimentTime.ByteArray[i]);
#pragma endregion
#pragma region AcelerometerX   
	Payload[Contador++] = (0x19);
	for (int i = (sizeof(ExperimentPackage.AcelerometerX.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.AcelerometerX.ByteArray[i]);
#pragma endregion			
#pragma region AcelerometerY 
	Payload[Contador++] = (0x1A);
	for (int i = (sizeof(ExperimentPackage.AcelerometerY.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.AcelerometerY.ByteArray[i]);
#pragma endregion
#pragma region AcelerometerZ   
	Payload[Contador++] = (0x1B);
	for (int i = (sizeof(ExperimentPackage.AcelerometerZ.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.AcelerometerZ.ByteArray[i]);
#pragma endregion			
#pragma region GiroscopeX	 
	Payload[Contador++] = (0x1C);
	for (int i = (sizeof(ExperimentPackage.GiroscopeX.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.GiroscopeX.ByteArray[i]);
#pragma endregion			
#pragma region GiroscopeY	  
	Payload[Contador++] = (0x1D);
	for (int i = (sizeof(ExperimentPackage.GiroscopeX.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.GiroscopeY.ByteArray[i]);
#pragma endregion	
#pragma region GiroscopeZ	  
	Payload[Contador++] = (0x1E);
	for (int i = (sizeof(ExperimentPackage.GiroscopeX.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.GiroscopeZ.ByteArray[i]);
#pragma endregion
#pragma region GravitationalAcceleration	   	  
	Payload[Contador++] = (0x1F);
	for (int i = (sizeof(ExperimentPackage.GravitationalAcceleration.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.GravitationalAcceleration.ByteArray[i]);
#pragma endregion
#pragma region CentripetalAcceleration	   	  
	Payload[Contador++] = (0x23);
	for (int i = (sizeof(ExperimentPackage.CentripetalAcceleration.Value) - 1); i >= 0; i--)
		Payload[Contador++] = (ExperimentPackage.CentripetalAcceleration.ByteArray[i]);
#pragma endregion
#pragma region CentrifugalAcceleration	   	  
	Payload[Contador++] = (0x24);
	for (int i = (sizeof(ExperimentPackage.CentrifugalAcceleration.Value) - 1); i >= 0; i--)
		Payload[Contador++] = (ExperimentPackage.CentrifugalAcceleration.ByteArray[i]);
#pragma endregion				 
#pragma region ResultantAcceleration	   	  
	Payload[Contador++] = (0x25);
	for (int i = (sizeof(ExperimentPackage.ResultantAcceleration.Value) - 1); i >= 0; i--)
		Payload[Contador++] = (ExperimentPackage.ResultantAcceleration.ByteArray[i]);
#pragma endregion	  
#pragma region ResultantSpeed	   	  
	Payload[Contador++] = (0x26);
	for (int i = (sizeof(ExperimentPackage.ResultantSpeed.Value) - 1); i >= 0; i--)
		Payload[Contador++] = (ExperimentPackage.ResultantSpeed.ByteArray[i]);
#pragma endregion
#pragma region ResultantRotation			  
	Payload[Contador++] = (0x27);
	Payload[Contador++] = (ExperimentPackage.ResultantRotation);
#pragma endregion	
#pragma region Temperature	  
	Payload[Contador++] = (0x0E);
	for (int i = (sizeof(ExperimentPackage.Temperature.Value) - 1); i >= 0; i--) 
		Payload[Contador++] = (ExperimentPackage.Temperature.ByteArray[i]);
#pragma endregion
#pragma region RpmX			  
	Payload[Contador++] = (0x0B);
	Payload[Contador++] = (ExperimentPackage.RpmX);
#pragma endregion
#pragma region RpmY		   
	Payload[Contador++] = (0x0C);
	Payload[Contador++] = (ExperimentPackage.RpmY);
#pragma endregion
#pragma region RpmZ		
	Payload[Contador++] = (0x0D);
	Payload[Contador++] = (ExperimentPackage.RpmZ);
#pragma endregion

	Serial.write(Payload, PackageLenght);
	Serial.write(CalculeCheckSum(Payload, PackageLenght));
}

void SerialWake() {	  
	/*Serial.write(0x7F);
	Serial.write(0x7F);
	Serial.write(0x01);
	Serial.write(0x02);
	Serial.write(0x01);
	Serial.write(0xA5);
	Serial.write(0xA6);*/	  

	Serial.write(0x7F);
	Serial.write(0x7F);
	Serial.write(0x02);
	Serial.write(0x02);
	Serial.write(0x01);
	Serial.write(0xA5);
	Serial.write(0xA6);

	/*Serial.write(0x7F);
	Serial.write(0x7F);
	Serial.write(0x03);
	Serial.write(0x02);
	Serial.write(0x01);
	Serial.write(0xA5);
	Serial.write(0xA6);*/
}	 
#pragma endregion

void setup() {
	ConfigurePID();

	Serial.begin(9600);
	ConfigPackage.SampleMass = 10;

	pinMode(MotorY1Port, 0x01);
	pinMode(MotorY2Port, 0x01);
	pinMode(pwmMotorYPort, 0x01);

	digitalWrite(MotorY1Port, 0x01);
	digitalWrite(MotorY2Port, 0x0);

	analogWrite(pwmMotorYPort, 0xFF);
	delay(500);
	randomSeed(A0);

	Wire.begin();			 
	Wire.setTimeout(50);
	Wire.beginTransmission(MPUAddress);
	Wire.write(0x6B);

	//Inicializa o MPU-6050
	Wire.write(0);
	Wire.endTransmission(true);
				  									 
	PhysicsValues.Acceleration.X = 0;
	PhysicsValues.Acceleration.Y = 0;
	PhysicsValues.Acceleration.Z = 0;
	PhysicsValues.Acceleration.Result = 0;

	PhysicsValues.AngularSpeed.X = 0;
	PhysicsValues.AngularSpeed.Y = 0;
	PhysicsValues.AngularSpeed.Z = 0;
	PhysicsValues.AngularSpeed.Result = 0;

	PhysicsValues.CentrifugalAcceleration = 0;
	PhysicsValues.CentripetalAcceleration = 0;
	PhysicsValues.ResultantRotation = 0;

	PhysicsValues.GravitationalAcceleration.X = 0;
	PhysicsValues.GravitationalAcceleration.Y = 0;
	PhysicsValues.GravitationalAcceleration.Z = 0;
	PhysicsValues.GravitationalAcceleration.Result = 0;
							 
	ExperimentPackage.ExperimentID.Value = 0;
	ExperimentPackage.ExperimentTime.Value = 0;

	ExperimentPackage.AcelerometerX.Value = 0;
	ExperimentPackage.AcelerometerY.Value = 0;
	ExperimentPackage.AcelerometerZ.Value = 0;

	ExperimentPackage.GiroscopeX.Value = 0;
	ExperimentPackage.GiroscopeY.Value = 0;
	ExperimentPackage.GiroscopeZ.Value = 0;

	ExperimentPackage.GravitationalAcceleration.Value = 0;
	ExperimentPackage.CentrifugalAcceleration.Value = 0;
	ExperimentPackage.CentripetalAcceleration.Value = 0;

	ExperimentPackage.ResultantAcceleration.Value = 0;
	ExperimentPackage.ResultantRotation = 0;
	ExperimentPackage.ResultantSpeed.Value = 0;

	ExperimentPackage.Temperature.Value = 0;

	ExperimentPackage.RpmX = 0;
	ExperimentPackage.RpmY = 0;
	ExperimentPackage.RpmZ = 0;

	SerialReader.enabled = true;
	SerialReader.setInterval(50);
	SerialReader.onRun(SerialRead);

	SerialWriter.enabled = true; 
	SerialWriter.setInterval(500);
	SerialWriter.onRun(SerialWrite);

	SerialAlive.enabled = true;
	SerialAlive.setInterval(2500);
	SerialAlive.onRun(SerialWake);

	DebugWriter.enabled = true;
	DebugWriter.setInterval(330);
	DebugWriter.onRun(DebugWrite);

	SensorReader.enabled = true;
	SensorReader.setInterval(500);
	SensorManager.onRun(SensorRead);

	MotorXWriter.enabled = true;
	MotorXWriter.setInterval(100);
	MotorXWriter.onRun(MotorXWrite);

	MotorXWriter.enabled = true;
	MotorXWriter.setInterval(100);
	MotorXWriter.onRun(MotorXWrite);

	MotorYWriter.enabled = true;
	MotorYWriter.setInterval(100);
	MotorYWriter.onRun(MotorYWrite);

	SensorManager.enabled = true;
	SensorManager.add(&SensorReader);

	MotorManager.enabled =true;
	MotorManager.add(&MotorXWriter);
	MotorManager.add(&MotorYWriter);
	MotorManager.add(&MotorZWriter);

	SerialManager.enabled = true;
	SerialManager.add(&SerialReader);
	SerialManager.add(&SerialWriter);
	SerialManager.add(&SerialAlive);
	//SerialManager.add(&DebugWriter);

	TaskManager.enabled = true;
	TaskManager.add(&SerialManager);
	TaskManager.add(&MotorManager);
	TaskManager.add(&SensorManager);
}

void loop() {
	if (TaskManager.shouldRun()) TaskManager.run();
}


