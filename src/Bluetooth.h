#ifndef BLUETOOTH_H
#define BLUETOOTH_H 

boolean BtDataComplete = false;  // whether the string is complete

char bt_packet[20]; 
int serialCount = 0;

const static char* getPIDValues = "GP;";
const static char* getSettings = "GS;";
const static char* getInfo = "GI;";
const static char* getKalman = "GK;";

const static char* setPValue = "SP,";
const static char* setIValue = "SI,";
const static char* setDValue = "SD,";
const static char* setKalman = "SK,";
const static char* setTargetAngle = "ST,";
const static char* setMaxAngle = "SA,";
const static char* setMaxTurning = "SU,";
const static char* setBackToSpot = "SB,";

const static char* imuBegin = "IB;";
const static char* imuStop = "IS;";

const static char* statusBegin = "RB;";
const static char* statusStop = "RS;";

const static char* sendStop = "CS;";
const static char* sendIMUValues = "CM,";
const static char* sendJoystickValues = "CJ,";
const static char* sendPairWithWii = "CPW;";
const static char* sendPairWithPS4 = "CPP;";

const static char* restoreDefaultValues = "CR;";

const static char* responsePIDValues = "P";
const static char* responseKalmanValues = "K";
const static char* responseSettings = "S";
const static char* responseInfo = "I";
const static char* responseIMU = "V";
const static char* responseStatus = "R";
const static char* responsePairConfirmation = "PC";

const static int HEADER_LEN = 3;


float bt_pitch = 0;
float bt_yaw_rate = 0;

void bluetooth_clear()
{
	memset(&bt_packet[0], 0, sizeof(bt_packet));
	serialCount = 0;
	BtDataComplete = false;
}

void decode_pitch_yaw(float pitch_yaw[], int size)
{
	int j=0;
	int k=0;
	char float_buf[14];
	for (int i=HEADER_LEN;i<serialCount;i++)
	{
		if ((bt_packet[i] != ',') && (bt_packet[i] !=';'))
		{
			float_buf[j++] = bt_packet[i];
		}
		else{
			float_buf[j] = '\0';
			if (k < size){
				pitch_yaw[k] = atof(float_buf);
			//	Serial.println(pitch_yaw[k]);
				k++;
				memset(&float_buf[0], 0, sizeof(float_buf));
				j = 0;
			}
			else{
				return;
			}
		}
	}
	
}

void processBluetooth()
{
	while (Bluetooth.available()) {
		// get the new byte:
		char inChar = (char)Bluetooth.read();

		bt_packet[serialCount++] = inChar;

		if (inChar == ';') {
			BtDataComplete = true;
		}
	}
	if (BtDataComplete) {
	//	for (int i=0;i<serialCount;i++)
	//		Serial.print(bt_packet[i]);
	//	Serial.println(serialCount);
		
		char header[HEADER_LEN+1];

		if (serialCount < HEADER_LEN){
			bluetooth_clear();
			return;
		}

		for (int i=0;i<HEADER_LEN;i++)
		{
			header[i] = bt_packet[i];
		}
		header[HEADER_LEN] = '\0';
		
		//classify by header
		if (strcmp(header, sendStop) == 0)
		{
		//	Serial.println("stop");
			bt_pitch = 0;
			bt_yaw_rate = 0;
		}
		if (strcmp(header, sendJoystickValues) == 0)
		{
			//Serial.println("joystick");
			float pitch_yaw[2] = {0};
			
			decode_pitch_yaw(pitch_yaw, 2);

			bt_pitch = pitch_yaw[1]*MAX_PITCH;
			bt_yaw_rate = pitch_yaw[0]*MAX_YAW_RATE;
		}
		if (strcmp(header, sendIMUValues) == 0)
		{
			//Serial.println("imu");
			float pitch_yaw[2]={0};
			decode_pitch_yaw(pitch_yaw, 2);
			
			const static float MAX_PHONE_ANGLE = 45.0f;
			pitch_yaw[0] = constrain(pitch_yaw[0], -MAX_PHONE_ANGLE, MAX_PHONE_ANGLE);
			pitch_yaw[1] = constrain(pitch_yaw[1], -MAX_PHONE_ANGLE, MAX_PHONE_ANGLE);
			
			applyDeadband(pitch_yaw[0], 5.0f);
			applyDeadband(pitch_yaw[1], 5.0f);
			bt_pitch = pitch_yaw[0]/MAX_PHONE_ANGLE*MAX_PITCH;
			bt_yaw_rate = pitch_yaw[1]/MAX_PHONE_ANGLE*MAX_YAW_RATE;

		}
		

	}
	bluetooth_clear();

}



#endif /* BLUETOOTH_H */
