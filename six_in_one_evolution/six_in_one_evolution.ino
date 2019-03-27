#include <WeELF328P.h>

#define NTD1 294
#define NTD2 330
#define NTD3 350
#define NTD4 393
#define NTD5 441
#define NTD6 495
#define NTD7 556
#define NTDL1 147
#define NTDL2 165
#define NTDL3 175
#define NTDL4 196
#define NTDL5 221
#define NTDL6 248
#define NTDL7 278
#define NTDH1 589
#define NTDH2 661
#define NTDH3 700
#define NTDH4 786
#define NTDH5 882
#define NTDH6 990
#define NTDH7 112

WeRGBLed rgb(OnBoard_RGB);
WeInfraredReceiver ir(PORT_2);
WeBuzzer buzzer(OnBoard_Buzzer);

WeUltrasonicSensor ultraSensor(NC);
WeLineFollower lineFollower(NC);
WeLEDPanelModuleMatrix7_21 ledPanel(NC);

WeDCMotor MotorL(M2);
WeDCMotor MotorR(M1);

enum{MODE_A, MODE_B, MODE_C, MODE_D, MODE_E, MODE_F};
byte mode = MODE_A;

enum{STOP, RUN_F, RUN_B, RUN_L, RUN_R}
motor_sta = STOP;

int moveSpeed = 150;
//int& line_speed = moveSpeed;

int speedSetLeft = 0;
int speedSetRight = 0;

bool speed_flag = false;
byte RGBUlt_flag = false;

long command_timestamp = 0;
uint8_t prev_mode = mode;
byte prev_RGBUlt_flag = RGBUlt_flag;
int prev_moveSpeed = moveSpeed;
bool bluetoothMode = false;
bool isJoystickMode = false;
int joystickSpeedL = 0;
int joystickSpeedR = 0;

void handle_command(uint8_t value)
{
	command_timestamp = millis();
	switch(value)
	{
	case IR_CONTROLLER_A:
		speed_flag = false;
		moveSpeed = 150;
		mode = MODE_A;
		Stop();
		buzzer.tone2(NTD1, 300);
		rgb.setColor(0,10,0,0);
		rgb.show();
		break;
	case IR_CONTROLLER_B:
		moveSpeed = 200;
		mode = MODE_B;
		Stop();
		buzzer.tone2(NTD2, 300);
		rgb.setColor(0,0,10,0);
		rgb.show();
		break;     
	case IR_CONTROLLER_C:
		mode = MODE_C;
		moveSpeed = 150;
		Stop();
		buzzer.tone2(NTD3, 300);
		rgb.setColor(0,0,0,10);
		rgb.show();
		break;
	case IR_CONTROLLER_D:
		speed_flag = true;
		moveSpeed = 200;
		buzzer.tone2(NTD4, 300);
		rgb.setColor(0,10,10,0);
		rgb.show();
		break;
	case IR_CONTROLLER_E:   
		speed_flag = true;
		moveSpeed = 255;
		mode = MODE_E;
		buzzer.tone2(NTD5, 300);
		break;       
	case IR_CONTROLLER_F:
		mode = MODE_F;
		moveSpeed = 100;
		Stop();
		buzzer.tone2(NTD6, 300);
		rgb.setColor(0,0,10,10);
		rgb.show();
		break;
	case IR_CONTROLLER_OK:
		RGBUlt_flag = (RGBUlt_flag + 1) % 3;
		if(RGBUlt_flag == 0){
			ultraSensor.setColor1(0, 0, 0);
    		ultraSensor.setColor2(0, 0, 0);
		}
		buzzer.tone2(NTD6, 300);
		break;
	case IR_CONTROLLER_UP:
		motor_sta = RUN_F;
		isJoystickMode = false;
		break;
	case IR_CONTROLLER_DOWN:
		motor_sta = RUN_B;
		isJoystickMode = false;
		break;
	case IR_CONTROLLER_RIGHT:
		motor_sta = RUN_R;
		isJoystickMode = false;
		break;
	case IR_CONTROLLER_LEFT:
		motor_sta = RUN_L;
		isJoystickMode = false;
		break;
	case IR_CONTROLLER_9:
		setMoveSpeed(NTDH2, 9);
		break;
	case IR_CONTROLLER_8:
		setMoveSpeed(NTDH1, 8);
		break;
	case IR_CONTROLLER_7:
		setMoveSpeed(NTD7, 7);
		break;
	case IR_CONTROLLER_6:
		setMoveSpeed(NTD6, 6);
		break;
	case IR_CONTROLLER_5:
		setMoveSpeed(NTD5, 5);
		break;
	case IR_CONTROLLER_4:
		setMoveSpeed(NTD4, 4);
		break;
	case IR_CONTROLLER_3:
		setMoveSpeed(NTD3, 3);
		break;
	case IR_CONTROLLER_2:
		setMoveSpeed(NTD2, 2);
		break;
	case IR_CONTROLLER_1:
		setMoveSpeed(NTD1, 0);
		break;
	}
}

void setMoveSpeed(uint16_t frequency, int level)
{
	buzzer.tone2(frequency, 300);
	moveSpeed = level * 16 + 75;
}

void SetDestSpeed(int value)
{
	const int speedStep = 100;
	const int sleepTime = 120;
	for(;;){
		int lspeed = speedStep + speedSetLeft;
		int rspeed = speedStep + speedSetRight;
		
		if(lspeed < value && rspeed < value){
			doRun(lspeed, rspeed);
			delay(sleepTime);
		}else if(lspeed < value){
			doRun(lspeed, value);
			delay(sleepTime);
		}else if(rspeed < value){
			doRun(value, rspeed);
			delay(sleepTime);
		}else{
			doRun(value, value);
			break;
		}
	}
}

void Forward()
{   
	if(speed_flag){
		doRun(moveSpeed, moveSpeed);
	}else{
		SetDestSpeed(moveSpeed);
	}
}

void Backward()
{
   doRun(-moveSpeed, -moveSpeed);
}
void TurnLeft()
{
   doRun(-moveSpeed, moveSpeed);
}
void TurnRight()
{
   doRun(moveSpeed,-moveSpeed);
}
void Stop()
{
	if(speed_flag){
		doRun(0, 0);
	}else{
		SetDestSpeed(0);
	}
}

void doRun(int lspeed, int rspeed)
{
	speedSetLeft = lspeed;
	speedSetRight = rspeed;
	motor_run(lspeed, rspeed);
}

void modeA()
{
	if(isJoystickMode){
		motor_run(joystickSpeedL, joystickSpeedR);
		return;
	}
	switch(motor_sta){
	case RUN_F:	Forward();	break;
	case RUN_B:	Backward();	break;
	case RUN_L:	TurnLeft();	break;
	case RUN_R:	TurnRight();break;
	case STOP:	Stop();		break;
	}
}

void setup()
{
	pinMode(OnBoard_Button, INPUT);
	Stop();

	for(int i=0;i<10;i++){
		rgb.setColorAt(0, i, 0, 0);  // led number, red, green, blue,
		rgb.show();
		delay(20);
	}
	buzzer.tone(NTD1, 500);
	for(int i=0;i<15;i++){
		rgb.setColorAt(0, 0, i, 0);  // led number, red, green, blue,
		rgb.show();
		delay(20);
	}
	buzzer.tone(NTD1, 500); 
	for(int i=0;i<15;i++){
		rgb.setColorAt(0, 0, 0, i);  // led number, red, green, blue,
		rgb.show();
		delay(20);   
	}
	buzzer.tone(NTD1, 500); 
	for(int i=0;i<10;i++){
		rgb.setColorAt(0, i, i, i);  // led number, red, green, blue,
		rgb.show();
		delay(20);   
	}
	buzzer.tone(NTD6, 600);
	
	loopSensor();
	ultraSensor.setColor1(0, 0, 0);
	ultraSensor.setColor2(0, 0, 0);
	ledPanel.setBrightness(7);
	uint8_t bitmap[] = {0x41,0x42,0x04,0x08,0x10,0x20,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x20,0x10,0x08,0x04,0x42,0x41};
	ledPanel.showBitmap(0, 0, bitmap);

	ir.begin();
	Serial.begin(115200);
}

void bindSensor(uint8_t sensorType, uint8_t port)
{
	switch(sensorType){
	case 1:
		ultraSensor.reset(port);
		break;
	case 2:
		lineFollower.reset(port);
		break;
	case 3:
		ledPanel.reset(port);
		break;
	}
}

void loopSensor()
{
	const uint8_t sensor_port[] = {PORT_A, PORT_B, PORT_C, PORT_D};
	WeOneWire portDetect;

	for(int i=0; i<4; ++i){
		uint8_t port = sensor_port[i];
		if(!digitalRead(port)){
			continue;
		}
		//delay(400);
		portDetect.reset(port);
		portDetect.reset();
		portDetect.write_byte(0x01);
		portDetect.respond();

		uint8_t sensorType = portDetect.read_byte();
		bindSensor(sensorType, port);
	}
}

void loop()
{
	if(bluetoothMode){
		get_serial_command();
	}else if(Serial.available()){
		bluetoothMode = true;
	}else if(ir.decode()){
		handle_command(ir.value >> 16 & 0xFF);
	}
	if(millis() - command_timestamp > 120){
		command_timestamp = millis();
		motor_sta = STOP;
	}
	switch(mode){
	case MODE_A: modeA(); break;
	case MODE_B: modeB(); break;
	case MODE_C: modeC(); break;
	case MODE_E: modeE(); break;
	case MODE_F: modeF(); break;
	}
	if(RGBUlt_flag == 1){
		mode_RGBult();
	}
	if(!digitalRead(OnBoard_Button)){
		pre_button();
	}
}

void modeB()
{
	randomSeed(analogRead(6));
	uint8_t d = ultraSensor.distanceCm();
	
	if(d >= 50 || d == 0){
		motor_run(moveSpeed, moveSpeed);
		delay(100);
	}else if (d > 15){
		switch(random(2)){
		case 0:
			motor_run(moveSpeed, -moveSpeed/5);
			break;
		case 1:
			motor_run(-moveSpeed/5, moveSpeed);
			break;
		}
		delay(200);
	}else{
		motor_run(-moveSpeed, -moveSpeed);
		delay(300);
		switch(random(2)){
		case 0:
			motor_run(moveSpeed, -moveSpeed);
			break;
		case 1:
			motor_run(-moveSpeed, moveSpeed);
			break;
		}
		delay(300);
	}
}

void modeC()
{
	const int base = 500;
	//static uint8_t line_speed = 100;
	static uint8_t flag = 0;

	lineFollower.startRead();
	bool L_IN = lineFollower.readSensor1() < base;
	bool R_IN = lineFollower.readSensor2() < base;

	if(L_IN && R_IN){
		motor_run(moveSpeed, moveSpeed);
	}else if(L_IN && !R_IN){
		flag = 1;
		motor_run(moveSpeed, moveSpeed);
	}else if(!L_IN && R_IN){
		flag = 2;
		motor_run(moveSpeed, moveSpeed);
	}else if(flag == 1){
		motor_run(-moveSpeed, moveSpeed);
	}else if(flag == 2){
		motor_run(moveSpeed, -moveSpeed);

	}
}

void modeE()
{
	Forward();
	delay(3000);
	mode = MODE_A;
	moveSpeed = 150;
}

void modeF()
{
	const int ult_speed = 150;
	static int f_cout = 0;

	if(f_cout > 20){
		motor_run(0, ult_speed);
		delay(100);
		f_cout--;
	}else if(f_cout < 20){
		motor_run(ult_speed, 0);
		delay(100);
		f_cout++;
	}

	if(f_cout == 21){
		f_cout = 0;
	}else if(f_cout == 19){
		f_cout = 40;
	}
}

void mode_RGBult()
{
	static float j, f, k;
	j += random(1, 6) / 6.0;
	f += random(1, 6) / 6.0;
	k += random(1, 6) / 6.0;
	float red   = 64 * (1 + sin(1 / 2.0 + j / 4.0));
	float green = 64 * (1 + sin(1 / 1.0 + f / 9.0 + 2.1));
	float blue  = 64 * (1 + sin(1 / 3.0 + k / 14.0 + 4.2));
	ultraSensor.setColor1(red, green, blue);
	ultraSensor.setColor2(red, green, blue);
}

void pre_button()
{
	Stop();
	mode = (mode + 1) % MODE_D;
	switch(mode){
	case MODE_A:
		buzzer.tone2(NTD1, 300);
		rgb.setColor(0,10,0,0);
		break;
	case MODE_B:
		buzzer.tone2(NTD2, 300);
		rgb.setColor(0,0,10,0);
		break;
	case MODE_C:
		buzzer.tone2(NTD3, 300);
		rgb.setColor(0,0,0,10);
		break;
	}
	rgb.show();
	delay(500);
}

void motor_run(int lspeed, int rspeed)
{
	MotorL.run(-lspeed);
	MotorR.run(rspeed);
}

void get_serial_command()
{
	const int buffer_len = 128;
	static char buffer[buffer_len];
	static int buffer_index = 0;

	while(Serial.available())
	{
		char nextChar = Serial.read();
		if(nextChar == '\n'){
			while(buffer[--buffer_index] == '\r');
			buffer[buffer_index+1] = ' ';
			buffer[buffer_index+2] = 0;
			buffer_index = 0;
			handle_serial_command(buffer);
			//memset(buffer, 0, buffer_len);
		}else{
			buffer[buffer_index] = nextChar;
			buffer_index = (buffer_index + 1) % buffer_len;
		}
	}
}

int nextInt(char **cmd)
{
	while(' ' != *(*cmd)++);
	return atoi(*cmd);
}

bool isCmd(char *buffer, char *cmd)
{
	while(*cmd){
		if(*buffer != *cmd){
			return false;
		}
		++buffer;
		++cmd;
	}
	return true;
}

void serial_reply(char *buffer, char *info)
{
	Serial.write(buffer, strlen(buffer));
	Serial.println(info);
}

void handle_serial_command(char *cmd)
{
	if(isCmd(cmd, "VER")){
		serial_reply(cmd, "6in1_A_1");
		return;
	}
	if(isCmd(cmd, "LFA")){
		serial_reply(cmd, "OK");
		ledPanel.clearScreen();
		return;
	}
	if(isCmd(cmd, "LF")){
		serial_reply(cmd, "OK");
		int x = nextInt(&cmd);
		int y = nextInt(&cmd);
		ledPanel.turnOffDot(x, y);
		return;
	}
	if(isCmd(cmd, "LO")){
		serial_reply(cmd, "OK");
		int x = nextInt(&cmd);
		int y = nextInt(&cmd);
		ledPanel.turnOnDot(x, y);
		return;
	}
	if(isCmd(cmd, "BZ")){
		serial_reply(cmd, "OK");
		int note = nextInt(&cmd);
		int hz   = nextInt(&cmd);
		buzzer.tone2(note, hz);
		return;
	}
	if(isCmd(cmd, "RGB")){
		serial_reply(cmd, "OK");
		int index = nextInt(&cmd);
		int r = nextInt(&cmd);
		int g = nextInt(&cmd);
		int b = nextInt(&cmd);
		ultraSensor.setColor(index, r, g, b);
		return;
	}
	if(isCmd(cmd, "JS")){
		serial_reply(cmd, "OK");
		isJoystickMode = true;
		joystickSpeedL = nextInt(&cmd);
		joystickSpeedR = nextInt(&cmd);
		return;
	}
	if(isCmd(cmd, "US")){
		uint8_t d = ultraSensor.distanceCm();
		Serial.write(cmd, strlen(cmd));
		Serial.println(d);
		return;
	}
	if(isCmd(cmd, "IR")){
		serial_reply(cmd, "OK");
		uint8_t code = atoi(cmd + 2);
		if(code == 1){
			mode = prev_mode;
			moveSpeed = prev_moveSpeed;
			RGBUlt_flag = prev_RGBUlt_flag;
			buzzer.tone2(NTD1, 300);
		}else if(code == 2){
			prev_RGBUlt_flag = RGBUlt_flag;
			prev_moveSpeed = moveSpeed;
			prev_mode = mode;

			mode = MODE_A;
			RGBUlt_flag = 0;
			ultraSensor.setColor1(0, 0, 0);
    		ultraSensor.setColor2(0, 0, 0);
			motor_run(0, 0);
			buzzer.tone2(NTD1, 300);
		}else{
			handle_command(code);
		}
		return;
	}
}
