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
WeRGBLed rgb_8(PORT_0);
WeInfraredReceiver ir(PORT_2);
WeBuzzer buzzer(OnBoard_Buzzer);

WeLineFollower lineFollower(PORT_A);
WeLineFollower lineLimit(PORT_B);
WeUltrasonicSensor ultraSensor(PORT_B);
WeLimitSwitch limitSwitch(PORT_C);
WeGyroSensor gyro(PORT_D);

WeEncoderMotor188 MotorR(PORT_3);
WeEncoderMotor188 MotorL(PORT_4);
WeDCMotor Motor1(M1);
WeDCMotor Motor2(M2);


enum{MODE_A, MODE_B, MODE_C, MODE_D};
byte mode = MODE_A;

enum{STOP, RUN_F, RUN_B, RUN_L, RUN_R, MOTOR1_L, MOTOR1_R, MOTOR2_L, MOTOR2_R,}
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
uint8_t RGB8_flag=0;
uint8_t motor_flag=0;
bool Encodemotor_flag=false;

boolean start_flag = false;
boolean move_flag = false;
long lasttime_angle = 0;
long lasttime_speed = 0;
uint16_t Measurement_speed_time=0;
double  CompAngleY, CompAngleX;
double  angle_speed = 0.0;
double  last_turn_setpoint_filter = 0.0;
double  last_speed_setpoint_filter = 0.0;
double  speed_Integral_average = 0.0;
int RELAX_ANGLE = 2;
bool flag_D=0;

typedef struct
{
  double P, I, D;
  double Setpoint, Output, Integral,differential, last_error;
} PID;
PID  PID_angle, PID_speed, PID_turn;
PID  PID_speed_left, PID_speed_right;

void handle_command(uint8_t value)
{
	command_timestamp = millis();
	switch(value)
	{
	case IR_CONTROLLER_A:
		moveSpeed = 150;
		mode = MODE_A;
		Encodemotor_flag=0;
    flag_D=0;
		Stop();
		buzzer.tone2(NTD1, 300);
		rgb.setColor(0,10,0,0);
		rgb.show();
		break;
	case IR_CONTROLLER_B:
		moveSpeed = 100;
		mode = MODE_B;
    flag_D=0;
		Stop();
		buzzer.tone2(NTD2, 300);
		rgb.setColor(0,0,10,0);
		rgb.show();
		break;     
	case IR_CONTROLLER_C:
		mode = MODE_C;
		moveSpeed = 100;
    flag_D=0;
		Stop();
		buzzer.tone2(NTD3, 300);
		rgb.setColor(0,0,0,10);
		rgb.show();
		break;
	case IR_CONTROLLER_D:
    mode = MODE_D;
    flag_D=1;
    RGB8_flag=0;
    RGBUlt_flag=0;
    gyro.begin();
    Stop();
		buzzer.tone2(NTD4, 300);
		rgb.setColor(0,10,10,0);
		rgb.show();
		break;
    
	case IR_CONTROLLER_E:  
		Encodemotor_flag=1;
    flag_D=0;
		buzzer.tone2(NTD5, 300);
		break;       
	case IR_CONTROLLER_F:   
    RGB8_flag =(RGB8_flag+1)%3;
    if(RGB8_flag==0){
      for(int j=0;j<8;j++){
        rgb_8.setColorAt(j, 0, 0, 0);  // led number, red, green, blue,
      }
      rgb_8.show();
    }
		buzzer.tone2(NTD6, 300);
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
		motor_sta = RUN_F; PID_speed.Setpoint=80;PID_turn.Setpoint = 0;move_flag = true;
		break;
	case IR_CONTROLLER_DOWN:
		motor_sta = RUN_B; PID_speed.Setpoint=-80;PID_turn.Setpoint = 0;move_flag = true; 
		break;
	case IR_CONTROLLER_RIGHT:
		motor_sta = RUN_R; PID_speed.Setpoint=0;PID_turn.Setpoint = 100;move_flag = true;
		break;
	case IR_CONTROLLER_LEFT:
		motor_sta = RUN_L;PID_speed.Setpoint=0;PID_turn.Setpoint = -100;move_flag = true;
		break;
    
	case IR_CONTROLLER_9:
    motor_sta = MOTOR1_R; 
		break;
	case IR_CONTROLLER_8:
    motor_sta = MOTOR2_R;
		break;
	case IR_CONTROLLER_7:
   motor_sta = MOTOR2_L;
		break;
	case IR_CONTROLLER_6:
    motor_sta = MOTOR1_L;   
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
    setMoveSpeed(NTD2, 1);
		break;
  case IR_CONTROLLER_0:
    setMoveSpeed(NTD1, 0);
    break;
	}
}

void setMoveSpeed(uint16_t frequency, int level)
{
	buzzer.tone2(frequency, 300);
	moveSpeed = level * 36 + 30;
}

// void SetDestSpeed(int value)
// {
// 	const int speedStep = 100;
// 	const int sleepTime = 120;
// 	for(;;){
// 		int lspeed = speedStep + speedSetLeft;
// 		int rspeed = speedStep + speedSetRight;
		
// 		if(lspeed < value && rspeed < value){
// 			doRun(lspeed, rspeed);
// 			delay(sleepTime);
// 		}else if(lspeed < value){
// 			doRun(lspeed, value);
// 			delay(sleepTime);
// 		}else if(rspeed < value){
// 			doRun(value, rspeed);
// 			delay(sleepTime);
// 		}else{
// 			doRun(value, value);
// 			break;
// 		}
// 	}
// }

void Forward()
{ 
  if(Encodemotor_flag==0){  
	doRun(moveSpeed, moveSpeed);
	// SetDestSpeed(moveSpeed);
  }
  else  MotorL.runSpeed(-moveSpeed/2);
}
void Backward()
{
  if(Encodemotor_flag==0){  
   doRun(-moveSpeed, -moveSpeed);
  }
  else MotorL.runSpeed(moveSpeed/2);
}
void TurnLeft()
{
  if(Encodemotor_flag==0){  
   doRun(-moveSpeed, moveSpeed);
  }
  else MotorR.runSpeed(moveSpeed/2);
}
void TurnRight()
{
   if(Encodemotor_flag==0){  
   doRun(moveSpeed,-moveSpeed);
   }
   else MotorR.runSpeed(-moveSpeed/2);
}
void Stop()
{
	MotorL.runSpeed(0);
  MotorR.runSpeed(0);
  Motor1.run(0);
  Motor2.run(0);
}

void doRun(int lspeed, int rspeed)
{
	// speedSetLeft = lspeed;
	// speedSetRight = rspeed;
	motor_run(lspeed, rspeed);
}

void motor_run(int lspeed, int rspeed)
{
	MotorL.runSpeed(-lspeed/2);
	MotorR.runSpeed(rspeed/2);
}

void motor1_limit(uint8_t motor_dir)
{
	int line_limit_value;
	bool line_flag=0;
	if((limitSwitch.read()==1)&&(motor_flag==0))
	{
	  motor_flag=motor_dir;     
	}
	else if(limitSwitch.read()==0)  motor_flag=0;
	lineLimit.startRead();
	if(lineLimit.readSensor1() == 1023){
	    line_limit_value = 0;
	}
	else{
			line_limit_value = lineLimit.readSensor1();
	}

	if(line_limit_value>500)
	{
	  line_flag=1;
	}
	else  line_flag=0;

	if((motor_dir==1)&&(motor_flag!=1))
	      Motor1.run(-moveSpeed);
	else if((motor_dir==2)&&(motor_flag!=2)&&(line_flag==0))
	      Motor1.run(moveSpeed);
	else  Motor1.run(0);
}

void modeA()
{
	switch(motor_sta){
	case RUN_F:	Forward();	break;
	case RUN_B:	Backward();	break;
	case RUN_L:	TurnLeft();	break;
	case RUN_R:	TurnRight();break;
	case MOTOR1_L :motor1_limit(1);break;
	case MOTOR1_R :motor1_limit(2);break;
	case MOTOR2_L :Motor2.run(-moveSpeed);break;
	case MOTOR2_R :Motor2.run(moveSpeed);break;
	case STOP:	Stop();		break;
	}
}

void setup()
{
	pinMode(OnBoard_Button, INPUT);
  Stop();

	for(int i=0;i<30;i++){
    for(int j=0;j<8;j++){
		rgb_8.setColorAt(j, i, i, i);  // led number, red, green, blue,
    }
		rgb_8.show();
		delay(20);   
	}
 for(int i=30;i>=0;i--){
    for(int j=0;j<8;j++){
    rgb_8.setColorAt(j, i, i, i);  // led number, red, green, blue,
    }
    rgb_8.show();
    delay(20);   
  }  
	buzzer.tone(NTD6, 800);
	ultraSensor.setColor1(0, 0, 0);
	ultraSensor.setColor2(0, 0, 0);
	ir.begin();
	Serial.begin(115200);
  delay(1000);
  PID_angle.Setpoint = -1;
  PID_angle.P = 26;          // 20;
  PID_angle.I = 3;           // 1;
  PID_angle.D = 0.2;         // 0.2;
  PID_speed.P = 0.02;        // 0.06
  PID_speed.I = 0.003;       // 0.005
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
	if(millis() - command_timestamp > 200){
		command_timestamp = millis();
    if(flag_D==0) motor_sta = STOP;
    PID_speed.Setpoint=0;PID_turn.Setpoint = 0;
	}
	switch(mode){
	case MODE_A: modeA(); break;
	case MODE_B: modeB(); break;
	case MODE_C: modeC(); break;
	case MODE_D: modeD(); break;
	}
 
	if(RGBUlt_flag == 1){
		mode_RGBult();
	}
   if(RGB8_flag > 0){
    mode_RGB8();
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
  RGB8_flag=0;
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

void modeD()
{
   gyro.update();
   balanced_model();
}
float RGB_j, RGB_f, RGB_k;
uint8_t RGB_num;
void mode_RGB8()
{ 
  if (RGB8_flag==1)
  {
    RGB_j += random(1, 6) / 6.0;
    RGB_f += random(1, 6) / 6.0;
    RGB_k += random(1, 6) / 6.0;
    for(uint8_t t = 0; t < 7; t++)
    {
      uint8_t red  = 10 * (1 + sin(t / 2.0 + RGB_j / 4.0) );
      uint8_t green = 10 * (1 + sin(t / 1.0 + RGB_f / 9.0 + 2.1) );
      uint8_t blue = 10 * (1 + sin(t / 3.0 + RGB_k / 14.0 + 4.2) );
      rgb_8.setColor(t, red, green, blue);    //(Red,Green,Blue)
    }
    rgb_8.show(); 
  }
 
  else if (RGB8_flag==2)
  {
    RGB_num++;
    if(RGB_num>10)
    {
      RGB_j += random(1, 6) / 6.0;
      RGB_f += random(1, 6) / 6.0;
      RGB_k += random(1, 6) / 6.0;   
      RGB_num=0;
    }
    uint8_t red  = 10 * (1 + sin(1 / 2.0 + RGB_j / 4.0) );
    uint8_t green = 10 * (1 + sin(1 / 1.0 + RGB_f / 9.0 + 2.1) );
    uint8_t blue = 10 * (1 + sin(1 / 3.0 + RGB_k / 14.0 + 4.2) );
    for(uint8_t t = 0; t < 7; t++)
    {
         rgb_8.setColor(RGB_num, red, green, blue);    //(Red,Green,Blue)
    }
    rgb_8.show(); 
  }
  delay(5);
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
	mode = (mode + 1) % 4;
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
  case MODE_D:
    buzzer.tone2(NTD4, 300);
    rgb.setColor(0,10,10,0);
    break;
	}
	rgb.show();
	delay(500);
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
		serial_reply(cmd, "12in1_A_1");  //12 in 1 kit
		return;
	}
	if(isCmd(cmd, "BZ")){
		int note = nextInt(&cmd);
		int hz   = nextInt(&cmd);
		buzzer.tone2(note, hz);
		return;
	}
	if(isCmd(cmd, "RGB")){
		int index = nextInt(&cmd);
		int r = nextInt(&cmd);
		int g = nextInt(&cmd);
		int b = nextInt(&cmd);
		ultraSensor.setColor(index, r, g, b);
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


int16_t agx_start_count;
void reset(void)
{
  if((start_flag == false) && (abs(gyro.getAngleX()) < 5))
  {
    agx_start_count++;
  }
  if((start_flag == true) && abs(gyro.getAngleX()) > 30)
  {
    agx_start_count = 0;
    MotorL.run(0);
    MotorR.run(0);
    PID_speed.Integral = 0;
    PID_angle.Setpoint = RELAX_ANGLE;
    PID_speed.Setpoint = 0;
    PID_turn.Setpoint = 0;
    MotorL.setPositionOrigin();
    MotorR.setPositionOrigin();
    PID_speed.Integral = 0;
    start_flag = false;
    last_speed_setpoint_filter = 0.0;
    last_turn_setpoint_filter = 0.0;
  }
  else if(agx_start_count > 20)
  {
    agx_start_count = 0;
    PID_speed.Integral = 0;
    MotorL.run(0);
    MotorR.run(0);
    PID_angle.Setpoint = RELAX_ANGLE;
    MotorL.setPositionOrigin();
    MotorR.setPositionOrigin();
    lasttime_speed = lasttime_angle = millis();
    start_flag = true;
    }
}
void balanced_model(void)
{
  reset();
  if(start_flag == true)
  {
    if((millis() - lasttime_angle) > 10)
    {
      PID_angle_compute();
      lasttime_angle = millis();
    }    
    if((millis() - lasttime_speed) > 100)
    {
      PID_speed_compute();
      last_turn_setpoint_filter  = last_turn_setpoint_filter * 0.8;
      last_turn_setpoint_filter  += PID_turn.Setpoint * 0.2;
      PID_turn.Output = last_turn_setpoint_filter;
      lasttime_speed = millis();
    }
  }
  else
  {
    MotorL.run(0);
    MotorR.run(0);
  } 
}

void PID_angle_compute(void)   //PID
{
  CompAngleX = -gyro.getAngleX();
  angle_speed = gyro.getGyroY();
  double error = CompAngleX - PID_angle.Setpoint;

  PID_angle.Integral += error;
  PID_angle.Integral = constrain(PID_angle.Integral,-100,100); 
  PID_angle.differential = angle_speed;
  PID_angle.Output = PID_angle.P * error + PID_angle.I * PID_angle.Integral + PID_angle.D * PID_angle.differential;

  double pwm_left = PID_angle.Output - PID_turn.Output;
  double pwm_right = -PID_angle.Output - PID_turn.Output;

  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  MotorL.run(pwm_left);
  MotorR.run(pwm_right);
}
long last_pos_1=0;
long last_pos_2=0;

void PID_speed_compute(void)
{
  uint16_t dt = millis() - Measurement_speed_time;
  long cur_pos_1=MotorL.getCurrentPosition();
  long cur_pos_2=MotorR.getCurrentPosition();
  float MotorL_CurrentSpeed=((cur_pos_1-last_pos_1)*(1000.0/dt)*60)/585.0;
  float MotorR_CurrentSpeed=((cur_pos_2-last_pos_2)*(1000.0/dt)*60)/585.0;
  double speed_now = (MotorR_CurrentSpeed - MotorL_CurrentSpeed)/2;
  last_pos_1=cur_pos_1;
  last_pos_2=cur_pos_2;
  Measurement_speed_time=millis();
  last_speed_setpoint_filter  = last_speed_setpoint_filter  * 0.8;
  last_speed_setpoint_filter  += PID_speed.Setpoint * 0.2;
 
  if((move_flag == true) && (abs(speed_now) < 8) && (PID_speed.Setpoint == 0))
  {
    move_flag = false;
    last_speed_setpoint_filter = 0;
    PID_speed.Integral = speed_Integral_average;
  }

  double error = speed_now - last_speed_setpoint_filter;
  PID_speed.Integral += error;

  if(move_flag == true) 
  { 
    PID_speed.Integral = constrain(PID_speed.Integral , -2000, 2000);
    PID_speed.Output = PID_speed.P * error + PID_speed.I * PID_speed.Integral;
    PID_speed.Output = constrain(PID_speed.Output , -8.0, 8.0);
  }
  else
  {  
    PID_speed.Integral = constrain(PID_speed.Integral , -2000, 2000);
    PID_speed.Output = PID_speed.P * speed_now + PID_speed.I * PID_speed.Integral;
    PID_speed.Output = constrain(PID_speed.Output , -10.0, 10.0);
    speed_Integral_average = 0.8 * speed_Integral_average + 0.2 * PID_speed.Integral;
  }
  PID_angle.Setpoint =  RELAX_ANGLE + PID_speed.Output;
}
