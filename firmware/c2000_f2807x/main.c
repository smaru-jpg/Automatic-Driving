#include "F2807x_device.h"
#include "F2807x_Examples.h"
#include <math.h>
#include <stdlib.h>




//서보모터 :  서보모터 제어를 위한 매크로 정의
// Period = (TBPRD + 1) / TBCLK
// TBCLK = SYSCLK /(HSPCLKDIV * CLKDIV)

//SYSCLK = 120 MHZ, HSPCLKDIV(6), CLKDIV(4) 계산하면 5MHz, 1클럭 당 20us 소요, TBPRD 1000으로 세팅하면 주기 20ms
//인줄 알았으나 오실로를 찍어보니 주기가 400us 가 나와서 조사해본 결과 F2807X 모델은 TBCLK대신 EPWMCLK를 사용하며, EPWMCLK = SYSCLK/2 임.
//
// servocenter 150
#define PWM_PERIOD      50000
#define SERVO_CENTER    3750   // 중립 (정지)
#define CCW_DUTY_RATIO  8
#define CW_DUTY_RATIO   7
#define CCW_CMPA        ((PWM_PERIOD / 100) * CCW_DUTY_RATIO)   // 반시계
#define CW_CMPA         ((PWM_PERIOD / 100) * CW_DUTY_RATIO)    // 시계

Uint32 pwmA = SERVO_CENTER;

//서보모터 : UART 수신 버퍼
#define RX_BUFFER_SIZE  32
char rxBuffer[RX_BUFFER_SIZE];
Uint16 rxIndex = 0;
Uint16 dataReady = 0;

//서보모터 :  조이스틱
int joystickX = 512;  // 중립값
int joystickY = 512;
// 조이스틱 데드존 설정 (중립 인식 범위)
#define DEADZONE_LOW    200
#define DEADZONE_HIGH   500



//짐벌 : 짐벌을 위한 매크로 정의
#define MPU9250_ADDR    0x68
#define WHO_AM_I        0x75
#define PWR_MGMT_1      0x6B
#define ACCEL_XOUT_H    0x3B
#define GYRO_XOUT_H     0x43

float Pitch = 0;  // 피치 각도 (전후 기울기) [radian]
float Roll = 0;   // 롤 각도 (좌우 기울기) [radian]

//짐벌 : 입력값을 MIN~MAX 범위로 제한하는 매크로
#define CLAMP(IN,MIN,MAX) (IN>MAX?MAX:IN<MIN?MIN:IN)
#define M_PI 3.14159265358979323846  // 라디안 변환용 상수



// 짐벌을 위한.. ~
Uint16 whoAmI = 0;
Uint16 i2cOK = 0;

int16 accelX, accelY, accelZ;
int16 gyroX, gyroY, gyroZ;

volatile Uint32 loopCount = 0;
volatile Uint16 initDone = 0;
volatile Uint32 timerCount = 0;  // 타이머 동작 확인용

float roll = 0.0f;
float pitch = 0.0f;
float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float dt = 0.01f;

float accelX_g, accelY_g, accelZ_g;     // g 단위
float gyroX_dps, gyroY_dps, gyroZ_dps;  // deg/s 단위

#define SYSTEM_CLOCK        120E6   // 시스템 클럭: 150MHz
#define TBCLK               60E6   // EPWM 타임베이스 클럭: 150MHz ( F2007x 모델은 /2
#define PWM_CARRIER         20E3    // PWM 주파수: 20kHz
#define PWM_DUTY_RATIO_A    0       // 초기 듀티비 A: 0%
#define PWM_DUTY_RATIO_B    0       // 초기 듀티비 B: 0%


float32 PwmCarrierFrequency;   // PWM 캐리어 주파수
float32 PwmDutyRatioA;         // PWM 채널 A 듀티비
float32 PwmDutyRatioB;         // PWM 채널 B 듀티비
float32 Cpu_Clk;               // CPU 클럭 (MHz)
float32 Timer_Prd1 = 500;      // Timer0 주기: 500us = 2kHz (IMU 데이터 수신)
float32 Timer_Prd2 = 500;      // Timer1 주기: 500us = 2kHz (모터 제어)
float32 Timer_prd4 = 900;      // Timer2 주기: 900us (미사용)


// PID 제어 파라미터 - Pitch 제어 (전후 안정화)
//===================================================================
float32 KP_Pitch = 39.7;       // 비례 게인
float32 KI_Pitch = 0.0;        // 적분 게인 (현재 미사용)
float32 KD_Pitch = 12.4;       // 미분 게인
float32 TARGET_Pitch = 0.0;    // 목표 피치 각도 (수평: 0도)

//===================================================================
// PID 제어 파라미터 - Roll 제어 (좌우 안정화)
//===================================================================
float32 KP_Roll = 39.7;        // 비례 게인
float32 KI_Roll = 0.0;         // 적분 게인 (현재 미사용)
float32 KD_Roll = 12.4;        // 미분 게인
float32 TARGET_Roll = 0.0;     // 목표 롤 각도 (수평: 0도)

//===================================================================
// 모터 출력 변수 (4개 리니어 액추에이터)
//===================================================================
float32 Motor1_Output = 0.0;   // 모터1 제어 출력 [-1.0 ~ 1.0]
float32 Motor2_Output = 0.0;   // 모터2 제어 출력
float32 Motor3_Output = 0.0;   // 모터3 제어 출력
float32 Motor4_Output = 0.0;   // 모터4 제어 출력

//===================================================================
// PID 제어기 출력
//===================================================================
float32 ControllerOutput = 0.0;   // 최종 제어 출력 (클램핑 후)
float32 ControllerOutput1 = 0.0;  // Pitch 제어 출력
float32 ControllerOutput2 = 0.0;  // Roll 제어 출력

//===================================================================
// PID 제어 계산용 변수
//===================================================================
/*
float32 dt = 0;   // 샘플링 주기 (Timer_Prd1 기반)
float32 dt2 = 0;  // Timer_Prd2 기반 샘플링 주기
float32 dt3 = 0;  // Timer_prd4 기반 샘플링 주기
*/
// Pitch PID 제어 변수
float32 PitchError = 0;         // 현재 오차
float32 PitchErrorPrev = 0.0;   // 이전 오차 (미분 계산용)
float32 P_Pitch_Term = 0;       // 비례항
float32 I_Pitch_Term = 0;       // 적분항
float32 D_Pitch_Term = 0;       // 미분항

// Roll PID 제어 변수
float32 RollError = 0;          // 현재 오차
float32 RollErrorPrev = 0;      // 이전 오차
float32 P_Roll_Term = 0;        // 비례항
float32 I_Roll_Term = 0;        // 적분항
float32 D_Roll_Term = 0;        // 미분항




//서보 관련 함수 선언
void GetJoystickData(void);
void InitScia(void);
void InitEpwm7(void);
void UpdateServo(int xValue, int yValue);
Uint16 receiveCount = 0;



//짐벌 관련 함수 선언
void InitI2C(void); //IMU 시작
void InitTimer0(void); // IMU 데이터 수신을 위한 타이머
Uint16 ReadByte(Uint16 reg);
void WriteByte(Uint16 reg, Uint16 data);
void UpdateAngle(void); //IMU 각속도,가속도를 ANGLE값으로 변환

void InitEPwm1Module(void);  // PWM1 모듈 초기화 (모터 1,2 제어)
void InitEPwm2Module(void);  // PWM2 모듈 초기화 (모터 3,4 제어)
interrupt void EPwm1Isr(void);  // PWM1 인터럽트 서비스 루틴
interrupt void EPwm2Isr(void);  // PWM2 인터럽트 서비스 루틴
__interrupt void cpu_timer0_isr(void);  // Timer0 ISR: IMU 데이터 수신 및 PID 계산
__interrupt void cpu_timer1_isr(void);  // Timer1 ISR: 모터 제어 출력



void PID_Pitch_controller();   // Pitch PID 제어 함수
void PID_Roll_controller();    // Roll PID 제어 함수
void origin_motor(void);       // 모터 원점 복귀 (초기화)
void Motor_contoller(float32 ControllerOutput1, float32 ControllerOutput2);  // 모터 제어 출력

//===================================================================
// PID 제어기 - Pitch (전후 기울기 안정화)
//===================================================================
void PID_Pitch_controller() {
    // 1. 오차 계산: 목표값 - 현재값
    PitchError = TARGET_Pitch - Pitch;

    // 2. PID 항 계산
    P_Pitch_Term = KP_Pitch * PitchError;                        // 비례항: 현재 오차에 비례
    I_Pitch_Term += KI_Pitch * PitchError * dt;                 // 적분항: 오차 누적
    D_Pitch_Term = KD_Pitch * ((PitchError - PitchErrorPrev) / dt); // 미분항: 오차 변화율

    // 3. 이전 오차 저장 (다음 미분 계산용)
    PitchErrorPrev = PitchError;

    // 4. 최종 제어 출력 = P + I + D
    ControllerOutput1 = P_Pitch_Term + I_Pitch_Term + D_Pitch_Term;
}

//===================================================================
// PID 제어기 - Roll (좌우 기울기 안정화)
//===================================================================
void PID_Roll_controller() {
    // Pitch와 동일한 PID 구조
    RollError = TARGET_Roll - Roll;

    P_Roll_Term = KP_Roll * RollError;
    I_Roll_Term += KI_Roll * RollError * dt;
    D_Roll_Term = KD_Roll * ((RollError - RollErrorPrev) / dt);
    RollErrorPrev = RollError;

    ControllerOutput2 = P_Roll_Term + I_Roll_Term + D_Roll_Term;
}

//===================================================================
// 모터 원점 복귀 함수
// - 시스템 시작 시 모터를 완전히 늘렸다가 줄여서 중간 위치로 이동
//===================================================================
void origin_motor(void)
{
    // 1단계: 모든 모터를 한 방향으로 완전히 이동 (상방향)
    PwmDutyRatioA = 0.5;  // 50% 듀티비로 구동
    PwmDutyRatioB = 0.5;

    // GPIO로 모터 방향 설정 (상방향)
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;   // 모터3 방향 설정
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
    GpioDataRegs.GPCSET.bit.GPIO64 = 1;   // 모터4 방향 설정
    GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;
    GpioDataRegs.GPCSET.bit.GPIO68 = 1;   // 모터2 방향 설정
    GpioDataRegs.GPCCLEAR.bit.GPIO69 = 1;
    GpioDataRegs.GPCSET.bit.GPIO70 = 1;   // 모터1 방향 설정
    GpioDataRegs.GPCCLEAR.bit.GPIO71 = 1;

    // PWM 레지스터 업데이트
    EPwm1Regs.TBPRD = (TBCLK / PwmCarrierFrequency) - 1;
    EPwm1Regs.CMPA.bit.CMPA = (EPwm1Regs.TBPRD + 1) * PwmDutyRatioA;
    EPwm1Regs.CMPB.bit.CMPB = (EPwm1Regs.TBPRD + 1) * PwmDutyRatioB;

    EPwm2Regs.TBPRD = (TBCLK / PwmCarrierFrequency) - 1;
    EPwm2Regs.CMPA.bit.CMPA = (EPwm2Regs.TBPRD + 1) * PwmDutyRatioA;
    EPwm2Regs.CMPB.bit.CMPB = (EPwm2Regs.TBPRD + 1) * PwmDutyRatioB;

    DELAY_US(2500000);  // 2.5초 대기 (완전히 이동할 때까지)

    // 2단계: 반대 방향으로 완전히 이동 (하방향)
    GpioDataRegs.GPCSET.bit.GPIO67 = 1;   // 모든 모터 방향 반전
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    GpioDataRegs.GPCSET.bit.GPIO65 = 1;
    GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;
    GpioDataRegs.GPCSET.bit.GPIO69 = 1;
    GpioDataRegs.GPCCLEAR.bit.GPIO68 = 1;
    GpioDataRegs.GPCSET.bit.GPIO71 = 1;
    GpioDataRegs.GPCCLEAR.bit.GPIO70 = 1;


    DELAY_US(10000000);  // 10초 대기 (완전히 이동 + 여유시간)

    // 3단계: 모터 정지
    PwmDutyRatioA = 0.0;
    PwmDutyRatioB = 0.0;
    EPwm1Regs.CMPA.bit.CMPA = 0;
    EPwm1Regs.CMPB.bit.CMPB = 0;
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;

    DELAY_US(1000000);  // 1초 정지 유지
}

//===================================================================
// 모터 제어 출력 함수
// - PID 출력을 받아서 4개 모터의 출력값과 방향을 결정
//===================================================================
void Motor_contoller(float32 ControllerOutput1, float32 ControllerOutput2) {

    // 전체 출력 클램핑 (-1 ~ 1 범위로 제한)
    ControllerOutput = CLAMP(ControllerOutput1 + ControllerOutput2, -1, 1);

    // Pitch 제어 적용 (전후 기울기 보정)
    // 앞이 기울면(+): 모터1,4는 늘리고(+), 모터2,3은 줄임(-)
    Motor1_Output += ControllerOutput1;
    Motor4_Output += ControllerOutput1;
    Motor2_Output -= ControllerOutput1;
    Motor3_Output -= ControllerOutput1;

    // Roll 제어 적용 (좌우 기울기 보정)
    // 오른쪽이 기울면(+): 모터1,2는 늘리고(+), 모터3,4는 줄임(-)
    Motor1_Output += ControllerOutput2;
    Motor2_Output += ControllerOutput2;
    Motor3_Output -= ControllerOutput2;
    Motor4_Output -= ControllerOutput2;

    // 각 모터 출력 클램핑
    Motor1_Output = CLAMP(Motor1_Output, -1, 1);
    Motor2_Output = CLAMP(Motor2_Output, -1, 1);
    Motor3_Output = CLAMP(Motor3_Output, -1, 1);
    Motor4_Output = CLAMP(Motor4_Output, -1, 1);

    // GPIO 핀 매핑:
    // GPIO68/69: 모터2 (INA/INB)
    // GPIO66/67: 모터3 (INA/INB)
    // GPIO64/65: 모터4 (INA/INB)
    // GPIO70/71: 모터1 (INA/INB)

    // 모터1 방향 설정
    if (Motor1_Output > 0) {
        GpioDataRegs.GPCSET.bit.GPIO68 = 1;     // 정방향
        GpioDataRegs.GPCCLEAR.bit.GPIO69 = 1;
    }
    else {
        GpioDataRegs.GPCSET.bit.GPIO69 = 1;     // 역방향
        GpioDataRegs.GPCCLEAR.bit.GPIO68 = 1;
    }

    // 모터2 방향 설정
    if (Motor2_Output > 0) {
        GpioDataRegs.GPCSET.bit.GPIO66 = 1;
        GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;
    }
    else {
        GpioDataRegs.GPCSET.bit.GPIO67 = 1;
        GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    }

    // 모터3 방향 설정
    if (Motor3_Output > 0) {
        GpioDataRegs.GPCSET.bit.GPIO64 = 1;
        GpioDataRegs.GPCCLEAR.bit.GPIO65 = 1;
    }
    else {
        GpioDataRegs.GPCSET.bit.GPIO65 = 1;
        GpioDataRegs.GPCCLEAR.bit.GPIO64 = 1;
    }

    // 모터4 방향 설정
    if (Motor4_Output > 0) {
        GpioDataRegs.GPCSET.bit.GPIO70 = 1;
        GpioDataRegs.GPCCLEAR.bit.GPIO71 = 1;
    }
    else {
        GpioDataRegs.GPCSET.bit.GPIO71 = 1;
        GpioDataRegs.GPCCLEAR.bit.GPIO70 = 1;
    }

    // PWM 듀티비 설정 (절댓값 사용 - 속도만 제어, 방향은 GPIO로 제어)
    PwmDutyRatioA = fabs(ControllerOutput);
    PwmDutyRatioB = fabs(ControllerOutput);
}




void main(void)
{
    EALLOW;
    WdRegs.WDCR.all = 0x0068;
    EDIS;

    InitSysCtrl();

    EALLOW;

    // 디버깅을 위한 LED
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;

    // I2C pins
    GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO32 = 3;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;

    GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO33 = 0;
    GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;

    //EPWM7A, EPWM7B (서보모터)
    GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;

    //EPWM1A,1B , EPWM2A,2B 세팅
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;


    GpioCtrlRegs.GPAGMUX1.bit.GPIO0 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;

    GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;


    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;




    //SCI-A rx만,,,
    GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;
    GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO28 = 0;    // ✅ GMUX = 0
    GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;


    EDIS;

    InitI2C();
    InitEpwm7();
    InitScia();
    DELAY_US(100000);


/* 삭제 희망 !!
    whoAmI = ReadByte(WHO_AM_I);

    if(whoAmI == 0x70 || whoAmI == 0x71 || whoAmI == 0x73)
    {
        i2cOK = 1;
        GpioDataRegs.GPBSET.bit.GPIO34 = 1;

        WriteByte(PWR_MGMT_1, 0x00);
        DELAY_US(100000);

        // 가속도 설정: ±4g
       WriteByte(0x1C, 0x08);  // ACCEL_CONFIG = 0x08 (±4g)
       DELAY_US(10000);

           // 자이로 설정: ±500°/s (현재 유지)
       WriteByte(0x1B, 0x08);  // GYRO_CONFIG = 0x08 (±500°/s)
       DELAY_US(10000);

        initDone = 1;

        InitTimer0();
    }
    */


    while(1)
    {


        // UART 데이터 수신
           while(SciaRegs.SCIRXST.bit.RXRDY == 1)
           {
               char receivedChar = SciaRegs.SCIRXBUF.all;

               if(receivedChar == '\n')
               {
                   rxBuffer[rxIndex] = '\0';
                   dataReady = 1;
                   receiveCount++;
                   rxIndex = 0;

                   break;
               }
               else
               {
                   if(rxIndex < RX_BUFFER_SIZE - 1)
                   {
                       rxBuffer[rxIndex++] = receivedChar;
                   }
               }
           }

           // 데이터 파싱 및 서보 제어
           if(dataReady)
           {
               dataReady = 0;
               GetJoystickData();
           }


        loopCount++;

        // Breakpoint here!
        if(loopCount % 1000000 == 0)
        {
            GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
        }
    }
}



//함수 순서
/*
 1. InitI2C (짐벌)
 2. InitTimer0 ( 짐벌 )
 GetJoystickData
 IniteScia(서보)
 3. InitEpwm ( 서보 )
 4. __cput_timer_ISR ( 짐벌, IMU데이터 )
 5. I2C REAN & WRITE DATA
 */
void InitI2C(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR9.bit.I2C_A = 1;
    EDIS;

    I2caRegs.I2CMDR.bit.IRS = 0;

    I2caRegs.I2CPSC.all = 11;
    I2caRegs.I2CCLKL = 50;
    I2caRegs.I2CCLKH = 50;

    I2caRegs.I2CMDR.bit.IRS = 1;
    DELAY_US(1000);
}

void InitTimer0(void)
{
    EALLOW;

    CpuSysRegs.PCLKCR0.bit.CPUTIMER0 = 1;

    PieVectTable.TIMER0_INT = &cpu_timer0_isr;

    EDIS;

    InitCpuTimers();

    ConfigCpuTimer(&CpuTimer0, 100, 10000);

    CpuTimer0Regs.TCR.all = 0x4000;

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // PIE 활성화
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;   // Timer0 인터럽트 활성화

    IER |= M_INT1;


    EINT;
    ERTM;
}

// ========== 조이스틱 데이터  ==========
void GetJoystickData(void)
{
    if(rxBuffer[0] == 'J')
    {
        char *xPtr = NULL;
        char *yPtr = NULL;
        int i;

        // 'X'와 'Y' 위치 찾기
        for(i = 0; rxBuffer[i] != '\0'; i++)
        {
            if(rxBuffer[i] == 'X') xPtr = &rxBuffer[i+1];
            if(rxBuffer[i] == 'Y') yPtr = &rxBuffer[i+1];
        }

        if(xPtr != NULL && yPtr != NULL)
        {
            joystickX = atoi(xPtr);
            joystickY = atoi(yPtr);

            // 범위 제한
            if(joystickX < 0) joystickX = 0;
            if(joystickX > 1023) joystickX = 1023;
            if(joystickY < 0) joystickY = 0;
            if(joystickY > 1023) joystickY = 1023;

            // 서보모터 제어
            UpdateServo(joystickX, joystickY);
        }
    }
}

void InitScia(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
    EDIS;

    DELAY_US(1000);

    SciaRegs.SCICTL1.bit.SWRESET = 0;

    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x0061;

    SciaRegs.SCICCR.bit.SCICHAR = 7;
    SciaRegs.SCICCR.bit.PARITYENA = 0;
    SciaRegs.SCICCR.bit.STOPBITS = 0;
    SciaRegs.SCICCR.bit.LOOPBKENA = 0;

    SciaRegs.SCICTL1.bit.RXENA = 1;
    SciaRegs.SCICTL1.bit.TXENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 0;
    SciaRegs.SCICTL2.bit.TXINTENA = 0;

    SciaRegs.SCICTL1.bit.SWRESET = 1;
}

// ========== EPWM1 초기화 ==========

void UpdateServo(int xValue, int yValue)
{
    Uint32 pwmB;

    // ===== X축 → 서보1 제어 =====
    if(xValue < DEADZONE_LOW)
    {
        // 조이스틱 왼쪽 → 서보1 CCW (반시계)
        pwmA += 50;
        if(pwmA > 20000) pwmA = 20000;
    }
    else if(xValue > DEADZONE_HIGH)
    {
        // 조이스틱 오른쪽 → 서보1 CW (시계)
        pwmA -= 50;
        if(pwmA < 50) pwmA = 50;
    }
    else
    {

    }

    // ===== Y축 → 서보2 제어 =====
    if(yValue < DEADZONE_LOW)
    {
        // 조이스틱 아래쪽 → 서보2 CCW (반시계)
        pwmB = CCW_CMPA;
    }
    else if(yValue > DEADZONE_HIGH)
    {
        // 조이스틱 위쪽 → 서보2 CW (시계)
        pwmB = CW_CMPA;
    }
    else
    {
        // 조이스틱 중립 → 서보2 정지
        pwmB = SERVO_CENTER;
    }

    // PWM 레지스터 업데이트
    EPwm7Regs.CMPA.bit.CMPA = pwmA;
    EPwm7Regs.CMPB.bit.CMPB = pwmB;
}

void InitEpwm7(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    EPwm7Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;
    EPwm7Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm7Regs.TBCTL.bit.PRDLD = TB_SHADOW;
    EPwm7Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;

    EPwm7Regs.TBCTL.bit.HSPCLKDIV = 0x3; // TBCLK / 6
    EPwm7Regs.TBCTL.bit.CLKDIV = 0X2;  // / TBCLK / 6 / 4  =  5MHz, 카운트당

    EPwm7Regs.TBPRD = PWM_PERIOD;
    EPwm7Regs.TBCTR = 0x0000;

    EPwm7Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
    EPwm7Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm7Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;

    // 초기값: 정지
    EPwm7Regs.CMPA.bit.CMPA = SERVO_CENTER;
    EPwm7Regs.CMPB.bit.CMPB = SERVO_CENTER;

    EPwm7Regs.AQCTLA.bit.ZRO = AQ_SET;
    EPwm7Regs.AQCTLA.bit.CAU = AQ_CLEAR;

    EPwm7Regs.AQCTLB.bit.ZRO = AQ_SET;
    EPwm7Regs.AQCTLB.bit.CBU = AQ_CLEAR;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;
}


__interrupt void cpu_timer0_isr(void)
{
    timerCount++;  // 타이머 동작 확인

    if(i2cOK && initDone)
    {
        Uint16 temp_h, temp_l;

        // Accel (간단하게 X만)
        temp_h = ReadByte(ACCEL_XOUT_H);
        temp_l = ReadByte(ACCEL_XOUT_H + 1);
        accelX = (int16)((temp_h << 8) | temp_l);

        temp_h = ReadByte(ACCEL_XOUT_H + 2);
        temp_l = ReadByte(ACCEL_XOUT_H + 3);
        accelY = (int16)((temp_h << 8) | temp_l);

        temp_h = ReadByte(ACCEL_XOUT_H + 4);
        temp_l = ReadByte(ACCEL_XOUT_H + 5);
        accelZ = (int16)((temp_h << 8) | temp_l);

        // Gyro
        temp_h = ReadByte(GYRO_XOUT_H);
        temp_l = ReadByte(GYRO_XOUT_H + 1);
        gyroX = (int16)((temp_h << 8) | temp_l);

        temp_h = ReadByte(GYRO_XOUT_H + 2);
        temp_l = ReadByte(GYRO_XOUT_H + 3);
        gyroY = (int16)((temp_h << 8) | temp_l);

        temp_h = ReadByte(GYRO_XOUT_H + 4);
        temp_l = ReadByte(GYRO_XOUT_H + 5);
        gyroZ = (int16)((temp_h << 8) | temp_l);

        UpdateAngle();
    }

    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

Uint16 ReadByte(Uint16 reg)
{
    Uint16 data;
    Uint16 timeout;

    I2caRegs.I2CSAR.all = MPU9250_ADDR;
    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CDXR.all = reg;
    I2caRegs.I2CMDR.all = 0x6620;

    timeout = 10000;
    while(I2caRegs.I2CSTR.bit.ARDY == 0 && timeout > 0)
    {
        timeout--;
    }
    if(timeout == 0) return 0xFFFF;

    I2caRegs.I2CCNT = 1;
    I2caRegs.I2CMDR.all = 0x6C20;

    timeout = 10000;
    while(I2caRegs.I2CSTR.bit.RRDY == 0 && timeout > 0)
    {
        timeout--;
    }
    if(timeout == 0) return 0xFFFF;

    data = I2caRegs.I2CDRR.all;

    timeout = 10000;
    while(I2caRegs.I2CMDR.bit.STP == 1 && timeout > 0)
    {
        timeout--;
    }

    return data;
}

void WriteByte(Uint16 reg, Uint16 data)
{
    Uint16 timeout;

    I2caRegs.I2CSAR.all = MPU9250_ADDR;
    I2caRegs.I2CCNT = 2;
    I2caRegs.I2CDXR.all = reg;
    I2caRegs.I2CMDR.all = 0x6E20;

    timeout = 10000;
    while(I2caRegs.I2CSTR.bit.XRDY == 0 && timeout > 0)
    {
        timeout--;
    }
    if(timeout == 0) return;

    I2caRegs.I2CDXR.all = data;

    timeout = 10000;
    while(I2caRegs.I2CSTR.bit.ARDY == 0 && timeout > 0)
    {
        timeout--;
    }

    I2caRegs.I2CMDR.bit.STP = 1;

    timeout = 10000;
    while(I2caRegs.I2CMDR.bit.STP == 1 && timeout > 0)
    {
        timeout--;
    }
}

void UpdateAngle(void)
{
    float accel_roll, accel_pitch;
    float gyro_roll_rate, gyro_pitch_rate;

    // Raw 데이터를 실제 단위로 변환
    accelX_g = (float)accelX / 16384.0f;    // ±2g 설정
    accelY_g = (float)accelY / 16384.0f;
    accelZ_g = (float)accelZ / 16384.0f;

    gyroX_dps = (float)gyroX / 65.5f;       // ±500°/s 설정
    gyroY_dps = (float)gyroY / 65.5f;
    gyroZ_dps = (float)gyroZ / 65.5f;

    // 각도 계산 (변환된 값 사용)
    accel_roll = atan2f(accelY_g, accelZ_g);
    accel_pitch = atan2f(-accelX_g, sqrtf(accelY_g*accelY_g + accelZ_g*accelZ_g));

    gyro_roll_rate = gyroX_dps * (3.14159f / 180.0f);   // deg/s → rad/s
    gyro_pitch_rate = gyroY_dps * (3.14159f / 180.0f);

    // 상보 필터
    Roll = 0.98f * (roll + gyro_roll_rate * dt) + 0.02f * accel_roll;
    Pitch = 0.98f * (pitch + gyro_pitch_rate * dt) + 0.02f * accel_pitch;

}

void InitEPwm1Module(void)
{
    // 카운터 모드 설정: Up-count (0에서 TBPRD까지 증가)
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;

    // 클럭 설정: SYSCLKOUT / (HSPCLKDIV × CLKDIV) = 150MHz / 1
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;  // ÷1
    EPwm1Regs.TBCTL.bit.CLKDIV = 0;     // ÷1

    // 위상 설정 (다른 PWM과 동기화 시 사용)
    EPwm1Regs.TBPHS.half.TBPHS = 0;     // 위상 = 0
    EPwm1Regs.TBCTL.bit.PHSEN = 0;      // 위상 로딩 비활성화

    // PWM 주기 설정: (150MHz / 20kHz) - 1 = 7499
    EPwm1Regs.TBPRD = (TBCLK / PWM_CARRIER) - 1;
    EPwm1Regs.TBCTR = 0;  // 카운터 초기화

    // 비교 값 설정 (듀티비 결정)
    EPwm1Regs.CMPA.bit.CMPA = (Uint16)((EPwm1Regs.TBPRD + 1) * PwmDutyRatioA);
    EPwm1Regs.CMPB.bit.CMPB = (Uint16)((EPwm1Regs.TBPRD + 1) * PwmDutyRatioB);

    // Shadow 레지스터 설정 (카운터=0일 때 업데이트)
    EPwm1Regs.TBCTL.bit.PRDLD = 0;      // TBPRD shadow 활성화
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 0; // CMPA shadow 활성화
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 0; // CTR=0에서 로드
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 0; // CMPB shadow 활성화
    EPwm1Regs.CMPCTL.bit.LOADBMODE = 0;

    // Action Qualifier 설정 (PWM 파형 생성)
    EPwm1Regs.AQCTLA.bit.ZRO = 2;  // CTR=0: EPWMxA를 High로 Set
    EPwm1Regs.AQCTLA.bit.CAU = 1;  // CTR=CMPA: EPWMxA를 Low로 Clear
    EPwm1Regs.AQCTLB.bit.ZRO = 2;  // CTR=0: EPWMxB를 High로 Set
    EPwm1Regs.AQCTLB.bit.CBU = 1;  // CTR=CMPB: EPWMxB를 Low로 Clear

    // 인터럽트 설정
    EPwm1Regs.ETSEL.bit.INTSEL = 1;  // CTR=0에서 인터럽트 발생
    EPwm1Regs.ETPS.bit.INTPRD = 1;   // 매 이벤트마다 인터럽트
    EPwm1Regs.ETSEL.bit.INTEN = 1;   // 인터럽트 활성화
}

//===================================================================
// PWM2 모듈 초기화 (PWM1과 동일한 구조)
//===================================================================
void InitEPwm2Module(void)
{
    EPwm2Regs.TBCTL.bit.CTRMODE = 0;
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
    EPwm2Regs.TBCTL.bit.CLKDIV = 0;
    EPwm2Regs.TBPHS.half.TBPHS = 0;
    EPwm2Regs.TBCTL.bit.PHSEN = 0;
    EPwm2Regs.TBPRD = (TBCLK / PWM_CARRIER) - 1;
    EPwm2Regs.TBCTR = 0;
    EPwm2Regs.CMPA.bit.CMPA = (Uint16)((EPwm2Regs.TBPRD + 1) * PwmDutyRatioA);
    EPwm2Regs.CMPB.bit.CMPB = (Uint16)((EPwm2Regs.TBPRD + 1) * PwmDutyRatioB);
    EPwm2Regs.TBCTL.bit.PRDLD = 0;
    EPwm2Regs.CMPCTL.bit.SHDWAMODE = 0;
    EPwm2Regs.CMPCTL.bit.LOADAMODE = 0;
    EPwm2Regs.CMPCTL.bit.SHDWBMODE = 0;
    EPwm2Regs.CMPCTL.bit.LOADBMODE = 0;
    EPwm2Regs.AQCTLA.bit.ZRO = 2;
    EPwm2Regs.AQCTLA.bit.CAU = 1;
    EPwm2Regs.AQCTLB.bit.ZRO = 2;
    EPwm2Regs.AQCTLB.bit.CBU = 1;
    EPwm2Regs.ETSEL.bit.INTSEL = 1;
    EPwm2Regs.ETPS.bit.INTPRD = 1;
    EPwm2Regs.ETSEL.bit.INTEN = 1;
}


/*

## 확인 순서

**1. 기본 동작 확인**
```
whoAmI = ?
i2cOK = ?
initDone = ?
```

**2. 타이머 확인**
```
timerCount 증가? → 타이머 동작 중
timerCount = 0? → 타이머 안 도는 중
```

**3. 센서 데이터 확인**
```
accelX, Y, Z = ?
gyroX, Y, Z = ?
```

**4. 각도 확인**
```
roll_deg = ?
pitch_deg = ?
*/
