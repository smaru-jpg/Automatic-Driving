# Firmware Sources

이 폴더에는 원본 텍스트 파일에서 MCU별로 분리한 펌웨어 소스가 들어 있습니다.

## STM32 motor recording

`stm32_recording/main.c`

- STM32 HAL 기반 모터 PWM 및 엔코더 제어
- 버튼 입력으로 주행 기록 시작/중지
- 기록된 모터 PWM 값 재생

원본 코드에서 참조하는 핵심 HAL 헤더와 설정은 `main.h`, CubeMX 생성 peripheral 초기화 파일 및 STM32 HAL 드라이버를 별도로 생성해야 합니다.

## STM32 joystick controller

`stm32_joystick/main.c`

- ADC 조이스틱 입력
- 주행 모드와 카메라 모드 메뉴
- UART 프로토콜 송수신
- I2C LCD 및 UART 중계

이 소스는 `main.h`, `i2c_lcd.h`, ADC/DMA/I2C/UART 초기화와 MCU 전용 HAL 드라이버를 포함하는 CubeMX STM32 프로젝트와 함께 사용해야 합니다.

## TI C2000 F2807x

`c2000_f2807x/main.c`

- MPU9250 가속도/자이로 각도 계산
- 서보 조이스틱 제어
- Roll/Pitch PID 제어
- EPWM 기반 4축 모터 출력

Texas Instruments Code Composer Studio와 C2000Ware 또는 F2807x 공용 장치 헤더가 필요합니다. 원본 프로젝트의 `F2807x_device.h`, `F2807x_Examples.h`, 시스템 초기화, 인터럽트 벡터 설정을 함께 구성해야 합니다.

## 주의사항

분리 작업은 두 번째 소스 간 경계와 각 플랫폼용 코드 덩어리를 복원한 것이며, MCU IDE 프로젝트 전체를 재생성한 것은 아닙니다. 실제 보드에서 빌드/검증하려면 각 소스가 사용하는 헤더와 주변장치 설정을 원본 프로젝트 구성과 일치시켜야 합니다.
