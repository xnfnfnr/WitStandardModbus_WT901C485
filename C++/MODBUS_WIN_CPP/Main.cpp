
#include "string.h"
#include "Math.h"
#include "windows.h"
#include "Com.h"

#include <stdio.h>
#include <conio.h>
#include "wit_c_sdk.h"

#include <setupapi.h>
#include <devguid.h>
#include <regstr.h>
#pragma comment(lib, "setupapi.lib")

static char s_cDataUpdate = 0;
int iComPort = -1;
int iBaud = 230400;
int iAddress = 0x50;

volatile LONG g_ComThreadStop = 0;

// ===== Named Pipe =====
HANDLE hPipe = INVALID_HANDLE_VALUE;

// ===== Graceful shutdown event =====
// SDmotionDlg sets this event to signal WitSimulate to shut down cleanly.
// This allows CloseCOMDevice() to be called so the USB serial driver
// releases properly (avoids needing USB unplug/replug on restart).
static HANDLE g_hShutdownEvent = INVALID_HANDLE_VALUE;

// ===== sensor connection =====
static volatile ULONGLONG g_lastUpdateTick = 0;

// ===== Config (ini 파일에서 읽기) =====
static int g_ConfigComPort = -1;			// 설정 파일에서 읽어온 COM 포트 번호
static DWORD g_DisconnectTimeoutMs = 5000;	// 이 시간 동안 데이터가 없으면 연결 끊김으로 판단
static DWORD g_RescanIntervalMs = 5000;		// 연결 끊김 감지 후 재스캔 간격
static DWORD g_ReadIntervalMs = 100;		// 읽기 간격

void SendToPipe(float pitch, float roll);
void ComRxCallBack(char* p_data, UINT32 uiSize);
static void DelayMs(uint16_t ms);
static void SensorUartSend(uint8_t* p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static int AutoScanSensor(void);
static int LoadConfig(void);
static int TryFindSensorOnPort(int comPort);
static int ConnectSensorOnPort(int comPort);
static int ScanAndReconnectSensor(void);

//static int EnumerateComPorts(int* ports, int maxPorts);

// Ctrl+C, Ctrl+Break, 창 닫힘, 시스템 종료 등 모든 종료 신호 처리
static BOOL WINAPI ConsoleCtrlHandler(DWORD ctrlType)
{
	printf("ConsoleCtrlHandler: ctrlType=%lu, closing COM port cleanly...\n", ctrlType);
	if (iComPort >= 0)
		CloseCOMDevice();
	if (hPipe != INVALID_HANDLE_VALUE)
	{
		CloseHandle(hPipe);
		hPipe = INVALID_HANDLE_VALUE;
	}
	return FALSE; // 기본 핸들러도 실행되도록
}

void main(void)
{
	float roll = 0.0f, pitch = 0.0f;

	// 어떤 방식으로 종료되어도 CloseCOMDevice()가 호출되도록 등록
	SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);

	// Create the shutdown event (non-signaled). SDmotionDlg will open
	// this by name and set it to trigger graceful exit.
	g_hShutdownEvent = CreateEventA(NULL, TRUE, FALSE, "TiltSensor_Shutdown");

	// 설정 파일 로드
	LoadConfig();

	// 초기 센서 연결
	if (!ScanAndReconnectSensor())
	{
		printf("Sensor not found at startup. Entering disconnected mode.\n");
		// 여기서 return하지 않고, 루프 안에서 주기적으로 재연결 시도
	}

	DWORD lastRescanTick = 0;

	while (1)
	{
		// Check for graceful shutdown signal from SDmotionDlg.
		// This ensures CloseCOMDevice() is called so the USB serial
		// driver is released cleanly (prevents needing USB unplug/replug).
		if (g_hShutdownEvent != INVALID_HANDLE_VALUE &&
			WaitForSingleObject(g_hShutdownEvent, 0) == WAIT_OBJECT_0)
		{
			printf("Shutdown signal received. Closing COM port cleanly...\n");
			if (iComPort >= 0)
				CloseCOMDevice();
			if (hPipe != INVALID_HANDLE_VALUE)
			{
				CloseHandle(hPipe);
				hPipe = INVALID_HANDLE_VALUE;
			}
			CloseHandle(g_hShutdownEvent);
			return;
		}

		ULONGLONG now = GetTickCount64();  // GetTickCount()?? GetTickCount64() ????
		ULONGLONG lastUpdate = InterlockedCompareExchange64((volatile LONGLONG*)&g_lastUpdateTick, 0, 0);

		// 1) 연결된 포트가 있으면 데이터 읽기 시도
		if (iComPort >= 0)
		{
			s_cDataUpdate = 0;
			WitReadReg(Roll, 2);

			// 실제로는 sReg에서 읽음 (참고: 콜백에서 이미 값이 업데이트됨)
			roll = (float)sReg[Roll] / 32768.0f * 180.0f;
			pitch = (float)sReg[Pitch] / 32768.0f * 180.0f;
		}

		// 2) 연결 상태 확인 (마지막 DISCONNECT_TIMEOUT_MS 내 업데이트 여부)
		int needRescan = 1;
		if (lastUpdate != 0 && (now - lastUpdate) <= g_DisconnectTimeoutMs)
			needRescan = 0;

		if (!needRescan)
		{
			// "연결됨 상태" 일 때 데이터 전송
			SendToPipe(roll, pitch);
			printf("\rRoll: %7.3f Pitch: %7.3f    ", roll, pitch);
		}
		else
		{
			// "연결 끊김 상태" 일 때 재스캔: DISCONNECT_TIMEOUT_MS마다 한 번씩
			if ((now - lastRescanTick) >= g_RescanIntervalMs)
			{
				lastRescanTick = now;

				if (!ScanAndReconnectSensor())
				{
					// 재연결 실패: 포트 닫기/초기화
					iComPort = -1;
					CloseCOMDevice(); // 혹시 열린 핸들이 있으면 닫기
				}
			}

			// 연결 끊김 신호 파이프 전송
			SendToPipe(-999.0f, -999.0f);
			printf("\rSensor Disconnected (scanning...)           ");
		}

		fflush(stdout);
		Sleep(g_ReadIntervalMs);
	}

	if (hPipe != INVALID_HANDLE_VALUE)
		CloseHandle(hPipe);
}

// pipe에 데이터 전송
void SendToPipe(float pitch, float roll)
{
	// 파이프에 연결
	while (hPipe == INVALID_HANDLE_VALUE)
	{
		hPipe = CreateFile(
			TEXT("\\\\.\\pipe\\TiltSensorPipe"),
			GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0,
			NULL);
	
		//if (hPipe == INVALID_HANDLE_VALUE)
		//{
		//	printf("Waiting for WPF...\n");
		//	Sleep(1000);
		//}
	}

	// 데이터 전송
	char buffer[64];
	sprintf_s(buffer, "%.2f,%.2f\n", pitch, roll);
	DWORD bytesWritten;
	BOOL success = WriteFile(hPipe, buffer, strlen(buffer), &bytesWritten, NULL);

	if (!success)
	{
		CloseHandle(hPipe);
		hPipe = INVALID_HANDLE_VALUE;
	}
}

void ComRxCallBack(char *p_data, UINT32 uiSize)
{
	for(UINT32 i = 0; i < uiSize; i++)
	{
		WitSerialDataIn(p_data[i]);
	}
}

static void DelayMs(uint16_t ms)
{
	Sleep(ms);
}

static void SensorUartSend(uint8_t* p_data, uint32_t uiSize)
{
	SendUARTMessageLength((const char*)p_data, uiSize);
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	s_cDataUpdate = 1;
	InterlockedExchange64((volatile LONGLONG*)&g_lastUpdateTick, (LONGLONG)GetTickCount64());	// 마지막 업데이트 시각 갱신
}

static int AutoScanSensor(void)
{
	const uint32_t c_uiBaud[7] = { 4800, 9600, 19200, 38400, 57600, 115200, 230400 };
	int i, iRetry;

	for (i = 0; i < 7; i++)
	{
		SetBaundrate(c_uiBaud[i]);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			Sleep(100);
			if (s_cDataUpdate != 0)
			{
				printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
				return 1; // 성공
			}
			iRetry--;
		} while (iRetry);
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");

	return 0; // 실패
}

static int LoadConfig(void)
{
	char exePath[MAX_PATH];
	char configPath[MAX_PATH];

	// 현재 실행 파일의 경로를 가져옴
	GetModuleFileNameA(NULL, exePath, MAX_PATH);
	char* lastSlash = strrchr(exePath, '\\');

	if (lastSlash)
		*lastSlash = '\0';

	// sensorConfig.ini 경로 생성
	sprintf_s(configPath, MAX_PATH, "%s\\sensorConfig.ini", exePath);

	// 설정값 읽기 (기본값 포함)
	g_ConfigComPort = GetPrivateProfileIntA("Sensor", "ComPort", -1, configPath);
	g_DisconnectTimeoutMs = GetPrivateProfileIntA("Sensor", "DISCONNECT_TIMEOUT_MS", 5000, configPath);
	g_RescanIntervalMs = GetPrivateProfileIntA("Sensor", "RESCAN_INTERVAL_MS", 5000, configPath);
	g_ReadIntervalMs = GetPrivateProfileIntA("Sensor", "READ_INTERVAL_MS", 100, configPath);


	if (g_ConfigComPort <= 0)
	{
		printf("Error: Invalid or missing ComPort in config file.\n");
		printf("Config file path: %s\n", configPath);
		printf("Please create sensorConfig.ini with:\n");

		return 0;
	}

	// 설정값 출력
	printf("=== Configuration Loaded ===\n");
	printf("Config file: %s\n", configPath);
	printf("COM Port: COM%d\n", g_ConfigComPort);
	printf("Disconnect Timeout: %d ms\n", g_DisconnectTimeoutMs);
	printf("Rescan Interval: %d ms\n", g_RescanIntervalMs);
	printf("Read Interval: %d ms\n", g_ReadIntervalMs);
	printf("============================\n\n");

	return 1;
}

//static int EnumerateComPorts(int* ports, int maxPorts)
//{
//	HDEVINFO hDevInfo = SetupDiGetClassDevs(&GUID_DEVCLASS_PORTS, 0, 0, DIGCF_PRESENT);
//	if (hDevInfo == INVALID_HANDLE_VALUE) return 0;
//
//	SP_DEVINFO_DATA devInfo;
//	devInfo.cbSize = sizeof(SP_DEVINFO_DATA);
//
//	int count = 0;
//
//	for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfo); i++)
//	{
//		char friendly[256];
//		DWORD regType = 0, size = 0;
//
//		if (!SetupDiGetDeviceRegistryPropertyA(
//			hDevInfo, &devInfo, SPDRP_FRIENDLYNAME,
//			&regType, (PBYTE)friendly, sizeof(friendly), &size))
//		{
//			continue;
//		}
//
//		char* p = strstr(friendly, "(COM");
//		if (!p) continue;
//
//		int com = 0;
//		if (sscanf_s(p, "(COM%d)", &com) == 1)
//		{
//			if (count < maxPorts)
//				ports[count++] = com;
//		}
//	}
//	SetupDiDestroyDeviceInfoList(hDevInfo);
//	return count;
//}

static int TryFindSensorOnPort(int comPort)
{
	// 포트 열기
	int openResult = (int)OpenCOMDevice(comPort, iBaud);
	if (openResult != 0)
		return 0;

	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(DelayMs);

	// 센서 탐색
	int ok = AutoScanSensor();
	CloseCOMDevice();

	return ok;
}

static int ConnectSensorOnPort(int comPort)
{
	// 이전 연결이 있을 경우 먼저 닫기 (중복 열기 방지)
	CloseCOMDevice();

	// 포트 열기
	int openResult = (int)OpenCOMDevice(comPort, iBaud);
	if (openResult != 0) // 실패
		return 0;

	// Wit SDK 초기화
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(DelayMs);

	// baud 자동 탐색 (연결 가능한 속도 확인)
	if (AutoScanSensor() == 0) // 실패
	{
		CloseCOMDevice();
		return 0;
	}

	// 연결 후 센서 unlock/baud/save 설정값 저장
	WitWriteReg(KEY, KEY_UNLOCK);       DelayMs(20);
	WitWriteReg(BAUD, WIT_BAUD_230400); DelayMs(20);
	WitWriteReg(SAVE, SAVE_PARAM);      DelayMs(20);

	iComPort = comPort;
	InterlockedExchange64((volatile LONGLONG*)&g_lastUpdateTick, (LONGLONG)GetTickCount64());
	return 1;
}

static int ScanAndReconnectSensor(void)
{
	if (g_ConfigComPort < 0)
	{
		printf("Error: COM port not configured\n");
		return 0;
	}

	// Close any existing COM connection first.
	// TryFindSensorOnPort calls OpenCOMDevice internally; if the port is
	// already open it will fail with access denied. Closing here also lets
	// the USB serial driver release cleanly before we re-scan.
	if (iComPort >= 0)
	{
		iComPort = -1;
		CloseCOMDevice();
		Sleep(300); // give the driver a moment to fully release
	}

	printf("Trying COM%d (from config)...\n", g_ConfigComPort);

	if (TryFindSensorOnPort(g_ConfigComPort) == 1)
	{
		if (ConnectSensorOnPort(g_ConfigComPort) == 1)
		{
			printf("\nReconnected: COM%d\n", g_ConfigComPort);
			return 1;
		}
	}

	return 0; // 설정된 포트에서도 센서를 찾지 못함
}
