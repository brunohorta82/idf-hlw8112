#include "HLW8112.hpp"
#include "stdio.h"
#include <esp_log.h>
#include "driver/uart.h"
#include "math.h"
#define HIGH 1
#define LOW 0
#define D_CAL_U 1
#define D_CAL_A_I 1
#define D_CAL_A_P 1
#define D_CAL_B_P 1
#define D_CAL_A_E 1
#define D_CAL_B_I 1
#define TICK_UART_RESPONSE 10
#define HWL_TAG "HLW8112"

int s_hlw_error_count = 0;
union IntData
{
	uint16_t inte;
	uint8_t byte[2];
};
union LongData
{
	uint32_t word;
	uint16_t inte[2];
	uint8_t byte[4];
};
SemaphoreHandle_t xSemaphore;
unsigned char u8_TxBuf[10];
unsigned char u8_RxBuf[10];
unsigned char u8_TX_Length;
unsigned char u8_RX_Length;
unsigned char u8_RX_Index;

unsigned char B_Rx_Finish;
unsigned char B_Rx_Data_ING;
unsigned char B_Read_Error;

unsigned int U16_TempData;

unsigned int U16_IFData;
unsigned int U16_RIFData;
unsigned int U16_LineFData;
unsigned int U16_AngleData;
unsigned int U16_PFData;
unsigned int U16_HFConst_RegData;

unsigned int U16_RMSIAC_RegData;
unsigned int U16_RMSIBC_RegData;
unsigned int U16_RMSUC_RegData;
unsigned int U16_PowerPAC_RegData;
unsigned int U16_PowerPBC_RegData;
unsigned int U16_PowerSC_RegData;
unsigned int U16_EnergyAC_RegData;
unsigned int U16_EnergyBC_RegData;
unsigned int U16_CheckSUM_RegData;
unsigned int U16_CheckSUM_Data;

unsigned int U16_Check_SysconReg_Data;
unsigned int U16_Check_EmuconReg_Data;
unsigned int U16_Check_Emucon2Reg_Data;

unsigned long U32_RMSIA_RegData;
unsigned long U32_RMSU_RegData;
unsigned long U32_POWERPA_RegData;
unsigned long U32_ENERGY_PA_RegData;

unsigned long U32_RMSIB_RegData;
unsigned long U32_POWERPB_RegData;
unsigned long U32_ENERGY_PB_RegData;

float F_AC_V;
float F_AC_I;
float F_AC_P;
float F_AC_E;
float F_AC_BACKUP_E;
float F_AC_PF;
float F_Angle;

float F_AC_I_B;
float F_AC_P_B;
float F_AC_E_B;
float F_AC_BACKUP_E_B;
float F_AC_LINE_Freq;
double valuePrecision(double value, double precision)
{
	return (floor((value * pow(10, precision) + 0.5)) / pow(10, precision));
}
unsigned char HLW8112_checkSum_Write(unsigned char u8_Reg_length)
{
	unsigned char i;
	unsigned char Temp_u8_checksum;
	unsigned int a;

	a = 0x0000;
	Temp_u8_checksum = 0;
	for (i = 0; i < (u8_Reg_length - 1); i++)
	{
		a += u8_TxBuf[i];
	}

	a = ~a;
	Temp_u8_checksum = a & 0xff;

	return Temp_u8_checksum;
}
void Start_Send_UartData(unsigned char len)
{
	uart_write_bytes(UART_NUM_1, (const char *)u8_TxBuf, len);
}
void Uart_HLW8112_WriteREG_EN()
{

	u8_TX_Length = 4;
	u8_RX_Length = 0;

	u8_TxBuf[0] = 0xa5;
	u8_TxBuf[1] = 0xea;
	u8_TxBuf[2] = 0xe5;
	u8_TxBuf[3] = HLW8112_checkSum_Write(u8_TX_Length);
	Start_Send_UartData(u8_TX_Length);
}
void Uart_HLW8112_WriteREG_DIS(void)
{

	u8_TX_Length = 4;
	u8_RX_Length = 0;

	u8_TxBuf[0] = 0xa5;
	u8_TxBuf[1] = 0xea;
	u8_TxBuf[2] = 0xdc;

	u8_TxBuf[3] = HLW8112_checkSum_Write(u8_TX_Length);

	Start_Send_UartData(u8_TX_Length);
}
void vTaskPmSerial(void *pvParameters)
{
	QueueHandle_t uart_queue;
	uart_event_t uart_event;
	if (uart_driver_install(UART_NUM_1, 1024, 1024, 20, &uart_queue, 0) != ESP_OK)
	{
		ESP_LOGE("TAG", "Driver installation failed");
	}
	while (true)
	{
		if (xQueueReceive(uart_queue, &uart_event, portMAX_DELAY))
		{
			switch (uart_event.type)
			{
			case UART_DATA:
			{
				B_Rx_Data_ING = 1;
				uart_read_bytes(UART_NUM_1, u8_RxBuf, u8_RX_Length, portMAX_DELAY);
				B_Rx_Finish = 0;
				u8_RX_Index = 0;
			}

			break;

			case UART_FIFO_OVF:
			{
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				uart_flush_input(UART_NUM_1);
				xQueueReset(uart_queue);
			}
			break;
			case UART_BREAK:
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				break;
			case UART_BUFFER_FULL:
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				break;
			case UART_FRAME_ERR:
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				break;
			case UART_PARITY_ERR:
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				break;
			case UART_DATA_BREAK:
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				break;
			case UART_PATTERN_DET:
				ESP_LOGW(HWL_TAG, "UART_FIFO_OVF");
				break;
				break;
			default:
				break;
			}
		}
	}
}

void Clear_RxBuf(void)
{
	unsigned char i;
	for (i = 0; i < 10; i++)
	{
		u8_RxBuf[i] = 0x00;
	}

	B_Rx_Data_ING = 0;
	B_Rx_Finish = 0;
	u8_RX_Index = 0;
}

unsigned char HLW8112_checkSum_Read(unsigned char u8_Reg_length)
{
	unsigned char i;
	unsigned char Temp_u8_checksum;
	unsigned int a;

	a = 0x0000;
	Temp_u8_checksum = 0;
	for (i = 0; i < (u8_Reg_length - 1); i++)
	{
		a += u8_RxBuf[i];
	}

	a = a + u8_TxBuf[0] + u8_TxBuf[1];
	a = ~a;

	Temp_u8_checksum = a & 0xff;

	return Temp_u8_checksum;
}

void Uart_HLW8112_Set_Channel_A()
{
	u8_TX_Length = 4;
	u8_RX_Length = 0;

	u8_TxBuf[0] = 0xa5;
	u8_TxBuf[1] = 0xea;
	u8_TxBuf[2] = 0x5a;
	u8_TxBuf[3] = HLW8112_checkSum_Write(u8_TX_Length);

	Start_Send_UartData(u8_TX_Length);
}

void Uart_Read_HLW8112_Reg(unsigned char ADDR_Reg, unsigned char u8_reg_length)
{

	u8_TxBuf[0] = 0xa5;
	u8_TxBuf[1] = ADDR_Reg;
	u8_TX_Length = 2;
	u8_RX_Length = u8_reg_length + 1;

	Clear_RxBuf();
	Start_Send_UartData(u8_TX_Length);
}

void Uart_Write_HLW8112_Reg(unsigned char ADDR_Reg, unsigned char u8_reg_length, unsigned long u32_data)
{
	unsigned char i;
	union LongData Temp_u32_a;

	u8_TxBuf[0] = 0xa5;
	u8_TxBuf[1] = ADDR_Reg | 0x80;

	Temp_u32_a.word = u32_data;
	for (i = 0; i < u8_reg_length; i++)
	{
		u8_TxBuf[i + 2] = Temp_u32_a.byte[u8_reg_length - 1 - i];
	}

	u8_TX_Length = 3 + u8_reg_length;
	u8_RX_Length = 0;

	u8_TxBuf[u8_TX_Length - 1] = HLW8112_checkSum_Write(u8_TX_Length);

	Start_Send_UartData(u8_TX_Length);
}

void Uart_HLW8112_Reset(void)
{

	u8_TX_Length = 4;
	u8_RX_Length = 0;

	u8_TxBuf[0] = 0xa5;
	u8_TxBuf[1] = 0xea;
	u8_TxBuf[2] = 0x96;

	u8_TxBuf[3] = HLW8112_checkSum_Write(u8_TX_Length);

	Start_Send_UartData(u8_TX_Length);
}
void Set_underVoltage(void)
{
	unsigned int a = 0;

	Uart_HLW8112_WriteREG_EN();
	Uart_Write_HLW8112_Reg(REG_SAGLVL_ADDR, 2, 0x4E1C);

	Uart_Read_HLW8112_Reg(REG_OVLVL_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_TempData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
		printf("OK REG_OVLVL_ADDR\r\n");
	}
	Uart_Write_HLW8112_Reg(REG_INT_ADDR, 2, 0x32D9);
	Uart_Read_HLW8112_Reg(REG_INT_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		a = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
		printf("OK REG_INT_ADDR\r\n");
	}
	Uart_Write_HLW8112_Reg(REG_IE_ADDR, 2, a);
	/*HLW8112_SPI_WriteReg(REG_IE_ADDR);
	HLW8112_SPI_WriteByte((a >> 8) | 0x08);
	HLW8112_SPI_WriteByte(a & 0xff);*/

	Uart_Read_HLW8112_Reg(REG_IE_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_TempData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
		printf("OK REG_IE_ADDR\r\n");
	}
	Uart_HLW8112_WriteREG_DIS();
}
unsigned char Judge_CheckSum_HLW8112_Calfactor(void)
{
	unsigned long a;
	unsigned char d;

	Uart_Read_HLW8112_Reg(REG_RMS_IAC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_RMSIAC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	Uart_Read_HLW8112_Reg(REG_RMS_IBC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_RMSIBC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	Uart_Read_HLW8112_Reg(REG_RMS_UC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_RMSUC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	Uart_Read_HLW8112_Reg(REG_POWER_PAC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_PowerPAC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	Uart_Read_HLW8112_Reg(REG_POWER_PBC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_PowerPBC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}
	Uart_Read_HLW8112_Reg(REG_POWER_SC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_PowerSC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	Uart_Read_HLW8112_Reg(REG_ENERGY_AC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_EnergyAC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}
	Uart_Read_HLW8112_Reg(REG_ENERGY_BC_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_EnergyBC_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	Uart_Read_HLW8112_Reg(REG_CHECKSUM_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_CheckSUM_RegData = (u8_RxBuf[0] << 8) + u8_RxBuf[1];
	}

	a = 0;
	a = ~(0xffff + U16_RMSIAC_RegData + U16_RMSIBC_RegData + U16_RMSUC_RegData +
		  U16_PowerPAC_RegData + U16_PowerPBC_RegData + U16_PowerSC_RegData +
		  U16_EnergyAC_RegData + U16_EnergyBC_RegData);

	U16_CheckSUM_Data = a & 0xffff;

	a = 0;

	if (U16_CheckSUM_Data == U16_CheckSUM_RegData)
	{
		d = 1;
	}
	else
	{
		d = 0;
	}

	return d;
}

void Init_HLW8112()
{
	const uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_EVEN,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 0,
		.source_clk = UART_SCLK_DEFAULT};
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, CONFIG_PIN_PM_RX, CONFIG_PIN_PM_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	xTaskCreatePinnedToCore(vTaskPmSerial, "vTaskPmSerial", 1024 * 5, NULL, configMAX_PRIORITIES - 9, NULL, 1);
	vTaskDelay(TICK_UART_RESPONSE);
	Uart_HLW8112_WriteREG_EN();
	vTaskDelay(TICK_UART_RESPONSE);
	Uart_HLW8112_Set_Channel_A();
	vTaskDelay(TICK_UART_RESPONSE);
	Uart_Write_HLW8112_Reg(REG_SYSCON_ADDR, 2, 0x0f04);
	vTaskDelay(TICK_UART_RESPONSE);
	Uart_Write_HLW8112_Reg(REG_EMUCON_ADDR, 2, 0x1003);
	vTaskDelay(TICK_UART_RESPONSE);
	Uart_Write_HLW8112_Reg(REG_EMUCON2_ADDR, 2, 0x0fff);
	vTaskDelay(TICK_UART_RESPONSE);
	Uart_HLW8112_WriteREG_DIS();
	vTaskDelay(TICK_UART_RESPONSE);
	Judge_CheckSum_HLW8112_Calfactor();
}

void Check_WriteReg_Success(void)
{
	Uart_Read_HLW8112_Reg(REG_SYSCON_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_Check_SysconReg_Data = (u8_RxBuf[0] << 8) + (u8_RxBuf[1]);
	}
	else
	{

		ESP_LOGE(HWL_TAG, "B_Read_Error REG_SYSCON_ADDR");
		B_Read_Error = 1;
	}

	Uart_Read_HLW8112_Reg(REG_EMUCON_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_Check_EmuconReg_Data = (u8_RxBuf[0] << 8) + (u8_RxBuf[1]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_EMUCON_ADDR");
		B_Read_Error = 1;
	}
	Uart_Read_HLW8112_Reg(REG_EMUCON2_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_Check_Emucon2Reg_Data = (u8_RxBuf[0] << 8) + (u8_RxBuf[1]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_EMUCON2_ADDR");
		B_Read_Error = 1;
	}
}

void Read_HLW8112_IA(void)
{
	float a;

	Uart_Read_HLW8112_Reg(REG_RMSIA_ADDR, 3);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U32_RMSIA_RegData = (unsigned long)(u8_RxBuf[0] << 16) + (unsigned long)(u8_RxBuf[1] << 8) + (unsigned long)(u8_RxBuf[2]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_RMSIA_ADDR");
		B_Read_Error = 1;
	}

	if ((U32_RMSIA_RegData & 0x800000) == 0x800000)
	{
		F_AC_I = 0;
	}
	else
	{
		a = (float)U32_RMSIA_RegData;
		a = a * U16_RMSIAC_RegData;
		a = a / 0x800000;
		a = a / 1;
		a = a / 1000;
		a = a * D_CAL_A_I;
		F_AC_I = a;
	}
}
void Read_HLW8112_IB(void)
{
	float a;

	Uart_Read_HLW8112_Reg(REG_RMSIB_ADDR, 3);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U32_RMSIB_RegData = (unsigned long)(u8_RxBuf[0] << 16) + (unsigned long)(u8_RxBuf[1] << 8) + (unsigned long)(u8_RxBuf[2]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_RMSIB_ADDR");
		B_Read_Error = 1;
	}

	if ((U32_RMSIB_RegData & 0x800000) == 0x800000)
	{
		F_AC_I_B = 0;
	}
	else
	{
		a = (float)U32_RMSIB_RegData;
		a = a * U16_RMSIBC_RegData;
		a = a / 0x800000;
		a = a / 1;
		a = a / 1000;
		a = a * D_CAL_B_I;
		F_AC_I_B = a;
	}
}
void Read_HLW8112_U(void)
{
	float a;

	Uart_Read_HLW8112_Reg(REG_RMSU_ADDR, 3);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U32_RMSU_RegData = (unsigned long)(u8_RxBuf[0] << 16) + (unsigned long)(u8_RxBuf[1] << 8) + (unsigned long)(u8_RxBuf[2]);
		s_hlw_error_count = 0;
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_RMSU_ADDR %d", s_hlw_error_count);
		s_hlw_error_count++;
		B_Read_Error = 1;
	}

	if ((U32_RMSU_RegData & 0x800000) == 0x800000)
	{
		F_AC_V = 0;
	}
	else
	{
		a = (float)U32_RMSU_RegData;
		a = a * U16_RMSUC_RegData;
		a = a / 0x400000;
		a = a / 1;
		a = a / 100;
		a = a * D_CAL_U;
		F_AC_V = a;
	}
}

void Read_HLW8112_PA(void)
{
	float a;
	float b;

	Uart_Read_HLW8112_Reg(REG_POWER_PA_ADDR, 4);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U32_POWERPA_RegData = (unsigned long)(u8_RxBuf[0] << 24) + (unsigned long)(u8_RxBuf[1] << 16) + (unsigned long)(u8_RxBuf[2] << 8) + (unsigned long)(u8_RxBuf[3]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_POWER_PA_ADDR");
		B_Read_Error = 1;
	}

	if (U32_POWERPA_RegData > 0x80000000)
	{
		b = ~U32_POWERPA_RegData;
		a = (float)b;
	}
	else
		a = (float)U32_POWERPA_RegData;
	a = a * U16_PowerPAC_RegData;
	a = a / 0x80000000;
	a = a / 1;
	a = a / 1;
	a = a * D_CAL_A_P;
	F_AC_P = a;
}
void Read_HLW8112_PB(void)
{
	float a;
	float b;

	Uart_Read_HLW8112_Reg(REG_POWER_PB_ADDR, 4);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U32_POWERPB_RegData = (unsigned long)(u8_RxBuf[0] << 24) + (unsigned long)(u8_RxBuf[1] << 16) + (unsigned long)(u8_RxBuf[2] << 8) + (unsigned long)(u8_RxBuf[3]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_POWER_PB_ADDR");
		B_Read_Error = 1;
	}

	if (U32_POWERPB_RegData > 0x80000000)
	{
		b = ~U32_POWERPB_RegData;
		a = (float)b;
	}
	else
		a = (float)U32_POWERPB_RegData;
	a = a * U16_PowerPBC_RegData;
	a = a / 0x80000000;
	a = a / 1;
	a = a / 1;
	a = a * D_CAL_B_P;
	F_AC_P_B = a;
}
void Read_HLW8112_EA(void)
{
	float a;
	Uart_Read_HLW8112_Reg(REG_ENERGY_PA_ADDR, 3);
	vTaskDelay(TICK_UART_RESPONSE);

	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U32_ENERGY_PA_RegData = (unsigned long)(u8_RxBuf[0] << 16) + (unsigned long)(u8_RxBuf[1] << 8) + (unsigned long)(u8_RxBuf[2]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_ENERGY_PB_ADDR");
		B_Read_Error = 1;
	}

	Uart_Read_HLW8112_Reg(REG_HFCONST_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		U16_HFConst_RegData = (unsigned int)(u8_RxBuf[0] << 8) + (unsigned int)(u8_RxBuf[1]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_HFCONST_ADDR");
		B_Read_Error = 1;
	}

	a = (float)U32_ENERGY_PA_RegData;

	a = a * U16_EnergyAC_RegData;
	a = a / 0x20000000;

	a = a / 1;
	a = a / 1;
	a = a * D_CAL_A_E;
	F_AC_E = a;

	F_AC_BACKUP_E = F_AC_E;
}

void Read_HLW8112_LineFreq(void)
{
	float a = 0;
	unsigned long b = 0;
	Uart_Read_HLW8112_Reg(REG_UFREQ_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		b = (unsigned long)(u8_RxBuf[0] << 8) + (unsigned long)(u8_RxBuf[1]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_UFREQ_ADDR");
		B_Read_Error = 1;
	}
	a = (float)b;
	a = 3579545 / (8 * a);
	F_AC_LINE_Freq = a;
}

void Read_HLW8112_PF(void)
{
	float a = 0;
	unsigned long b = 0;

	Uart_Read_HLW8112_Reg(REG_PF_ADDR, 3);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		b = (unsigned long)(u8_RxBuf[0] << 16) + (unsigned long)(u8_RxBuf[1] << 8) + (unsigned long)(u8_RxBuf[2]);
	}
	else
	{
		ESP_LOGE(HWL_TAG, "B_Read_Error REG_PF_ADDR");
		B_Read_Error = 1;
	}

	if (b > 0x800000)
	{
		a = (float)(0xffffff - b + 1) / 0x7fffff;
	}
	else
	{
		a = (float)b / 0x7fffff;
	}

	if (F_AC_P < 0.3) // Ð¡ÓÚ0.3W(Less than 0.3w),Power factor data is inaccurate
		a = 0;

	F_AC_PF = a;
}

void Read_HLW8112_Angle()
{
	float a = 0;
	unsigned long b = 0;
	Uart_Read_HLW8112_Reg(REG_ANGLE_ADDR, 2);
	vTaskDelay(TICK_UART_RESPONSE);
	if (u8_RxBuf[u8_RX_Length - 1] == HLW8112_checkSum_Read(u8_RX_Length))
	{
		b = (unsigned long)(u8_RxBuf[0] << 8) + (unsigned long)(u8_RxBuf[1]);
	}
	else
	{
		ESP_LOGE("HLW8112", "B_Read_Error REG_ANGLE_ADDR");
		B_Read_Error = 1;
	}

	if (F_AC_PF < 55) // Linear frequency: 50 Hz,
	{
		a = b;
		a = a * 0.0805;
		F_Angle = a;
	}
	else
	{
		// Linear frequency: 60 Hz
		a = b;
		a = a * 0.0965;
		F_Angle = a;
	}

	if (F_AC_P < 0.5) //(When the power is less than 0.5, there is no load and the phase angle is 0)
	{
		F_Angle = 0;
	}

	if (F_Angle < 90)
	{
		a = F_Angle;
		printf("Current lead voltage: %f\n ", a); // Current lead voltage
	}
	else if (F_Angle < 180)
	{
		a = 180 - F_Angle;
		printf(" Current hysteresis voltage: %f\n ", a); // Current hysteresis voltage
	}
	else if (F_Angle < 360)
	{
		a = 360 - F_Angle;
		printf("Current hysteresis voltage: %f\n ", a); // Current hysteresis voltage
	}
	else
	{
		a = F_Angle - 360;
		printf("Current lead voltage: %f\n ", a); // Current lead voltage
	}
}

int Calculate_HLW8112_MeterData(struct PowerReadings *rA, struct PowerReadings *rB)
{
	struct PowerReadings *a = rA;
	struct PowerReadings *b = rB;
	// Check_WriteReg_Success();
	Read_HLW8112_U();
	Read_HLW8112_IA();
	Read_HLW8112_PA();
	Read_HLW8112_EA();
	//	Read_HLW8112_Angle();
	//	Read_HLW8112_PF();
	Read_HLW8112_IB();
	Read_HLW8112_PB();
	a->voltage = F_AC_V;
	a->current = F_AC_I;
	a->power = valuePrecision(a->current, 2) == 0.0 ? 0.0 : F_AC_P;
	b->voltage = F_AC_V;
	b->current = F_AC_I_B;
	b->power = valuePrecision(b->current, 2) == 0.0 ? 0.0 : F_AC_P_B;
#ifdef CONFIG_POWER_METTER_DEBUG
	ESP_LOGI(HWL_TAG, "CHANNEL A:  %f Volts  %f Amps %f Watts %f KWH", F_AC_V, F_AC_I, F_AC_P, F_AC_E);
	ESP_LOGI(HWL_TAG, "CHANNEL B:  %f Volts  %f Amps %f Watts %f KWH", F_AC_V, F_AC_I_B, F_AC_P_B, F_AC_E_B);
#endif
	return s_hlw_error_count;
}
