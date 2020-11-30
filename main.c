#include "stm32f4xx_hal.h"
#include "math.h"
void GPIO_Config(void);
void I2C2_Config(void);
void IMU_Config(void);
uint8_t IMU_Check(void);
void IMU_Read(void);
void oxi_setup(void);
void OXIMETER_READ_SAMPLE(void);
void IMU_Calc(void);
void IMU_Steps(void);
void OXI_IR_DSP(void);
void OXI_R_DSP(void);
uint32_t time_ms;
float data_filt_oxi = 0.0;
float variance_oxi = 0.0;
float n_variance_oxi=0.0;
#define INTENSITY_MATCHING_THRES 40000
#define mpu6050Address	0xD0
#define IMU_CONFIG_REG 0x1A
#define IMU_SMPLRT_REG 0x19
#define IMU_ACCEL_CONFIG_REG 0x1C
#define IMU_FIFO_EN_REG 0x23
#define IMU_ACCEL_XOUT_H 0x3B
#define IMU_WHO_AM_I 0x75
#define IMU_PWR_MGMT_1 0x6B
#define lambda1 650e-9
#define lambda2 950e-9

void GPIO_I2C1_Config(void);
void I2C1_Config(void);
uint32_t time1=0.0;
uint32_t time2=0.0;
#define OXI_ADDRESS (0xAE)

double R = 0;
uint8_t OXI_INT_STATUS = 0x00;
uint8_t OXI_INT_ENABLE = 0x01;
uint8_t OXI_FIFO_WR_PTR = 0x02;
uint8_t OXI_FIFO_OVF_CTR = 0x03;
uint8_t OXI_FIFO_RD_PTR = 0x04;
uint8_t OXI_FIFO_DATA = 0x05;
uint8_t OXI_MODE_CONFIG = 0x06;
uint8_t OXI_SP02_CONFIG = 0x07;
uint8_t OXI_LED_CONFIG = 0x09;
uint8_t OXI_DEVICE_ID = 0xFF;
uint16_t bpm=0xffff;
I2C_HandleTypeDef I2CHandle;
uint8_t oxi_id, oxi_fifo_regs[3];		
uint8_t oxi_sample[4];							//holds raw data from the oximeter
uint16_t ir_data, r_data;						//holds the processed data (two 16-bit words, IR_LED and R_LED values)
uint32_t oxi_count=0;								
uint8_t red_pa=0x08;
uint8_t ir_pa=0x0F;
uint8_t led_config;
uint8_t intStatus;
float w_ir, prev_w_ir=0, w_r, prev_w_r=0;
float ir_data_filtered, ir_data_final, ir_data_mean, r_data_filtered;
float IMU_var=0;	
float IMU_avg=0;	
float sum=0;
float mag=0;
float IMU_var_sum = 0;
uint8_t IMU_who_am_I;
uint32_t IMU_count = 0;
I2C_HandleTypeDef myI2Chandle;


double sq_sum_r = 0, sq_sum_ir = 0;
double r_rms = 0, ir_rms = 0, spo2 = 0;

float acc_smooth=0.0;
float mag_avg = 0.0;
float mag_var = 0.0;
float IMU_norm_mag = 0;
float thres_peak = 2.1;
float thres_end = 2.03;
/////
int check_thres_peak = 0;
int check_thres_end = 0;
float span[4];
uint8_t steps=0;
int special_flag=0;
float end_oxi[3];
/////
float start[3]={0.0,0.0,0.0};
float end[3]={0.0,0.0,0.0};
//float next_vector[3]={0.0,0.0,0.0};
int flag_start=0;

//I2C variables 
uint8_t i2c2Buf[8];
int16_t ax,ay,az;
float Xaccel,Yaccel,Zaccel;

//Butterworth
float v[2] = {0.0, 0.0};
float result = 0;

float thres_peak_oxi = 1;
float thres_end_oxi_oxi = 0;
/////
int check_thres_peak_oxi = 0;
int check_thres_end_oxi_oxi = 0;
float span_oxi[4];
float start_oxi[3];
float mag_avg_oxi=0.0;
float oxi_smooth=0.0;
int flag_start_oxi=0;
int data1,data2;
int32_t SER_GetChar (void);
int32_t SER_PutChar(int32_t);
int32_t UART4_IRQHandler(void)
{	
  // RX IRQ part 
		
		data1=SER_PutChar(0xAA);
		data2=data1;
		//HAL_Delay(1);
	//	spo2=0;
		data1=SER_PutChar(spo2);
		data2=data1;
		
	// RX IRQ part 
		data1=SER_PutChar((bpm>>8));
		data2=data1;
		//HAL_Delay(1);
		
		data1=SER_PutChar(bpm);
		data2=data1;
		//HAL_Delay(1);
} 

void SER_Init (void){
	
	
	RCC->APB1ENR|=(1UL<<19);//Enable USART 4 clock
	RCC->AHB1ENR|=(1UL<<2);//Enable GPIOC clock
	GPIOC->MODER &=0XFF0FFFFF;
	GPIOC->MODER |=0X00A00000;
	GPIOC->AFR[1]|=0X00008800;//PC10 UART4_Tx, PC11 UART4_Rx (AF8)
	UART4->BRR=0x1117;
	
	UART4->CR1=0X200C;
	UART4->CR1 |= 0x0020; // Enable RX interrupt
	NVIC_EnableIRQ(UART4_IRQn); // Enable IRQ for UART4 in NVIC 
	
}


int main(void)
{
	time_ms = 0;
	HAL_Init();
	GPIO_Config();
	I2C1_Config();
	oxi_setup();
	I2C2_Config();
	IMU_who_am_I = IMU_Check(); 
	IMU_Config();
	//SER_Init();
	while(1)
	{
		OXIMETER_READ_SAMPLE();
		OXI_IR_DSP();
		OXI_R_DSP();
		IMU_Read();
		IMU_Calc();
		IMU_Steps();
		//NVIC_SetPendingIRQ(UART4_IRQn);
		//HAL_Delay(50);
	}
}

void GPIO_Config(void)
{
	//Enable Ports clocks
//	__HAL_RCC_GPIOB_CLK_ENABLE();
	uint32_t tmpreg;
	RCC->AHB1ENR |= 1<<1;
	RCC->APB1ENR |= 1<<21;
	tmpreg = RCC->AHB1ENR;
	tmpreg = RCC->APB1ENR;
		
	GPIO_InitTypeDef myPinInit;

	
	GPIOB->AFR[0] |= (4<<24) | (4<<28);
	GPIOB->AFR[1] |= (4<<8) | (4<<12);
	GPIOB->MODER &= ~(3<<12);
	GPIOB->MODER |= 2<<12;
	GPIOB->MODER &= ~(3<<14);
	GPIOB->MODER |= 2<<14;
	GPIOB->MODER &= ~(3<<6);
	GPIOB->MODER |= 2<<6;
	GPIOB->MODER &= ~(3<<20);
	GPIOB->MODER |= 2<<20;
	GPIOB->MODER &= ~(3<<22);
	GPIOB->MODER |= 2<<22;
	GPIOB->OTYPER |= (1<<6) | (1<<7);
	GPIOB->OTYPER |= (1<<10) | (1<<11);
	GPIOB->OSPEEDR &= ~(3<<6);
	GPIOB->OSPEEDR |= (3<<6);
	GPIOB->OSPEEDR &= ~(3<<12);
	GPIOB->OSPEEDR |= (3<<12);
	GPIOB->OSPEEDR &= ~(3<<14);
	GPIOB->OSPEEDR |= (3<<14);
	GPIOB->OSPEEDR &= ~(3<<20);
	GPIOB->OSPEEDR |= (3<<20);
	GPIOB->OSPEEDR &= ~(3<<22);
	GPIOB->OSPEEDR |= (3<<22);
	GPIOB->PUPDR &= ~(3<<8);
	GPIOB->PUPDR &= ~(1<<8);
	GPIOB->PUPDR &= ~(3<<12);
	GPIOB->PUPDR &= ~(1<<12);
	GPIOB->PUPDR &= ~(3<<14);
	GPIOB->PUPDR &= ~(1<<14);
	GPIOB->PUPDR &= ~(3<<20);
	GPIOB->PUPDR &= ~(1<<20);
	GPIOB->PUPDR &= ~(3<<22);
	GPIOB->PUPDR &= ~(1<<22);
	//	myPinInit.Pin = GPIO_PIN_6 | GPIO_PIN_7;
//	myPinInit.Mode = GPIO_MODE_AF_OD;
//	myPinInit.Pull = GPIO_PULLUP;
//	myPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	myPinInit.Alternate = GPIO_AF4_I2C1;
//	HAL_GPIO_Init(GPIOB, &myPinInit);
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
	//I2C pins config
//	myPinInit.Pin = GPIO_PIN_10 | GPIO_PIN_11;
//	myPinInit.Mode = GPIO_MODE_AF_OD;
//	myPinInit.Pull = GPIO_PULLUP;
//	myPinInit.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	myPinInit.Alternate = GPIO_AF4_I2C2;
//	HAL_GPIO_Init(GPIOB, &myPinInit);
	
	//Systick interrupt enable for HAL_Delay function
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
int32_t SER_PutChar(int32_t ch){

	while (!(UART4->SR & 0X0080));
	UART4->SR&= 0XFFBF;
	UART4->DR=(ch &0xFF);
	return(ch);
	
}


void I2C2_Config(void)
{
	//Enable I2C peripheral clock
	__HAL_RCC_I2C2_CLK_ENABLE();
	
	myI2Chandle.Instance = I2C2;
	myI2Chandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	myI2Chandle.Init.ClockSpeed = 100000;
	myI2Chandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	myI2Chandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	myI2Chandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	myI2Chandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	myI2Chandle.Init.OwnAddress1 = 0;
	myI2Chandle.Init.OwnAddress2 = 0;
	HAL_I2C_Init(&myI2Chandle);
}

void I2C1_Config(void)
{
	__HAL_RCC_I2C1_CLK_ENABLE();
	
	I2CHandle.Instance = I2C1;
	I2CHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2CHandle.Init.ClockSpeed = 100000;
	I2CHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
	I2CHandle.Init.DutyCycle = I2C_DUTYCYCLE_2;
	I2CHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
	I2CHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
	I2CHandle.Init.OwnAddress1 = 0;
	I2CHandle.Init.OwnAddress2 = 0;
	HAL_I2C_Init(&I2CHandle);
}

void SysTick_Handler(void)
{
	time_ms = time_ms + 1;
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void IMU_Config()
{
	//Disable sleep mode
	i2c2Buf[0] = IMU_PWR_MGMT_1;
	i2c2Buf[1] = 0x00;
	HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 2, 10);

//Set config register
	i2c2Buf[0] = IMU_CONFIG_REG;
	i2c2Buf[1] = 0x01; 	//DLPF_CFG = 1
	HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 2, 10);

	//Sampling rate
	i2c2Buf[0] = IMU_SMPLRT_REG;
	i2c2Buf[1] = 0x01; 	//Sampling rate = 0.5kHz
	HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 2, 10);
	
	//Set accelerometer range
	i2c2Buf[0] = IMU_ACCEL_CONFIG_REG;			
	i2c2Buf[1] = 0x00;		//+/-2g range
	HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 2, 10);
	
	//Enable Accel FIFO
	i2c2Buf[0] = 35;
	i2c2Buf[1] = 0x08;
	HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 2, 10);
}

uint8_t IMU_Check()
{
	//Request to read from Who am I register
	i2c2Buf[0] = IMU_WHO_AM_I;	

	HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 1, 10);
	
	//Read who am I data
	i2c2Buf[1] = 0x00;
	HAL_I2C_Master_Receive(&myI2Chandle, mpu6050Address|0x01, &i2c2Buf[1], 1, 10);
	return i2c2Buf[1];
}

void IMU_Read()
{

		//Read accelerometer data
		i2c2Buf[0] = IMU_ACCEL_XOUT_H;			//Register address: X_axis H
		HAL_I2C_Master_Transmit(&myI2Chandle, mpu6050Address, i2c2Buf, 1, 10);
		//Read data
		i2c2Buf[1] = 0x00;
		HAL_I2C_Master_Receive(&myI2Chandle, mpu6050Address|0x01, &i2c2Buf[1], 6, 10);	//Read 6 bytes so all accel values are read
		
		ax = -(i2c2Buf[1]<<8 | i2c2Buf[2]);
		ay = -(i2c2Buf[3]<<8 | i2c2Buf[4]);
		az = 	(i2c2Buf[5]<<8 | i2c2Buf[6]);

		Xaccel = ax/8192.0;
		Yaccel = ay/8192.0;
		Zaccel = az/8192.0;
}

void OXI_Check(void)
{
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, &OXI_DEVICE_ID, 1, 10);
	HAL_I2C_Master_Receive(&I2CHandle, OXI_ADDRESS, &oxi_id, 1, 10);
}
void OXI_FIFO_Clear(void)
{
	uint8_t fifo_config[4] = {OXI_FIFO_WR_PTR, 0x00, 0x00, 0x00};						//FIFO Config
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, fifo_config, 4, 10);
}
void OXI_MODE_Config(void)
{
	uint8_t mode_config1[2] = {OXI_MODE_CONFIG, 0x03};						//Oximeter mode config
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, mode_config1, 2, 10);
	uint8_t mode_config2[2] = {OXI_SP02_CONFIG, 0x47};						//Oximeter mode config
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, mode_config2, 2, 10);
	uint8_t mode_config3[2] = {OXI_LED_CONFIG, 0x8F};						//Oximeter mode config
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, mode_config3, 2, 10);
	uint8_t mode_config4[2] = {OXI_INT_ENABLE, 0x90};						//Oximeter mode config
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, mode_config4, 2, 10);
}

void OXIMETER_READ_SAMPLE(void)
{
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, &OXI_INT_STATUS, 1, 10);
	HAL_I2C_Master_Receive(&I2CHandle, OXI_ADDRESS, &intStatus, 4, 10);

	if(intStatus & 0x80)
	{
		OXI_FIFO_Clear();
	}
	
//	while(!(intStatus & 0b00010000));
		
	HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, &OXI_FIFO_DATA, 1, 10);	//Read the FIFO Data: one sample = 4 bytes
	HAL_I2C_Master_Receive(&I2CHandle, OXI_ADDRESS, oxi_sample, 4, 10);
	ir_data = (oxi_sample[0]<<8 | oxi_sample[1]);
	r_data = (oxi_sample[2]<<8 | oxi_sample[3]);
}

void oxi_setup(void)
{
	OXI_Check();
	OXI_FIFO_Clear();
	OXI_MODE_Config();
}

void IMU_Calc()
{
	IMU_count = IMU_count + 1;
	mag = sqrt(Xaccel*Xaccel + Yaccel*Yaccel + Zaccel*Zaccel);
	sum = sum + mag;
	IMU_avg = sum/IMU_count;
	//////
	IMU_var_sum = IMU_var_sum + ((mag - IMU_avg)*(mag - IMU_avg));
	//////
	if(IMU_count-1>0)
	IMU_var = IMU_var_sum/(IMU_count-1);
}

void IMU_Steps()
{
	if(IMU_count > 1000)
	{
		mag_avg = IMU_avg;
	}
	else
	{
		mag_avg = 2.18;
	}
	
	if(IMU_count > 300)
	{
		mag_var = IMU_var;
	}
	else
	{
		mag_var = 0.125;
	}
	
	IMU_norm_mag = mag-mag_avg;
	float data_filt = mag_avg;
	float variance = mag_var;
	float n_variance=0.0;
	float K=variance/(variance+1);
	n_variance=((variance*(1-K)))+10;
	acc_smooth = data_filt+K*(mag-data_filt);
	span[0]=span[1];
	span[1]=span[2];
	span[2]=span[3];
	span[3]=acc_smooth;

	if(span[3]>span[2] && span[2]>span[1] && span[1]<span[0] && check_thres_peak==0)
	{
			start[2]=span[3];
			start[1]=span[2];
			start[0]=span[1];
			flag_start=1;
	}
	if (acc_smooth>thres_peak && check_thres_peak==0)
		{
		steps++;
		check_thres_peak=1;
		}
	if (acc_smooth<thres_end && check_thres_peak==1)
		{
		check_thres_end=1;
		check_thres_peak=0;
		}
	if(span[3]>span[2] && span[2]>span[1] && span[1]<span[0]&& flag_start==1 && check_thres_end==1)
	{
			end[2]=span[3];
			end[1]=span[2];
			end[0]=span[1];
			flag_start=0;
			check_thres_end=0;
	}
	//////////////
}

/////
//uint16_t Buffer_ir[10000] = {0};
//uint16_t array_num = 0, i = 0;
//double Dc_filtered_array[9999] = {0};
//double mean_filtered_array[9999] = {0};


float ir_data_buf[5] = {0};
uint16_t ir_data_buf_count=0;

void IR_DC_removal(void)
{
	double alpha = 0.95;
	w_ir = ir_data + alpha*prev_w_ir;
	ir_data_filtered = w_ir - prev_w_ir;
	
	if(ir_data_buf_count==5)///////////////////////
	{
		ir_data_buf_count=0;
	}
	
	ir_data_buf[0] = ir_data_buf[1];
	ir_data_buf[1] = ir_data_buf[2];
	ir_data_buf[2] = ir_data_buf[3];
	ir_data_buf[3] = ir_data_buf[4];
	ir_data_buf[4] = ir_data_filtered;
    
	
	prev_w_ir = w_ir;
}
void meanfilter(void)
{
	float sum_m = 0;
	float mean_m = 0;
	for(int i = 0; i<5; i++)
	{
		sum_m += ir_data_buf[i];
	}
	mean_m = sum_m/5;
	ir_data_mean = mean_m - ir_data_filtered;
}

float sum_oxi = 0.0, OXI_avg = 0.0, OXI_var_sum_oxi = 0.0 , OXI_var = 0;
int OXI_count=0;
void OXI_calc()
{
    OXI_count = OXI_count + 1;
		if (ir_data_final>0)
			sum_oxi = sum_oxi + ir_data_final;
		else
			sum_oxi = sum_oxi - ir_data_final;
    OXI_avg = sum_oxi/OXI_count;
    //////
    OXI_var_sum_oxi = OXI_var_sum_oxi + ((ir_data_final - OXI_avg)*(ir_data_final - OXI_avg));
    //////
    if(OXI_count-1>0)
    OXI_var = OXI_var_sum_oxi/(OXI_count-1);
}

float mag_var_oxi = 0.0, OXI_norm_mag = 0.0; 
void OXI_PEAKS()
{
    mag_avg_oxi = 0.3;
    mag_var_oxi = 4.5; 
    OXI_norm_mag = ir_data_final-mag_avg_oxi;
    data_filt_oxi = mag_avg_oxi;
    variance_oxi = mag_var_oxi;
    float n_variance_oxi=0.0;
    float K_oxi=variance_oxi/(variance_oxi+1);
    n_variance_oxi=((variance_oxi*(1-K_oxi)))+10;
    oxi_smooth = data_filt_oxi+K_oxi*(ir_data_final-data_filt_oxi);
    span_oxi[0]=span_oxi[1];
    span_oxi[1]=span_oxi[2];
    span_oxi[2]=span_oxi[3];
    span_oxi[3]=oxi_smooth;

    if(span_oxi[3]>span_oxi[2] && span_oxi[2]>span_oxi[1] && span_oxi[1]<span_oxi[0] && check_thres_peak_oxi==0)
    {
            start_oxi[2]=span_oxi[3];
            start_oxi[1]=span_oxi[2];
            start_oxi[0]=span_oxi[1];
            flag_start_oxi=1;
    }
    if (oxi_smooth>thres_peak_oxi && check_thres_peak_oxi==0)
        {
        check_thres_peak_oxi=1;
        }
    if (span_oxi[3]<span_oxi[2] && span_oxi[2]<span_oxi[1] && span_oxi[1]>span_oxi[0] && check_thres_peak==1 && special_flag==0)
    {
        time2=time1;
        time1=time_ms;
        special_flag=1;
        bpm=(60000/(time1-time2));
    }
    if (oxi_smooth<thres_end_oxi_oxi && check_thres_peak_oxi==1)
        {
        check_thres_end_oxi_oxi=1;
        check_thres_peak_oxi=0;
        }
    if(span_oxi[3]>span_oxi[2] && span_oxi[2]>span_oxi[1] && span_oxi[1]<span_oxi[0]&& flag_start_oxi==1 && check_thres_end_oxi_oxi==1)
    {
            end_oxi[2]=span_oxi[3];
            end_oxi[1]=span_oxi[2];
            end_oxi[0]=span_oxi[1];
            flag_start_oxi=0;
            check_thres_end_oxi_oxi=0;
            special_flag=0;
    }
}
void ButterworthFilter()
{
	v[0] = v[1];
	v[1] = (2.4523727527856026e-1*ir_data_mean) + (0.50952544949442879485 * v[0]);
	ir_data_final = v[0] + v[1];
}

void OXI_IR_DSP()
{
	IR_DC_removal();
	meanfilter();
	ButterworthFilter();
	OXI_calc();
	OXI_PEAKS();
}

void R_DC_Removal()
{
	double alpha = 0.95;
	
	w_r = r_data + alpha*prev_w_r;
	r_data_filtered = w_r - prev_w_r;

	prev_w_r = w_r;
}

void R_Calc()
{
	oxi_count = oxi_count + 1;
//	if(oxi_count > 5)
//	{
//		r_rms = 0;
//		ir_rms = 0;
//		oxi_count = 0;
//	}
	sq_sum_r = sq_sum_r + r_data_filtered*r_data_filtered;
	r_rms = sqrt(sq_sum_r/oxi_count);
	sq_sum_ir = sq_sum_ir + ir_data_final*ir_data_final;
	ir_rms = sqrt(sq_sum_ir/oxi_count);
	R = (log(r_rms)*lambda1)/(log(ir_rms)*lambda2);
	spo2 = 110 - 20*R;
}

//void setLEDCurrents()
//{
//	while(w_ir-w_r > INTENSITY_MATCHING_THRES | w_r-w_ir > INTENSITY_MATCHING_THRES)
//	{
//		if(w_ir - w_r > INTENSITY_MATCHING_THRES && red_pa<=0x0F)
//		{
//			red_pa += 1;
//			led_config = (red_pa<<4) | ir_pa;
//			uint8_t led_config_arr[2] = {OXI_LED_CONFIG, led_config};						//Oximeter mode config
//			HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, led_config_arr, 2, 10);
//		}
//		if(w_r-w_ir > INTENSITY_MATCHING_THRES && red_pa>=0x00)
//		{
//			red_pa -= 1;
//			led_config = (red_pa<<4) | ir_pa;
//			uint8_t led_config_arr[2] = {OXI_LED_CONFIG, led_config};						//Oximeter mode config
//			HAL_I2C_Master_Transmit(&I2CHandle, OXI_ADDRESS, led_config_arr, 2, 10);
//		}
//	}
//}

void OXI_R_DSP()
{
	R_DC_Removal();
//	setLEDCurrents();
	R_Calc();
}
