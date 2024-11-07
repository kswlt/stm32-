#include "mpu6050.h"
#include"math.h"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;


float Ax, Ay, Az, Gx, Gy, Gz;

float roll_a, pitch_a;


float roll_g, pitch_g, yaw_g;


//欧拉角
float roll, pitch, yaw;



void MPU6050_Init(void)
{
	uint8_t check;
	uint8_t Data;
//文档最后一页
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	//为啥不能挂载在同一条IIC总线上，如果改成I21就一点反应都木有

	if (check == 0x68)
	{
		//电源管理1
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		//电源管理2
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);

		
	
		//采样频率分频器寄存器
		Data = 0x09;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		//加速度计配置
		Data = 0x18;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		//陀螺仪配置
		Data = 0x18;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read_Accel(void)
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	//文档第23页    8g   2048LSB/g
	Ax = Accel_X_RAW / 2048.0;
	Ay = Accel_Y_RAW / 2048.0;
	Az = Accel_Z_RAW  / 2048.0 ;
}

void MPU6050_Read_Gyro(void)
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	Gx = Gyro_X_RAW / 16.4;
	Gy = Gyro_Y_RAW / 16.4;
	Gz = Gyro_Z_RAW / 16.4;
	//25页
}


void MPU6050_Read_Result(void){


	uint8_t Rec_Data_A[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_A, 6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data_A[0] << 8 | Rec_Data_A[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data_A[2] << 8 | Rec_Data_A[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data_A[4] << 8 | Rec_Data_A[5]);
	Ax = Accel_X_RAW / 2048.0;
	Ay = Accel_Y_RAW / 2048.0;
	Az = Accel_Z_RAW  / 2048.0;

	uint8_t Rec_Data_G[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_G, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data_G[0] << 8 | Rec_Data_G[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data_G[2] << 8 | Rec_Data_G[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data_G[4] << 8 | Rec_Data_G[5]);
	Gx = Gyro_X_RAW / 16.384;
	Gy = Gyro_Y_RAW / 16.384;
	Gz = Gyro_Z_RAW / 16.384;


	//使用加速度计算欧拉角
	roll_a = atan2(Ay, Az) /3.14f * 180;
	pitch_a = - atan2(Ax, Az) /3.14f * 180;

	//使用角速度计算欧拉角
	yaw_g = yaw + Gz * 0.005;
	roll_g = roll + Gx * 0.005;
	pitch_g = pitch + Gy * 0.005;
//0.05积分
/*https://blog.csdn.net/hbsyaaa/article/details108186892?fromshare=blogdetail&sharetype=blogdetail&sharerId=108186892&sharerefer=PC&sharesource=m0_70625833&sharefrom=from_link*/
	const float alpha = 0.9;
	roll = roll + (roll_a - roll_g ) *alpha;
	pitch = pitch + (pitch_a - pitch_g) *alpha;
	yaw = yaw_g;

}







