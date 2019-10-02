/*
 * IMU.c
 *
 *  Created on: 16.01.2019
 *      Author: cc-isn
 */

#include "IMU.h"
#include "SM1.h"
#include "WAIT1.h"

float biasGyro[3];
float biasACC;
IMU_data_t imuData;

/*!
* \brief Writes a byte and reads the value
* \param val Value to write. This value will be shifted out
* \return The value shifted in
*/
static int16_t SPI_WriteRead(int16_t val) {
	int16_t ch;

  while (SM1_GetCharsInTxBuf()!=0) {} /* wait until tx is empty */
  while (SM1_SendChar(val)!=ERR_OK) {} /* send character */
  while (SM1_GetCharsInTxBuf()!=0) {} /* wait until data has been sent */
  while (SM1_GetCharsInRxBuf()==0) {} /* wait until we receive data */
  while (SM1_RecvChar(&ch)!=ERR_OK) {} /* get data */
  SM1_ClearRxBuf();
  return ch;
}

void RF_WriteRegister(int8_t reg, int16_t val) {
  (void)SPI_WriteRead((reg<<8)|val); /* write register command */
}

int16_t RF_ReadRegister(int8_t reg) {
  return SPI_WriteRead(0b1000000000000000|(reg<<8)); /* write register command */
}

bool initIMU(void)
{
	uint16_t data;
	uint8_t result;
	data = RF_ReadRegister(0x0F);
	data = RF_ReadRegister(0x0F);
	result=(uint8_t)data;

	if(result == 0x69){
		RF_WriteRegister(0x12, (int16_t)0x00);//multiple byte access NOT

		RF_WriteRegister(0x10, (int16_t)0x08);//+- 4g full scale
		RF_WriteRegister(0x11, (int16_t)0x0C);//500dps full scale

		RF_WriteRegister(0x10, (int16_t)0x88);//1.66kHz Samplingrate
		RF_WriteRegister(0x11, (int16_t)0x8C);// 1.66kHz Samplingrate

		return TRUE;
	} else {
		return FALSE;
	}


}




/*******************************************************************************
* Function Name  : mems_status_t LSM6DSL_ACC_GYRO_GetRawAccData(uint8_t *buff)
* Description    : Read GetAccData output register
* Input          : pointer to [uint8_t]
* Output         : GetAccData buffer uint8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
void LSM6DSL_ACC_GYRO_GetRawAccData_1(uint8_t *buff)
{
uint8_t i, j, k;
  uint8_t numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<numberOfByteForDimension;i++ )
  {
		uint16_t data;
		data = RF_ReadRegister(0x28+k);
		buff[k]=(uint8_t)data;
		k++;
  }
}

/*******************************************************************************
* Function Name  : mems_status_t LSM6DSL_ACC_GYRO_GetRawGyroData(uint8_t *buff)
* Description    : Read GetGyroData output register
* Input          : pointer to [uint8_t]
* Output         : GetGyroData buffer uint8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
void LSM6DSL_ACC_GYRO_GetRawGyroData_1(uint8_t *buff)
{
	uint8_t i, j, k;
  uint8_t numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<numberOfByteForDimension;i++ )
  {
		uint16_t data;
		data = RF_ReadRegister(0x26+k);
		buff[k]=(uint8_t)data;
		k++;
  }
}


/**
// * @brief  Read raw data from LSM6DSL Accelerometer
 * @param  pData the pointer where the accelerometer raw data are stored
 * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
 */
void Get_X_AxesRaw_1(int16_t *pData)
{
  uint8_t regValue[2] = {0, 0};

  /* Read output registers from LSM6DSL_ACC_GYRO_OUTX_L_XL to LSM6DSL_ACC_GYRO_OUTZ_H_XL. */
  LSM6DSL_ACC_GYRO_GetRawAccData_1(regValue);

  /* Format the data. */
  *pData = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );


}

/**
 * @brief  Read raw data from LSM6DSL Gyroscope
 * @param  pData the pointer where the gyroscope raw data are stored
 * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
 */
void Get_G_AxesRaw_1(int16_t *pData)
{
  uint8_t regValue[2] = {0, 0};

  /* Read output registers from LSM6DSL_ACC_GYRO_OUTX_L_G to LSM6DSL_ACC_GYRO_OUTZ_H_G. */
 LSM6DSL_ACC_GYRO_GetRawGyroData_1(regValue);

  /* Format the data. */
  *pData = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );

}



/**
 * @brief  Read data from LSM6DSL Accelerometer
 * @param  pData the pointer where the accelerometer data are stored
 * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
 */
void Get_X_Axes_1(IMU_data_t *pData)
{
  int16_t dataRaw;


  /* Read raw data from LSM6DSL output register. */
  Get_X_AxesRaw_1(&dataRaw);

  /* Calculate the data. */
  pData->raw_Values.x_Axis_Acc_mg = dataRaw * 0.122f;
  pData->unit_Values.x_Axis_Acc_ms2 = (pData->raw_Values.x_Axis_Acc_mg - pData->bias_Values.bias_x_Axis_Acc_mg)*0.001f*-9.81f;
  pData->unit_Values.x_Axis_Acc_velocity = pData->unit_Values.x_Axis_Acc_velocity + (pData->unit_Values.x_Axis_Acc_ms2*SAMPLETIME);
}



/**
 * @brief  Read data from LSM6DSL Gyroscope
 * @param  pData the pointer where the gyroscope data are stored
 * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
 */
void Get_G_Axes_1(IMU_data_t *pData)
{
  int16_t dataRaw;


  /* Read raw data from LSM6DSL output register. */
  Get_G_AxesRaw_1(&dataRaw);

  /* Calculate the data. */
  pData->raw_Values.z_Axis_Gyro_mdps = dataRaw * 70.0f;//17.5f at 500dps;
  pData->unit_Values.z_Axis_Gyro_dps = (pData->raw_Values.z_Axis_Gyro_mdps - pData->bias_Values.bias_z_Axis_Gyro_mdps)*0.001;
  pData->unit_Values.z_Axis_Gyro_rps = (pData->unit_Values.z_Axis_Gyro_dps* DEGREE_TO_RADIANT);
  pData->unit_Values.z_Axis_Gyro_angle = pData->unit_Values.z_Axis_Gyro_angle + (pData->unit_Values.z_Axis_Gyro_dps*SAMPLETIME);

}




/**
 * TODO: Macro for sample time in Waitms and makro for number of recorded bias values/samples
 */
void biasCalc(void)
{
	for(int i = 0; i<10000; i++){
		calcIMU_data(&imuData);
		biasACC += imuData.raw_Values.x_Axis_Acc_mg;
		biasGyro[0] += imuData.raw_Values.x_Axis_Gyro_mdps;
		biasGyro[1] += imuData.raw_Values.y_Axis_Gyro_mdps;
		biasGyro[2] += imuData.raw_Values.z_Axis_Gyro_mdps;
		WAIT1_Waitms(1);
	}
	biasACC = biasACC*0.0001;
	biasGyro[0]  = biasGyro[0]*0.0001;
	biasGyro[1]  = biasGyro[1]*0.0001;
	biasGyro[2]  = biasGyro[2]*0.0001;
}

float getGyroBias_Z(void){
 return biasGyro[2];
}
float getGyroBias_Y(void){
 return biasGyro[1];
}
float getGyroBias_X(void){
 return biasGyro[0];
}
float getAccBias(void){
	return biasACC;
}












#if 1


/*******************************************************************************
* Function Name  : mems_status_t LSM6DSL_ACC_GYRO_GetRawGyroData(uint8_t *buff)
* Description    : Read GetGyroData output register
* Input          : pointer to [uint8_t]
* Output         : GetGyroData buffer uint8_t
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
void LSM6DSL_ACC_GYRO_GetRawGyroData(uint8_t *buff)
{
	uint8_t i, j, k;
  uint8_t numberOfByteForDimension;

  numberOfByteForDimension=6/3;

  k=0;
  for (i=0; i<3;i++ )
  {
	for (j=0; j<numberOfByteForDimension;j++ )
	{
		uint16_t data;
		data = RF_ReadRegister(0x22+k);
		buff[k]=(uint8_t)data;
		k++;
	}
  }
}




/**
 * @brief  Read raw data from LSM6DSL Gyroscope
 * @param  pData the pointer where the gyroscope raw data are stored
 * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
 */
void Get_G_AxesRaw(int16_t *pData)
{
  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};

  /* Read output registers from LSM6DSL_ACC_GYRO_OUTX_L_G to LSM6DSL_ACC_GYRO_OUTZ_H_G. */
 LSM6DSL_ACC_GYRO_GetRawGyroData(regValue);

  /* Format the data. */
  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );
}


void Get_G_Axes(IMU_data_t *pData)
{
	  int16_t dataRaw[3];


	  /* Read raw data from LSM6DSL output register. */
	  Get_G_AxesRaw(dataRaw);

  /* Calculate the data. */

	  pData->raw_Values.x_Axis_Gyro_mdps = dataRaw[0] * 70.0f;//17.5f at 500dps;
	  pData->unit_Values.x_Axis_Gyro_dps = (pData->raw_Values.x_Axis_Gyro_mdps - pData->bias_Values.bias_x_Axis_Gyro_mdps)*0.001;
	  pData->unit_Values.x_Axis_Gyro_rps = (pData->unit_Values.x_Axis_Gyro_dps* DEGREE_TO_RADIANT);
	  pData->unit_Values.x_Axis_Gyro_angle = pData->unit_Values.x_Axis_Gyro_angle + (pData->unit_Values.x_Axis_Gyro_dps*SAMPLETIME);


	  pData->raw_Values.y_Axis_Gyro_mdps = dataRaw[1] * 70.0f;//17.5f at 500dps;
	  pData->unit_Values.y_Axis_Gyro_dps = (pData->raw_Values.y_Axis_Gyro_mdps - pData->bias_Values.bias_y_Axis_Gyro_mdps)*0.001;
	  pData->unit_Values.y_Axis_Gyro_rps = (pData->unit_Values.y_Axis_Gyro_dps* DEGREE_TO_RADIANT);
	  pData->unit_Values.y_Axis_Gyro_angle = pData->unit_Values.y_Axis_Gyro_angle + (pData->unit_Values.y_Axis_Gyro_dps*SAMPLETIME);


  pData->raw_Values.z_Axis_Gyro_mdps = dataRaw[2] * 70.0f;//17.5f at 500dps;
  pData->unit_Values.z_Axis_Gyro_dps = (pData->raw_Values.z_Axis_Gyro_mdps - pData->bias_Values.bias_z_Axis_Gyro_mdps)*0.001;
  pData->unit_Values.z_Axis_Gyro_rps = (pData->unit_Values.z_Axis_Gyro_dps* DEGREE_TO_RADIANT);
  pData->unit_Values.z_Axis_Gyro_angle = pData->unit_Values.z_Axis_Gyro_angle + (pData->unit_Values.z_Axis_Gyro_dps*SAMPLETIME);

}


#endif


/**
 *
 */
void calcIMU_data(IMU_data_t *pData)
{
	  Get_G_Axes_1(pData);
	  Get_X_Axes_1(pData);
}























#if 0
//
///**
// * @brief  Read Accelerometer Sensitivity
// * @param  pfData the pointer where the accelerometer sensitivity is stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_X_Sensitivity(float *pfData)
//{
//	uint8_t adr = 0x6A; //IMU Address when SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b)
//  uint8_t fullScale;
//
//  /* Read actual full scale selection from sensor. */
//  GI2C1_ReadByteAddress8(adr, 0x10, &fullScale);
//  fullScale &= 0x0C;//Maskierung
//
//
//  /* Store the sensitivity based on actual full scale. */
//  switch( fullScale )
//  {
//    case 0x00:
//      *pfData = ( float )0.061;
//      break;
//    case 0x04:
//      *pfData = ( float )0.122;
//      break;
//    case 0x08:
//      *pfData = ( float )0.244 ;
//      break;
//    case 0x0C:
//      *pfData = ( float )0.488;
//      break;
//    default:
//      *pfData = -1.0f;
//  }
//}
//
///**
// * @brief  Read Gyroscope Sensitivity
// * @param  pfData the pointer where the gyroscope sensitivity is stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_G_Sensitivity(float *pfData)
//{
//	uint8_t adr = 0x6A; //IMU Address when SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b)
//  uint8_t fullScale;
//
//  /* Read actual full scale selection from sensor. */
//  GI2C1_ReadByteAddress8(adr, 0x11, &fullScale);
//  fullScale &= 0x0C;//Maskierung
//
//
//    /* Store the sensitivity based on actual full scale. */
//    switch(fullScale)
//    {
//      case 0x00:
//        *pfData = ( float )08.750;
//        break;
//      case 0x04:
//        *pfData = ( float )17.500;
//        break;
//      case 0x08:
//        *pfData = ( float )35.000;
//        break;
//      case 0x0C:
//        *pfData = ( float )70.000;
//        break;
//      default:
//        *pfData = -1.0f;
//    }
//}
//
///*******************************************************************************
//* Function Name  : mems_status_t LSM6DSL_ACC_GYRO_GetRawAccData(uint8_t *buff)
//* Description    : Read GetAccData output register
//* Input          : pointer to [uint8_t]
//* Output         : GetAccData buffer uint8_t
//* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
//*******************************************************************************/
//void LSM6DSL_ACC_GYRO_GetRawAccData(uint8_t *buff)
//{
//	uint8_t adr = 0x6A; //IMU Address when SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b)
//  uint8_t i, j, k;
//  uint8_t numberOfByteForDimension;
//
//  numberOfByteForDimension=6/3;
//
//  k=0;
//  for (i=0; i<3;i++ )
//  {
//	for (j=0; j<numberOfByteForDimension;j++ )
//	{
//		GI2C1_ReadByteAddress8(adr, 0x28+k, &buff[k]);
//		k++;
//	}
//  }
//}
//
///*******************************************************************************
//* Function Name  : mems_status_t LSM6DSL_ACC_GYRO_GetRawGyroData(uint8_t *buff)
//* Description    : Read GetGyroData output register
//* Input          : pointer to [uint8_t]
//* Output         : GetGyroData buffer uint8_t
//* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
//*******************************************************************************/
//void LSM6DSL_ACC_GYRO_GetRawGyroData(uint8_t *buff)
//{
//	uint8_t adr = 0x6A; //IMU Address when SDO/SA0 pin is connected to ground, the LSb value is ‘0’ (address 1101010b)
//  uint8_t i, j, k;
//  uint8_t numberOfByteForDimension;
//
//  numberOfByteForDimension=6/3;
//
//  k=0;
//  for (i=0; i<3;i++ )
//  {
//	for (j=0; j<numberOfByteForDimension;j++ )
//	{
//		GI2C1_ReadByteAddress8(adr, 0x22+k, &buff[k]);
//		k++;
//	}
//  }
//}
//
//
///**
// * @brief  Read raw data from LSM6DSL Accelerometer
// * @param  pData the pointer where the accelerometer raw data are stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_X_AxesRaw(int16_t *pData)
//{
//  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
//
//  /* Read output registers from LSM6DSL_ACC_GYRO_OUTX_L_XL to LSM6DSL_ACC_GYRO_OUTZ_H_XL. */
//  LSM6DSL_ACC_GYRO_GetRawAccData(regValue);
//
//  /* Format the data. */
//  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
//  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
//  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );
//
//}
//
///**
// * @brief  Read raw data from LSM6DSL Gyroscope
// * @param  pData the pointer where the gyroscope raw data are stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_G_AxesRaw(int16_t *pData)
//{
//  uint8_t regValue[6] = {0, 0, 0, 0, 0, 0};
//
//  /* Read output registers from LSM6DSL_ACC_GYRO_OUTX_L_G to LSM6DSL_ACC_GYRO_OUTZ_H_G. */
// LSM6DSL_ACC_GYRO_GetRawGyroData(regValue);
//
//  /* Format the data. */
//  pData[0] = ( ( ( ( int16_t )regValue[1] ) << 8 ) + ( int16_t )regValue[0] );
//  pData[1] = ( ( ( ( int16_t )regValue[3] ) << 8 ) + ( int16_t )regValue[2] );
//  pData[2] = ( ( ( ( int16_t )regValue[5] ) << 8 ) + ( int16_t )regValue[4] );
//}
//
//
//
///**
// * @brief  Read data from LSM6DSL Accelerometer
// * @param  pData the pointer where the accelerometer data are stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_X_Axes(float *pData)
//{
//  int16_t dataRaw[3];
//  float sensitivity = 0;
//
//  /* Read raw data from LSM6DSL output register. */
//  Get_X_AxesRaw(dataRaw);
//
//
//  /* Get LSM6DSL actual sensitivity. */
//  Get_X_Sensitivity(&sensitivity);
//
//  /* Calculate the data. */
//  pData[0] = ( float )((( dataRaw[0] * sensitivity )-biasACC_X)/1000);
//  pData[1] = ( float )((( dataRaw[1] * sensitivity )-biasACC_Y)/1000);
//  pData[2] = ( float )((( dataRaw[2] * sensitivity )-biasACC_Z)/1000);
//
//}
//
//
///**
// * @brief  Read data from LSM6DSL Gyroscope
// * @param  pData the pointer where the gyroscope data are stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_G_Axes_Int(void)
//{
//  int16_t dataRaw[3];
//  float sensitivity = 0;
//
//  /* Read raw data from LSM6DSL output register. */
//  Get_G_AxesRaw(dataRaw);
//
//
//  /* Get LSM6DSL actual sensitivity. */
//  Get_G_Sensitivity(&sensitivity);
//
//  /* Calculate the data. */
//  gyroscope[0] = ( float )((( dataRaw[0] * sensitivity )-biasGyro_X)/1000);
//  gyroscope[1] = ( float )((( dataRaw[1] * sensitivity )-biasGyro_Y)/1000);
//  gyroscope[2] = ( float )((( dataRaw[2] * sensitivity )-biasGyro_Z)/1000);
//
//  angle = angle + (gyroscope[2]*0.0008966);
//}
//
///**
// * @brief  Read data from LSM6DSL Gyroscope
// * @param  pData the pointer where the gyroscope data are stored
// * @retval LSM6DSL_STATUS_OK in case of success, an error code otherwise
// */
//void Get_G_Axes(float *pData)
//{
//  int16_t dataRaw[3];
//  float sensitivity = 0;
//
//  /* Read raw data from LSM6DSL output register. */
//  Get_G_AxesRaw(dataRaw);
//
//
//  /* Get LSM6DSL actual sensitivity. */
//  Get_G_Sensitivity(&sensitivity);
//
//  /* Calculate the data. */
//  pData[0] = ( float )((( dataRaw[0] * sensitivity )-biasGyro_X)/1000);
//  pData[1] = ( float )((( dataRaw[1] * sensitivity )-biasGyro_Y)/1000);
//  pData[2] = ( float )((( dataRaw[2] * sensitivity )-biasGyro_Z)/1000);
//
//  angle = angle + (gyroscope[2]*0.00102);
//}

#endif
