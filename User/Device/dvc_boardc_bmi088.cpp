#include "dvc_boardc_bmi088.h"
#include "dvc_boardc_bmi088_middleware.h"
#include "dvc_boardc_bmi088_reg.h"
#include "dvc_dwt.h"
#include "math.h"

float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t res = 0;
static uint8_t write_reg_num = 0;
static uint8_t error = BMI088_NO_ERROR;
float gyroDiff[3], gNormDiff;

uint8_t caliOffset = 1;
int16_t caliCount = 0;

IMU_Data_t BMI088;
IMU_Real_Data_t BMI088_Real_Data;
SPI_HandleTypeDef *BMI088_SPI;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};

uint8_t Class_BoardC_BMI::init(SPI_HandleTypeDef *hspi)
{
    if(hspi == NULL)
    {
      return 0;
    }
    else if(hspi->Instance==SPI1)
    {
        SPI_Manage_Object = &SPI1_Manage_Object;
    }
    else if(hspi->Instance==SPI2)
    {
        SPI_Manage_Object = &SPI2_Manage_Object;
    }
    else if(hspi->Instance==SPI3)
    {
        SPI_Manage_Object = &SPI3_Manage_Object;
    }
    else
    {
        return 0;
    }
     
		Delay_Init();
    while (BMI088_Init());  
	return 1;
}


uint8_t Class_BoardC_BMI::BMI088_Init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    // BMI088_GPIO_init();
    // BMI088_com_init();

    // self test pass and init
    if (bmi088_accel_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_ACCEL_ERROR;
    }
    else
    {
        error |= BMI088_Accel_Init();
    }

    if (bmi088_gyro_self_test() != BMI088_NO_ERROR)
    {
        error |= BMI088_SELF_TEST_GYRO_ERROR;
    }
    else
    {
        error |= BMI088_Gyro_Init();
    }

    return error;
}

uint8_t Class_BoardC_BMI::BMI088_Accel_Init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    Delay_Ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}

uint8_t Class_BoardC_BMI::BMI088_Gyro_Init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    Delay_Ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], &res);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}

uint8_t Class_BoardC_BMI::bmi088_accel_self_test(void)
{

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
        {
            {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
            {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
            {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_ERROR},
            {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR},
            {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_ERROR}

        };

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    // reset  bmi088 accel sensor and wait for > 50ms
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    Delay_Ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    // set the accel register
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], &res);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        Delay_Ms(BMI088_LONG_DELAY_TIME);
    }

    // self test include postive and negative
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], &res);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
        {
            return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
        }
        // accel conf and accel range  . the two register set need wait for > 50ms
        Delay_Ms(BMI088_LONG_DELAY_TIME);

        // read response accel
        BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

        self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
        self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
        self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
    }

    //set self test off
    BMI088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_SELF_TEST, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
    {
        return BMI088_ACC_SELF_TEST_ERROR;
    }

    //reset the accel sensor
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    Delay_Ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
    {
        return BMI088_SELF_TEST_ACCEL_ERROR;
    }

    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_ERROR;
}

uint8_t Class_BoardC_BMI::bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //check commiunication is normal
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    Delay_Ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
    Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);

    BMI088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    Delay_Ms(BMI088_LONG_DELAY_TIME);

    do
    {

        BMI088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, &res);
        Delay_Us(BMI088_COM_WAIT_SENSOR_TIME);
        retry++;
    } while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    if (res & BMI088_GYRO_BIST_FAIL)
    {
        return BMI088_SELF_TEST_GYRO_ERROR;
    }

    return BMI088_NO_ERROR;
}

void Class_BoardC_BMI::BMI088_read_gyro_who_am_i(void)
{
    uint8_t buf;
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &buf);
}


void Class_BoardC_BMI::BMI088_read_accel_who_am_i(void)
{
    uint8_t buf;
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &buf);
}

void Class_BoardC_BMI::BMI088_Read(IMU_Data_t *bmi088)
{
    static uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    static int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    bmi088->Accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    bmi088->Accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    bmi088->Accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
	
    if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        if (caliOffset)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[0];
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[1];
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN - bmi088->GyroOffset[2];
        }
        else
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            bmi088->Gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            bmi088->Gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            bmi088->Gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
        }
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    bmi088->Temperature = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}


uint8_t Class_BoardC_BMI::BMI088_read_write_byte(uint8_t txdata)
{
	SPI_Manage_Object->Tx_Buffer[0]=txdata;
    SPI_Send_Receive_Data(SPI_Manage_Object->SPI_Handler,CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, 1, 1);
    //HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
	return SPI_Manage_Object->Rx_Buffer[0];
}

void Class_BoardC_BMI::BMI088_accel_write_single_reg(uint8_t reg,uint8_t data) 
{   
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin); 	
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);	
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
}
	
void Class_BoardC_BMI::BMI088_accel_read_single_reg(uint8_t reg,uint8_t* data) 
{   
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);                                                         
    BMI088_read_write_byte((reg) | 0x80);   
    BMI088_read_write_byte(0x55);         
    (*data) = BMI088_read_write_byte(0x55);  
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
}

//û����	
void Class_BoardC_BMI::BMI088_accel_write_muli_reg(uint8_t reg,uint8_t* data,uint8_t len) 
{
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
    BMI088_read_write_byte( reg );
    while( len != 0 )
    {
        BMI088_read_write_byte( *data );
        data ++;
        len --;
    }
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);
}

void Class_BoardC_BMI::BMI088_accel_read_muli_reg(uint8_t reg,uint8_t*  data,uint8_t  len) 
{                
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);                                                   
    BMI088_read_write_byte((reg) | 0x80);      
    BMI088_read_write_byte(reg | 0x80);
    while (len != 0)
    {  
        *data = BMI088_read_write_byte(0x55);
        data++;
        len--;
    } 
    HAL_GPIO_TogglePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin);                          
}
	
void Class_BoardC_BMI::BMI088_gyro_write_single_reg(uint8_t reg,uint8_t  data) 
{   
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);                                                            
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data); 
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);                    
}
	
void Class_BoardC_BMI::BMI088_gyro_read_single_reg(uint8_t reg,uint8_t*  data)  
{                                                               
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    BMI088_read_write_byte(reg | 0x80);
    (*data) = BMI088_read_write_byte(0x55);
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);		
}

//û����	
void Class_BoardC_BMI::BMI088_gyro_write_muli_reg(uint8_t reg,uint8_t*  data,uint8_t len) 
{ 
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    BMI088_read_write_byte( reg );
    while( len != 0 )
    {

        BMI088_read_write_byte( 0x55 );
        data ++;
        len --;
    }
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
}

void Class_BoardC_BMI::BMI088_gyro_read_muli_reg(uint8_t reg,uint8_t*  data,uint8_t  len)  
{    
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    BMI088_read_write_byte( reg | 0x80 );
    while( len != 0 )
    {
        *data = BMI088_read_write_byte(0x55);
        data ++;
        len --;
    }	
    HAL_GPIO_TogglePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);	
}




void Class_BoardC_BMI::Delay_Init(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;

}

void Class_BoardC_BMI::Delay_Us(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void Class_BoardC_BMI::Delay_Ms(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}