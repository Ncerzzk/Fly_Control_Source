/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "base.h"
#include "i2c.h"
#include "mpu9250.h"
#include "USART.h"



// HMC5883L, default address 0x1E
// PB12 connected to MAG_DRDY on rev4 hardware
// PC14 connected to MAG_DRDY on rev5 hardware

/* CTRL_REGA: Control Register A
 * Read Write
 * Default value: 0x10
 * 7:5  0   These bits must be cleared for correct operation.
 * 4:2 DO2-DO0: Data Output Rate Bits
 *             DO2 |  DO1 |  DO0 |   Minimum Data Output Rate (Hz)
 *            ------------------------------------------------------
 *              0  |  0   |  0   |            0.75
 *              0  |  0   |  1   |            1.5
 *              0  |  1   |  0   |            3
 *              0  |  1   |  1   |            7.5
 *              1  |  0   |  0   |           15 (default)
 *              1  |  0   |  1   |           30
 *              1  |  1   |  0   |           75
 *              1  |  1   |  1   |           Not Used
 * 1:0 MS1-MS0: Measurement Configuration Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Normal
 *              0  |  1   |  Positive Bias
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Not Used
 *
 * CTRL_REGB: Control RegisterB
 * Read Write
 * Default value: 0x20
 * 7:5 GN2-GN0: Gain Configuration Bits.
 *             GN2 |  GN1 |  GN0 |   Mag Input   | Gain       | Output Range
 *                 |      |      |  Range[Ga]    | [LSB/mGa]  |
 *            ------------------------------------------------------
 *              0  |  0   |  0   |  ?.88Ga      |   1370     | 0xF800?0x07FF (-2048:2047)
 *              0  |  0   |  1   |  ?.3Ga (def) |   1090     | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  0   |  ?.9Ga       |   820      | 0xF800?0x07FF (-2048:2047)
 *              0  |  1   |  1   |  ?.5Ga       |   660      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  0   |  ?.0Ga       |   440      | 0xF800?0x07FF (-2048:2047)
 *              1  |  0   |  1   |  ?.7Ga       |   390      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  0   |  ?.6Ga       |   330      | 0xF800?0x07FF (-2048:2047)
 *              1  |  1   |  1   |  ?.1Ga       |   230      | 0xF800?0x07FF (-2048:2047)
 *                               |Not recommended|
 *
 * 4:0 CRB4-CRB: 0 This bit must be cleared for correct operation.
 *
 * _MODE_REG: Mode Register
 * Read Write
 * Default value: 0x02
 * 7:2  0   These bits must be cleared for correct operation.
 * 1:0 MD1-MD0: Mode Select Bits
 *             MS1 | MS0 |   MODE
 *            ------------------------------
 *              0  |  0   |  Continuous-Conversion Mode.
 *              0  |  1   |  Single-Conversion Mode
 *              1  |  0   |  Negative Bias
 *              1  |  1   |  Sleep Mode
 */

#define MAG_ADDRESS 0x3c 				//0x1E
#define MAG_DATA_REGISTER 0x03

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16f)       // X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16f)       // Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08f)       // Z axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0f / 390.0f)    // Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0f / 390.0f)    // High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

static float magGain[3] = { 1.0f, 1.0f, 1.0f };
#define HMC_Write(address,data)   I2C_ByteWrite(I2C_USE,MAG_ADDRESS,data,address)

void hmc5883lRead(int16_t *magData);
void Adjust_HMC(void);


#define min(a,b) (a)>(b)?(b):(a)

float Mag_X,Mag_Y,Mag_Z;

void hmc5883lInit()
{
    int16_t magADC[3];
		int16_t temp;
    int i;
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.
    int bret = 1;           // Error indicator
		
		MPU_Single_Write(INT_PIN_CFG,0x02);    //MPU6500 开启路过模式
		
    delay_us(50*1000);
		HMC_Write(HMC58X3_R_CONFA,0x010 + HMC_POS_BIAS);
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    HMC_Write(HMC58X3_R_CONFB, 0x60); // Set the Gain to 2.5Ga (7:5->011)
    delay_us(100*1000);
    hmc5883lRead(magADC);

    for (i = 0; i < 10; i++) {  // Collect 10 samples
        HMC_Write(HMC58X3_R_MODE, 1);
        delay_us(50*1000);
        hmc5883lRead(magADC);       // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged rather than taking the max.
        xyz_total[0] += magADC[0];
        xyz_total[1] += magADC[1];
        xyz_total[2] += magADC[2];

        // Detect saturation.
				temp=min(magADC[1], magADC[2]);
				temp=min(magADC[0], temp);
        if (-4096 >= temp) {
            bret = 0;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
    }

    // Apply the negative bias. (Same gain)
    HMC_Write(HMC58X3_R_CONFA, 0x010 + HMC_NEG_BIAS);   // Reg A DOR = 0x010 + MS1, MS0 set to negative bias.
    for (i = 0; i < 10; i++) {
        HMC_Write(HMC58X3_R_MODE, 1);
        delay_us(50*1000);
        hmc5883lRead(magADC);               // Get the raw values in case the scales have already been changed.

        // Since the measurements are noisy, they should be averaged.
        xyz_total[0] -= magADC[0];
        xyz_total[1] -= magADC[1];
        xyz_total[2] -= magADC[2];

        // Detect saturation.
				temp=min(magADC[1], magADC[2]);
				temp=min(magADC[0],temp);
        if (-4096 >= temp) {
            bret =0;
            break;              // Breaks out of the for loop.  No sense in continuing if we saturated.
        }
    }

    magGain[0] = fabs(660.0f * HMC58X3_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[0]);
    magGain[1] = fabs(660.0f * HMC58X3_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[1]);
    magGain[2] = fabs(660.0f * HMC58X3_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[2]);

    // leave test mode
    HMC_Write(HMC58X3_R_CONFA, 0x78);   // Configuration Register A  -- 0 11 110 00  num samples: 8 ; output rate: 75Hz ; normal measurement mode
    HMC_Write(HMC58X3_R_CONFB, 0x20);   // Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    HMC_Write(HMC58X3_R_MODE, 0x00);    // Mode register             -- 000000 00    continuous Conversion Mode
    delay_us(100*1000);
    magGain[0] = 0.95f;
    magGain[1] = 1.05f;
    magGain[2] = 1.005f;
    if (!bret) {                // Something went wrong so get a best guess
        magGain[0] = 1.0f;
        magGain[1] = 1.0f;
        magGain[2] = 1.0f;
    }
}

void hmc5883lRead(int16_t *magData)
{
    uint8_t buf[6];
    int16_t mag[3];
		I2C_BufferRead(I2C_USE,MAG_ADDRESS,buf,MAG_DATA_REGISTER,6);
    //i2cRead(MAG_ADDRESS, MAG_DATA_REGISTER, 6, buf);
    // During calibration, magGain is 1.0, so the read returns normal non-calibrated values.
    // After calibration is done, magGain is set to calculated gain values.
    mag[0] = (int16_t)(buf[0] << 8 | buf[1]) ;//* magGain[0];   //X
    mag[2] = (int16_t)(buf[2] << 8 | buf[3]) ;//* magGain[2];		//Z
    mag[1] = (int16_t)(buf[4] << 8 | buf[5]);// * magGain[1];		//Y
	
		magData[0]=mag[0];
		magData[1]=mag[1];
		magData[2]=mag[2];
}

int MAG_Cnt;
void HMC_Get_Mag(void){
	int16_t temp[3];
	MAG_Cnt++;
	if(MAG_Cnt>=8){
		MPU_Single_Write(INT_PIN_CFG,0x02);    //MPU6500 开启路过模式
		hmc5883lRead(temp);
		Mag_X=(float)temp[0]/1090.0;
		Mag_Y=(float)temp[1]/1090.0;
		Mag_Z=(float)temp[2]/1090.0;

		MAG_Cnt=0;
		Adjust_HMC();
		//uprintf(USART,"%f,%f,%f\r\n",Mag_X,Mag_Y,Mag_Z);
	}
}
typedef struct {
float x;
float y;
float z;
} Axis3f;
 
Axis3f MagRaw;       //磁力计原始数据
Axis3f tmp3f;            //临时变量
Axis3f MagOffset;   // 磁力计偏移量参数
float B[6];                   // 磁力计校准B参数

void Adjust_HMC(void){
	B[0]=0.93613258676408;
	B[1]=0.00267184778915339;
	B[2]=0.0259231678709882;
	B[3]=0.953676023637478;
	B[4]=0.0035936506696941;
	B[5]=1.12085263700066;
	MagOffset.x=-0.0114886290660598;
	MagOffset.y=-0.11362460257489;
	MagOffset.z=0.0678062294292163;
	
	tmp3f.x = Mag_X - MagOffset.x;
	tmp3f.y = Mag_Y - MagOffset.y;
	tmp3f.z = Mag_Z - MagOffset.z;
	Mag_X = B [0]*tmp3f.x + B [1]*tmp3f.y + B [2]*tmp3f.z;
	Mag_Y = B [1]*tmp3f.x + B [3]*tmp3f.y + B [4]*tmp3f.z;
	Mag_Z = B [2]*tmp3f.x + B [4]*tmp3f.y + B [5]*tmp3f.z;	
}