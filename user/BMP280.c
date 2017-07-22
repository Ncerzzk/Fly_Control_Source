/*
 *****************************************************************************
 *
 * (C) All rights reserved by ROBERT BOSCH GMBH
 *
 ****************************************************************************/
/*  Date: 2013/05/06
 *  Revision: 1.3 (Pressure and Temperature compensation code is revision 1.1)
 *
 */

/*****************************************************************************
* Copyright (C) 2013 Bosch Sensortec GmbH
*
* <BMP280 API>
*
* Usage:        <API for accessing the BMP280 sensor>
*
*****************************************************************************/
/****************************************************************************/
/*  Disclaimer
*
* Common:
* Bosch Sensortec products are developed for the consumer goods industry.
* They may only be used within the parameters of the respective valid
* product data sheet.  Bosch Sensortec products are provided with the
* express understanding that there is no warranty of fitness for a
* particular purpose.They are not fit for use in life-sustaining,
* safety or security sensitive systems or any system or device
* that may lead to bodily harm or property damage if the system
* or device malfunctions. In addition,Bosch Sensortec products are
* not fit for use in products which interact with motor vehicle systems.
* The resale and or use of products are at the purchasers own risk and
* his own responsibility. The examination of fitness for the intended use
* is the sole responsibility of the Purchaser.
*
* The purchaser shall indemnify Bosch Sensortec from all third party
* claims, including any claims for incidental, or consequential damages,
* arising from any product use not covered by the parameters of
* the respective valid product data sheet or not approved by
* Bosch Sensortec and reimburse Bosch Sensortec for all costs in
* connection with such claims.
*
* The purchaser must monitor the market for the purchased products,
* particularly with regard to product safety and inform Bosch Sensortec
* without delay of all security relevant incidents.
*
* Engineering Samples are marked with an asterisk (*) or (e).
* Samples may vary from the valid technical specifications of the product
* series. They are therefore not intended or fit for resale to third
* parties or for use in end products. Their sole purpose is internal
* client testing. The testing of an engineering sample may in no way
* replace the testing of a product series. Bosch Sensortec assumes
* no liability for the use of engineering samples.
* By accepting the engineering samples, the Purchaser agrees to indemnify
* Bosch Sensortec from all claims arising from the use of engineering
* samples.
*
* Special:
* This software module (hereinafter called "Software") and any information
* on application-sheets (hereinafter called "Information") is provided
* free of charge for the sole purpose to support your application work.
* The Software and Information is subject to the following
* terms and conditions:
*
* The Software is specifically designed for the exclusive use for
* Bosch Sensortec products by personnel who have special experience
* and training. Do not use this Software if you do not have the
* proper experience or training.
*
* This Software package is provided `` as is `` and without any expressed
* or implied warranties,including without limitation, the implied warranties
* of merchantability and fitness for a particular purpose.
*
* Bosch Sensortec and their representatives and agents deny any liability
* for the functional impairment
* of this Software in terms of fitness, performance and safety.
* Bosch Sensortec and their representatives and agents shall not be liable
* for any direct or indirect damages or injury, except as
* otherwise stipulated in mandatory applicable law.
*
* The Information provided is believed to be accurate and reliable.
* Bosch Sensortec assumes no responsibility for the consequences of use
* of such Information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of Bosch. Specifications mentioned in the Information are
* subject to change without notice.
*
* It is not allowed to deliver the source code of the Software
* to any third party without permission of
* Bosch Sensortec.
*/
/****************************************************************************/

#include "BMP280.h"
struct bmp280_t *p_bmp280;                      /**< pointer to BMP280 */
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief routine to initialize the function pointers
 *
 *
 *
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_init(struct bmp280_t *bmp280)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	p_bmp280 = bmp280;                         /* assign BMP280 ptr */
	p_bmp280->dev_addr = BMP280_I2C_ADDRESS;   /* preset BMP280 I2C_addr */
	comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
		BMP280_CHIPID_REG, &v_data_u8r, 1);    /* read Chip Id */
	p_bmp280->chip_id = v_data_u8r;

	bmp280_get_calib_param(); /* readout bmp280 calibparam structure */
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated temperature
 *
 *
 *
 * \param signed long utemperature : Pointer holding
 * 	                           the uncompensated temperature.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_ut(BMP280_S32_t *utemperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[3] = {0};
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_TEMPERATURE_MSB_REG, a_data_u8r, 3);
		*utemperature = (BMP280_S32_t)((( \
		(BMP280_U32_t) (a_data_u8r[0])) << SHIFT_LEFT_12_POSITION) | \
		(((BMP280_U32_t)(a_data_u8r[1])) << SHIFT_LEFT_4_POSITION) \
		| ((BMP280_U32_t)a_data_u8r[2] >> SHIFT_RIGHT_4_POSITION));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *                    and returns the value in 0.01 degree Centigrade
 *                    Output value of "5123" equals 51.23 DegC.
 *
 *
 *
 *  \param signed long : value of uncompensated temperature.
 *
 *
 *
 *  \return
 *			signed long : actual temperature
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t v_x1_u32r = 0;
	BMP280_S32_t v_x2_u32r = 0;
	BMP280_S32_t temperature = 0;

	v_x1_u32r  = ((((adc_T >> 3) - ((BMP280_S32_t)
		p_bmp280->cal_param.dig_T1 << 1))) * \
		((BMP280_S32_t)p_bmp280->cal_param.dig_T2)) >> 11;
	v_x2_u32r  = (((((adc_T >> 4) - \
		((BMP280_S32_t)p_bmp280->cal_param.dig_T1)) * ((adc_T >> 4) - \
		((BMP280_S32_t)p_bmp280->cal_param.dig_T1))) >> 12) * \
		((BMP280_S32_t)p_bmp280->cal_param.dig_T3)) >> 14;
	p_bmp280->cal_param.t_fine = v_x1_u32r + v_x2_u32r;
	temperature  = (p_bmp280->cal_param.t_fine * 5 + 128) >> 8;

	return temperature;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure.
 *
 *
 *
 *
 *  \param signed long upressure : Pointer holding the uncompensated pressure.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_up(BMP280_S32_t *upressure)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[3] = {0};
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
		BMP280_PRESSURE_MSB_REG, a_data_u8r, 3);
		*upressure = (BMP280_S32_t)((((BMP280_U32_t)(a_data_u8r[0])) \
		<< SHIFT_LEFT_12_POSITION) | (((BMP280_U32_t)(a_data_u8r[1])) \
		<< SHIFT_LEFT_4_POSITION) | ((BMP280_U32_t)a_data_u8r[2] >>\
		SHIFT_RIGHT_4_POSITION));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns the value in Pascal(Pa)
 *                          Output value of "96386" equals 96386 Pa =
 *                          963.86 hPa = 963.86 millibar

 *
 *
 *
 *  \param signed long : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			unsigned long : actual pressure
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
unsigned int bmp280_compensate_P_int32(BMP280_S32_t adc_P)
{
	BMP280_S32_t v_x1_u32r = 0;
	BMP280_S32_t v_x2_u32r = 0;
	BMP280_U32_t pressure = 0;

	v_x1_u32r = (((BMP280_S32_t)p_bmp280->cal_param.t_fine) >> 1) - \
		(BMP280_S32_t)64000;
	v_x2_u32r = (((v_x1_u32r >> 2) * (v_x1_u32r >> 2)) >> 11) * \
		((BMP280_S32_t)p_bmp280->cal_param.dig_P6);
	v_x2_u32r = v_x2_u32r + ((v_x1_u32r * \
		((BMP280_S32_t)p_bmp280->cal_param.dig_P5)) << 1);
	v_x2_u32r = (v_x2_u32r >> 2) + \
		(((BMP280_S32_t)p_bmp280->cal_param.dig_P4) << 16);
	v_x1_u32r = (((p_bmp280->cal_param.dig_P3 * (((v_x1_u32r >> 2) * \
		(v_x1_u32r >> 2)) >> 13)) >> 3) + \
		((((BMP280_S32_t)p_bmp280->cal_param.dig_P2) * \
		v_x1_u32r) >> 1)) >> 18;
	v_x1_u32r = ((((32768+v_x1_u32r)) * \
		((BMP280_S32_t)p_bmp280->cal_param.dig_P1))	>> 15);
	if (v_x1_u32r == 0)
		return 0; /* Avoid exception caused by division by zero */
	pressure = (((BMP280_U32_t)(((BMP280_S32_t)1048576) - adc_P) - \
		(v_x2_u32r >> 12))) * 3125;
	if (pressure < 0x80000000)
		pressure = (pressure << 1) / ((BMP280_U32_t)v_x1_u32r);
	else
		pressure = (pressure / (BMP280_U32_t)v_x1_u32r) * 2;
	v_x1_u32r = (((BMP280_S32_t)p_bmp280->cal_param.dig_P9) * \
		((BMP280_S32_t)(((pressure >> 3) * (pressure >> 3)) >> 13))) \
		>> 12;
	v_x2_u32r = (((BMP280_S32_t)(pressure >> 2)) * \
		((BMP280_S32_t)p_bmp280->cal_param.dig_P8)) >> 13;
	pressure = (BMP280_U32_t)((BMP280_S32_t)pressure + \
		((v_x1_u32r + v_x2_u32r + p_bmp280->cal_param.dig_P7) >> 4));

	return pressure;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads uncompensated pressure and temperature
 *
 *
 * \param signed long upressure: Pointer holding
 *	                    the uncompensated pressure.
 * \param signed long utemperature: Pointer holding
 *			           the uncompensated temperature.
 *
 *  \return
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_uput(BMP280_S32_t *upressure,\
		BMP280_S32_t *utemperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[6] = {0};
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
		BMP280_PRESSURE_MSB_REG, a_data_u8r, 6);
		*upressure = (BMP280_S32_t)((((BMP280_U32_t)(a_data_u8r[0])) \
		<< SHIFT_LEFT_12_POSITION) | (((BMP280_U32_t)(a_data_u8r[1])) \
		<< SHIFT_LEFT_4_POSITION) | ((BMP280_U32_t)a_data_u8r[2] >>\
		SHIFT_RIGHT_4_POSITION));

		/* Temperature */
		*utemperature = (BMP280_S32_t)((( \
		(BMP280_U32_t) (a_data_u8r[3])) << SHIFT_LEFT_12_POSITION) | \
		(((BMP280_U32_t)(a_data_u8r[4])) << SHIFT_LEFT_4_POSITION) \
		| ((BMP280_U32_t)a_data_u8r[5] >> SHIFT_RIGHT_4_POSITION));
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads pressure and temperature
 *
 *
 *  \param unsigned long pressure : Pointer holding the compensated pressure.
 *  \param signed long temperature : Pointer holding
 *                          the compensated temperature.
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_pt(BMP280_U32_t *pressure,\
		BMP280_S32_t *temperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	BMP280_S32_t upressure = 0;
	BMP280_S32_t utemperature = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = bmp280_read_uput(&upressure, &utemperature);
		*temperature = bmp280_compensate_T_int32(utemperature);
		*pressure = bmp280_compensate_P_int32(upressure);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief reads calibration parameters used for calculation
 *
 *
 *
 *
 *  \param  None
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_calib_param()
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char a_data_u8r[26] = {0};
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_DIG_T1_LSB_REG, a_data_u8r, 24);

		p_bmp280->cal_param.dig_T1 = (BMP280_U16_t)(((\
			(BMP280_U16_t)((unsigned char)a_data_u8r[1])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[0]);
		p_bmp280->cal_param.dig_T2 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[3])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[2]);
		p_bmp280->cal_param.dig_T3 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[5])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[4]);
		p_bmp280->cal_param.dig_P1 = (BMP280_U16_t)(((\
			(BMP280_U16_t)((unsigned char)a_data_u8r[7])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[6]);
		p_bmp280->cal_param.dig_P2 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[9])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[8]);
		p_bmp280->cal_param.dig_P3 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[11])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[10]);
		p_bmp280->cal_param.dig_P4 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[13])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[12]);
		p_bmp280->cal_param.dig_P5 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[15])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[14]);
		p_bmp280->cal_param.dig_P6 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[17])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[16]);
		p_bmp280->cal_param.dig_P7 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[19])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[18]);
		p_bmp280->cal_param.dig_P8 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[21])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[20]);
		p_bmp280->cal_param.dig_P9 = (BMP280_S16_t)(((\
			(BMP280_S16_t)((signed char)a_data_u8r[23])) << \
			SHIFT_LEFT_8_POSITION) | a_data_u8r[22]);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the temperature oversampling setting
 *
 *
 *
 *
 *  \param unsigned char value : Pointer holding the osrs_t value
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_osrs_t(\
		unsigned char *value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_OSRST__REG, &v_data_u8r, 1);
		*value = BMP280_GET_BITSLICE(v_data_u8r, \
			BMP280_CTRLMEAS_REG_OSRST);

		p_bmp280->osrs_t = *value;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 *Description: *//**\brief Used to set the temperature oversampling setting
 *
 *
 *
 *
 *  \param unsigned char value : Value of the temperature oversampling setting.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_osrs_t(\
	unsigned char value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_OSRST__REG, &v_data_u8r, 1);
		v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
			BMP280_CTRLMEAS_REG_OSRST, value);
		comres += p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_OSRST__REG, &v_data_u8r, 1);

		p_bmp280->osrs_t = value;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the pressure oversampling setting
 *
 *
 *
 *
 *  \param  *  \param unsigned char value : Pointer holding the osrs_p value.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_osrs_p(\
	unsigned char *value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_OSRSP__REG, &v_data_u8r, 1);
		*value = BMP280_GET_BITSLICE(v_data_u8r, \
			BMP280_CTRLMEAS_REG_OSRSP);

		p_bmp280->osrs_p = *value;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to set the pressure oversampling setting
 *
 *
 *
 *
 *  \param unsigned char value : Value of the pressure oversampling setting.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_osrs_p(\
	unsigned char value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_OSRSP__REG, &v_data_u8r, 1);
		v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
			BMP280_CTRLMEAS_REG_OSRSP, value);
		comres += p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_OSRSP__REG, &v_data_u8r, 1);

		p_bmp280->osrs_p = value;
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to get the Operational Mode from the sensor
 *
 *
 *
 *
 *  \param  *  \param unsigned char mode : Pointer holding the mode value.
 *              0       -> BMP280_SLEEP_MODE
 *              1 and 2 -> BMP280_FORCED_MODE
 *              3       -> BMP280_NORMAL_MODE
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_mode(unsigned char *mode)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_mode_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CTRLMEAS_REG_MODE__REG, &v_mode_u8r, 1);
		*mode = BMP280_GET_BITSLICE(v_mode_u8r, \
			BMP280_CTRLMEAS_REG_MODE);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to set the Operational Mode for the sensor
 *
 *
 *
 *
 *  \param unsigned char mode : Value of the mode
 *              0       -> BMP280_SLEEP_MODE
 *              1 and 2 -> BMP280_FORCED_MODE
 *              3       -> BMP280_NORMAL_MODE
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_mode(unsigned char mode)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_mode_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		if (mode < BMP280_Four_U8X) {
			v_mode_u8r = (p_bmp280->osrs_t << \
				SHIFT_LEFT_5_POSITION) + (p_bmp280->osrs_p << \
				SHIFT_LEFT_2_POSITION) + mode;
			comres += p_bmp280->BMP280_BUS_WRITE_FUNC(\
				p_bmp280->dev_addr,	\
				BMP280_CTRLMEAS_REG_MODE__REG, &v_mode_u8r, 1);
		} else {
			comres = E_BMP280_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Used to reset the sensor
 * The value 0xB6 is written to the 0xE0 register the device is reset using the
 * complete power-on-reset procedure.
 * Softreset can be easily set using bmp280_set_softreset().
 *
 * Usage Hint : bmp280_set_softreset()
 *
 *  \param
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_softreset()
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = BMP280_SOFT_RESET_CODE;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr, \
			BMP280_RESET_REG, &v_data_u8r, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Gets the sensor communication type
 *
 *
 *
 *
 *  \param  unsigned char enable_disable : Pointer holding the
 *                                         spi3 enable or disable state.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_spi3(unsigned char *enable_disable)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_SPI3WEN__REG, &v_data_u8r, 1);
		*enable_disable = BMP280_GET_BITSLICE(v_data_u8r, \
			BMP280_CONFIG_REG_SPI3WEN);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Sets the sensor communication type to 3 wire SPI
 *
 *
 *
 *
 *  \param unsigned char enable_disable : Value of the enable or disable
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_spi3(unsigned char enable_disable)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_SPI3WEN__REG, &v_data_u8r, 1);
		v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
			BMP280_CONFIG_REG_SPI3WEN, enable_disable);
		comres += p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr, \
				BMP280_CONFIG_REG_SPI3WEN__REG, &v_data_u8r, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads filter setting value
 *
 *
 *
 *
 *  \param  *  \param unsigned char *value : Pointer holding the filter value.
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_filter(unsigned char *value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_FILTER__REG, &v_data_u8r, 1);
		*value = BMP280_GET_BITSLICE(v_data_u8r, \
			BMP280_CONFIG_REG_FILTER);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes filter setting to sensor
 *
 *
 *
 *
 *  \param unsigned char value : Value of the filter setting
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_filter(unsigned char value)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_FILTER__REG, &v_data_u8r, 1);
		v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
			BMP280_CONFIG_REG_FILTER, value);
		comres += p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_FILTER__REG, &v_data_u8r, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Reads standby duration time from the sensor
 *
 *  \param * \param unsigned char *time : Pointer holding
 *                        the standby duration time value.
 *              0x00 - BMP280_STANDBYTIME_1_MS
 *              0x01 - BMP280_STANDBYTIME_63_MS
 *              0x02 - BMP280_STANDBYTIME_125_MS
 *              0x03 - BMP280_STANDBYTIME_250_MS
 *              0x04 - BMP280_STANDBYTIME_500_MS
 *              0x05 - BMP280_STANDBYTIME_1000_MS
 *              0x06 - BMP280_STANDBYTIME_2000_MS
 *              0x07 - BMP280_STANDBYTIME_4000_MS
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_standbydur(unsigned char *time)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_TSB__REG, &v_data_u8r, 1);
		*time = BMP280_GET_BITSLICE(v_data_u8r, BMP280_CONFIG_REG_TSB);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes standby duration time from the sensor
 *
 * Normal mode comprises an automated perpetual cycling between an (active)
 * Measurement period and an (inactive) standby period.
 * The standby time is determined by the contents of the register t_sb.
 * Standby time can be set using BME280_STANDBYTIME_125_MS.
 *
 * Usage Hint : bme280_set_standbydur(BME280_STANDBYTIME_125_MS)
 *
 *  \param unsigned char time : Value of the standby duration
 *              0x00 - BMP280_STANDBYTIME_1_MS
 *              0x01 - BMP280_STANDBYTIME_63_MS
 *              0x02 - BMP280_STANDBYTIME_125_MS
 *              0x03 - BMP280_STANDBYTIME_250_MS
 *              0x04 - BMP280_STANDBYTIME_500_MS
 *              0x05 - BMP280_STANDBYTIME_1000_MS
 *              0x06 - BMP280_STANDBYTIME_2000_MS
 *              0x07 - BMP280_STANDBYTIME_4000_MS
 *
 *
 *  \param unsigned char time : Value of the standby duration
 *
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_standbydur(unsigned char time)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_TSB__REG, &v_data_u8r, 1);
		v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
			BMP280_CONFIG_REG_TSB, time);
		comres += p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr, \
			BMP280_CONFIG_REG_TSB__REG, &v_data_u8r, 1);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Writes the working mode to the sensor
 *
 *
 *
 *
 *  \param unsigned char : Mode to be set
 *				0 -> BMP280_ULTRALOWPOWER_MODE
 *				1 -> BMP280_LOWPOWER_MODE
 *				2 -> BMP280_STANDARDRESOLUTION_MODE
 *				3 -> BMP280_HIGHRESOLUTION_MODE
 *				4 -> BMP280_ULTRAHIGHRESOLUTION_MODE
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_set_workmode(unsigned char mode)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		if (mode <= BMP280_Four_U8X) {
			comres += p_bmp280->BMP280_BUS_READ_FUNC(\
				p_bmp280->dev_addr,	BMP280_CTRLMEAS_REG,\
				&v_data_u8r, 1);
			switch (mode) {
			case BMP280_ULTRALOWPOWER_MODE:
				p_bmp280->osrs_t = BMP280_ULTRALOWPOWER_OSRS_T;
				p_bmp280->osrs_p = BMP280_ULTRALOWPOWER_OSRS_P;
				break;
			case BMP280_LOWPOWER_MODE:
				p_bmp280->osrs_t = BMP280_LOWPOWER_OSRS_T;
				p_bmp280->osrs_p = BMP280_LOWPOWER_OSRS_P;
				break;
			case BMP280_STANDARDRESOLUTION_MODE:
				p_bmp280->osrs_t = \
				BMP280_STANDARDRESOLUTION_OSRS_T;
				p_bmp280->osrs_p = \
				BMP280_STANDARDRESOLUTION_OSRS_P;
				break;
			case BMP280_HIGHRESOLUTION_MODE:
				p_bmp280->osrs_t = BMP280_HIGHRESOLUTION_OSRS_T;
				p_bmp280->osrs_p = BMP280_HIGHRESOLUTION_OSRS_P;
				break;
			case BMP280_ULTRAHIGHRESOLUTION_MODE:
				p_bmp280->osrs_t = \
				BMP280_ULTRAHIGHRESOLUTION_OSRS_T;
				p_bmp280->osrs_p = \
				BMP280_ULTRAHIGHRESOLUTION_OSRS_P;
				break;
			}
			v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
				BMP280_CTRLMEAS_REG_OSRST, p_bmp280->osrs_t);
			v_data_u8r = BMP280_SET_BITSLICE(v_data_u8r, \
				BMP280_CTRLMEAS_REG_OSRSP, p_bmp280->osrs_p);
			comres += p_bmp280->BMP280_BUS_WRITE_FUNC(\
				p_bmp280->dev_addr,	BMP280_CTRLMEAS_REG,\
				&v_data_u8r, 1);
		} else {
			comres = E_BMP280_OUT_OF_RANGE;
		}
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief Read both uncompensated pressure and temperature
 *							 in forced mode
 *
 *
 *  \param signed long upressure: Pointer holding the uncompensated pressure.
 *  \param signed long utemperature: Pointer holding
 *                       the uncompensated temperature.
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_get_forced_uput(BMP280_S32_t *upressure,\
		BMP280_S32_t *utemperature)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	unsigned char v_data_u8r = 0;
	unsigned char v_waittime_u8r = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		v_data_u8r = (p_bmp280->osrs_t << SHIFT_LEFT_5_POSITION) + \
			(p_bmp280->osrs_p << SHIFT_LEFT_2_POSITION) + \
			BMP280_FORCED_MODE;
		comres += p_bmp280->BMP280_BUS_WRITE_FUNC(\
				p_bmp280->dev_addr,	BMP280_CTRLMEAS_REG,\
				&v_data_u8r, 1);
		bmp280_compute_wait_time(&v_waittime_u8r);
		p_bmp280->delay_msec(v_waittime_u8r);
		comres += bmp280_read_uput(upressure, utemperature);
	}
	return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API gives data to the given register and
 *                          the data is written in the corresponding register
 *							address
 *
 *
 *
 *  \param unsigned char addr, unsigned char data, unsigned char len
 *          addr -> Address of the register
 *          data -> Data to be written to the register
 *          len  -> Length of the Data
 *
 *
 *
 *  \return communication results.
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_write_register(unsigned char addr,
	    unsigned char *data, unsigned char len)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres = p_bmp280->BMP280_BUS_WRITE_FUNC(p_bmp280->dev_addr,
			addr, data, len);
   }
   return comres;
}
/* Compiler Switch if applicable
#ifdef

#endif
*/
/*******************************************************************************
 * Description: *//**\brief This API reads the data from the given register
 *							address
 *
 *
 *
 *  \param unsigned char addr, unsigned char *data, unsigned char len
 *         addr -> Address of the register
 *         data -> address of the variable, read value will be kept
 *         len  -> Length of the data
 *
 *
 *
 *
 *  \return results of communication routine
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_read_register(unsigned char addr,
		unsigned char *data, unsigned char len)
{
	BMP280_RETURN_FUNCTION_TYPE comres;
	if (p_bmp280 == BMP280_NULL) {
		comres = E_BMP280_NULL_PTR;
	} else {
		comres += p_bmp280->BMP280_BUS_READ_FUNC(p_bmp280->dev_addr,
			addr, data, len);
   }
	return comres;
}
#ifdef BMP280_ENABLE_FLOAT
/*******************************************************************************
 * Description: *//**\brief Reads actual temperature from uncompensated temperature
 *							and returns the value in Degree centigrade
 *                          Output value of "51.23" equals 51.23 DegC.
 *
 *
 *
 *  \param signed long : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			double : actual temperature in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double v_x1_u32r = 0;
	double v_x2_u32r = 0;
	double temperature = 0;

	v_x1_u32r  = (((double)adc_T) / 16384.0 - \
		((double)p_bmp280->cal_param.dig_T1) / 1024.0) * \
		((double)p_bmp280->cal_param.dig_T2);
	v_x2_u32r  = ((((double)adc_T) / 131072.0 - \
		((double)p_bmp280->cal_param.dig_T1) / 8192.0) * \
		(((double)adc_T) / 131072.0 - \
		((double)p_bmp280->cal_param.dig_T1) / 8192.0)) * \
		((double)p_bmp280->cal_param.dig_T3);
	p_bmp280->cal_param.t_fine = (BMP280_S32_t)(v_x1_u32r + v_x2_u32r);
	temperature  = (v_x1_u32r + v_x2_u32r) / 5120.0;


	return temperature;
}
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *							and returns pressure in Pa as double.
 *                          Output value of "96386.2"
 *                          equals 96386.2 Pa = 963.862 hPa.
 *
 *
 *
 *  \param signed int : value of uncompensated pressure
 *
 *
 *
 *  \return
 *			double : actual pressure in floating point
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
double bmp280_compensate_P_double(BMP280_S32_t adc_P)
{
	double v_x1_u32r = 0;
	double v_x2_u32r = 0;
	double pressure = 0;

	v_x1_u32r = ((double)p_bmp280->cal_param.t_fine/2.0) - 64000.0;
	v_x2_u32r = v_x1_u32r * v_x1_u32r * \
		((double)p_bmp280->cal_param.dig_P6) / 32768.0;
	v_x2_u32r = v_x2_u32r + v_x1_u32r * \
		((double)p_bmp280->cal_param.dig_P5) * 2.0;
	v_x2_u32r = (v_x2_u32r / 4.0) + \
		(((double)p_bmp280->cal_param.dig_P4) * 65536.0);
	v_x1_u32r = (((double)p_bmp280->cal_param.dig_P3) * \
		v_x1_u32r * v_x1_u32r / 524288.0 + \
		((double)p_bmp280->cal_param.dig_P2) * v_x1_u32r) / 524288.0;
	v_x1_u32r = (1.0 + v_x1_u32r / 32768.0) * \
		((double)p_bmp280->cal_param.dig_P1);
	if (v_x1_u32r == 0.0)
		return 0; /* Avoid exception caused by division by zero */
	pressure = 1048576.0 - (double)adc_P;
	pressure = (pressure - (v_x2_u32r / 4096.0)) * 6250.0 / v_x1_u32r;
	v_x1_u32r = ((double)p_bmp280->cal_param.dig_P9) * \
		pressure * pressure / 2147483648.0;
	v_x2_u32r = pressure * ((double)p_bmp280->cal_param.dig_P8) / 32768.0;
	pressure = pressure + (v_x1_u32r + v_x2_u32r + \
		((double)p_bmp280->cal_param.dig_P7)) / 16.0;

	return pressure;
}
#endif
#if defined(BMP280_ENABLE_INT64) && defined(BMP280_64BITSUPPORT_PRESENT)
/*******************************************************************************
 * Description: *//**\brief Reads actual pressure from uncompensated pressure
 *                          and returns the value in Pa as unsigned 32 bit
 *                          integer in Q24.8 format (24 integer bits and
 *                          8 fractional bits). Output value of "24674867"
 *                          represents 24674867 / 256 = 96386.2 Pa = 963.862 hPa
 *
 *
 *
 *  \param signed long : value of uncompensated temperature
 *
 *
 *
 *  \return
 *			unsigned long : actual pressure
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ******************************************************************************/
BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t v_x1_s64r = 0;
	BMP280_S64_t v_x2_s64r = 0;
	BMP280_S64_t pressure = 0;
	v_x1_s64r = ((BMP280_S64_t)p_bmp280->cal_param.t_fine) - 128000;
	v_x2_s64r = v_x1_s64r * v_x1_s64r * \
		(BMP280_S64_t)p_bmp280->cal_param.dig_P6;
	v_x2_s64r = v_x2_s64r + ((v_x1_s64r * \
		(BMP280_S64_t)p_bmp280->cal_param.dig_P5) << 17);
	v_x2_s64r = v_x2_s64r + \
		(((BMP280_S64_t)p_bmp280->cal_param.dig_P4) << 35);
	v_x1_s64r = ((v_x1_s64r * v_x1_s64r * \
		(BMP280_S64_t)p_bmp280->cal_param.dig_P3) >> 8) + \
		((v_x1_s64r * (BMP280_S64_t)p_bmp280->cal_param.dig_P2) << 12);
	v_x1_s64r = (((((BMP280_S64_t)1) << 47) + v_x1_s64r)) * \
		((BMP280_S64_t)p_bmp280->cal_param.dig_P1) >> 33;
	if (v_x1_s64r == 0)
		return 0; /* Avoid exception caused by division by zero */
	pressure = 1048576 - adc_P;
	pressure = (((pressure << 31) - v_x2_s64r) * 3125) / v_x1_s64r;
	v_x1_s64r = (((BMP280_S64_t)p_bmp280->cal_param.dig_P9) * \
		(pressure >> 13) * (pressure >> 13)) >> 25;
	v_x2_s64r = (((BMP280_S64_t)p_bmp280->cal_param.dig_P8) * \
		pressure) >> 19;
	pressure = ((pressure + v_x1_s64r + v_x2_s64r) >> 8) + \
		(((BMP280_S64_t)p_bmp280->cal_param.dig_P7) << 4);
	return (BMP280_U32_t)pressure;
}
#endif
/*******************************************************************************
 * Description: *//**\brief Computing waiting time for sensor data read
 *
 *
 *
 *
 *  \param
 *			unsigned char : value of time
 *
 *
 *  \return
 *
 *
 ******************************************************************************/
/* Scheduling:
 *
 *
 *
 * Usage guide:
 *
 *
 * Remarks:
 *
 ****************************************************************************/
BMP280_RETURN_FUNCTION_TYPE bmp280_compute_wait_time(unsigned char \
	*v_delaytime_u8r)
{
	BMP280_RETURN_FUNCTION_TYPE comres = 0;

	*v_delaytime_u8r = (T_INIT_MAX + T_MEASURE_PER_OSRS_MAX * \
		(((1 << p_bmp280->osrs_t) >> 1) + ((1 << p_bmp280->osrs_p) \
		>> 1)) + (p_bmp280->osrs_p ? T_SETUP_PRESSURE_MAX : 0) + 15) \
		/ 16;
	return comres;
}
