#ifndef __MS5611_H
#define __MS5611_H

void ms5611_start_ut(void);
void ms5611_get_ut(void);
void ms5611_start_up(void);
void ms5611_get_up(void);
void ms5611_calculate(void);

void ms5611_init(void);
extern int32_t pressure,temperature;
#endif