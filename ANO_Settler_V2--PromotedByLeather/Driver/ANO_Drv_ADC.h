#ifndef __ANO_DRV_ADC_H
#define	__ANO_DRV_ADC_H

#include "include.h"

void ADC1_Init(void);
void NVIC_Configuration(void);
void MYDMA_Enable(void);
extern __IO uint16_t ADC_ConvertedValue;

#endif /* __ADC_H */


