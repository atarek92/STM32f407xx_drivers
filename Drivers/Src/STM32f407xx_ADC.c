/*
 * STM32f407xx_ADC.c
 *
 *  Created on: Apr 14, 2022
 *      Author: atarek
 */
#include "STM32f407xx_ADC.h"

/*********************************************************************
 * @fn      		  - ADC_PeriClockControl
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void ADC_PeriClockControl(ADC_RegDef_t *pADCx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pADCx == ADC1)
		{
			ADC1_PCLK_EN();
		}else if (pADCx == ADC2)
		{
			ADC2_PCLK_EN();
		}else if (pADCx == ADC3)
		{
			ADC3_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}


/*********************************************************************
 * @fn      		  - ADC_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void ADC_Init(ADC_Handle_t *pADCHandle)
{

	/* Enable the clock access to the ADC module*/
	ADC_PeriClockControl(pADCHandle->pADCx, ENABLE);

	/* Set Sequencer length */
	pADCHandle->pADCx->ADC_SQR1 = 0;

	/* Set ADC channel*/
	pADCHandle->pADCx->ADC_SQR3 |= pADCHandle->ADC_Config.ADC_Channel << ADC_SQR_COV_1;

	/* Set conversion t continues */
	pADCHandle->pADCx->ADC_CR2 |= pADCHandle->ADC_Config.ADC_ConverterMode << ADC_CR2_CONT;

	/* Enable ADC module*/
	pADCHandle->pADCx->ADC_CR2 |= 1 << ADC_CR2_ADON;

	/* Start conversion */
	pADCHandle->pADCx->ADC_CR2 |= 1 << ADC_CR2_SWSTART;

}

/*********************************************************************
 * @fn      		  - ADC_ReadData
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

uint32_t ADC_ReadData(ADC_Handle_t *pADCHandle)
{
	uint8_t end_of_conv_flag = pADCHandle->pADCx->ADC_SR & (1 << ADC_SR_EOC);
	while(! end_of_conv_flag)
	{
		end_of_conv_flag = pADCHandle->pADCx->ADC_SR & (1 << ADC_SR_EOC);
	}
	return (pADCHandle->pADCx->ADC_DR);
}
