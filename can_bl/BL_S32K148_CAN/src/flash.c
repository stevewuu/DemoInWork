/*
 * flash.c
 *
 *  Created on: 2017Äê7ÔÂ20ÈÕ
 *      Author: 4337
 */
#include "S32K148_features.h"
#include "flash.h"
#include "flash_driver.h"


/* Function declarations */
void FTFC_IRQHandler(void);
/* If target is flash, insert this macro to locate callback function into RAM */
START_FUNCTION_DECLARATION_RAMSECTION
void CCIF_Callback(void)
END_FUNCTION_DECLARATION_RAMSECTION

status_t ret;
flash_callback_t pCallBack;

flash_ssd_config_t flashSSDConfig;
const flash_user_config_t Flash_InitConfig0 = 
{
    .PFlashBase  = 0x00000000U,
    .PFlashSize  = 0x00080000U,
    .DFlashBase  = 0x10000000U,
    .EERAMBase   = 0x14000000U,
    .CallBack    = NULL_CALLBACK
};

/**
 *  \brief flash_init
 *  
 *  \return None
 *  
 *  \details flash init
 */
void flash_init()
{
	 
// disable cache for flash writer to register
//	MSCM->OCMDR[0u] |= MSCM_OCMDR_OCM1(0xFu);
//	MSCM->OCMDR[1u] |= MSCM_OCMDR_OCM1(0xFu);
//	MSCM->OCMDR[2u] |= MSCM_OCMDR_OCM1(0xFu);

	// global variable,local can not work???
	ret = FLASH_DRV_Init(&Flash_InitConfig0, &flashSSDConfig);
	if (ret != STATUS_SUCCESS)
	{
		while(1);
	}
}
/**
 *  \brief flash_erase_sector
 *  
 *  \param [in] addr erase sector address
 *  \return None
 *  
 *  \details erase a sector,if failed stuck it
 */
void flash_erase_sector(uint32_t addr)
{
	ret = FLASH_DRV_EraseSector(&flashSSDConfig, addr, FEATURE_FLS_PF_BLOCK_SECTOR_SIZE);
	if (ret != STATUS_SUCCESS)
	{
		while(1);
	}
}
/**
 *  \brief flash_program_data
 *  
 *  \param [in] addr address to program data
 *  \param [in] pdata buffer pointer to Programed data
 *  \param [in] byte_size size of data
 *  \return None
 *  
 *  \details program data to flash, if failed will stuck it 
 */
void flash_program_data(uint32_t addr, uint8_t *pdata, uint32_t byte_size)
{
    /* Disable Callback */
	flashSSDConfig.CallBack = NULL_CALLBACK;
	//ret = FlashProgram(&flashSSDConfig, addr, byte_size, pdata, pCmdSequence);
	ret = FLASH_DRV_Program(&flashSSDConfig, addr, byte_size, pdata);
	if (ret != STATUS_SUCCESS)
	{
		while(1);
	}
}
/**
 *  \brief FTFC_IRQHandler
 *  
 *  \return None
 *  
 *  \details Flash interrupt handler
 */
void FTFC_IRQHandler(void)
{
    /* Disable Flash Command Complete interrupt */
    FTFx_FCNFG &= (~FTFx_FCNFG_CCIE_MASK);

    return;
}

/*!
  \brief Callback function for Flash operations
*/
START_FUNCTION_DEFINITION_RAMSECTION
void CCIF_Callback(void)
{
    /* Enable interrupt for Flash Command Complete */
    if ((FTFx_FCNFG & FTFx_FCNFG_CCIE_MASK) == 0u)
    {
        FTFx_FCNFG |= FTFx_FCNFG_CCIE_MASK;
    }
}
END_FUNCTION_DEFINITION_RAMSECTION
