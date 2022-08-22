#include "sai_controller.h"

#include "error_handler.h"
#include "main.h"
#include "sys_headers.h"

static SAI_InitTypeDef hsai_init = {
    .AudioMode = SAI_MODEMASTER_TX,
    .Synchro = SAI_ASYNCHRONOUS,
    .OutputDrive = SAI_OUTPUTDRIVE_DISABLE,
    .NoDivider = SAI_MASTERDIVIDER_ENABLE,
    .FIFOThreshold = SAI_FIFOTHRESHOLD_1QF,
    .AudioFrequency = SAI_AUDIO_FREQUENCY_96K,
    .SynchroExt = SAI_SYNCEXT_DISABLE,
    .MonoStereoMode = SAI_STEREOMODE,
    .CompandingMode = SAI_NOCOMPANDING,
    .TriState = SAI_OUTPUT_NOTRELEASED,
};

static SAI_HandleTypeDef hsai_BlockA2;
static DMA_HandleTypeDef hdma_sai2_a;

/**
 * @brief SAI2 Initialization Function
 * @param None
 * @retval None
 */
void MX_SAI2_Init(void) {
    /* USER CODE BEGIN SAI2_Init 0 */
    hsai_BlockA2.Instance = SAI2_Block_A;
    hsai_BlockA2.Init = hsai_init;
    /* USER CODE END SAI2_Init 0 */

    /* USER CODE BEGIN SAI2_Init 1 */

    /* USER CODE END SAI2_Init 1 */

    if (HAL_SAI_InitProtocol(&hsai_BlockA2,
                             SAI_I2S_STANDARD,
                             SAI_PROTOCOL_DATASIZE_32BIT,
                             2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SAI2_Init 2 */

    /* USER CODE END SAI2_Init 2 */
}
