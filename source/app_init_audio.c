/* ----------------------------------------------------------------------------
 * Copyright (c) 2017 Semiconductor Components Industries, LLC (d/b/a
 * ON Semiconductor), All Rights Reserved
 *
 * This code is the property of ON Semiconductor and may not be redistributed
 * in any form without prior written permission from ON Semiconductor.
 * The terms of use and warranty for this code are covered by contractual
 * agreements between ON Semiconductor and the licensee.
 *
 * This is Reusable Code.
 *
 * ----------------------------------------------------------------------------
 * app_init.c
 * - Audio application initialization function
 * ------------------------------------------------------------------------- */

#include <app_audio.h>
#include <output_driver.h>
#include <rsl10.h>
#include "queue.h"

extern struct queue_t audio_queue;

/// global memory buffer where asrc samples are stored
int16_t od_out_buffer[OD_OUT_BUFFER_SIZE];
int16_t asrc_sync_register[4] = {0};


/* Define the LPDSP32 Context */
LPDSP32Context lpdsp32;


/* ----------------------------------------------------------------------------
 * Function      : void App_CodecInitialize(void)
 * ----------------------------------------------------------------------------
 * Description   : Initialize the codec subsystem
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
static void App_CodecInitialize(void)
{
    /* Initialise the dsp context */
    lpdsp32.state    = DSP_STARTING;
    lpdsp32.channels.action  = PACKED | DECODE_RESET | DECODE;

    /* Set the framesize and block length */
    lpdsp32.channels.frameSize       = FRAME_LENGTH;
    lpdsp32.channels.blockSize       = FRAME_LENGTH;

    /* Set the mode and channel, at the moment the channel is encoded as the
     * top four bits of the field
     */
    lpdsp32.channels.modeAndChannel  = 0x00 | CODEC_MODE;

    /* Sample rate is not needed in G722 */
    lpdsp32.channels.sampleRate      = 0;

    /* Create a codec located in the configuration area */
    lpdsp32.codec = populateG722PLCDSPCodec(Buffer.configuration,
                                            CODEC_CONFIGURATION_SIZE);
    if (!codecIsCodec(lpdsp32.codec))
    {
        return;
    }

    /* initialise the codec */
    codecInitialise(lpdsp32.codec);

    /* Configure the static parts of the codec */
    codecSetStatusBuffer(lpdsp32.codec, Buffer.configuration,
                         CODEC_CONFIGURATION_SIZE);
    codecSetOutputBuffer(lpdsp32.codec, Buffer.output, CODEC_OUTPUT_SIZE);

    /* In the current implementation, the management of the data into the codec
     * is handled by a queing mechanism, as such only one input buffer is
     * required, this can be hard coded here.
     */
    codecSetInputBuffer(lpdsp32.codec, Buffer.input, CODEC_INPUT_SIZE);

    /* we have a handshake protocol to ensure the DSP is alive and in synch
     * before we try to use it.
     */
    dspHandshake(lpdsp32.codec);

    lpdsp32.outgoing = Buffer.output;
    lpdsp32.incoming = Buffer.input;

    lpdsp32.state = DSP_IDLE;
    lpdsp32.channels.action = DECODE_RESET | DECODE;
    codecSetParameters(lpdsp32.codec, &lpdsp32.channels);
}


/* ----------------------------------------------------------------------------
 * Function      : void Audio_Initialize_System(void)
 * ----------------------------------------------------------------------------
 * Description   : Initialize audio functions (TX/RX)
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void Audio_Initialize_System(void)
{

    /// initialize codec
    App_CodecInitialize();

    /// initialize output driver interface

    /// DMA DSP->ASRC config
    Sys_DMA_ChannelConfig(
       ASRC_IN_IDX,
       RX_DMA_ASRC_IN,
       FRAME_LENGTH,
       0,
       (uint32_t)lpdsp32.outgoing,
       (uint32_t)&ASRC->IN
       );

    /// DMA ASRC->Memory config
    Sys_DMA_ChannelConfig(
        ASRC_OUT_IDX,
        RX_DMA_ASRC_OUT_MEM,
		OD_OUT_BUFFER_SIZE,
        0,
        (uint32_t)&ASRC->OUT,
        (uint32_t)od_out_buffer);

     /// DMA Memory->OD config
    Sys_DMA_ChannelConfig(
        OD_IN_IDX,
        RX_DMA_IN_OD,
        OD_OUT_BUFFER_SIZE,
        OD_OUT_BUFFER_SIZE/2,
        (uint32_t) od_out_buffer,
        (uint32_t) &AUDIO->OD_DATA);


    /// ASRC configuration
    Sys_ASRC_IntEnableConfig(INT_EBL_ASRC_IN_ERR | INT_EBL_ASRC_UPDATE_ERR);
    ASRC_CTRL->ASRC_DISABLE_ALIAS = ASRC_DISABLED_BITBAND;

    /// audio sink period cnt config
    AUDIOSINK_CTRL->PERIOD_CNT_START_ALIAS = 1;

    /// audiosink clock counter configuration
    Sys_Audiosink_ResetCounters();
    Sys_Audiosink_InputClock(0,AUDIOSINK_CLK_SRC_DMIC_OD);
    Sys_Audiosink_Config(AUDIO_SINK_PERIODS_16, 0, 0);
    Sys_BBIF_SyncConfig(SYNC_ENABLE | SYNC_SOURCE_BLE_RX, 0, SLAVE_CONNECT);

    /// reset audio sink parameters
    Reset_Audio_Sync_Param(&env_sync, &env_audio);

    // debug pin configuration
    Sys_DIO_Config(GPIO_DBG_PACK_RECV, DIO_MODE_GPIO_OUT_0);
    Sys_DIO_Config(GPIO_DBG_ASRC_ISR, DIO_MODE_GPIO_OUT_0);
    Sys_DIO_Config(GPIO_DBG_PACK_STREAM, DIO_MODE_GPIO_OUT_0);
    Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);
    Sys_GPIO_Set_Low(GPIO_DBG_ASRC_ISR);
    Sys_GPIO_Set_Low(GPIO_DBG_PACK_STREAM);

    //  set IRQ priorities
    NVIC_SetPriority(DSP0_IRQn, 2);
    NVIC_SetPriority(TIMER_IRQn(TIMER_RENDER), 1);

    QueueInit(&audio_queue);

    APP_ResetPrevSeqNumber();
}
