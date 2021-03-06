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
 * app_func.c
 * - Audio application functions
 * ------------------------------------------------------------------------- */

#include "app.h"
#include "app_audio.h"
#include "queue.h"
#include <ble_asha.h>
#include <output_driver.h>
#include <app_audio_playback_sync.h>

/// forward declaration to avoid compiler warnings
extern struct ASHA_Env_t asha_env;
extern void DMA3_IRQHandler(void);

/// Global variables

int8_t volumeShift = 0;
uint8_t seqNum_prev=0;
uint8_t seqNum_prev_stream=0;

sync_param_t env_sync;
audio_frame_param_t env_audio;

struct queue_t audio_queue;

/* Enable / disable PLC feature */
bool plc_enable = true;

static uint16_t m_samples_to_correct = 0;


audio_frame_param_t * APP_Audio_GetInstance(void)
{
	return &env_audio;
}

uint8_t APP_GetSeqNum(void)
{
	return seqNum_prev_stream;
}

void APP_CorrectAudioStream(uint16_t samples)
{
	m_samples_to_correct = samples;
	DMA->CTRL0[OD_IN_IDX] |= (1U << DMA_CTRL0_COMPLETE_INT_ENABLE_Pos);
	NVIC_SetPriority(DMA0_IRQn,0);
	NVIC_EnableIRQ(DMA0_IRQn);

}

void APP_Audio_Transfer(uint8_t *audio_buff, uint16_t audio_length, uint8_t seqNum)
{
    uint8_t i;

    // sync tracking
    if (seqNum == 1)
    {
    	//Sys_GPIO_Set_High(GPIO_DBG_PACK_STREAM);
        //Sys_GPIO_Set_Low(GPIO_DBG_PACK_STREAM);
    }
    if( ((uint8_t) (seqNum - seqNum_prev - 1)) != 0 ) {
    //    PRINTF("\r\n APP_Audio_Transfer: seqNum_prev = %d, seqNum = %d\r\n", seqNum_prev, seqNum);
    }

    seqNum_prev = seqNum;


    // Transient if this is the first packet received after a start
    if (env_audio.state == LINK_TRANSIENT)
    {
        /* Add credits to avoid starvation */
        APP_Audio_FlushQueue();
    	ASHA_AddCredits(1);
    }


    // insert the audio samples to the queue
    uint8_t num_inserted = 0;
    for (i = 0; i < audio_length; i += ENCODED_FRAME_LENGTH)
    {
        QueueInsert(&audio_queue, &audio_buff[i], GOOD_PACKET,seqNum);
        num_inserted++;
    }

    // TODO: For a 10 ms connection interval, a credit has to be sent immediatelly.
    // Find out why this is needed!

    env_sync.cntr_connection = 0;

    if (env_audio.state == LINK_TRANSIENT)
    {
		NVIC_DisableIRQ(RF_RXSTOP_IRQn);
		RF_REG_WRITE(IRQ_CONF, 0x0);
       	//Sys_GPIO_Set_High(GPIO_DBG_PACK_STREAM);
    	APP_Audio_Start();
    	seqNum_prev_stream = seqNum-2;
    	PRINTF("Save seqNumPrev:%d!\n",seqNum_prev_stream);
    }
}

void APP_Audio_Start(void)
{
    Sys_GPIO_Set_Low(GPIO_DBG_ASRC_ISR);

	PRINTF("Established\r\n");

    env_audio.binaural_active = false;

    /// Enable output driver interface
    od_enable();

    env_sync.cntr_transient = 0;
    env_sync.audio_sink_cnt = 0;
    env_sync.flag_ascc_phase = false;

    ASRC_CTRL->ASRC_ENABLE_ALIAS = ASRC_ENABLED_BITBAND;
    Sys_ASRC_Reset();
    Sys_Audiosink_ResetCounters();

    AUDIOSINK_CTRL->PERIOD_CNT_START_ALIAS = 1;

    // ASCC interrupts
    NVIC_ClearPendingIRQ(AUDIOSINK_PHASE_IRQn);
    NVIC_ClearPendingIRQ(AUDIOSINK_PERIOD_IRQn);
    NVIC_EnableIRQ(AUDIOSINK_PHASE_IRQn);
    NVIC_EnableIRQ(AUDIOSINK_PERIOD_IRQn);

    // LPDSP32 interrupt
    NVIC_EnableIRQ(DSP0_IRQn);

    // Timer interrupts
    NVIC_EnableIRQ(TIMER_IRQn(TIMER_RENDER));
    NVIC_EnableIRQ(TIMER_IRQn(TIMER_START_STREAM));

    /// DMA DSP->ASRC config
    Sys_DMA_ChannelConfig(
       ASRC_IN_IDX,
       RX_DMA_ASRC_IN,
       FRAME_LENGTH,
       0,
       (uint32_t)lpdsp32.outgoing,
       (uint32_t)&ASRC->IN
       );

    // DMA ASRC->Memory config
    Sys_DMA_ChannelConfig(
	   ASRC_OUT_IDX,
	   RX_DMA_ASRC_OUT_MEM,
	   640,
	   0,
	   (uint32_t)&ASRC->OUT,
	   (uint32_t)od_out_buffer);

	// DMA Memory->OD config
    Sys_DMA_ChannelConfig(
	   OD_IN_IDX,
	   RX_DMA_IN_OD,
	   640,
	   0,
	   (uint32_t) od_out_buffer,
	   (uint32_t) &AUDIO->OD_DATA);

    Sys_DMA_ClearChannelStatus(ASRC_OUT_IDX);
    Sys_DMA_ClearChannelStatus(OD_IN_IDX);

    // dma channel for output driver is not enabled yet
    Sys_DMA_ChannelDisable(ASRC_OUT_IDX);
    Sys_DMA_ChannelDisable(OD_IN_IDX);

    /* Enable ASRC block */
    Sys_ASRC_StatusConfig(ASRC_ENABLE);


	env_audio.audio_state = AUDIO_START;
    env_audio.state = LINK_ESTABLISHED;
    env_audio.onset_cnt = 0;

}

void APP_Audio_Disconnect(void)
{

	APP_ResetPrevSeqNumber();

    Reset_Audio_Sync_Param(&env_sync, &env_audio);


    memset(od_out_buffer,0,OD_OUT_BUFFER_SIZE* sizeof(int16_t));

    PRINTF("\r\nDisconnect\r\n");

    env_sync.cntr_connection = 0;

    ASRC_CTRL->ASRC_DISABLE_ALIAS = ASRC_DISABLED_BITBAND;

    // Disable pcm interface
    od_disable();

    NVIC_DisableIRQ(DMA0_IRQn);
    NVIC_DisableIRQ(DMA3_IRQn);

    Sys_DMA_ChannelDisable(OD_IN_IDX);
    Sys_DMA_ChannelDisable(ASRC_OUT_IDX);

    /* ASCC interrupt */
    NVIC_DisableIRQ(AUDIOSINK_PHASE_IRQn);
    NVIC_DisableIRQ(AUDIOSINK_PERIOD_IRQn);

    /* LPDSP32 interrupt */
    NVIC_DisableIRQ(DSP0_IRQn);


    /* Timer interrupts */
    NVIC_DisableIRQ(TIMER_IRQn(TIMER_RENDER));

    Sys_Timers_Stop(1U << TIMER_RENDER);

    APP_Audio_FlushQueue();

    Sys_ASRC_StatusConfig(ASRC_DISABLE);

    env_audio.audio_state = AUDIO_IDLE;
    env_audio.render_state = FIRST_FRAME;
    env_audio.state = LINK_DISCONNECTED;
}

void APP_Audio_FlushQueue(void)
{
    uint16_t queue_len;
    uint16_t i;

    queue_len = QueueCount(&audio_queue);
    for (i = 0; i < queue_len; ++i)
        QueueFree(&audio_queue);
}

/* ----------------------------------------------------------------------------
 * Function      : void Volume_Set(int8_t volume)
 * ----------------------------------------------------------------------------
 * Description   : Normalizes the volume range from [-128, 0] to [16, 0]. The
 * 				   new scale is used in Volume_Set() to adjust by shifting the
 * 				   samples to the right
 * Inputs        : - volume	- Volume [-128 to 0 range]
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void Volume_Set(int8_t volume)
{
    volumeShift = 16 - ((volume + 128) / 8);
}

/* ----------------------------------------------------------------------------
 * Function      : void Volume_Shift_Subframe (int16_t *src)
 * ----------------------------------------------------------------------------
 * Description   : Adjust the audio level by a simple shift.
 * Inputs        : - src	- pointer to the audio frame
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void Volume_Shift_Subframe(int16_t *src)
{
    /* Shift subframe samples to change the volume */
	for (uint16_t i = 0; i < SUBFRAME_LENGTH; i++)
    {
    	src[i] = src[i]>>volumeShift;
    }

}

/* ----------------------------------------------------------------------------
 * Function      : void APP_ResetPrevSeqNumber(void)
 * ----------------------------------------------------------------------------
 * Description   : Initialize ASHA sequence number control after a disconnect.
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void APP_ResetPrevSeqNumber(void)
{
    seqNum_prev = 0xFF;
}


static void render_timer_for_10ms()
{
    // break here when the buffer is not yet filled completly
    if (env_audio.audio_state != AUDIO_PLAY)
    {
    	asrc_reconfig(&env_sync);
    	return;
    }

    	// get the audio samples from the queue
    	uint8_t seq_num = 0xFF;

    	env_audio.frame_in = QueueFront(&audio_queue, &env_audio.packet_state,&seq_num);
    	if (env_audio.frame_in != NULL)
    	{

    		 // check if the sequence number is out of sync => enable PLC
    		 if ((uint8_t)(seq_num - seqNum_prev_stream-1)!=0)
    		 {

    			// TODO: think of what should happen when we have old packets in queue
    			if (seqNum_prev_stream >= seq_num)
    			{
    				PRINTF("Out of sequence! seqNum = %d. Should be: %d\r\n", seq_num,seqNum_prev_stream+1);
    				QueueFree(&audio_queue);
    				QueueFree(&audio_queue);
    			}
    			else
    			{
    				env_audio.packet_state = BAD_PACKET;
    				PRINTF("Packet missing! seqNum = %d.\r\n", seqNum_prev_stream+1);
    			}
    		 }
    		 else
    		 {

    			 memcpy(lpdsp32.incoming, env_audio.frame_in, ENCODED_FRAME_LENGTH * sizeof(uint8_t));
    			 QueueFree(&audio_queue);

    			 // render tracking
    			 if (seq_num == 1)
    			 {
    				 //Sys_GPIO_Set_High(GPIO_DBG_PACK_STREAM);
    			 }
    		 }
    	}

    	else
    	{
    		env_audio.packet_state = BAD_PACKET;
    		PRINTF("Audio Buffer underflow. Enable PLC!\r\n");
    	}

    	seqNum_prev_stream++;

}

static void render_timer_for_20ms()
{

    if (env_audio.render_state == FIRST_FRAME)
    {
    	env_sync.sink_cnt_render = ((Sys_Audiosink_Counter() % 322) * 1000000 / 16129) + (Sys_Audiosink_PeriodCounter() % env_sync.audio_sink_period_cnt / 16) + env_sync.audio_sink_phase_cnt / 16;  ;
    	if ((QueueCount(&audio_queue) %2) != 0)
    	{
    		QueueFree(&audio_queue);
    	}

		seqNum_prev_stream++;
    }

	// get the audio samples from the queue
	uint8_t seq_num = 0xFF;

	env_audio.frame_in = QueueFront(&audio_queue, &env_audio.packet_state,&seq_num);
	if (env_audio.frame_in != NULL)
	{

		 // check if the sequence number is out of sync => enable PLC
		 if ((uint8_t)(seq_num - seqNum_prev_stream-1)!=0)
		 {

			// TODO: think of what should happen when we have old packets in queue
			if (seqNum_prev_stream >= seq_num)
			{
				//PRINTF("Out of sequence! seqNum = %d. Should be: %d\r\n", seq_num,seqNum_prev_stream+1);
				QueueFree(&audio_queue);
				QueueFree(&audio_queue);
				QueueFree(&audio_queue);
			}
			else
			{
				env_audio.packet_state = BAD_PACKET;
				//PRINTF("Packet missing! seqNum = %d.\r\n", seqNum_prev_stream+1);
			}
		 }
		 else
		 {

			 memcpy(lpdsp32.incoming, env_audio.frame_in, ENCODED_FRAME_LENGTH * sizeof(uint8_t));
			 QueueFree(&audio_queue);

			 // render tracking
			 if (seq_num == 1)
			 {
				 //Sys_GPIO_Set_High(GPIO_DBG_PACK_STREAM);
			 }

		 }
	}

	else
	{
		env_audio.packet_state = BAD_PACKET;
		//PRINTF("Audio Buffer underflow. Enable PLC!\r\n");
	}

	if (env_audio.render_state == SECOND_FRAME)
	{
		ASHA_AddCredits(1);
	}

	env_audio.render_state = (env_audio.render_state + 1) % 2;
}




/* ----------------------------------------------------------------------------
 * Function      : void rendering_timer_isr(void)
 * ----------------------------------------------------------------------------
 * Description   : rendering timer interrupt handler
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void TIMER_IRQ_FUNC(TIMER_RENDER)(void)
{


    /* for accuracy start the rendering timer in free-run mode once */

	switch (env_audio.audio_state)
	{

	case AUDIO_START:

		 if (!env_sync.timer_free_run)
		{
			Sys_Timers_Stop(1U << TIMER_RENDER);
			Sys_Timer_Set_Control(TIMER_RENDER, TIMER_FREE_RUN | (20000 - 1) |
			TIMER_SLOWCLK_DIV2);
			Sys_Timers_Start(1U << TIMER_RENDER);
		}

		asrc_reconfig(&env_sync);

		if (env_audio.onset_cnt++ == 3)
		{
			env_audio.audio_state = AUDIO_ENABLE;
			Sys_DMA_ChannelEnable(ASRC_OUT_IDX);
			env_audio.onset_cnt = 0;
		}

		ASHA_AddCredits(1);

		return;


	case AUDIO_ENABLE:

		Sys_DMA_ChannelEnable(OD_IN_IDX);
		env_audio.audio_state = AUDIO_PLAY;
		Sys_GPIO_Set_Low(GPIO_DBG_ASRC_ISR);
		if (!env_sync.timer_free_run)
		{
			Sys_Timers_Stop(1U << TIMER_RENDER);
			Sys_Timer_Set_Control(TIMER_RENDER, TIMER_FREE_RUN | (20000 - 1) |
			TIMER_SLOWCLK_DIV2);
			Sys_Timers_Start(1U << TIMER_RENDER);
		}
		ASHA_AddCredits(1);
		return;

	case AUDIO_PLAY:
	{

	    if (!env_sync.timer_free_run)
	    {
	        Sys_Timers_Stop(1U << TIMER_RENDER);
	        Sys_Timer_Set_Control(TIMER_RENDER, TIMER_FREE_RUN | (10000 - 1) |
	        TIMER_SLOWCLK_DIV2);
	        Sys_Timers_Start(1U << TIMER_RENDER);
	    }


	    if (asha_env.con_int == 8)
	    {
	    	render_timer_for_10ms();
	    }
	    else
	    {
	    	render_timer_for_20ms();
	    }

	    env_sync.timer_free_run = true;

		/* Check if the decoder has finished on the frame */
		ASSERT(lpdsp32.state == DSP_IDLE);
		lpdsp32.state = DSP_BUSY;

		if (plc_enable)
		{
			if (env_audio.packet_state == GOOD_PACKET)
			{
				lpdsp32.channels.action &= ~DECODE_PLC;

			}
			else if (env_audio.packet_state == BAD_PACKET)
			{
				lpdsp32.channels.action |= DECODE_PLC;
			}
			else
			{
				lpdsp32.channels.action |= DECODE_PLC;
			}
		}
		else
		{
			/* No PLC performed */
			lpdsp32.channels.action &= ~DECODE_PLC;
		}

		codecSetParameters(lpdsp32.codec, &lpdsp32.channels);
		codecDecode(lpdsp32.codec);

		lpdsp32.channels.action &= ~DECODE_RESET;

		env_sync.cntr_connection++;
		//Sys_GPIO_Set_Low(GPIO_DBG_PACK_STREAM);
	}
	break;

	default:
		break;

	}

}


void TIMER_IRQ_FUNC(TIMER_START_STREAM)(void)
{

}


/* ----------------------------------------------------------------------------
 * Function      : void DSP0_IRQHandler(void)
 * ----------------------------------------------------------------------------
 * Description   : LPDSP32 decoder interrupt handler (RX)
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
//#define DEBUG_BUFFER
void DSP0_IRQHandler(void)
{
    /* Check if there is any sub-frame from prev frame waiting to be processed */
    ASSERT(!env_audio.proc); //TODO: investigate why this is being triggered.
    env_audio.proc = true;

   /// Buffer level information. Set the define flag to observe buffer levels on the UART
   #ifdef DEBUG_BUFFER


       uint32_t buffer_in  = DMA->WORD_CNT[ASRC_OUT_IDX];
       uint32_t buffer_out = DMA->WORD_CNT[OD_IN_IDX];
       uint32_t asrc = ASRC->OUTPUT_CNT;
       uint32_t tmp = 0;

             if (buffer_in < buffer_out)
             {
                 tmp =buffer_in + 2 * 160;
             }
             uint32_t buffer_level = tmp - buffer_out;

             PRINTF("buf_asrc: %d buffer_out: %d level:%d asrc:%d\n",buffer_in,buffer_out,buffer_level,asrc);
#endif

   env_audio.frame_dec = (int16_t *) lpdsp32.outgoing;

    if (env_sync.flag_ascc_phase)
    {
        asrc_reconfig(&env_sync);
        env_sync.flag_ascc_phase = false;
    }

    Volume_Shift_Subframe(env_audio.frame_dec);

    Sys_ASRC_StatusConfig(ASRC_ENABLE);
    Sys_DMA_ChannelEnable(ASRC_IN_IDX);
    NVIC_EnableIRQ(DMA3_IRQn);

    env_audio.frame_idx = 0;

    lpdsp32.state = DSP_IDLE;
}

/// DMA3 IRQHandler (DSP->ASRC)
void DMA3_IRQHandler()
{
	while (ASRC_CTRL->ASRC_PROC_STATUS_ALIAS != ASRC_IDLE_BITBAND);
	Sys_ASRC_StatusConfig(ASRC_DISABLE);
	Sys_DMA_ChannelDisable(ASRC_IN_IDX);
}


/// DMA4 IRQHandler
void DMA0_IRQHandler()
{
#define RX_DMA_IN_OD_CORR        ( DMA_LITTLE_ENDIAN           \
                                | DMA_PRIORITY_1              \
                                | DMA_TRANSFER_M_TO_P         \
                                | DMA_ADDR_CIRC               \
                                | DMA_COMPLETE_INT_ENABLE    \
                                | DMA_SRC_ADDR_INC            \
                                | DMA_SRC_WORD_SIZE_32        \
                                | DMA_DEST_OD                 \
                                | DMA_DEST_ADDR_STATIC        \
                                | DMA_DEST_WORD_SIZE_16       \
                                | DMA_ENABLE                 )

	static uint8_t cnt = 0;

	if (cnt++ < 5)
	{
		return;
	}


	if (m_samples_to_correct >0)
	{
		Sys_DMA_ClearChannelStatus(OD_IN_IDX);

	    // DMA ASRC->Memory config
	    Sys_DMA_ChannelConfig(
	       OD_IN_IDX,
		   RX_DMA_IN_OD_CORR,
		   m_samples_to_correct,
		   0,
		   (uint32_t)(od_out_buffer+640-m_samples_to_correct),
		   (uint32_t)&AUDIO->OD_DATA);



	    m_samples_to_correct = 0;
	}
	else
	{
		Sys_DMA_ClearChannelStatus(OD_IN_IDX);

		// DMA Memory->OD config
	    Sys_DMA_ChannelConfig(
		   OD_IN_IDX,
		   RX_DMA_IN_OD,
		   640,
		   0,
		   (uint32_t) od_out_buffer,
		   (uint32_t) &AUDIO->OD_DATA);


		Sys_DMA_ChannelEnable(OD_IN_IDX);
		NVIC_DisableIRQ(DMA0_IRQn);

		cnt = 0;
	}
}

/* ----------------------------------------------------------------------------
 * Function      : void AUDIOSINK_PHASE_IRQHandler(void)
 * ----------------------------------------------------------------------------
 * Description   : ASCC phase interrupt handler (TX/RX)
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void AUDIOSINK_PHASE_IRQHandler(void)
{

	Sys_GPIO_Toggle(GPIO_DBG_ASRC_ISR);

#if SIMUL
    static uint32_t sim_missed = 0;
    sim_missed = (sim_missed + 1) % 100;

    if (sim_missed < 1)
#else    /* if SIMUL */
    if (AUDIOSINK_CTRL->PHASE_CNT_MISSED_STATUS_ALIAS)
#endif    /* if SIMUL */
    {
        env_sync.phase_cnt_missed = true;
    }
    else
    {

        /* Start the rendered timer */
        Sys_Timers_Stop(1U << TIMER_RENDER);
        Sys_Timer_Set_Control(TIMER_RENDER,
                TIMER_SHOT_MODE | (RENDER_TIME_US -1 +3500/*TODO: android_support_env.channel_delay*/) |
                TIMER_SLOWCLK_DIV2);
        Sys_Timers_Start(1U << TIMER_RENDER);
        env_sync.timer_free_run = false;

        /* Get audio sink phase count */
        env_sync.audio_sink_phase_cnt = Sys_Audiosink_PhaseCounter();
        if (!env_sync.phase_cnt_missed)
        {
            /* Get audio sink count */
            env_sync.audio_sink_cnt = Sys_Audiosink_Counter();
            env_sync.audio_sink_cnt = env_sync.audio_sink_cnt << SHIFT_BIT;
            env_sync.audio_sink_cnt += ((((env_sync.audio_sink_phase_cnt_prev - env_sync.audio_sink_phase_cnt))
                    << SHIFT_BIT) / env_sync.audio_sink_period_cnt);
        }

        /* store audio sink count phase for the next time */
        env_sync.audio_sink_phase_cnt_prev = env_sync.audio_sink_phase_cnt;
        env_sync.phase_cnt_missed = false;
    }

    env_sync.flag_ascc_phase = true;

    AUDIOSINK_CTRL->CNT_RESET_ALIAS = CNT_RESET_BITBAND;
    AUDIOSINK->PHASE_CNT = 0;
    AUDIOSINK_CTRL->PHASE_CNT_START_ALIAS = PHASE_CNT_START_BITBAND;
}

/* ----------------------------------------------------------------------------
 * Function      : void AUDIOSINK_PERIOD_IRQHandler(void)
 * ----------------------------------------------------------------------------
 * Description   : ASCC period interrupt handler (TX/RX)
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void AUDIOSINK_PERIOD_IRQHandler(void)
{
    env_sync.audio_sink_period_cnt = Sys_Audiosink_PeriodCounter() / (AUDIOSINK->CFG + 1);
    AUDIOSINK->PERIOD_CNT = 0;
    AUDIOSINK_CTRL->PERIOD_CNT_START_ALIAS = PERIOD_CNT_START_BITBAND;
}

/* ----------------------------------------------------------------------------
 * Function      : void asrc_reconfig(void)
 * ----------------------------------------------------------------------------
 * Description   : Configure ASRC (TX/RX)
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void asrc_reconfig(sync_param_t *sync_param)
{
    static int64_t asrc_inc_carrier;

   // Sys_ASRC_ResetOutputCount();

	sync_param->Cr = (2*asha_env.con_int *10) << SHIFT_BIT;
	sync_param->Ck = sync_param->audio_sink_cnt;

	// calculate phase increment depending on the connection interval
	if ((sync_param->Ck <= ((2*asha_env.con_int *10) - ASRC_CFG_THR_MIN) << SHIFT_BIT)
			|| (sync_param->Ck >= ((2*asha_env.con_int *10) + ASRC_CFG_THR_MAX) << SHIFT_BIT))
	{
		sync_param->Ck = sync_param->Ck_prev;
		sync_param->avg_ck_outputcnt = 0;
	}

	/* Store Ck to apply on the next packet if the audio sink value is out of
	 * range
	 */
	sync_param->Ck_prev = sync_param->Ck;

	/* Configure ASRC base on new Ck */
	asrc_inc_carrier = (((sync_param->Cr - sync_param->Ck) << 29) / sync_param->Ck);
	asrc_inc_carrier &= 0xFFFFFFFF;

	if (asrc_inc_carrier != ASRC->PHASE_INC)
	{
		Sys_ASRC_Config(asrc_inc_carrier, LOW_DELAY | ASRC_DEC_MODE1);
	}
}

/* ----------------------------------------------------------------------------
 * Function      : void ASRC_ERROR_IRQHandler(void)
 * ----------------------------------------------------------------------------
 * Description   : ASRC error interrupt handler (TX/RX)
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void ASRC_ERROR_IRQHandler(void)
{
    ASSERT(ASRC_CTRL->ASRC_IN_ERR_ALIAS == 0); ASSERT(ASRC_CTRL->ASRC_UPDATE_ERR_ALIAS == 0);

    ASRC_CTRL->ASRC_UPDATE_ERR_CLR_ALIAS = CLR_ASRC_UPDATE_ERR_BITBAND;
    ASRC_CTRL->ASRC_IN_ERR_CLR_ALIAS = CLR_ASRC_IN_ERR_BITBAND;
}

/* ----------------------------------------------------------------------------
 * Function      : void Reset_Audio_Sync_Param(sync_param_t *sync_param,
 *                                          audio_frame_param_t *audio_param)
 * ----------------------------------------------------------------------------
 * Description   : Reset the audio and sync instances
 * Inputs        : - sync_param  - pointer to the sync structure
 *                 - audio_param - pointer to the audio structure
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void Reset_Audio_Sync_Param(sync_param_t *sync_param, audio_frame_param_t *audio_param)
{
	if (asha_env.con_int == 8)
	{
		 sync_param->Ck_prev = (160) << SHIFT_BIT;
	}
	else
	{
		sync_param->Ck_prev = (320) << SHIFT_BIT;
	}
    sync_param->Cr = 0;
    sync_param->Ck = 0;
    sync_param->audio_sink_cnt = 0;
    sync_param->avg_ck_outputcnt = 0;
    sync_param->audio_sink_period_cnt = 0;
    sync_param->audio_sink_phase_cnt = 0;
    sync_param->audio_sink_phase_cnt_prev = 0;
    sync_param->asrc_cnt_cnst = 0;
    sync_param->phase_cnt_missed = false;
    sync_param->flag_ascc_phase = false;
    sync_param->cntr_connection = 0;
    sync_param->cntr_transient = 0;
    sync_param->timer_free_run = false;

    audio_param->proc = false;
    audio_param->frame_idx = 0;
    audio_param->state = LINK_DISCONNECTED;
    audio_param->packet_state = BAD_PACKET;

    Sys_Timers_Stop(1U << TIMER_START_STREAM);
    Sys_Timer_Set_Control(TIMER_START_STREAM, TIMER_SHOT_MODE | (200 - 1) |
    TIMER_SLOWCLK_DIV2);
}
