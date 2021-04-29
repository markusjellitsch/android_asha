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


/// forward declaration to avoid compiler warnings
extern struct ASHA_Env_t asha_env;
extern void DMA3_IRQHandler(void);
extern uint8_t asha_side;
extern struct gapm_start_connection_cmd startConnectionCmd;
extern struct gapm_start_advertise_cmd advertiseCmd;

/// Global variables
uint32_t primEvtCnt = 0, secEvtCnt = 0, invalidRxCnt = 0;
uint8_t sentValue = 0;
uint32_t cntr = 0;
uint32_t cntr_coded = 0;
int8_t volumeShift = 0;
uint8_t seqNum_prev;
uint8_t seqNum_prev_stream;

sync_param_t env_sync;
audio_frame_param_t env_audio;

struct queue_t audio_queue;

/* Enable / disable PLC feature */
bool plc_enable = true;

static bool m_start_rendering = 0;

static uint8_t m_sync_buffer[5] = {0};

static uint32_t m_local_sync = 0;


static uint16_t m_samples_to_correct = 0;

extern void BLE_COEX_RX_TX_IRQHandler(void);

typedef enum
{
	FIRST_FRAME,
	SECOND_FRAME
}state_t;
static state_t state = FIRST_FRAME;


void BLE_COEX_RX_TX_IRQHandler(void)
{
	static uint8_t state = 0;
	volatile uint32_t bbif_status =  *((uint32_t *)0x40001404);
	volatile uint32_t link_label = (bbif_status & BBIF_STATUS_LINK_LABEL_Mask) >> BBIF_STATUS_LINK_LABEL_Pos;
	volatile uint32_t link_format = (bbif_status & BBIF_STATUS_LINK_FORMAT_Mask) >> BBIF_STATUS_LINK_FORMAT_Pos;

	if (link_label == 1)
	{
		if (state == 0)
		{
			state = 1;
		}
		else
		{
			Sys_GPIO_Set_High(GPIO_DBG_PACK_RECV);
			APP_CalculateSync();
			Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);
			NVIC_DisableIRQ(BLE_COEX_RX_TX_IRQn);
			state = 0;
		}
	}
}


bool APP_Start_Binaural_Sync(void)
{
	if (!asha_env.binaural || !m_start_rendering)
	{
		return false;
	}

    Sys_Timers_Stop(1U << TIMER_SIMUL);
    Sys_Timer_Set_Control(TIMER_SIMUL, TIMER_SHOT_MODE | (208000 - 1) |
    TIMER_SLOWCLK_DIV2);
    Sys_Timers_Start(1U << TIMER_SIMUL);
    NVIC_EnableIRQ(TIMER_IRQn(TIMER_SIMUL));

	asha_env.binaural = false;
	return true;
}

void APP_CalculateSync(void)
{
	__disable_irq();
	// we have to timestamp
	uint8_t seqNum100 = 0;
	if (seqNum_prev_stream > 100)
	{
		seqNum100 = 255-seqNum_prev_stream + 100 -1;
	}
	else
	{
		seqNum100 = 100 - seqNum_prev_stream - 1;
	}
	if (state == SECOND_FRAME)
	{
		seqNum100 -= 1;
	}

	uint32_t sink_cnt = Sys_Audiosink_Counter() % 322;
	uint32_t phase_cnt = Sys_Audiosink_PhaseCounter() / 16;

	int tmp = 0;
	if (sink_cnt < 133)
	{
		tmp = 322 - 133 + sink_cnt;
	}
	else tmp = sink_cnt - 133;


	uint32_t time_to_next_packet =  20000 - (tmp * 1000000 /16129) - phase_cnt;
	uint32_t time_to_packet100 = time_to_next_packet + (seqNum100 * 20000);

	if (asha_side == ASHA_CAPABILITIES_SIDE_LEFT)
	{
		time_to_packet100 += 370;
	}

	m_local_sync =  time_to_packet100;



	m_sync_buffer[0] =(uint8_t)(time_to_packet100 & 0xFF);
	time_to_packet100 = time_to_packet100 >> 8;
	m_sync_buffer[1] =(uint8_t)(time_to_packet100 & 0xFF);
	time_to_packet100 = time_to_packet100 >> 8;
	m_sync_buffer[2] =(uint8_t)(time_to_packet100 & 0xFF);
	time_to_packet100 = time_to_packet100 >> 8;
	m_sync_buffer[3] =(uint8_t)(time_to_packet100 & 0xFF);
	m_sync_buffer[4] = seqNum_prev_stream;
	__enable_irq();
}


void APP_StartSyncCapture()
{
    NVIC_EnableIRQ(BLE_COEX_RX_TX_IRQn);
    NVIC_SetPriority(BLE_COEX_RX_TX_IRQn,0);
    while(BBIF_COEX_STATUS->BLE_IN_PROCESS_ALIAS || BBIF_COEX_STATUS->BLE_RX_ALIAS || BBIF_COEX_STATUS->BLE_TX_ALIAS);
	*((uint32_t *)BBIF_COEX_INT_CFG_BASE) = (BLE_TX_EVENT_TRANSITION);
}

void APP_AdaptRenderDelay(uint32_t remote_sync,uint8_t seqNum)
{

	PRINTF("Local Sync %d seqNum: %d\n",m_local_sync,seqNum);

	if (m_local_sync > remote_sync)
	{
		PRINTF("The other Peripheral will adjust Render delay!\n");
		return;
	}

	uint32_t time_dif = remote_sync - m_local_sync;
	if (time_dif > 20000)
	{
		PRINTF("Delay > 1 conn_int!\n");
		time_dif -= 20000;
	}

    NVIC_EnableIRQ(TIMER_IRQn(TIMER_SIMUL));


    m_samples_to_correct = (uint16_t)((double)(time_dif * 16129 / 1000000)+0.5);
	PRINTF("Time Dif: %d us. Samples: %d SeqNum: %d\n",time_dif,seqNum,m_samples_to_correct);

	DMA->CTRL0[OD_IN_IDX] |= (1U << DMA_CTRL0_COMPLETE_INT_ENABLE_Pos);
	NVIC_SetPriority(DMA0_IRQn,0);
	NVIC_EnableIRQ(DMA0_IRQn);
}

uint8_t *APP_GetSyncInfo(void)
{
	return m_sync_buffer;
}


/* ----------------------------------------------------------------------------
 * Function      : void APP_Audio_Transfer(const uint8_t *audio_buff,
 *                                    uint8_t audio_length, uint8_t seqNum)
 * ----------------------------------------------------------------------------
 * Description   : Handles a received Android Audio frame
 * Inputs        : audio_buff    - pointer of the received frame buffer
 *                 audio_length  -
 *                 seqNum        -
 * Outputs       : None
 * Assumptions   : None
 * -------------------------------------------------------------------
 * ------ */
void APP_Audio_Transfer(uint8_t *audio_buff, uint16_t audio_length, uint8_t seqNum)
{
    uint8_t i;

    // sync tracking
    if (seqNum == 1)
    {
    	//Sys_GPIO_Set_High(GPIO_DBG_PACK_RECV);
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
    	APP_Audio_Start();
    	seqNum_prev_stream = seqNum-1;
    	PRINTF("Save seqNumPrev:%d!\n",seqNum_prev_stream);

    }


   // Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);

	APP_Start_Binaural_Sync();
}

void APP_Audio_Start(void)
{
    PRINTF("Established\r\n");

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
    NVIC_EnableIRQ(AUDIOSINK_PHASE_IRQn);
    NVIC_EnableIRQ(AUDIOSINK_PERIOD_IRQn);

    // LPDSP32 interrupt
    NVIC_EnableIRQ(DSP0_IRQn);

    NVIC_DisableIRQ(BLE_COEX_RX_TX_IRQn);

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

	Sys_DMA_ChannelEnable(OD_IN_IDX);

    Sys_Timers_Stop(1U << TIMER_START_STREAM);
	if (asha_env.con_int == 8)
	{
		 Sys_Timer_Set_Control(TIMER_START_STREAM, TIMER_FREE_RUN | (10000*(ASHA_L2CC_INITIAL_CREDITS-2) - 1) |   TIMER_SLOWCLK_DIV2);
	}
	else
	{
		 Sys_Timer_Set_Control(TIMER_START_STREAM, TIMER_FREE_RUN | (20000*(ASHA_L2CC_INITIAL_CREDITS-3) - 1) |   TIMER_SLOWCLK_DIV2);
	}

	Sys_Timers_Start(1U << TIMER_START_STREAM);

    /* Enable ASRC block */
    Sys_ASRC_StatusConfig(ASRC_ENABLE);

    env_audio.state = LINK_ESTABLISHED;
}

void APP_Audio_Disconnect(void)
{

	APP_ResetPrevSeqNumber();

    Reset_Audio_Sync_Param(&env_sync, &env_audio);

    m_start_rendering = 0;

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
    if (!m_start_rendering)
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
    				 Sys_GPIO_Set_High(GPIO_DBG_PACK_STREAM);
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


	// break here when the buffer is not yet filled completly
    if (!m_start_rendering)
    {
    	asrc_reconfig(&env_sync);
    	state = FIRST_FRAME;
    	//ASHA_AddCredits(1);
    	return;
    }

    if (state == FIRST_FRAME)
    {
    	if ((QueueCount(&audio_queue) %2) != 0)
    	{
    		QueueFree(&audio_queue);
    	}
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
				 Sys_GPIO_Set_High(GPIO_DBG_PACK_STREAM);
			 }

		 }
	}

	else
	{
		env_audio.packet_state = BAD_PACKET;
		//PRINTF("Audio Buffer underflow. Enable PLC!\r\n");
	}

	if (state == SECOND_FRAME)
	{
		seqNum_prev_stream++;
		ASHA_AddCredits(1);
	}

	state = (state + 1) % 2;
}


void TIMER_IRQ_FUNC(TIMER_SIMUL)(void)
{

	if (asha_side == ASHA_CAPABILITIES_SIDE_RIGHT)
	{
		 PRINTF("Start SYNC mode (SCAN)"); // TODO
		 GAPM_StartConnectionCmd(&startConnectionCmd);
	}
	else
	{
		PRINTF("Start SYNC mode (ADV)");
		GAPM_StartAdvertiseCmd(&advertiseCmd);
	}
    NVIC_DisableIRQ(TIMER_IRQn(TIMER_SIMUL));

    Sys_Timers_Stop(1U << TIMER_SIMUL);
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

	 Sys_GPIO_Toggle(GPIO_DBG_ASRC_ISR);


    /* for accuracy start the rendering timer in free-run mode once */
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

    if (!m_start_rendering)
    {
    	return;
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
    Sys_GPIO_Set_Low(GPIO_DBG_PACK_STREAM);
}


void TIMER_IRQ_FUNC(TIMER_START_STREAM)(void)
{
	Sys_Timers_Stop(1U << TIMER_START_STREAM);
	m_start_rendering = true;
	NVIC_DisableIRQ(TIMER_IRQn(TIMER_START_STREAM));

	Sys_DMA_ChannelEnable(ASRC_OUT_IDX);
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
void DMA4_IRQHandler()
{
#define RX_DMA_ASRC_OUT_MEM_CORR     (DMA_SRC_ASRC                  | \
                                 	 DMA_TRANSFER_P_TO_M            | \
									 DMA_LITTLE_ENDIAN              | \
									 DMA_PRIORITY_0                 | \
									 DMA_COMPLETE_INT_ENABLE       	| \
									 DMA_COUNTER_INT_DISABLE        | \
									 DMA_DEST_WORD_SIZE_32          | \
									 DMA_SRC_WORD_SIZE_16           | \
									 DMA_SRC_ADDR_STATIC            | \
									 DMA_DEST_ADDR_INC	            | \
									 DMA_ADDR_CIRC                   | \
									 DMA_ENABLE)

	static uint8_t cnt = 0;

	if (cnt++ < 5)
	{
		return;
	}


	if (m_samples_to_correct >0)
	{
		Sys_DMA_ClearChannelStatus(ASRC_OUT_IDX);

		volatile uint32_t word_cnt = DMA->WORD_CNT[ASRC_OUT_IDX];

	    // DMA ASRC->Memory config
	    Sys_DMA_ChannelConfig(
		   ASRC_OUT_IDX,
		   RX_DMA_ASRC_OUT_MEM_CORR,
		   640-m_samples_to_correct,
		   0,
		   (uint32_t)&ASRC->OUT,
		   (uint32_t)(od_out_buffer+m_samples_to_correct));



	    m_samples_to_correct = 0;
	}
	else
	{
		Sys_DMA_ClearChannelStatus(ASRC_OUT_IDX);

	    // DMA ASRC->Memory config
	    Sys_DMA_ChannelConfig(
		   ASRC_OUT_IDX,
		   RX_DMA_ASRC_OUT_MEM,
		   640,
		   0,
		   (uint32_t)&ASRC->OUT,
		   (uint32_t)od_out_buffer);


		Sys_DMA_ChannelEnable(ASRC_OUT_IDX);
		NVIC_DisableIRQ(DMA4_IRQn);

		cnt = 0;
	}
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
                TIMER_SHOT_MODE | (RENDER_TIME_US -1 +3750/*TODO: android_support_env.channel_delay*/) |
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
    int64_t asrc_inc_carrier;

    Sys_ASRC_ResetOutputCount();

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
    Sys_ASRC_Config(asrc_inc_carrier, LOW_DELAY | ASRC_INT_MODE);
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
