
#include <rsl10.h>
#include <app_audio_playback_sync.h>
#include <ble_asha.h>
#include <ble_gap.h>
#include <ble_l2c.h>
#include <app_trace.h>



extern void BLE_COEX_RX_TX_IRQHandler(void);
extern struct gapm_start_connection_cmd startConnectionCmd;
extern struct gapm_start_advertise_cmd advertiseCmd;
extern struct ASHA_Env_t asha_env;
extern uint8_t asha_side;

static asha_sync_info_list_t m_local_sync_info = {0};
static uint8_t m_sync_buffer_idx = 0;

static uint32_t m_local_sync = 0;

static uint16_t m_samples_to_correct = 0;
static audio_frame_param_t * m_audio_env = 0;

bool sync_enabled = true;


static void app_aps_msg_handler(ke_msg_id_t const msg_id, void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{

	uint8_t conidx = KE_IDX_GET(src_id);


	switch (msg_id)
	{

		case GAPC_CONNECTION_REQ_IND:
		{
			const struct gapc_connection_req_ind* p = param;

            if (conidx == 0) asha_env.con_int = p->con_interval;
            else if (conidx ==1)
            {
            	app_aps_start_tx_rx_sync_capture();

            	PRINTF("Binaural Link Connected\n");
				if (asha_side <= ASHA_CAPABILITIES_SIDE_RIGHT)
				{
					PRINTF("Start l2CAP Link\n");
					struct l2cc_lecb_connect_cmd cmd;

					cmd.le_psm = ASHA_LE_PSM_SYNC;
					cmd.pkt_id = 0;
					cmd.local_mtu = 100;
					cmd.local_mps = 100;
					cmd.local_cid = 0;
					cmd.local_credit = 0xFFFF;
					cmd.operation = L2CC_LECB_CONNECT;

					L2CC_LecbConnectCmd(conidx, &cmd);
				}
            }
		}
		break;


		case L2CC_CMP_EVT:
		{
			struct l2cc_cmp_evt * evt = param;

			if (evt->operation == L2CC_LECB_CONNECT)
			{
				if (evt->status == 0)
				{
					L2CC_LecbSduSendCmd(1,asha_env.peer_cid_sync,sizeof(asha_sync_info_list_t),app_aps_get_sync_info());
				}
			}
		}
		break;


		case L2CC_LECB_CONNECT_REQ_IND:
		{
			const struct l2cc_lecb_connect_req_ind* p = param;

			if (p->le_psm == ASHA_LE_PSM_SYNC)
			{
				struct l2cc_lecb_connect_req_ind const *ind = (struct l2cc_lecb_connect_req_ind const *)param;

				struct l2cc_lecb_connect_cfm cfm = {
					.le_psm = ind->le_psm,
					.peer_cid = ind->peer_cid,
					.local_mps = ind->peer_mps,
					.local_mtu = ind->peer_mtu,
					.accept = true,
					.local_cid = 0,
					.local_credit = 8,
				};

				L2CC_LecbConnectCfm(conidx, &cfm);
			}

		}
		break;


		case L2CC_LECB_SDU_RECV_IND:
		{

			const struct l2cc_lecb_sdu_recv_ind* p = param;
			struct asha_audio_received *rcv_p;

			if(p->sdu.cid == asha_env.local_cid)
			{

				if (m_audio_env->state == LINK_TRANSIENT)
				{

				}

				app_aps_start_binaural_link();
			}
			else
			{
				asha_sync_info_list_t t_play100_list = *((asha_sync_info_list_t *)(p->sdu.data));

				if (app_aps_calculate_left_right_offset(t_play100_list,3))
				{
					GAPC_DisconnectCmd(conidx,CO_ERROR_REMOTE_USER_TERM_CON);
					m_audio_env->binaural_active = false;
				}
			}
		}
		break;

	}

}

///@brief Initialize Audio Playback Synchronization
bool app_aps_init(void)
{
	MsgHandler_Add(TASK_ID_GAPM,app_aps_msg_handler);
	MsgHandler_Add(TASK_ID_L2CC, app_aps_msg_handler);
	MsgHandler_Add(TASK_ID_GAPC, app_aps_msg_handler);

	m_audio_env = APP_Audio_GetInstance();

	return true;
}


///@brief Start the  APS binaural link
bool app_aps_start_binaural_link(void)
{
	if (!asha_env.binaural || (m_audio_env->audio_state != AUDIO_PLAY))
	{
		return false;
	}
	// binaural link is delayed
    Sys_Timers_Stop(1U << TIMER_START_APS);
    Sys_Timer_Set_Control(TIMER_START_APS, TIMER_SHOT_MODE | (208000 - 1) |
    TIMER_SLOWCLK_DIV2);
    Sys_Timers_Start(1U << TIMER_START_APS);
    NVIC_EnableIRQ(TIMER_IRQn(TIMER_START_APS));

    asha_env.binaural = false;
	m_audio_env->binaural_active = true;

	return true;
}
#define USE_COMPLEX_ASCC

#ifdef USE_COMPLEX_ASCC

///@brief Compute sync info
bool app_aps_compute_sync_info(void)
{
	#define CONN_INT_MS					20.0
	#define OD_FREQ_HZ					16129.0
	#define AUDIO_SINK_CLOCK_CNT 		(CONN_INT_MS * OD_FREQ_HZ / 1000.0)
	#define T_PLAY_US					7000
	#define PERIPHERAL_OFFSET			54


	__disable_irq();

	uint16_t dma_cnt = DMA->WORD_CNT[OD_IN_IDX] & 0xFFFF;

	// current ASCC info
	uint32_t sink_cnt = Sys_Audiosink_Counter() % (uint32_t)(AUDIO_SINK_CLOCK_CNT);
	uint32_t phase_cnt = Sys_Audiosink_PhaseCounter() / 16;
	uint32_t period_cnt = Sys_Audiosink_PeriodCounter() % env_sync.audio_sink_period_cnt;

	// current sequence number
	uint8_t seq_num_dif = 0;
	uint8_t seq_num_current = APP_GetSeqNum();

	// sequence number dif to 100
	if (seq_num_current > 100)
	{
		seq_num_dif = 0xFF-seq_num_current + 100 -1;
	}
	else
	{
		seq_num_dif = 100 - seq_num_current - 1;
	}

//	if (m_audio_env->render_state == SECOND_FRAME)
//	{
//		seq_num_dif -= 1;
//	}

	// determine t_play next
	int tmp = 0;
	uint32_t t_play_next = 0;

	tmp = (int)((double)sink_cnt * 1000000.0 /  OD_FREQ_HZ);
	tmp += phase_cnt;
	tmp += period_cnt/16;

	// wrap around check
	if (tmp > T_PLAY_US)
	{
		t_play_next = CONN_INT_MS *1000 - tmp + T_PLAY_US;
	}
	else t_play_next = T_PLAY_US - tmp;


	// determine t_play_100
	uint32_t t_play100 = t_play_next + (seq_num_dif * CONN_INT_MS *1000);

	// apply the measured peripheral offset
	if (asha_side == ASHA_CAPABILITIES_SIDE_LEFT)
	{
		t_play100 += PERIPHERAL_OFFSET;
	}

	m_local_sync =  t_play100;

	asha_sync_info_t info;
	info.t_play100 = t_play100;
	info.od_cnt = dma_cnt ;
	m_local_sync_info.list[m_sync_buffer_idx++] = info;

	__enable_irq();


	return true;
}
#else

///@brief Compute sync info
bool app_aps_compute_sync_info(void)
{
	__disable_irq();

	uint16_t dma_cnt = DMA->WORD_CNT[OD_IN_IDX] & 0xFFFF;
	uint32_t sink_cnt = Sys_Audiosink_Counter() % 322;
	uint32_t phase_cnt =    env_sync.audio_sink_phase_cnt  / 16;
	uint32_t period_cnt = Sys_Audiosink_PeriodCounter() % env_sync.audio_sink_period_cnt;

	// we have to timestamp
	uint8_t seqNum100 = 0;
	uint8_t seqNum_prev_stream = APP_GetSeqNum();

	if (seqNum_prev_stream > 100)
	{
		seqNum100 = 0xFF-seqNum_prev_stream + 100 -1;
	}
	else
	{
		seqNum100 = 100 - seqNum_prev_stream - 1;
	}
	if (m_audio_env->render_state == SECOND_FRAME)
	{
		//seqNum100 -= 1;
	}


	uint32_t time_to_next_packet = 0;
	int tmp = (sink_cnt * 1000000 / 16129) + phase_cnt + period_cnt / 16;
	if (tmp <  env_sync.sink_cnt_render)
	{
		time_to_next_packet =  (322 * 1000000 / 16129) - env_sync.sink_cnt_render + tmp;
	}
	else time_to_next_packet = tmp - env_sync.sink_cnt_render;

	time_to_next_packet = (322 * 1000000 / 16129) - time_to_next_packet;

	uint32_t time_to_packet100 = time_to_next_packet + (seqNum100 * 20000);

	if (asha_side == ASHA_CAPABILITIES_SIDE_LEFT)
	{
		time_to_packet100 += 52;
		phase_cnt += 52;
	}

	m_local_sync =  time_to_packet100;

	asha_sync_info_t info;
	info.t_play100 =time_to_packet100;
	info.audio_sink_cnt = sink_cnt;
	info.audio_period_cnt = period_cnt / 16;
	info.audio_phase_cnt = phase_cnt;
	info.seq_num = seqNum_prev_stream;
	info.od_cnt = dma_cnt ;
	m_local_sync_info.list[m_sync_buffer_idx++] = info;

	__enable_irq();


	return true;
}


#endif



#ifdef USE_COMPLEX_ASCC
bool app_aps_calculate_left_right_offset(asha_sync_info_list_t remote_sync_info,uint8_t num_entries)
{

	PRINTF("Received Sync!\n");

	uint32_t time_dif_list[3] = {0};
	uint8_t num_behind = 0;
	uint32_t time_dif = 0xFFFFFF;
	uint8_t time_dif_index = 0;

	for (uint8_t i = 0; i < num_entries;i++)
	{
		uint32_t t_play_local = m_local_sync_info.list[i].t_play100;
		uint32_t t_play_remote =remote_sync_info.list[i].t_play100;
		int32_t t_lro =(int)(t_play_remote) - (int)(t_play_local);

		if (t_play_local > t_play_remote)
		{
			num_behind++;
			time_dif_list[i]= 0xFFFFFFFF;
		}
		else
		{
			time_dif_list[i]= t_play_remote - t_play_local;
			if (time_dif_list[i] < time_dif)
			{
				time_dif = time_dif_list[i];
				time_dif_index = i;
			}
		}

		PRINTF("Num: %d, local:%d, remote: %d time_dif: %d\n",i,t_play_local,t_play_remote,t_lro);
	}

	if (num_behind > 1)
	{
		PRINTF("The other peripheral is the leading device and will correct left-right-offset!\n");
		return false;
	}

	if (time_dif > 20000)
	{
		PRINTF("Delay > 1 conn_int!\n");
		time_dif -= 20000;
	}

	m_samples_to_correct = (uint16_t)((double)(time_dif * 16129 / 1000000)+0.5);

	uint16_t dma_cnt_local = m_local_sync_info.list[time_dif_index].od_cnt;
	uint16_t dma_cnt_remote = remote_sync_info.list[time_dif_index].od_cnt;
	uint16_t dma_cnt_dif = 0;

	if (dma_cnt_local > dma_cnt_remote)
	{
		dma_cnt_dif = dma_cnt_local - dma_cnt_remote;
	}
	else dma_cnt_dif = 640 - dma_cnt_remote +dma_cnt_local;

	dma_cnt_dif %= 322;

	PRINTF("Time Dif: %d us. Samples: %d DmaDif:%d\n",time_dif,m_samples_to_correct,dma_cnt_dif);

	APP_CorrectAudioStream(m_samples_to_correct);

	return true;

}
#else
bool app_aps_calculate_left_right_offset(asha_sync_info_list_t remote_sync_info,uint8_t num_entries)
{

	PRINTF("Received Sync!\n");

	uint32_t time_dif_list[3] = {0};
	uint8_t num_behind = 0;
	uint32_t time_dif = 0xFFFFFF;
	uint8_t time_dif_index = 0;

	for (uint8_t i = 0; i < num_entries;i++)
	{
		uint8_t local_seq_num = m_local_sync_info.list[i].seq_num;
		uint16_t local_audio_sink_cnt =  m_local_sync_info.list[i].audio_sink_cnt;
		uint16_t local_audio_period_cnt =  m_local_sync_info.list[i].audio_period_cnt;
		uint16_t local_audio_phase_cnt =  m_local_sync_info.list[i].audio_phase_cnt;

		uint8_t remote_seq_num = remote_sync_info.list[i].seq_num;
		uint16_t remote_audio_sink_cnt =  remote_sync_info.list[i].audio_sink_cnt;
		uint16_t remote_audio_period_cnt =  remote_sync_info.list[i].audio_period_cnt;
		uint16_t remote_audio_phase_cnt =  remote_sync_info.list[i].audio_phase_cnt;


		uint32_t t_sync_local = (local_audio_sink_cnt * 1000000 / 16129) + local_audio_period_cnt + local_audio_phase_cnt;
		uint32_t t_sync_remote = (remote_audio_sink_cnt * 1000000 / 16129) + remote_audio_period_cnt + remote_audio_phase_cnt;

		uint32_t t_play_render = (133 * 1000000 / 16129);
		uint32_t t_play_local = (322 * 1000000/16129) - t_sync_local;
		uint32_t t_play_remote = (322 * 1000000/16129) - t_sync_remote;


		if (t_sync_local > t_play_render)
		{
			t_play_local = (322 * 1000000/16129) - t_sync_local + t_play_render;
		}

		if (t_sync_remote > t_play_render)
		{
			t_play_remote = (322 * 1000000/16129) - t_sync_remote + t_play_render;
		}


		int32_t t_lro =(int)(t_sync_local) - (int)(t_sync_remote);

		if (t_play_local > t_play_remote)
		{
			if (local_seq_num <= remote_seq_num)
			{
				num_behind++;
				time_dif_list[i]= 0xFFFFFFFF;
			}
			else
			{
				time_dif_list[i] = (322 * 1000000/16129)- t_play_local + t_play_remote;
			}
		}
		else
		{
			if (local_seq_num <= remote_seq_num)
			{
				num_behind++;
				time_dif_list[i]= 0xFFFFFFFF;
			}
			else
			{
				time_dif_list[i] =  t_play_local - t_play_remote;
			}
		}

		if (time_dif_list[i] < time_dif)
		{
			time_dif = time_dif_list[i];
			time_dif_index = i;
		}

		PRINTF("Num: %d, local: %d in %d, remote: %d in %d time_dif: %d\n",i,local_seq_num,t_play_local,remote_seq_num,t_play_remote,t_lro);
	}

	if (num_behind > 1)
	{
		PRINTF("The other peripheral is the leading device and will correct left-right-offset!\n");
		return false;
	}

	if (time_dif > 20000)
	{
		PRINTF("Delay > 1 conn_int!\n");
		time_dif -= 20000;
	}

	m_samples_to_correct = (uint16_t)((double)(time_dif * 16129 / 1000000)+0.5);

	uint16_t dma_cnt_local = m_local_sync_info.list[time_dif_index].od_cnt;
	uint16_t dma_cnt_remote = remote_sync_info.list[time_dif_index].od_cnt;
	uint16_t dma_cnt_dif = 0;

	if (dma_cnt_local > dma_cnt_remote)
	{
		dma_cnt_dif = dma_cnt_local - dma_cnt_remote;
	}
	else dma_cnt_dif = 640 - dma_cnt_remote +dma_cnt_local;

	dma_cnt_dif %= 322;

	PRINTF("Time Dif: %d us. Samples: %d DmaDif:%d\n",time_dif,m_samples_to_correct,dma_cnt_dif);

	APP_CorrectAudioStream(m_samples_to_correct);

	return true;

}
#endif

#ifdef USE_BLE_COEX_SYNC

void app_aps_start_tx_rx_sync_capture(void)
{

	while(BBIF_COEX_STATUS->BLE_IN_PROCESS_ALIAS || BBIF_COEX_STATUS->BLE_RX_ALIAS || BBIF_COEX_STATUS->BLE_TX_ALIAS);

	SYSCTRL_RF_ACCESS_CFG->RF_IRQ_ACCESS_ALIAS = RF_IRQ_ACCESS_ENABLE_BITBAND;

	NVIC_EnableIRQ(BLE_COEX_RX_TX_IRQn);
	NVIC_SetPriority(BLE_COEX_RX_TX_IRQn,0);
	if (asha_side ==ASHA_CAPABILITIES_SIDE_RIGHT){
		*((uint32_t *)BBIF_COEX_INT_CFG_BASE) = (BLE_TX_EVENT_RISING_EDGE);
	}
	else
	{
		*((uint32_t *)BBIF_COEX_INT_CFG_BASE) = (BLE_RX_EVENT_RISING_EDGE);
	}
}

#else

bool app_aps_start_tx_rx_sync_capture(void)
{
	while(BBIF_COEX_STATUS->BLE_IN_PROCESS_ALIAS || BBIF_COEX_STATUS->BLE_RX_ALIAS || BBIF_COEX_STATUS->BLE_TX_ALIAS);


	SYSCTRL_RF_ACCESS_CFG->RF_IRQ_ACCESS_ALIAS = RF_IRQ_ACCESS_ENABLE_BITBAND;

	// enable ble_snyc pulse
	*((uint32_t *)BB_COEXIFCNTL0_BASE) = 0x1;
//	BB_COEXIFCNTL2->RX_ANT_DELAY_BYTE = 0xf;
//	BB_COEXIFCNTL2->TX_ANT_DELAY_BYTE = 0xf;

	if (asha_side ==ASHA_CAPABILITIES_SIDE_RIGHT){
		RF_REG_WRITE(IRQ_CONF, 0x1);
		NVIC_EnableIRQ(RF_TX_IRQn);
		NVIC_SetPriority(RF_TX_IRQn,0);
		//BBIF_COEX_CTRL->RX_ALIAS = 1;
		//BBIF_COEX_CTRL->TX_ALIAS = 0;
	}
	else
	{
		RF_REG_WRITE(IRQ_CONF, 0x2);
		NVIC_EnableIRQ(RF_RXSTOP_IRQn);
		NVIC_SetPriority(RF_RXSTOP_IRQn,0);
		//BBIF_COEX_CTRL->RX_ALIAS = 0;
		//BBIF_COEX_CTRL->TX_ALIAS = 1;
	}

	return true;
}
#endif


uint8_t *app_aps_get_sync_info(void)
{
	return (uint8_t*)m_local_sync_info.list;
}


void TIMER_IRQ_FUNC(TIMER_START_APS)(void)
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
    NVIC_DisableIRQ(TIMER_IRQn(TIMER_START_APS));

    Sys_Timers_Stop(1U << TIMER_START_APS);
}


void BLE_COEX_RX_TX_IRQHandler(void)
{
	static uint8_t state = 0;
	volatile uint32_t bbif_status =  *((uint32_t *)0x40001404);
	volatile uint32_t link_label = (bbif_status & BBIF_STATUS_LINK_LABEL_Mask) >> BBIF_STATUS_LINK_LABEL_Pos;

	if (link_label == 1)
	{
		if (state == 5)
		{
			state++;
		}
		else
		{
			Sys_GPIO_Set_High(GPIO_DBG_PACK_RECV);
			Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);
			//NVIC_DisableIRQ(BLE_COEX_RX_TX_IRQn);
			state = 0;
		}
	}
}


void RF_TX_IRQHandler(void)
{
	static uint8_t state = 0;
	volatile uint32_t bbif_status =  *((uint32_t *)0x40001404);
	volatile uint32_t link_label = (bbif_status & BBIF_STATUS_LINK_LABEL_Mask) >> BBIF_STATUS_LINK_LABEL_Pos;
	uint32_t status = RF_REG_READ(IRQ_STATUS);


	if (link_label == 1)
	{
		if (state <3)
		{
			//Sys_GPIO_Set_High(GPIO_DBG_PACK_RECV);
			app_aps_compute_sync_info();
			//Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);
			state++;
		}
		else
		{
			m_sync_buffer_idx = 0;
			state = 0;
	    	NVIC_DisableIRQ(RF_TX_IRQn);
	    	RF_REG_WRITE(IRQ_CONF, 0x0);
		}
	}
	else
	{

//		Sys_GPIO_Set_High(GPIO_DBG_ASRC_ISR);
//		Sys_GPIO_Set_Low(GPIO_DBG_ASRC_ISR);
	}
}

void RF_RXSTOP_IRQHandler(void)
{

	static uint8_t state = 0;
	volatile uint32_t bbif_status =  *((uint32_t *)0x40001404);
	volatile uint32_t link_label = (bbif_status & BBIF_STATUS_LINK_LABEL_Mask) >> BBIF_STATUS_LINK_LABEL_Pos;
	uint32_t status = RF_REG_READ(IRQ_STATUS);

	if (m_audio_env->audio_state == AUDIO_IDLE)
	{

			//Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);
	}
	else
	{

		if (link_label != 1)
		{
//			Sys_GPIO_Set_High(GPIO_DBG_ASRC_ISR);
//			Sys_GPIO_Set_Low(GPIO_DBG_ASRC_ISR);
			return;
		}
		if (state <3)
		{
			//Sys_GPIO_Set_High(GPIO_DBG_PACK_RECV);
			app_aps_compute_sync_info();
			//Sys_GPIO_Set_Low(GPIO_DBG_PACK_RECV);
			state++;
		}
		else
		{
			m_sync_buffer_idx = 0;
			state = 0;
			NVIC_DisableIRQ(RF_RXSTOP_IRQn);
			RF_REG_WRITE(IRQ_CONF, 0x0);
		}
	}
}


