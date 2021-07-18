
/* ----------------------------------------------------------------------------
 * Copyright (c) 2018 Semiconductor Components Industries, LLC (d/b/a
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
 * ble_asha.c
 * - Android Audio Streaming Hearing Aid (ASHA) service source file.
 *   The Android ASHA service is defined at
 *   https://source.android.com/devices/bluetooth/asha
 * ------------------------------------------------------------------------- */

#include <ble_gap.h>
#include <ble_gatt.h>
#include <ble_l2c.h>
#include <msg_handler.h>
#include <app_trace.h>
#include <ble_asha.h>
#include <app_audio.h>




/* Global variable definition */
struct ASHA_Env_t asha_env;

/* ASHA GATT services, as defined in
 * https://source.android.com/devices/bluetooth/asha#asha-gatt-services */
const struct att_db_desc asha_att_db[] =
{
    /**** Service 0: ASHA ****/
    CS_SERVICE_UUID_16(ASHA_SERVICE, ASHA_SERVICE_UUID),

    /* ASHA ReadOnlyProperties characteristic  */
    CS_CHAR_UUID_128(ASHA_READONLYPROPERTIES_CHAR_IDX,
            ASHA_READONLYPROPERTIES_VAL_IDX,
            ASHA_READONLYPROPERTIES_CHAR_UUID,
            PERM(RD, ENABLE),
            ASHA_READONLYPROPERTIES_CHAR_LENGTH,
            &(asha_env.readOnlyProperties),
            ASHA_ReadOnlyProperties_Callback),
    /* Characteristic User Description descriptor */
    CS_CHAR_USER_DESC(ASHA_READONLYPROPERTIES_USER_DESC_IDX,
            sizeof(ASHA_READONLYPROPERTIES_CHAR_NAME) - 1,
            ASHA_READONLYPROPERTIES_CHAR_NAME,
            NULL),

    /* ASHA AudioControlPoint characteristic */
    CS_CHAR_UUID_128(ASHA_AUDIOCONTROLPOINT_CHAR_IDX,
            ASHA_AUDIOCONTROLPOINT_VAL_IDX,
            ASHA_AUDIOCONTROLPOINT_CHAR_UUID,
            PERM(WRITE_COMMAND, ENABLE) | PERM(WRITE_REQ, ENABLE), /* Specification says WRITE_COMMAND but in practice the central sends a WRITE_REQ, so we enabled both just in case. */
            ASHA_AUDIOCONTROLPOINT_CHAR_LENGTH,
            &(asha_env.audioControlPoint),
            ASHA_AudioControlPoint_Callback),
    /* Characteristic User Description descriptor */
    CS_CHAR_USER_DESC(ASHA_AUDIOCONTROLPOINT_USER_DESC_IDX,
            sizeof(ASHA_AUDIOCONTROLPOINT_CHAR_NAME) - 1,
            ASHA_AUDIOCONTROLPOINT_CHAR_NAME,
            NULL),

    /* ASHA AudioStatusPoint characteristic  */
    CS_CHAR_UUID_128(ASHA_AUDIOSTATUSPOINT_CHAR_IDX,        /* attidx_char  */
            ASHA_AUDIOSTATUSPOINT_VAL_IDX,                  /* attidx_val   */
            ASHA_AUDIOSTATUSPOINT_CHAR_UUID,                /* uuid         */
            PERM(RD, ENABLE) | PERM(NTF, ENABLE),           /* perm         */
            ASHA_AUDIOSTATUSPOINT_CHAR_LENGTH,              /* length       */
            &(asha_env.audioStatusPoint),                   /* data         */
            ASHA_AudioStatusPoint_Callback),                /* callback     */
    /* Client Characteristic Configuration descriptor */
    CS_CHAR_CCC(ASHA_AUDIOSTATUSPOINT_VAL_CCC0,             /* attidx       */
            &(asha_env.audioStatusPointCCC),                /* data         */
            NULL),                                          /* callback     */
    /* Characteristic User Description descriptor */
    CS_CHAR_USER_DESC(ASHA_AUDIOSTATUSPOINT_USER_DESC_IDX,  /* attidx       */
            sizeof(ASHA_AUDIOSTATUSPOINT_CHAR_NAME) - 1,    /* length       */
            ASHA_AUDIOSTATUSPOINT_CHAR_NAME,                /* data         */
            NULL),                                          /* callback     */

    /* ASHA Volume characteristic  */
    CS_CHAR_UUID_128(ASHA_VOLUME_CHAR_IDX,                  /* attidx_char  */
            ASHA_VOLUME_VAL_IDX,                            /* attidx_val   */
            ASHA_VOLUME_CHAR_UUID,                          /* uuid         */
            PERM(RD, ENABLE) | PERM(WRITE_COMMAND, ENABLE), /* perm         */
            ASHA_VOLUME_CHAR_LENGTH,                        /* length       */
            &(asha_env.volume),                             /* data         */
            ASHA_Volume_Callback),                          /* callback     */
    /* Characteristic User Description descriptor */
    CS_CHAR_USER_DESC(ASHA_VOLUME_USER_DESC_IDX,            /* attidx       */
            sizeof(ASHA_VOLUME_CHAR_NAME) - 1,              /* length       */
            ASHA_VOLUME_CHAR_NAME,                          /* data         */
            NULL),                                          /* callback     */

    /* ASHA LE_PSM characteristic  */
    CS_CHAR_UUID_128(ASHA_LE_PSM_CHAR_IDX,                  /* attidx_char  */
            ASHA_LE_PSM_VAL_IDX,                            /* attidx_val   */
            ASHA_LE_PSM_CHAR_UUID,                          /* uuid         */
            PERM(RD, ENABLE),                               /* perm         */
            ASHA_LE_PSM_CHAR_LENGTH,                        /* length       */
            &(asha_env.lePSM),                              /* data         */
            ASHA_LE_PSM_Callback),                          /* callback     */
    /* Characteristic User Description descriptor */
    CS_CHAR_USER_DESC(ASHA_LE_PSM_USER_DESC_IDX,            /* attidx       */
            sizeof(ASHA_LE_PSM_CHAR_NAME) - 1,              /* length       */
            ASHA_LE_PSM_CHAR_NAME,                          /* data         */
            NULL),                                          /* callback     */
};

/* ----------------------------------------------------------------------------
 * Function      : void ASHA_Initialize(void)
 * ----------------------------------------------------------------------------
 * Description   : Initialize ASHA service environment and message handler
 * Inputs        : None
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void ASHA_Initialize(const struct ReadOnlyProperties_t *readOnlyProperties,
        void (*appCallback)(enum ASHA_Operation_t op, void *param))
{
    memset(&asha_env, 0, sizeof(struct ASHA_Env_t));
    memcpy(&(asha_env.readOnlyProperties), readOnlyProperties, sizeof(struct ReadOnlyProperties_t));
    asha_env.lePSM = ASHA_LE_PSM;
    asha_env.appCallback = appCallback;

    MsgHandler_Add(GAPM_CMP_EVT, ASHA_MsgHandler);
    MsgHandler_Add(TASK_ID_L2CC, ASHA_MsgHandler);
    MsgHandler_Add(GAPC_CONNECTION_REQ_IND, ASHA_MsgHandler);
    MsgHandler_Add(GAPC_PARAM_UPDATED_IND, ASHA_MsgHandler);
    MsgHandler_Add(TASK_ID_GAPC, ASHA_MsgHandler);
    asha_env.conidx = 0xFF;
}

void ASHA_AddCredits(uint16_t credits)
{
    L2CC_LecbAddCmd(asha_env.conidx, asha_env.local_cid, credits);
}

/* ----------------------------------------------------------------------------
 * Function      : void ASHA_MsgHandler(ke_msg_id_t const msg_id,
 *                                      void const *param,
 *                                      ke_task_id_t const dest_id,
 *                                      ke_task_id_t const src_id)
 * ----------------------------------------------------------------------------
 * Description   : Handle all events related to the ASHA service
 * Inputs        : - msg_id     - Kernel message ID number
 *                 - param      - Message parameter
 *                 - dest_id    - Destination task ID number
 *                 - src_id     - Source task ID number
 * Outputs       : return value - Indicate if the message was consumed;
 *                                compare with KE_MSG_CONSUMED
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void ASHA_MsgHandler(ke_msg_id_t const msg_id, void const *param,
                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);

    switch(msg_id)
    {
        case GAPM_CMP_EVT:
        {
            const struct gapm_cmp_evt* p = param;
            if(p->operation == GAPM_SET_DEV_CONFIG && p->status == GAP_ERR_NO_ERROR)
            {
                /* Register a LE Protocol/Service Multiplexer for ASHA */
                GAPM_LepsmRegisterCmd(ASHA_LE_PSM, TASK_APP, 0); //TODO: generalize SEC level param
                GAPM_LepsmRegisterCmd(ASHA_LE_PSM_SYNC, TASK_APP, 0);
                PRINTF("\r\n sending GAPM_LEPSM_REG_CMD...");
            }
            else if(p->operation == GAPM_LEPSM_REG &&  p->status == GAP_ERR_NO_ERROR)
            {
                PRINTF("\r\n GAPM_LEPSM_REG: LEPSM registered successfully");
            }

        }
        break;

        case GAPC_CONNECTION_REQ_IND:
        {

        }
        break;

        case GAPC_DISCONNECT_IND:
		{
			if (conidx == asha_env.conidx){
				asha_env.conidx = 0xFF;
				asha_env.appCallback(ASHA_AUDIO_STOP,0);
			}
		}
		break;

        case GAPC_PARAM_UPDATED_IND:
		{
			asha_env.con_int = ((struct gapc_param_updated_ind*) param)->con_interval;
			if (asha_env.con_int == 16 || asha_env.con_int == 8)
			{
				PRINTF("Updated the conn interval\n");
				uint8_t statusSuccess = 0;
				GATTC_SendEvtCmd(asha_env.conidx, GATTC_NOTIFY, 0, GATTM_GetHandle(ASHA_AUDIOSTATUSPOINT_VAL_IDX),
				sizeof(statusSuccess), &statusSuccess);
			}
		}
		break;

        case L2CC_LECB_CONNECT_REQ_IND:
        {
        	// only allow one Audio Stream at a time
        	//if (asha_env.conidx != 0xFF)
        	//{
        	//	break;
        	//}

        	/* If L2CC connection request has ASHA LE_PSM, accept it */
            const struct l2cc_lecb_connect_req_ind* p = param;

            PRINTF("\r\n ASHA_MsgHandler: L2CC_LECB_CONNECT_REQ_IND le_psm=0x%X",p->le_psm);
            if(p->le_psm == ASHA_LE_PSM)
            {

            	struct l2cc_lecb_connect_req_ind const *ind = (struct l2cc_lecb_connect_req_ind const *)param;

                struct l2cc_lecb_connect_cfm cfm = {
                    .le_psm = ind->le_psm,
                    .peer_cid = ind->peer_cid,
                    .local_mps = ind->peer_mps,
                    .local_mtu = ind->peer_mtu,
                    .accept = true,
                    .local_cid = 0,
                    .local_credit = ASHA_L2CC_INITIAL_CREDITS,
                };

                L2CC_LecbConnectCfm(conidx, &cfm);

                asha_env.conidx = conidx;
            }

        }
        break;

        case L2CC_CMP_EVT:
        {
        	struct l2cc_cmp_evt * evt = param;

        	if (evt->operation == L2CC_LECB_CONNECT)
        	{

        		PRINTF("L2CC_CMP_EVT: op:%d , status %d\n",evt->operation,evt->status);
        	}
        	else if ((evt->operation == L2CC_LECB_SDU_SEND) && (evt->status != 0))
        	{
        		PRINTF("L2CC_CMP_EVT: op:%d , status %d\n",evt->operation,evt->status);
        	}

        }
        break;
        case L2CC_LECB_CONNECT_IND:
        {
            const struct l2cc_lecb_connect_ind* ind = param;
            asha_env.audioStatusPoint = 0;
            if(ind->le_psm == ASHA_LE_PSM)
            {

                asha_env.local_cid = ind->local_cid;
                asha_env.peer_cid = ind->peer_cid;

                PRINTF("\r\n ASHA_MsgHandler: L2CC_LECB_CONNECT_IND: peer_cid=%d", ind->peer_cid);
            }
            else if (ind->le_psm == ASHA_LE_PSM_SYNC)
            {
            	asha_env.local_cid_sync = ind->local_cid;
				asha_env.peer_cid_sync = ind->peer_cid;
            	PRINTF("\r\n ASHA_MsgHandler: L2CC_LECB_CONNECT_IND: peer_cid=%d", ind->peer_cid);
            }
        }
        break;

        case L2CC_LECB_SDU_RECV_IND:
        {
        	// only allow one audio stream connection at at time
//        	if (conidx != asha_env.conidx)
//            {
//            	break;
//            }

        	const struct l2cc_lecb_sdu_recv_ind* p = param;
            struct asha_audio_received *rcv_p;

            if(p->sdu.cid == asha_env.local_cid)
            {

            	rcv_p = (void*)param+offsetof(struct l2cc_lecb_sdu_recv_ind, sdu.credit);
            	asha_env.appCallback(ASHA_AUDIO_RCVD, rcv_p);
            }


        }
        break;
    }
}

uint8_t ASHA_ReadOnlyProperties_Callback(uint8_t conidx, uint16_t attidx, uint16_t handle,
                                         uint8_t *toData, const uint8_t *fromData, uint16_t lenData,
                                         uint16_t operation)
{
    memcpy(toData, fromData, lenData);
    PRINTF("\r\nASHA_ReadOnlyProperties_Callback: conidx=%d length=%d", conidx, lenData);
    return ATT_ERR_NO_ERROR;
}

uint8_t ASHA_AudioControlPoint_Callback(uint8_t conidx, uint16_t attidx, uint16_t handle,
                                        uint8_t *toData, const uint8_t *fromData, uint16_t lenData,
                                        uint16_t operation)
{
    uint8_t opCode = fromData[0];
    if((opCode == ASHA_AUDIOCONTROLPOINT_START) ||
       (opCode == ASHA_AUDIOCONTROLPOINT_STOP))
    {
        /* Update audioControlPoint */
        memcpy(toData, fromData, lenData);

        /* Update volume */
        asha_env.volume = asha_env.audioControlPoint.volume;

        PRINTF("\r\nASHA_AudioControlPoint_Callback: opCode=%d codec=%d audioType=%d volume=%d\r\n",
                asha_env.audioControlPoint.opCode, asha_env.audioControlPoint.codec,
                asha_env.audioControlPoint.audioType, asha_env.audioControlPoint.volume);

        /* Call application callback */
        if(opCode == ASHA_AUDIOCONTROLPOINT_START)
        {
            struct asha_audio_start param = {
                .codec = asha_env.audioControlPoint.codec,
                .audiotype = asha_env.audioControlPoint.audioType,
                .volume = asha_env.audioControlPoint.volume,
				.other_state =asha_env.audioControlPoint.otherstate
            };

            // start the stream
            asha_env.appCallback(ASHA_AUDIO_START, &param);

            if (asha_env.con_int == 16 || asha_env.con_int == 8)
            {
                PRINTF("Allow Central to start audio streaming!\n");
            	uint8_t statusSuccess = 0;
                GATTC_SendEvtCmd(asha_env.conidx, GATTC_NOTIFY, 0, GATTM_GetHandle(ASHA_AUDIOSTATUSPOINT_VAL_IDX),
                sizeof(statusSuccess), &statusSuccess);
            }
        }
        else
        {
            asha_env.appCallback(ASHA_AUDIO_STOP, NULL);
            uint8_t statusSuccess = 0;
			GATTC_SendEvtCmd(asha_env.conidx, GATTC_NOTIFY, 0, GATTM_GetHandle(ASHA_AUDIOSTATUSPOINT_VAL_IDX),
			sizeof(statusSuccess), &statusSuccess);
        }

        asha_env.audioStatusPoint = 0;
    }
    else if (opCode == ASHA_AUDIOCONTROLPOINT_STATUS)
    {

    	uint8_t param =fromData[1];

    	asha_env.appCallback(ASHA_AUDIO_STATUS, &param);

        asha_env.audioStatusPoint = 0;
    }
    else
    {
        PRINTF("\r\nASHA_AudioControlPoint_Callback: ERROR (invalid opCode=%d)",
                fromData[0]);
        asha_env.audioStatusPoint = 1;
    }
    return ATT_ERR_NO_ERROR;
}

uint8_t ASHA_AudioStatusPoint_Callback(uint8_t conidx, uint16_t attidx, uint16_t handle,
                                       uint8_t *toData, const uint8_t *fromData, uint16_t lenData,
                                       uint16_t operation)
{
    memcpy(toData, fromData, lenData);
    PRINTF("\r\nASHA_AudioStatusPoint_Callback: conidx=%d length=%d", conidx, lenData);
    return ATT_ERR_NO_ERROR;
}

uint8_t ASHA_Volume_Callback(uint8_t conidx, uint16_t attidx, uint16_t handle,
                             uint8_t *toData, const uint8_t *fromData, uint16_t lenData,
                             uint16_t operation)
{
    memcpy(toData, fromData, lenData);
    asha_env.audioControlPoint.volume = asha_env.volume;
    PRINTF("\r\nASHA_Volume_Callback: conidx=%d volume=%d", conidx, asha_env.volume);
    struct asha_volume_change param = {
        .volume = asha_env.volume
    };
    asha_env.appCallback(ASHA_VOLUME_CHANGE, &param);
    return ATT_ERR_NO_ERROR;
}

uint8_t ASHA_LE_PSM_Callback(uint8_t conidx, uint16_t attidx, uint16_t handle,
                             uint8_t *toData, const uint8_t *fromData, uint16_t lenData,
                             uint16_t operation)
{
    memcpy(toData, fromData, lenData);
    PRINTF("\r\nASHA_LE_PSM_Callback: conidx=%d, length=%d, le_psm=0x%2X%2X", conidx, lenData, fromData[1], fromData[0]);
    return ATT_ERR_NO_ERROR;
}

int8_t ASHA_GetVolume(void)
{
	return asha_env.volume;
}

