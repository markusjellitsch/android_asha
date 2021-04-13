/* ----------------------------------------------------------------------------
 * Copyright (c) 2018 Semiconductor Components Industries, LLC (d/b/a
 * ON Semiconductor), All Rights Reserved
 *
 * Copyright (C) RivieraWaves 2009-2018
 *
 * This module is derived in part from example code provided by RivieraWaves
 * and as such the underlying code is the property of RivieraWaves [a member
 * of the CEVA, Inc. group of companies], together with additional code which
 * is the property of ON Semiconductor. The code (in whole or any part) may not
 * be redistributed in any form without prior written permission from
 * ON Semiconductor.
 *
 * The terms of use and warranty for this code are covered by contractual
 * agreements between ON Semiconductor and the licensee.
 *
 * This is Reusable Code.
 *
 * ----------------------------------------------------------------------------
 * app_msg_handler.c
 * - Application message handlers
 * ------------------------------------------------------------------------- */

#include <app.h>
#include <app_audio.h>
#include <ble_asha.h>

extern struct ASHA_Env_t asha_env;

/* Configuration pre-set in app_config.c */
extern struct gapm_set_dev_config_cmd  devConfigCmd;
extern struct gapm_start_advertise_cmd advertiseCmd;
extern const union  gapc_dev_info_val* getDevInfoCfm[];
extern const struct att_db_desc asha_att_db[];

extern audio_frame_param_t env_audio;

/* ----------------------------------------------------------------------------
 * Function      : void APP_ASHA_CallbackHandler(enum ASHA_Operation_t op, void *param)
 * ----------------------------------------------------------------------------
 * Description   : Application callback function to interact with ble_asha
 *                 service. Handle ASHA events to start/stop streaming or
 *                 change volume.
 * Inputs        : - op     - Kernel message ID number
 *                 - param  - Operation parameter to be type casted to:
 *                            - For op == ASHA_VOLUME_CHANGE,
 *                              param == struct asha_volume_change*
 *                            - For op == ASHA_AUDIO_START,
 *                              param == struct asha_audio_start*
 *                            - For op == ASHA_AUDIO_STOP,
 *                              param == NULL
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void APP_ASHA_CallbackHandler(enum ASHA_Operation_t op, void *param)
{
    switch(op)
    {
        case ASHA_VOLUME_CHANGE:
        {
            struct asha_volume_change* p = param;
            Volume_Set(p->volume);
            PRINTF("\r\nAPP_ASHA_CallbackHandler - ASHA_VOLUME_CHANGE: volume=%d\r\n", p->volume);
        }
        break;

        case ASHA_AUDIO_START:
        {
            struct asha_audio_start* p = param;
            Volume_Set(p->volume);
            APP_ResetPrevSeqNumber();
            PRINTF("\r\nAPP_ASHA_CallbackHandler - ASHA_AUDIO_START: codec=%d audiotype=%d volume=%d\r\n",
                    p->codec, p->audiotype, p->volume);

            if( env_audio.state == LINK_DISCONNECTED )
            {
                /* We don't directly start the rendering IRQs, we'll delay that
                 * to after we received an audio packet */
                env_audio.state = LINK_TRANSIENT;
            }
        }
        break;

        case ASHA_AUDIO_STOP:
        {
            PRINTF("\r\nAPP_ASHA_CallbackHandler - ASHA_AUDIO_STOP\r\n");
            APP_Audio_Disconnect();
        }
        break;

        case ASHA_AUDIO_RCVD:
        {

        	// start audio streaming by inserting audio data to the queue
        	struct asha_audio_received *rcv_p = (struct asha_audio_received *) param;
            APP_Audio_Transfer(rcv_p->data, rcv_p->length - 1, rcv_p->seq_number);
        }
        break;
    }
}

/* ----------------------------------------------------------------------------
 * Function      : void APP_GAPM_GATTM_Handler(ke_msg_id_t const msg_id,
 *                                     void const *param,
 *                                     ke_task_id_t const dest_id,
 *                                     ke_task_id_t const src_id)
 * ----------------------------------------------------------------------------
 * Description   : Handle GAPM/GATTM messages that need application action
 * Inputs        : - msg_id     - Kernel message ID number
 *                 - param      - Message parameter
 *                 - dest_id    - Destination task ID number
 *                 - src_id     - Source task ID number
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void APP_GAPM_GATTM_Handler(ke_msg_id_t const msg_id, void const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    switch(msg_id)
    {
        case GAPM_CMP_EVT:
        {
            const struct gapm_cmp_evt *p = param;
			/* Reset completed. Apply device configuration. */
            if(p->operation == GAPM_RESET)
            {
                GAPM_SetDevConfigCmd(&devConfigCmd);
                /* Trigger a GAPM_CMP_EVT / GAPM_SET_DEV_CONFIG when finished.
                 * BASS (ble_bass.c) and CUSTOMSS (app_customss.c) monitor this
                 * event before adding the profile/service to the stack. */
            }
            else if(p->operation == GAPM_SET_DEV_CONFIG && p->status == GAP_ERR_NO_ERROR)
            {
                /* Add custom services' attribute database (in this application only ASHA service) */
                GATTM_AddAttributeDatabase(asha_att_db, ASHA_MAX_IDX);
            }
            else if((p->operation == GAPM_RESOLV_ADDR) && /* IRK not found for address */
                    (p->status == GAP_ERR_NOT_FOUND))
            {
                struct gapc_connection_cfm cfm;
                uint8_t conidx = KE_IDX_GET(dest_id);
                APP_SetConnectionCfmParams(conidx, &cfm);
                GAPC_ConnectionCfm(conidx, &cfm); /* Confirm connection without LTK. */
                PRINTF("\r\nGAPM_CMP_EVT / GAPM_RESOLV_ADDR. conidx=%d Status = NOT FOUND", conidx);
            }
        }
        break;

        case GATTM_ADD_SVC_RSP:
        case GAPM_PROFILE_ADDED_IND:
        {
            /* If all expected profiles/services have been added */
            if(GAPM_GetProfileAddedCount() == APP_NUM_STD_PRF &&
               GATTM_GetServiceAddedCount() == APP_NUM_CUSTOM_SVC)
            {
                GAPM_StartAdvertiseCmd(&advertiseCmd); /* Start advertising */

                /* Start the LED periodic timer. LED blinks according to the
                 * number of connected peers. See APP_LED_Timeout_Handler. */
                ke_timer_set(APP_LED_TIMEOUT, TASK_APP, TIMER_SETTING_MS(200));
            }
        }
        break;

        case GAPM_ADDR_SOLVED_IND: /* Private address resolution was successful */
        {
            PRINTF("\r\nGAPM_ADDR_SOLVED_IND");
            struct gapc_connection_cfm cfm;
            uint8_t conidx = KE_IDX_GET(dest_id);
            APP_SetConnectionCfmParams(conidx, &cfm);
            GAPC_ConnectionCfm(conidx, &cfm); /* Send connection confirmation with LTK */
        }
        break;
    }
}

/* ----------------------------------------------------------------------------
 * Function      : void APP_GAPC_MsgHandler(ke_msg_id_t const msg_id,
 *                                     void const *param,
 *                                     ke_task_id_t const dest_id,
 *                                     ke_task_id_t const src_id)
 * ----------------------------------------------------------------------------
 * Description   : Handle GAPC messages that need application action
 * Inputs        : - msg_id     - Kernel message ID number
 *                 - param      - Message parameter
 *                 - dest_id    - Destination task ID number
 *                 - src_id     - Source task ID number
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void ble_gapc_main_handler(ke_msg_id_t const msg_id, void const *param,
                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t conidx = KE_IDX_GET(src_id);

    switch(msg_id)
    {
        case GAPC_CONNECTION_REQ_IND:
        {
            Sys_GPIO_Set_High(0);
            const struct gapc_connection_req_ind* p = param;
            PRINTF("\r\nGAPC_CONNECTION_REQ_IND: con_interval=%d, con_latency = %d, sup_to = %d, clk_accuracy = %d",
                    p->con_interval,
                    p->con_latency,
                    p->sup_to,
                    p->clk_accuracy);
            PRINTF("\r\n  ADDR: ");
            for (uint8_t i = 0;i < GAP_BD_ADDR_LEN; i++)
                PRINTF("%d ", p->peer_addr.addr[i]);
            PRINTF("\r\n  ADDR_TYPE: %d", p->peer_addr_type);

            Sys_GPIO_Set_Low(0);

#if CFG_BOND_LIST_IN_NVR2
            if(GAP_IsAddrPrivateResolvable(p->peer_addr.addr, p->peer_addr_type) &&
               BondList_Size() > 0)
            {
                PRINTF("\r\n    Starting GAPM_ResolvAddrCmd\r\n");
                GAPM_ResolvAddrCmd(conidx, p->peer_addr.addr, 0, NULL);
            }
            else
#endif
            {
                struct gapc_connection_cfm cfm;
                APP_SetConnectionCfmParams(conidx, &cfm);
                GAPC_ConnectionCfm(conidx, &cfm); /* Send connection confirmation */
            }

            /* If not yet connected to all peers, keep advertising */
            if((GAPC_GetConnectionCount() < APP_NB_PEERS))
            {
                GAPM_StartAdvertiseCmd(&advertiseCmd);
            }

            if (conidx == 0) asha_env.con_int = p->con_interval;
        }
        break;

        case GAPC_DISCONNECT_IND:
        {
            GAPM_StartAdvertiseCmd(&advertiseCmd);
            PRINTF("\r\nGAPC_DISCONNECT_IND: reason = %d", ((struct gapc_disconnect_ind*)param)->reason);
        }
        break;

        case GAPC_GET_DEV_INFO_REQ_IND:
        {
            const struct gapc_get_dev_info_req_ind* p = param;


            GAPC_GetDevInfoCfm(conidx, p->req, getDevInfoCfm[p->req]);
            PRINTF("\r\nGAPC_GET_DEV_INFO_REQ_IND: req = %d", p->req);
        }
        break;

        case GAPC_PARAM_UPDATE_REQ_IND:
        {
            GAPC_ParamUpdateCfm(conidx, true, 0xFF, 0xFF);

            PRINTF("\r\nGAPC_PARAM_UPDATE_REQ_IND: intv_min=%d, intv_max = %d, latency = %d, time_out = %d",
            		((struct gapc_conn_param*) param)->intv_min,
					((struct gapc_conn_param*) param)->intv_max,
					((struct gapc_conn_param*) param)->latency,
					((struct gapc_conn_param*) param)->time_out);
        }
        break;

#if CFG_BOND_LIST_IN_NVR2
        case GAPC_ENCRYPT_REQ_IND:
        {
            const struct gapc_encrypt_req_ind* p = param;
            /* Accept request if bond information is valid & EDIV/RAND match */
            bool found = (GAPC_IsBonded(conidx) &&
                          p->ediv == GAPC_GetBondInfo(conidx)->EDIV &&
                          !memcmp(p->rand_nb.nb, GAPC_GetBondInfo(conidx)->RAND, GAP_RAND_NB_LEN));

            PRINTF("\r\nGAPC_ENCRYPT_REQ_IND: bond information %s", (found ? "FOUND" : "NOT FOUND"));
            PRINTF("\r\n    GAPC_isBonded=%d GAPC_EDIV=%d  GAPC_rand= %d %d %d %d   ",
                    GAPC_IsBonded(conidx), GAPC_GetBondInfo(conidx)->EDIV,
                    GAPC_GetBondInfo(conidx)->RAND[0], GAPC_GetBondInfo(conidx)->RAND[1],
                    GAPC_GetBondInfo(conidx)->RAND[2], GAPC_GetBondInfo(conidx)->RAND[3]);
            PRINTF("\r\n    p_EDIV=%d  p_rand= %d %d %d %d   ", p->ediv,
                   p->rand_nb.nb[0], p->rand_nb.nb[1], p->rand_nb.nb[2], p->rand_nb.nb[3]);

            GAPC_EncryptCfm(conidx, found, GAPC_GetBondInfo(conidx)->LTK);
        }
        break;

        case GAPC_ENCRYPT_IND:
        {
            PRINTF("\r\nGAPC_ENCRYPT_IND: Link encryption is ON");
        }
        break;

        case GAPC_BOND_REQ_IND:
        {
            const struct gapc_bond_req_ind* p = param;
            switch (p->request)
            {
                case GAPC_PAIRING_REQ:
                {
                    bool accept = BondList_Size() < APP_BONDLIST_SIZE;
                    union gapc_bond_cfm_data pairingRsp =
                    {
                        .pairing_feat =
                        {
                            .iocap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT,
                            .oob = GAP_OOB_AUTH_DATA_NOT_PRESENT,
                            .key_size = KEY_LEN,
                            .ikey_dist = (GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY),
                            .rkey_dist = (GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY | GAP_KDIST_SIGNKEY),
                        }
                    };
#ifdef SECURE_CONNECTION
                    if (p->data.auth_req & GAP_AUTH_SEC_CON)
                    {
                        pairingRsp.pairing_feat.auth = GAP_AUTH_REQ_SEC_CON_BOND;
                        pairingRsp.pairing_feat.sec_req = GAP_SEC1_SEC_CON_PAIR_ENC;
                    }
                    else
#endif    /* ifdef SECURE_CONNECTION */
                    {
                        pairingRsp.pairing_feat.auth = GAP_AUTH_REQ_NO_MITM_BOND;
                        pairingRsp.pairing_feat.sec_req = GAP_NO_SEC;
                    }
                    PRINTF("\r\nGAPC_BOND_REQ_IND / GAPC_PAIRING_REQ: accept = %d conidx=%d", accept, conidx);
                    GAPC_BondCfm(conidx, GAPC_PAIRING_RSP, accept, &pairingRsp);
                }

                break;

                case GAPC_LTK_EXCH: /* Prepare and send random LTK (legacy only) */
                {
                    PRINTF("\r\nGAPC_BOND_REQ_IND / GAPC_LTK_EXCH");
                    union gapc_bond_cfm_data ltkExch;
                    ltkExch.ltk.ediv = co_rand_hword();
                    for(uint8_t i = 0, i2 = GAP_RAND_NB_LEN; i < GAP_RAND_NB_LEN; i++, i2++)
                    {
                        ltkExch.ltk.randnb.nb[i] = co_rand_byte();
                        ltkExch.ltk.ltk.key[i] = co_rand_byte();
                        ltkExch.ltk.ltk.key[i2] = co_rand_byte();
                    }
                    GAPC_BondCfm(conidx, GAPC_LTK_EXCH, true, &ltkExch); /* Send confirmation */
                }
                break;

                case GAPC_TK_EXCH: /* Prepare and send TK */
                {
                    PRINTF("\r\nGAPC_BOND_REQ_IND / GAPC_TK_EXCH");
                    /* IO Capabilities are set to GAP_IO_CAP_NO_INPUT_NO_OUTPUT in this application.
                     * Therefore TK exchange is NOT performed. It is always set to 0 (Just Works algorithm). */
                }
                break;

                case GAPC_IRK_EXCH:
                {
                    PRINTF("\r\nGAPC_BOND_REQ_IND / GAPC_IRK_EXCH");
                    union gapc_bond_cfm_data irkExch;
                    memcpy(irkExch.irk.addr.addr.addr, GAPM_GetDeviceConfig()->addr.addr, GAP_BD_ADDR_LEN);
                    irkExch.irk.addr.addr_type = GAPM_GetDeviceConfig()->addr_type;
                    memcpy(irkExch.irk.irk.key, GAPM_GetDeviceConfig()->irk.key, GAP_KEY_LEN);
                    GAPC_BondCfm(conidx, GAPC_IRK_EXCH, true, &irkExch); /* Send confirmation */
                }
                break;

                case GAPC_CSRK_EXCH:
                {
                    PRINTF("\r\nGAPC_BOND_REQ_IND / GAPC_CSRK_EXCH");
                    union gapc_bond_cfm_data csrkExch;
                    Device_Param_Read(PARAM_ID_CSRK, csrkExch.csrk.key);
                    GAPC_BondCfm(conidx, GAPC_CSRK_EXCH, true, &csrkExch); /* Send confirmation */
                }
                break;
            }
        }
        break;
#endif /* CFG_BOND_LIST_IN_NVR2 */
    }
}

/* ----------------------------------------------------------------------------
 * Function      : void APP_LED_Timeout_Handler(ke_msg_idd_t const msg_id,
 *                                              void const *param,
 *                                              ke_task_id_t const dest_id,
 *                                              ke_task_id_t const src_id)
 * ----------------------------------------------------------------------------
 * Description   : Control GPIO "LED_DIO_NUM" behavior using a timer.
 *                 Possible LED behaviors:
 *                     - If the device is advertising but it has not connected
 *                       to any peer: the LED blinks every 200 ms.
 *                     - If the device is advertising and it is connecting to
 *                       fewer than BLE_CONNECTION_MAX peers: the LED blinks
 *                       every 2 seconds according to the number of connected
 *                       peers (i.e., blinks once if one peer is connected,
 *                       twice if two peers are connected, etc.).
 *                     - If the device is connected to BLE_CONNECTION_MAX peers
 *                       the LED is steady on.
 * Inputs        : - msg_id     - Kernel message ID number
 *                 - param      - Message parameter (unused)
 *                 - dest_id    - Destination task ID number
 *                 - src_id     - Source task ID number
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
void APP_LED_Timeout_Handler(ke_msg_id_t const msg_id, void const *param,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    static uint8_t toggle_cnt = 0;
    uint8_t connectionCount = GAPC_GetConnectionCount();

    /* Blink LED according to the number of connections */
    switch (connectionCount)
    {
        case 0:
        {
            ke_timer_set(APP_LED_TIMEOUT, TASK_APP, TIMER_SETTING_MS(200));
            Sys_GPIO_Toggle(LED_DIO_NUM); /* Toggle LED_DIO_NUM every 200ms */
            toggle_cnt = 0;
        }
        break;

        case APP_NB_PEERS:
        {
            ke_timer_set(APP_LED_TIMEOUT, TASK_APP, TIMER_SETTING_MS(200));
            Sys_GPIO_Set_High(LED_DIO_NUM); /* LED_DIO_NUM steady high */
            toggle_cnt = 0;
        }
        break;

        default: /* connectionCount is between 1 and APP_NB_PEERS (exclusive) */
        {
            if (toggle_cnt >= connectionCount * 2)
            {
                toggle_cnt = 0;
                ke_timer_set(APP_LED_TIMEOUT, TASK_APP, TIMER_SETTING_S(2)); /* Schedule timer for a long 2s break */
                Sys_GPIO_Set_High(LED_DIO_NUM); /* LED_DIO_NUM steady high until next 2s blinking period */
            }
            else
            {
                toggle_cnt++;
                Sys_GPIO_Toggle(LED_DIO_NUM);
                ke_timer_set(APP_LED_TIMEOUT, TASK_APP, TIMER_SETTING_MS(200));
            }
        }
    }
}

