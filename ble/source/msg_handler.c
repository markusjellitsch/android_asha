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
 * msg_handler.c
 * - Message Handler source file
 * ------------------------------------------------------------------------- */
#include <rsl10.h>
#include <msg_handler.h>
#include <ble_gap.h>
#include <ble_gatt.h>
#if RTE_BLE_L2CC_ENABLE
#include <ble_l2c.h>
#endif /* RTE_BLE_L2CC_ENABLE */

#include <stdlib.h>



static MsgHandler_t msgHandlers[64];
static uint64_t used_handlers = 0;

static bool MsgHandler_field_used(uint8_t index){
	if(index>64){
		return true;
	}
	if((used_handlers & (1U << index)) > 0){
		return true;
	}
	return false;
}

static void MsgHandler_set_field_used(uint8_t index){
	used_handlers = used_handlers | (1U<< index);
}

static void MsgHandler_set_field_unused(uint8_t index){
	used_handlers = used_handlers & ~(1U<< index);
}

static MsgHandler_t* MsgHandler_get_next_free_field(){
	for (int var = 0; var < 64; ++var) {
		if(!MsgHandler_field_used(var)){
			MsgHandler_set_field_used(var);
			return &msgHandlers[var];
		}
	}
	return NULL;
}



/* ----------------------------------------------------------------------------
 * Function      : bool MsgHandler_Add(ke_msg_id_t const msg_id,
 *                               void (*callback)(ke_msg_id_t const msg_id,
 *                               void const *param, ke_task_id_t const dest_id,
 *                               ke_task_id_t const src_id))
 * ----------------------------------------------------------------------------
 * Description   : Add a message handler to the linked list of handlers.
 * Inputs        : msg_id   - A task identifier, such as TASK_ID_GAPM
 *                            or a message identifier, such as GAPM_CMP_EVT;
 *                 callback - A callback function associated to the msg_id
 * Outputs       : true if successful, false otherwise
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
bool MsgHandler_Add(ke_msg_id_t const msg_id,
                    void (*callback)(ke_msg_id_t const msg_id, void const *param,
                    ke_task_id_t const dest_id, ke_task_id_t const src_id))
{
    MsgHandler_Remove(msg_id, callback); /* Avoid handler duplication, in case it already exists */

    MsgHandler_t *newElem = MsgHandler_get_next_free_field();

    if(!newElem) /* Malloc error */
    {
#if MEDEL_DEBUG
    	while(1);
#endif
        return false;
    }

    newElem->msg_id = msg_id;
    newElem->callback = callback;

    return true;
}

/* ----------------------------------------------------------------------------
 * Function      : bool MsgHandler_Remove(ke_msg_id_t const msg_id,
 *                               void (*callback)(ke_msg_id_t const msg_id,
 *                               void const *param, ke_task_id_t const dest_id,
 *                               ke_task_id_t const src_id))
 * ----------------------------------------------------------------------------
 * Description   : Remove message handler from list
 * Inputs        : msg_id   - A task identifier, such as TASK_ID_GAPM
 *                            or a message identifier, such as GAPM_CMP_EVT;
 *                 callback - A callback function associated to the msg_id
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
bool MsgHandler_Remove(ke_msg_id_t const msg_id,
                       void (*callback)(ke_msg_id_t const msg_id, void const *param,
                                        ke_task_id_t const dest_id, ke_task_id_t const src_id))
{
	for (int var = 0; var < 64; ++var) {
		if(MsgHandler_field_used(var) && msgHandlers[var].msg_id == msg_id && msgHandlers[var].callback == callback){
			MsgHandler_set_field_unused(var);
			return true;
		}
	}
    return false;
}

/* ----------------------------------------------------------------------------
 * Function      : int MsgHandler_Notify(ke_msg_id_t const msg_id,
 *                               void const *param, ke_task_id_t const dest_id,
 *                               ke_task_id_t const src_id)
 * ----------------------------------------------------------------------------
 * Description   : Search the lists and call back the functions associated with
 *                 the msg_id, following the priority order (HIGH to LOW). This
 *                 function was designed to be used as the default handler of
 *                 the kernel and shall NOT be called directly by the
 *                 application. To notify an event, the application should
 *                 enqueue a message in the kernel, in order to avoid chaining
 *                 the context of function calls (stack overflow).
 * Inputs        : msg_id   - A task identifier, such as TASK_ID_GAPM
 *                            or a message identifier, such as GAPM_CMP_EVT;
 *                 param    - Message parameter
 *                 dest_id  - destination task
 *                 src_id   - source task
 * Outputs       : None
 * Assumptions   : None
 * ------------------------------------------------------------------------- */
int MsgHandler_Notify(ke_msg_id_t const msg_id, void *param,
                      ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    uint8_t task_id = KE_IDX_GET(msg_id);

    /* First notify abstraction layer handlers */
    switch(task_id)
    {
        case TASK_ID_GAPC:
            GAPC_MsgHandler(msg_id, param, dest_id, src_id);
            break;
        case TASK_ID_GAPM:
            GAPM_MsgHandler(msg_id, param, dest_id, src_id);
            break;
        case TASK_ID_GATTC:
            GATTC_MsgHandler(msg_id, param, dest_id, src_id);
            break;
        case TASK_ID_GATTM:
            GATTM_MsgHandler(msg_id, param, dest_id, src_id);
            break;
#if RTE_BLE_L2CC_ENABLE
        case TASK_ID_L2CC:
			L2CC_MsgHandler(msg_id, param, dest_id, src_id);
			break;
#endif /* RTE_BLE_L2CC_ENABLE */
    }


    /* Notify subscribed application/profile handlers */
	for (int var = 0; var < 64; ++var) {
        /* If message ID matches or the handler should be called for all
         * messages of this task type */
		if(MsgHandler_field_used(var) && (msgHandlers[var].msg_id == msg_id || msgHandlers[var].msg_id == task_id)){
			msgHandlers[var].callback(msg_id, param, dest_id, src_id);
		}
	}
    return KE_MSG_CONSUMED;
}
