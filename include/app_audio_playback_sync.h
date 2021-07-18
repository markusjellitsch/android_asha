

#ifndef APP_AUDIO_PLAYBACK_SNYC_H_
#define APP_AUDIO_PLAYBACK_SNYC_H_

#include <stdbool.h>
#include <stdint.h>
#include <app_audio.h>

#define RFREG_BASE                      0x40010000
#define IRQ_CONF                        0x0D
#define IRQ_STATUS                      0xD8
#define RF_REG_WRITE(addr, value)       (*(volatile uint8_t *)(RFREG_BASE + addr)) = (value)
#define RF_REG_READ(addr)               (*(volatile uint8_t *)(RFREG_BASE + addr))


#define APS_NUM_SYNCS 			3

typedef struct __attribute__((packed))
{
	uint32_t t_play100;
	uint16_t audio_sink_cnt;
	uint16_t audio_period_cnt;
	uint16_t audio_phase_cnt;
	uint16_t od_cnt;
	uint8_t seq_num;
}asha_sync_info_t;

typedef struct
{
	asha_sync_info_t list[APS_NUM_SYNCS];
}asha_sync_info_list_t;



bool app_aps_init(void);
bool app_aps_start_binaural_link(void);
bool app_aps_compute_sync_info(void);
bool app_aps_start_tx_rx_sync_capture(void);
bool app_aps_calculate_left_right_offset(asha_sync_info_list_t remote_sync_info,uint8_t num_entries);
uint8_t *app_aps_get_sync_info(void);

#endif    /* APP_AUDIO_H_ */
