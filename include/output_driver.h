

#ifndef OUTPUT_DRIVER_H
#define OUTPUT_DRIVER_H

/* ----------------------------------------------------------------------------
 * If building with a C++ compiler, make all of the definitions in this header
 * have a C binding.
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
extern "C"
{
#endif    /* ifdef __cplusplus */

/* ----------------------------------------------------------------------------
 * Include files
 * --------------------------------------------------------------------------*/
#include <rsl10.h>

#define DECIMATE_BY(n)        \
 (((((n) - 64) / 8) << AUDIO_CFG_DEC_RATE_Pos) & AUDIO_CFG_DEC_RATE_Mask)



/* ----------------------------------------------------------------------------
 * Defines
 * --------------------------------------------------------------------------*/
#define OD_P_DIO                        11
#define OD_N_DIO                        10
#define AUDIO_DMIC0_GAIN                0x800
#define AUDIO_DMIC1_GAIN                0x800
#define AUDIO_OD_GAIN                   0x800
#define DMIC_CLK_DIO                    15
#define DMIC_DATA_DIO                   14
#define AUDIO_CONFIG                    (OD_AUDIOCLK                | \
                                         DMIC_AUDIOCLK              | \
                                         DECIMATE_BY(200)           | \
                                         OD_UNDERRUN_PROTECT_ENABLE | \
                                         OD_DATA_LSB_ALIGNED        | \
                                         DMIC0_DATA_MSB_ALIGNED     | \
                                         DMIC1_DATA_MSB_ALIGNED     | \
                                         OD_DMA_REQ_ENABLE         | \
                                         DMIC0_DMA_REQ_DISABLE      | \
                                         DMIC1_DMA_REQ_DISABLE      | \
                                         OD_INT_GEN_DISABLE         | \
                                         DMIC0_INT_GEN_ENABLE       | \
                                         DMIC1_INT_GEN_DISABLE      | \
                                         OD_ENABLE                 | \
                                         DMIC0_DISABLE               | \
                                         DMIC1_DISABLE)


/* ----------------------------------------------------------------------------
 * Function Prototypes
 * --------------------------------------------------------------------------*/
void od_initiaize(void);
void od_enable(void);
void od_disable(void);

/* ----------------------------------------------------------------------------
 * Close the 'extern "C"' block
 * ------------------------------------------------------------------------- */
#ifdef __cplusplus
}
#endif    /* ifdef __cplusplus */

#endif    /* PCM_H */
