#ifndef _G722_PLC_DSP_DM_HI_H_
#define _G722_PLC_DSP_DM_HI_H_ 1

/* Auto generated file
 */

#include <stdint.h>

#ifndef _MEMORY_DESCRIPTION_
#define _MEMORY_DESCRIPTION_ 1

typedef struct
{
    void        *buffer;
    uint32_t fileSize;
    uint32_t memSize;
    uint32_t vAddress;
} memoryDescription;

#endif    /* ifndef _MEMORY_DESCRIPTION_ */

extern memoryDescription __attribute__ ((section(".dsp"))) g722_plc_dsp_DM_Hi_SegmentList[];

#endif    /* ifndef _G722_PLC_DSP_DM_HI_H_ */
