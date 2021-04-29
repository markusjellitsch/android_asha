
#include <rsl10.h>
#include <output_driver.h>


void od_initiaize(void)
{

    /* set audio clock to 4 MHz */
    Sys_Clocks_SystemClkPrescale1(PWM0CLK_PRESCALE_1 | PWM1CLK_PRESCALE_1 |
                                     UARTCLK_PRESCALE_1 | AUDIOCLK_PRESCALE_4 |
                                     AUDIOSLOWCLK_PRESCALE_2);

    Sys_Audio_Set_Config(AUDIO_CONFIG);

    /* Configure DMIC parameters */
    //Sys_Audio_Set_DMICConfig(DMIC0_DCRM_CUTOFF_20HZ | DMIC1_DCRM_CUTOFF_20HZ |
     //                        DMIC1_DELAY_DISABLE | DMIC0_FALLING_EDGE |
     //                        DMIC1_RISING_EDGE, 0);


    /* Configure OD parameters */
    Sys_Audio_Set_ODConfig(DCRM_CUTOFF_10HZ | DITHER_ENABLE | OD_RISING_EDGE);

    AUDIO->OD_GAIN    = AUDIO_OD_GAIN;
    //AUDIO->DMIC0_GAIN = AUDIO_DMIC0_GAIN;
    //AUDIO->DMIC1_GAIN = AUDIO_DMIC1_GAIN;

     //DIO_JTAG_SW_PAD_CFG->CM3_JTAG_DATA_EN_ALIAS = 0;
     //Sys_Audio_DMICDIOConfig(DIO_6X_DRIVE | DIO_LPF_DISABLE | DIO_NO_PULL,
     //                         DMIC_CLK_DIO, DMIC_DATA_DIO, DIO_MODE_AUDIOCLK);


    /* Configure DIO used for OD */
    Sys_Audio_ODDIOConfig(DIO_6X_DRIVE | DIO_LPF_DISABLE | DIO_NO_PULL,
                          OD_P_DIO, OD_N_DIO);
}


void od_enable(void)
{
    AUDIO_CFG->OD_ENABLE_ALIAS |= OD_ENABLE_BITBAND;
}
void od_disable(void)
{
    AUDIO->OD_DATA = 0x0000;
    AUDIO_CFG->OD_ENABLE_ALIAS = OD_DISABLE_BITBAND;
}
