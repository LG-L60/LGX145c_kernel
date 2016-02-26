/*******************************************************************************
 *
 * Filename:
 * ---------
 * audio_volume_custom_default.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 * This file is the header of audio customization related parameters or definition.
 *
 * Author:
 * -------
 * Chipeng chang
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 *
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#ifndef AUDIO_VOLUME_CUSTOM_DEFAULT_H
#define AUDIO_VOLUME_CUSTOM_DEFAULT_H

//<2014-03-22-35172-PengXu,[5503][Audio]update audio parameters for lo2
#define AUD_VOLUME_RING \
32, 48, 64, 80, 96, 112, 128,     \
32, 48, 64, 80, 96, 112, 128,     \
84, 88, 92, 96, 108, 120, 132

#define AUD_VOLUME_KEY \
112, 136, 160, 184, 208, 232, 255,     \
112, 136, 160, 184, 208, 232, 255,     \
112, 136, 160, 184, 208, 232, 255

#define AUD_VOLUME_MIC \
0, 0, 180, 152, 180, 150, 0,     \
0, 0, 192, 192, 192, 150, 0,     \
0, 0, 208, 168, 255, 0, 0

#define AUD_VOLUME_FMR \
32, 48, 64, 80, 96, 112, 128,     \
88, 100, 112, 124, 136, 148, 160,     \
68, 80, 92, 104, 116, 128, 140

#define AUD_VOLUME_SPH \
60, 76, 92, 108, 120, 136, 152,     \
64, 76, 88, 100, 112, 124, 136,     \
56, 72, 88, 104, 120, 136, 152

#define AUD_VOLUME_SID \
0, 0, 0, 240, 0, 0, 0,     \
0, 0, 32, 0, 0, 0, 0,     \
0, 0, 0, 0, 0, 0, 0

#define AUD_VOLUME_MEDIA \
32, 48, 64, 80, 96, 112, 160,     \
84, 96, 108, 120, 132, 144, 156,     \
60, 72, 84, 96, 108, 120, 132

#define AUD_VOLUME_MATV \
32, 48, 64, 80, 96, 112, 128,     \
88, 100, 112, 124, 136, 148, 160,     \
68, 80, 92, 104, 116, 128, 140

#endif
