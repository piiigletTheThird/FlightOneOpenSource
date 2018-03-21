#include "includes.h"


// use variable record but instead of storing address of variable, store offset based on address of field, that way it works with the record loaded from file

extern volatile int logMe;
volatile uint32_t rfCustomReplyBufferPointer = 0;
volatile uint32_t rfCustomReplyBufferPointerSent = 0;
volatile uint32_t configSize = 0;

volatile int headerWritten;
volatile int headerToWrite;
volatile int julian = -99;

#define LARGE_RF_BUFFER_SIZE 9000
main_config mainConfig;
uint32_t resetBoard = 0;
volatile int activeProfile = PROFILE1;
char rf_custom_out_buffer[RF_BUFFER_SIZE];
char rfCustomSendBufferAdder[RF_BUFFER_SIZE];
char rfCustomSendBuffer[LARGE_RF_BUFFER_SIZE];
unsigned char rfReplyBuffer[HID_EPIN_SIZE];

static int  SetVariable(char *inString);
static int  ValidateConfig(uint32_t addresConfigStart);
static void SetValueOrString(int position, char *value);
static void SetValue(int position, char *value);
static void DlflStatusDump(void);
static int  GetValueFromString(char *string, const string_comp_rec thisStringCompTable[], uint32_t sizeOfArray);
static void DoIdleStop(void);

const string_comp_rec vtxStringCompTable[] = {
		//telemetry.h.h
		{"a1", VTX_CH_A1 },
		{"a2", VTX_CH_A2 },
		{"a3", VTX_CH_A3 },
		{"a4", VTX_CH_A4 },
		{"a5", VTX_CH_A5 },
		{"a6", VTX_CH_A6 },
		{"a7", VTX_CH_A7 },
		{"a8", VTX_CH_A8 },

		{"b1", VTX_CH_B1 },
		{"b2", VTX_CH_B2 },
		{"b3", VTX_CH_B3 },
		{"b4", VTX_CH_B4 },
		{"b5", VTX_CH_B5 },
		{"b6", VTX_CH_B6 },
		{"b7", VTX_CH_B7 },
		{"b8", VTX_CH_B8 },

		{"e1", VTX_CH_E1 },
		{"e2", VTX_CH_E2 },
		{"e3", VTX_CH_E3 },
		{"e4", VTX_CH_E4 },
		{"e5", VTX_CH_E5 },
		{"e6", VTX_CH_E6 },
		{"e7", VTX_CH_E7 },
		{"e8", VTX_CH_E8 },

		{"f1", VTX_CH_F1 },
		{"f2", VTX_CH_F2 },
		{"f3", VTX_CH_F3 },
		{"f4", VTX_CH_F4 },
		{"f5", VTX_CH_F5 },
		{"f6", VTX_CH_F6 },
		{"f7", VTX_CH_F7 },
		{"f8", VTX_CH_F8 },

		{"r1", VTX_CH_R1 },
		{"r2", VTX_CH_R2 },
		{"r3", VTX_CH_R3 },
		{"r4", VTX_CH_R4 },
		{"r5", VTX_CH_R5 },
		{"r6", VTX_CH_R6 },
		{"r7", VTX_CH_R7 },
		{"r8", VTX_CH_R8 },

};

const string_comp_rec stringCompTable[] =
{
		//mixer.h
		{"ESC_MULTISHOT",    ESC_MULTISHOT },
		{"ESC_ONESHOT",      ESC_ONESHOT },
		{"ESC_PWM",          ESC_PWM },
		{"ESC_ONESHOT42",    ESC_ONESHOT42 },
		{"ESC_DSHOT600",     ESC_DSHOT600 },
		{"ESC_DSHOT300",     ESC_DSHOT300 },
		{"ESC_DSHOT150",     ESC_DSHOT150 },
		{"ESC_MULTISHOT25",  ESC_MULTISHOT25 },
		{"ESC_MULTISHOT125", ESC_MULTISHOT125 },
		{"ESC_DDSHOT",       ESC_DDSHOT },
//		{"ESC_MEGAVOLT",     ESC_MEGAVOLT },

		//mixer.h
		{"MIXER_X1234",     MIXER_X1234 },
		{"MIXER_X1234RY",   MIXER_X1234RY },
		{"MIXER_X1234_3D",  MIXER_X1234_3D },
		{"MIXER_X4213",     MIXER_X4213 },
		{"MIXER_X4213RY",   MIXER_X4213RY },
		{"MIXER_X4213_3D",  MIXER_X4213_3D },
		{"MIXER_CUSTOM",    MIXER_CUSTOM },

		//gyro.h
		{"LOOP_L1",   LOOP_L1   },
		{"LOOP_M1",   LOOP_M1   },
		{"LOOP_M2",   LOOP_M2   },
		{"LOOP_M4",   LOOP_M4   },
		{"LOOP_M8",   LOOP_M8   },
		{"LOOP_H16",  LOOP_H16  },
		{"LOOP_H1",   LOOP_H1   },
		{"LOOP_H2",   LOOP_H2   },
		{"LOOP_H4",   LOOP_H4   },
		{"LOOP_H8",   LOOP_H8   },
		{"LOOP_H32",  LOOP_H32  },
		{"LOOP_UH16", LOOP_UH16 },
		{"LOOP_UH1",  LOOP_UH1  },
		{"LOOP_UH2",  LOOP_UH2  },
		{"LOOP_UH4",  LOOP_UH4  },
		{"LOOP_UH8",  LOOP_UH8  },
		{"LOOP_UH32", LOOP_UH32 },

		//rx.h
		{"NO_EXPO",      NO_EXPO      },
		{"SKITZO_EXPO",  SKITZO_EXPO  },
		{"TARANIS_EXPO", TARANIS_EXPO },
		{"FAST_EXPO",    FAST_EXPO    },
		{"ACRO_PLUS",    ACRO_PLUS    },
		{"KISS_EXPO",    KISS_EXPO    },
		{"NO_EXPO",      NO_EXPO      },

		//rx.h
//		{"USING_MANUAL",           USING_MANUAL},
		{"USING_SPEK_R",           USING_SPEK_R},
		{"USING_SPEK_T",           USING_SPEK_T},
		{"USING_SBUS_R",           USING_SBUS_R},
		{"USING_SBUS_T",           USING_SBUS_T},
		{"USING_SUMD_R",           USING_SUMD_R},
		{"USING_SUMD_T",           USING_SUMD_T},
		{"USING_IBUS_R",           USING_IBUS_R},
		{"USING_IBUS_T",           USING_IBUS_T},
		{"USING_CPPM_R",           USING_CPPM_R},
		{"USING_CPPM_T",           USING_CPPM_T},
		{"USING_CRSF_R",           USING_CRSF_R},
		{"USING_CRSF_T",           USING_CRSF_T},
		{"USING_CRSF_B",           USING_CRSF_B},
	};

const config_variables_rec valueTable[] =
{

		{ "mixer_type", 		typeUINT,  "mixr", &mainConfig.mixerConfig.mixerType,					0, MIXER_END, MIXER_X1234, "" },

		{ "famx", 				typeUINT,  "mixr", &mainConfig.mixerConfig.foreAftMixerFixer,			0, 110, 0, "" },
		{ "bounce_guard", 		typeFLOAT, "mixr", &mainConfig.mixerConfig.bounceGuard,					0, 0.2f, 0.1f, "" },
		{ "mixer_style", 		typeUINT,  "mixr", &mainConfig.mixerConfig.mixerStyle,					0, 1, 0, "" },
		{ "esc_protocol", 		typeUINT,  "mixr", &mainConfig.mixerConfig.escProtocol,					0, ESC_PROTOCOL_END, ESC_MULTISHOT, "" },
		{ "bit_reverse_esc_1",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[0],			0, 1, -1, "" },
		{ "bit_reverse_esc_2",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[1],			0, 1, -1, "" },
		{ "bit_reverse_esc_3",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[2],			0, 1, -1, "" },
		{ "bit_reverse_esc_4",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[3],			0, 1, -1, "" },
		{ "bit_reverse_esc_5",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[4],			0, 1, -1, "" },
		{ "bit_reverse_esc_6",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[5],			0, 1, -1, "" },
		{ "bit_reverse_esc_7",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[6],			0, 1, -1, "" },
		{ "bit_reverse_esc_8",	typeINT,   "mixr", &mainConfig.mixerConfig.bitReverseEsc[7],			0, 1, -1, "" },
		{ "esc_frequency", 		typeUINT,  "mixr", &mainConfig.mixerConfig.escUpdateFrequency,			0, 32000, 32000, "" },
		{ "idle_percent", 		typeFLOAT, "mixr", &mainConfig.mixerConfig.idlePercent,					0, 15.0, 6, "" },
		{ "idle_percent_inv",	typeFLOAT, "mixr", &mainConfig.mixerConfig.idlePercentInverted,			0, 15.0, 8, "" },
		{ "spin_rec_str",		typeFLOAT, "mixr", &mainConfig.mixerConfig.spinRecoveryStrength,		0, 1500.0, 750.0, "" },
		{ "quopa_style",		typeUINT,  "mixr", &mainConfig.mixerConfig.quopaStyle,					0, 5, 0, "" },
		
		{ "mout1", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[0],				0, 7, 0, "" },
		{ "mout2", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[1],				0, 7, 1, "" },
		{ "mout3", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[2],				0, 7, 2, "" },
		{ "mout4", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[3],				0, 7, 3, "" },
		{ "mout5", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[4],				0, 7, 4, "" },
		{ "mout6", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[5],				0, 7, 5, "" },
		{ "mout7", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[6],				0, 7, 6, "" },
		{ "mout8", 				typeUINT,  "mixr", &mainConfig.mixerConfig.motorOutput[7],				0, 7, 7, "" },

		{ "led_count",	 		typeUINT,  "leds", &mainConfig.ledConfig.ledCount,						2, WS2812_MAX_LEDS, 16, "" },
		{ "led_red",	 		typeUINT,  "leds", &mainConfig.ledConfig.ledRed,						0, 255, 10, "" },
		{ "led_green",	 		typeUINT,  "leds", &mainConfig.ledConfig.ledGreen,						0, 255, 0, "" },
		{ "led_blue",	 		typeUINT,  "leds", &mainConfig.ledConfig.ledBlue,						0, 255, 0, "" },
		{ "led_mode",	 		typeUINT,  "leds", &mainConfig.ledConfig.ledMode,						0, 255, 0, "" },
		{ "led_with_usb",	 	typeUINT,  "leds", &mainConfig.ledConfig.ledOnWithUsb,					0, 1, 0, "" },

		{ "telem_smartaudio",	typeUINT,  "telm", &mainConfig.telemConfig.telemSmartAudio,				0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "telem_sport",		typeUINT,  "telm", &mainConfig.telemConfig.telemSport,					0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "telem_spek",	 		typeUINT,  "telm", &mainConfig.telemConfig.telemSpek,					0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "telem_msp",	 		typeUINT,  "telm", &mainConfig.telemConfig.telemMsp,					0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "telem_rfosd",	 	typeUINT,  "telm", &mainConfig.telemConfig.telemRfOsd,					0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "telem_tramp",	 	typeUINT,  "telm", &mainConfig.telemConfig.telemTramp,					0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "telem_crsf",		 	typeUINT,  "telm", &mainConfig.telemConfig.telemCrsf,					0, TELEM_NUM-1, TELEM_OFF, "" },
		
		{ "telem_mavlink", 		typeUINT,  "telm", &mainConfig.telemConfig.telemMav,					0, TELEM_NUM-1, TELEM_OFF, "" },
		{ "adc_current_factor", typeFLOAT, "telm", &mainConfig.telemConfig.adcCurrFactor,				0, 60.0, 34.2, "" },
		{ "vtx_pitmode_type",	typeUINT,  "telm", &mainConfig.telemConfig.vtxPitmodeType,				0, 1, 0, "" },
		{ "vbat_buzzer",		typeUINT,  "telm", &mainConfig.telemConfig.vbatbuzzer,					0, 1, 1, "" },
		{ "vbat_cutoff",		typeFLOAT, "telm", &mainConfig.telemConfig.vbatCutoff,					0, 4.0f, 3.8f, "" },
		{ "bat_size",			typeUINT,  "telm", &mainConfig.telemConfig.batSize,						0, 50000, 1300, "" },
		{ "crsf_otx_cur_hack",	typeUINT,  "telm", &mainConfig.telemConfig.crsfOtxCurHack,				0, 1, 1, "" },
		
		{ "gyro_rotation", 		typeUINT,  "gyro", &mainConfig.gyroConfig.gyroRotation,					0, CW315_INV, CW0, "" },
		{ "board_calibrated", 	typeUINT,  "gyro", &mainConfig.gyroConfig.boardCalibrated,				0, 1,  0, "" },
		{ "man_gy_roll_angle",  typeFLOAT, "gyro", &mainConfig.gyroConfig.minorBoardRotation[X],		-360.0f, 360.0f, 0, "" },
		{ "man_gy_pitch_angle", typeFLOAT, "gyro", &mainConfig.gyroConfig.minorBoardRotation[Y],		-360.0f, 360.0f, 0, "" },
		{ "man_gy_yaw_angle", 	typeFLOAT, "gyro", &mainConfig.gyroConfig.minorBoardRotation[Z], 		-360.0f, 360.0f, 0, "" },



		{ "rf_loop_ctrl", 		typeUINT,   "gyro", &mainConfig.gyroConfig.loopCtrl, 						0, LOOP_UH32, LOOP_UH32, "" },
		{ "drunk", 				typeINT,    "gyro", &mainConfig.gyroConfig.drunk, 							0, 2, 1, "" },
		{ "skunk", 				typeINT,    "gyro", &mainConfig.gyroConfig.skunk, 							0, 2, 2, "" },
 
		{ "craft_name", 		typeSTRING, "rate", &mainConfig.craftName,								 	0, 0, 0, "RF1FTW\0" },
		{ "pname1", 			typeSTRING, "rate", &mainConfig.tuneProfile[0].profileName,				 	0, 0, 0, "Profile1\0" },
		{ "stick_curve1", 		typeINT,    "rate", &mainConfig.tuneProfile[0].rcRates.useCurve, 			0, EXPO_CURVE_END, ACRO_PLUS, "" },
 		{ "rc_smoothing1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.rcSmoothingFactor,	0.0, 4.0, 1.0, "" },
 		{ "pitch_rate1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.rates[PITCH],		0, 1500, 400, "" },
 		{ "roll_rate1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.rates[ROLL],			0, 1500, 400, "" },
 		{ "yaw_rate1", 			typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.rates[YAW],			0, 1500, 400, "" },
 		{ "pitch_acrop1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.acroPlus[PITCH],		0, 300, 140, "" },
 		{ "roll_acrop1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.acroPlus[ROLL],		0, 300, 140, "" },
 		{ "yaw_acrop1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.acroPlus[YAW],		0, 300, 140, "" },
 		{ "pitch_expo1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.curveExpo[PITCH],	0, 100, 50, "" },
 		{ "roll_expo1", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.curveExpo[ROLL],		0, 100, 50, "" },
 		{ "yaw_expo1", 			typeFLOAT,  "rate", &mainConfig.tuneProfile[0].rcRates.curveExpo[YAW],		0, 100, 50, "" },
 
 		{ "yaw_wc1", 			typeUINT,   "pids", &mainConfig.tuneProfile[0].pidConfig[YAW].wc, 			0, 30, 0, "" },
 		{ "roll_wc1", 			typeUINT,   "pids", &mainConfig.tuneProfile[0].pidConfig[ROLL].wc, 			0, 30, 0, "" },
 		{ "pitch_wc1", 			typeUINT,   "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].wc, 		0, 30, 0, "" },
 		{ "yaw_kp1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[YAW].kp, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_kp1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[ROLL].kp, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_kp1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].kp, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "yaw_ki1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[YAW].ki, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_ki1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[ROLL].ki, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_ki1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].ki, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "yaw_kd1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[YAW].kd, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_kd1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[ROLL].kd, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_kd1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].kd, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "kd_limit1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[0].kdLimit, 		0.1, 1.0, 0.35, "" },
 		{ "ki_limit1", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[0].kiLimit, 		0.1, 1.0, 0.15, "" },
 		{ "slp1", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].slp, 		0, 25.0, 05.0, "" },
 		{ "sli1", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].sli, 		0, 25.0, 00.1, "" },
 		{ "sla1", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].sla, 		0, 75.0, 35.0, "" },
 		{ "sld1", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[0].pidConfig[PITCH].sld, 		0, 0.90, 0.03, "" },
 
		{ "res_redux1",			typeINT,   "filt", &mainConfig.tuneProfile[0].filterConfig[0].resRedux,		0, 1, 0, "" },
 		{ "yaw_rap1", 			typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].gyro.r, 	0, 1000, 0.6f, "" },
 		{ "roll_rap1", 			typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ROLL].gyro.r, 	0, 1000, 1.2f, "" },
 		{ "pitch_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[PITCH].gyro.r, 	0, 1000, 1.2f, "" },
 		{ "yaw_ga1", 			typeINT,   "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].ga, 		0, 31, 0, "" },
 		{ "roll_ga1", 			typeINT,   "filt", &mainConfig.tuneProfile[0].filterConfig[ROLL].ga, 		0, 31, 0, "" },
 		{ "pitch_ga1", 			typeINT,   "filt", &mainConfig.tuneProfile[0].filterConfig[PITCH].ga, 		0, 31, 0, "" },
 		{ "filter_type1",		typeINT,   "filt", &mainConfig.tuneProfile[0].filterConfig[0].filterType, 	0, 1, 1, "" },
 		{ "yaw_quick1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].gyro.q, 	0, 10000, 52.000, "" },
 		{ "roll_quick1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ROLL].gyro.q, 	0, 10000, 110.000, "" },
 		{ "pitch_quick1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[PITCH].gyro.q, 	0, 10000, 110.000, "" },
 		{ "yaw_kd_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].kd.r, 		0, 100, 55.000, "" },
 		{ "roll_kd_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ROLL].kd.r, 		0, 100, 55.000, "" },
 		{ "pitch_kd_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[PITCH].kd.r, 	0, 100, 55.000, "" },
 		{ "x_vector_quick1", 	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ACCX].acc.q, 	0, 10, 2.0000, "" },
 		{ "x_vector_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ACCX].acc.r, 	0, 10, 025.00, "" },
 		{ "y_vector_quick1", 	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ACCY].acc.q, 	0, 10, 2.0000, "" },
 		{ "y_vector_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ACCY].acc.r, 	0, 10, 025.00, "" },
 		{ "z_vector_quick1", 	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ACCZ].acc.q, 	0, 10, 2.0000, "" },
 		{ "z_vector_rap1", 		typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ACCZ].acc.r, 	0, 10, 025.00, "" },
 		{ "tpa_kp_curve_type1",	typeINT,   "mixr", &mainConfig.tuneProfile[0].filterConfig[0].tpaKpCurveType,				0, 1, 0, "" },
		{ "tpa_ki_curve_type1",	typeINT,   "mixr", &mainConfig.tuneProfile[0].filterConfig[0].tpaKiCurveType,				0, 1, 0, "" },
		{ "tpa_kd_curve_type1",	typeINT,   "mixr", &mainConfig.tuneProfile[0].filterConfig[0].tpaKdCurveType,				0, 1, 0, "" },
 
 		{ "pname2", 			typeSTRING, "rate", &mainConfig.tuneProfile[1].profileName,				 	0, 0, 0, "Profile2\0" },
 		{ "stick_curve2", 		typeINT,    "rate", &mainConfig.tuneProfile[1].rcRates.useCurve, 			0, EXPO_CURVE_END, ACRO_PLUS, "" },
 		{ "rc_smoothing2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.rcSmoothingFactor,	0.0, 4.0, 1.0, "" },
 		{ "pitch_rate2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.rates[PITCH],		0, 1500, 400, "" },
 		{ "roll_rate2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.rates[ROLL],			0, 1500, 400, "" },
 		{ "yaw_rate2", 			typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.rates[YAW],			0, 1500, 400, "" },
 		{ "pitch_acrop2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.acroPlus[PITCH],		0, 300, 140, "" },
 		{ "roll_acrop2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.acroPlus[ROLL],		0, 300, 140, "" },
 		{ "yaw_acrop2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.acroPlus[YAW],		0, 300, 140, "" },
 		{ "pitch_expo2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.curveExpo[PITCH],	0, 100, 50, "" },
 		{ "roll_expo2", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.curveExpo[ROLL],		0, 100, 50, "" },
 		{ "yaw_expo2", 			typeFLOAT,  "rate", &mainConfig.tuneProfile[1].rcRates.curveExpo[YAW],		0, 100, 50, "" },
 
		{ "yaw_wc2", 			typeUINT,   "pids", &mainConfig.tuneProfile[1].pidConfig[YAW].wc, 			0, 30, 0, "" },
 		{ "roll_wc2", 			typeUINT,   "pids", &mainConfig.tuneProfile[1].pidConfig[ROLL].wc, 			0, 30, 0, "" },
 		{ "pitch_wc2", 			typeUINT,   "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].wc, 		0, 30, 0, "" },
 		{ "yaw_kp2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[YAW].kp, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_kp2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[ROLL].kp, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_kp2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].kp, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "yaw_ki2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[YAW].ki, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_ki2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[ROLL].ki, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_ki2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].ki, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "yaw_kd2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[YAW].kd, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_kd2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[ROLL].kd, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_kd2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].kd, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "kd_limit2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[0].kdLimit, 		0.1, 1.0, 0.35, "" },
 		{ "ki_limit2", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[0].kiLimit, 		0.1, 1.0, 0.15, "" },
 		{ "slp2", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].slp, 		0, 25.0, 05.0, "" },
 		{ "sli2", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].sli, 		0, 25.0, 00.1, "" },
 		{ "sla2", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].sla, 		0, 75.0, 35.0, "" },
 		{ "sld2", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[1].pidConfig[PITCH].sld, 		0, 0.90, 0.03, "" },
 
		{ "res_redux2",			typeINT,   "filt", &mainConfig.tuneProfile[1].filterConfig[0].resRedux,		0, 1, 0, "" },
 		{ "yaw_rap2", 			typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[YAW].gyro.r, 	0, 1000, 0.6f, "" },
 		{ "roll_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ROLL].gyro.r, 	0, 1000, 1.2f, "" },
 		{ "pitch_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[PITCH].gyro.r, 	0, 1000, 1.2f, "" },
 		{ "yaw_ga2", 			typeINT,   "filt", &mainConfig.tuneProfile[1].filterConfig[YAW].ga, 		0, 31, 0, "" },
 		{ "roll_ga2", 			typeINT,   "filt", &mainConfig.tuneProfile[1].filterConfig[ROLL].ga, 		0, 31, 0, "" },
 		{ "pitch_ga2", 			typeINT,   "filt", &mainConfig.tuneProfile[1].filterConfig[PITCH].ga, 		0, 31, 0, "" },
 		{ "filter_type2",		typeINT,   "filt", &mainConfig.tuneProfile[1].filterConfig[0].filterType, 	0, 1, 1, "" },
 		{ "yaw_quick2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[YAW].gyro.q, 	0, 10000, 52.000, "" },
 		{ "roll_quick2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ROLL].gyro.q, 	0, 10000, 110.000, "" },
 		{ "pitch_quick2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[PITCH].gyro.q, 	0, 10000, 110.000, "" },
 		{ "yaw_kd_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[YAW].kd.r, 		0, 100, 55.000, "" },
 		{ "roll_kd_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ROLL].kd.r, 		0, 100, 55.000, "" },
 		{ "pitch_kd_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[PITCH].kd.r, 	0, 100, 55.000, "" },
 		{ "x_vector_quick2", 	typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ACCX].acc.q, 	0, 10, 2.0000, "" },
 		{ "x_vector_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ACCX].acc.r, 	0, 10, 025.00, "" },
 		{ "y_vector_quick2", 	typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ACCY].acc.q, 	0, 10, 2.0000, "" },
 		{ "y_vector_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ACCY].acc.r, 	0, 10, 025.00, "" },
 		{ "z_vector_quick2", 	typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ACCZ].acc.q, 	0, 10, 2.0000, "" },
 		{ "z_vector_rap2", 		typeFLOAT, "filt", &mainConfig.tuneProfile[1].filterConfig[ACCZ].acc.r, 	0, 10, 025.00, "" },
		{ "tpa_kp_curve_type2",	typeINT,   "mixr", &mainConfig.tuneProfile[1].filterConfig[0].tpaKpCurveType,				0, 1, 0, "" },
		{ "tpa_ki_curve_type2",	typeINT,   "mixr", &mainConfig.tuneProfile[1].filterConfig[0].tpaKiCurveType,				0, 1, 0, "" },
		{ "tpa_kd_curve_type2",	typeINT,   "mixr", &mainConfig.tuneProfile[1].filterConfig[0].tpaKdCurveType,				0, 1, 0, "" },
 
 
 		{ "pname3", 			typeSTRING, "rate", &mainConfig.tuneProfile[2].profileName,				 	0, 0, 0, "Profile3" },
 		{ "stick_curve3", 		typeINT,    "rate", &mainConfig.tuneProfile[2].rcRates.useCurve, 			0, EXPO_CURVE_END, ACRO_PLUS, "" },
 		{ "rc_smoothing3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.rcSmoothingFactor,	0.0, 4.0, 1.0, "" },
 		{ "pitch_rate3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.rates[PITCH],		0, 1500, 400, "" },
 		{ "roll_rate3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.rates[ROLL],			0, 1500, 400, "" },
 		{ "yaw_rate3", 			typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.rates[YAW],			0, 1500, 400, "" },
 		{ "pitch_acrop3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.acroPlus[PITCH],		0, 300, 140, "" },
 		{ "roll_acrop3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.acroPlus[ROLL],		0, 300, 140, "" },
 		{ "yaw_acrop3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.acroPlus[YAW],		0, 300, 140, "" },
 		{ "pitch_expo3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.curveExpo[PITCH],	0, 100, 50, "" },
 		{ "roll_expo3", 		typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.curveExpo[ROLL],		0, 100, 50, "" },
 		{ "yaw_expo3", 			typeFLOAT,  "rate", &mainConfig.tuneProfile[2].rcRates.curveExpo[YAW],		0, 100, 50, "" },
 
		{ "yaw_wc3", 			typeUINT,   "pids", &mainConfig.tuneProfile[2].pidConfig[YAW].wc, 			0, 30, 0, "" },
 		{ "roll_wc3", 			typeUINT,   "pids", &mainConfig.tuneProfile[2].pidConfig[ROLL].wc, 			0, 30, 0, "" },
 		{ "pitch_wc3", 			typeUINT,   "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].wc, 		0, 30, 0, "" },
 		{ "yaw_kp3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[YAW].kp, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_kp3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[ROLL].kp, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_kp3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].kp, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "yaw_ki3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[YAW].ki, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_ki3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[ROLL].ki, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_ki3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].ki, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "yaw_kd3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[YAW].kd, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "roll_kd3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[ROLL].kd, 			0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "pitch_kd3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].kd, 		0, 200, DEFAULT_PID_CONFIG_VALUE, "" },
 		{ "kd_limit3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[0].kdLimit, 		0.1, 1.0, 0.35, "" },
 		{ "ki_limit3", 			typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[0].kiLimit, 		0.1, 1.0, 0.15, "" },
 		{ "slp3", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].slp, 		0, 25.0, 05.0, "" },
 		{ "sli3", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].sli, 		0, 25.0, 00.1, "" },
 		{ "sla3", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].sla, 		0, 75.0, 35.0, "" },
 		{ "sld3", 				typeFLOAT,  "pids", &mainConfig.tuneProfile[2].pidConfig[PITCH].sld, 		0, 0.90, 0.03, "" },

		{ "res_redux3",			typeINT,   "filt", &mainConfig.tuneProfile[2].filterConfig[0].resRedux,		0, 1, 0, "" },
 		{ "yaw_rap3", 			typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[YAW].gyro.r, 	0, 1000, .6f, "" },
 		{ "roll_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ROLL].gyro.r, 	0, 1000, 1.2f, "" },
 		{ "pitch_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[PITCH].gyro.r, 	0, 1000, 1.2f, "" },
 		{ "yaw_ga3", 			typeINT,   "filt", &mainConfig.tuneProfile[2].filterConfig[YAW].ga, 		0, 31, 0, "" },
 		{ "roll_ga3", 			typeINT,   "filt", &mainConfig.tuneProfile[2].filterConfig[ROLL].ga, 		0, 31, 0, "" },
 		{ "pitch_ga3", 			typeINT,   "filt", &mainConfig.tuneProfile[2].filterConfig[PITCH].ga, 		0, 31, 0, "" },
 		{ "filter_type3",		typeINT,   "filt", &mainConfig.tuneProfile[2].filterConfig[0].filterType, 	0, 1, 1, "" },
 		{ "yaw_quick3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[YAW].gyro.q, 	0, 10000, 52.000, "" },
 		{ "roll_quick3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ROLL].gyro.q, 	0, 10000, 110.000, "" },
 		{ "pitch_quick3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[PITCH].gyro.q, 	0, 10000, 110.000, "" },
 		{ "yaw_kd_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[YAW].kd.r, 		0, 100, 55.000, "" },
 		{ "roll_kd_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ROLL].kd.r, 		0, 100, 55.000, "" },
 		{ "pitch_kd_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[PITCH].kd.r, 	0, 100, 55.000, "" },
 		{ "x_vector_quick3", 	typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ACCX].acc.q, 	0, 10, 2.0000, "" },
 		{ "x_vector_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ACCX].acc.r, 	0, 10, 025.00, "" },
 		{ "y_vector_quick3", 	typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ACCY].acc.q, 	0, 10, 2.0000, "" },
 		{ "y_vector_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ACCY].acc.r, 	0, 10, 025.00, "" },
 		{ "z_vector_quick3", 	typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ACCZ].acc.q, 	0, 10, 2.0000, "" },
 		{ "z_vector_rap3", 		typeFLOAT, "filt", &mainConfig.tuneProfile[2].filterConfig[ACCZ].acc.r, 	0, 10, 025.00, "" },
		{ "tpa_kp_curve_type3",	typeINT,   "mixr", &mainConfig.tuneProfile[2].filterConfig[0].tpaKpCurveType,				0, 1, 0, "" },
		{ "tpa_ki_curve_type3",	typeINT,   "mixr", &mainConfig.tuneProfile[2].filterConfig[0].tpaKiCurveType,				0, 1, 0, "" },
		{ "tpa_kd_curve_type3",	typeINT,   "mixr", &mainConfig.tuneProfile[2].filterConfig[0].tpaKdCurveType,				0, 1, 0, "" },
 

#ifdef STM32F446xx	//TODO remove target specific ifdefs
		{ "rx_protocol",		typeUINT, "rccf", &mainConfig.rcControlsConfig.rxProtcol,				0, USING_RX_END - 1, USING_SPEK_T, "" },
		{ "rx_usart",			typeUINT, "rccf", &mainConfig.rcControlsConfig.rxUsart,					0, MAX_USARTS - 1, ENUM_USART5, "" },
#else
		{ "rx_protocol", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.rxProtcol, 				0, USING_RX_END-1, USING_SPEK_T, "" },
		{ "rx_usart", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.rxUsart, 				0, MAX_USARTS-1, ENUM_USART1, "" },
#endif

		{ "arm_method", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.armMethod,				0, 1, 1, "" },
		{ "rx_inv_direction", 	typeUINT,  "rccf", &mainConfig.rcControlsConfig.rxInvertDirection,		0, 2, 0, "" },

		{ "pitch_deadband", 	typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[PITCH], 		0, 0.1, 0.015, "" },
		{ "roll_deadband", 		typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[ROLL], 		0, 0.1, 0.015, "" },
		{ "yaw_deadband", 		typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[YAW], 			0, 0.1, 0.015, "" },
		{ "throttle_deadband", 	typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[THROTTLE], 	0, 0.1, 0, "" },
		{ "aux1_deadband", 		typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[AUX1], 		0, 0.1, 0, "" },
		{ "aux2_deadband", 		typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[AUX2], 		0, 0.1, 0, "" },
		{ "aux3_deadband", 		typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[AUX3], 		0, 0.1, 0, "" },
		{ "aux4_deadband", 		typeFLOAT, "rccf", &mainConfig.rcControlsConfig.deadBand[AUX4], 		0, 0.1, 0, "" },

		{ "pitch_midrc",		typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[PITCH], 			0, 21480, 1024, "" },
		{ "roll_midrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[ROLL], 			0, 21480, 1024, "" },
		{ "yaw_midrc", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[YAW], 			0, 21480, 1024, "" },
		{ "throttle_midrc", 	typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[THROTTLE],		0, 21480, 1024, "" },
		{ "aux1_midrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[AUX1], 			0, 21480, 1024, "" },
		{ "aux2_midrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[AUX2], 			0, 21480, 1024, "" },
		{ "aux3_midrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[AUX3], 			0, 21480, 1024, "" },
		{ "aux4_midrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.midRc[AUX4], 			0, 21480, 1024, "" },

		{ "pitch_minrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[PITCH], 			0, 21480, 0, "" },
		{ "roll_minrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[ROLL], 			0, 21480, 0, "" },
		{ "yaw_minrc", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[YAW], 			0, 21480, 0, "" },
		{ "throttle_minrc", 	typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[THROTTLE], 		0, 21480, 0, "" },
		{ "aux1_minrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[AUX1], 			0, 21480, 0, "" },
		{ "aux2_minrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[AUX2], 			0, 21480, 0, "" },
		{ "aux3_minrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[AUX3], 			0, 21480, 0, "" },
		{ "aux4_minrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.minRc[AUX4], 			0, 21480, 0, "" },

		{ "pitch_maxrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[PITCH], 			0, 21480, 2048, "" },
		{ "roll_maxrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[ROLL], 			0, 21480, 2048, "" },
		{ "yaw_maxrc", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[YAW], 			0, 21480, 2048, "" },
		{ "throttle_maxrc", 	typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[THROTTLE], 		0, 21480, 2048, "" },
		{ "aux1_maxrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[AUX1], 			0, 21480, 2048, "" },
		{ "aux2_maxrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[AUX2], 			0, 21480, 2048, "" },
		{ "aux3_maxrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[AUX3], 			0, 21480, 2048, "" },
		{ "aux4_maxrc", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.maxRc[AUX4], 			0, 21480, 2048, "" },

		{ "pitch_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[PITCH], 		0, 100, 2, "" },
		{ "roll_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[ROLL], 		0, 100, 1, "" },
		{ "yaw_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[YAW], 		0, 100, 3, "" },
		{ "throttle_map", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[THROTTLE],	0, 100, 0, "" },
		{ "aux1_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX1], 		0, 100, 4, "" },
		{ "aux2_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX2], 		0, 100, 5, "" },
		{ "aux3_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX3], 		0, 100, 6, "" },
		{ "aux4_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX4], 		0, 100, 7, "" },
		{ "aux5_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX5], 		0, 100, 100, "" },
		{ "aux6_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX6], 		0, 100, 100, "" },
		{ "aux7_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX7], 		0, 100, 100, "" },
		{ "aux8_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX8], 		0, 100, 100, "" },
		{ "aux9_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX9], 		0, 100, 100, "" },
		{ "aux10_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX10], 		0, 100, 100, "" },
		{ "aux11_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX11], 		0, 100, 100, "" },
		{ "aux12_map", 			typeUINT,  "rccf", &mainConfig.rcControlsConfig.channelMap[AUX12], 		0, 100, 100, "" },

		{ "rc_calibrated", 		typeUINT,  "rccf", &mainConfig.rcControlsConfig.rcCalibrated,			0, 1, 0, "" },

		{ "bind", 	            typeUINT,  "rccf", &mainConfig.rcControlsConfig.bind, 	                0, 32, 0, "" },
		{ "short_throw", 	    typeUINT,  "rccf", &mainConfig.rcControlsConfig.shortThrow, 	        0, 1, 1, "" },

/*		{ "omega0", 	    	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].omega0, 0, 1000, 1, "" },
		{ "omega1_yaw",     	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].omega1, 0, 1000, 30.0f, "" },
		{ "omega1_roll",     	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ROLL].omega1, 0, 1000, 40.0f, "" },
		{ "omega1_pitch",    	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[PITCH].omega1, 0, 1000, 40.0f, "" },
		{ "omega2_yaw",     	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[YAW].omega2, 0, 1000, 0.088f, "" },
		{ "omega2_roll",     	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[ROLL].omega2, 0, 1000, 0.088f, "" },
		{ "omega2_pitch",     	typeFLOAT, "filt", &mainConfig.tuneProfile[0].filterConfig[PITCH].omega2, 0, 1000, 0.088f, "" },
*/
		/*
		{ "omega_drive", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaDrive, 	     		    0, 4000000000, 0, "" },
		{ "omega_route", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaDrive, 	     		    0, 4000000000, 0, "" },
		{ "omega_in_low", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaInLow, 	     		    0, 4000000000, 0, "" },
		{ "omega_in_mid", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaInMid, 	     		    0, 4000000000, 0, "" },
		{ "omega_in_high", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaInHigh, 	     		0, 4000000000, 0, "" },
		{ "omega_out_low", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaOutLow, 	     		0, 4000000000, 0, "" },
		{ "omega_out_mid", 	    typeUINT,  "mixr", &mainConfig.mixerConfig.omegaOutMid, 	     		0, 4000000000, 0, "" },
		{ "omega_out_high", 	typeUINT,  "mixr", &mainConfig.mixerConfig.omegaOutHigh, 	     		0, 4000000000, 0, "" },
		{ "omega_rate", 		typeUINT,  "mixr", &mainConfig.mixerConfig.omegaRate, 	     		    0, 4000000000, 0, "" },
*/
};

void RemoveNull(char *string, uint32_t stringLength)
{
    for (uint32_t x=strlen(string);x<stringLength;x++)
    {
        string[x] = ' ';
    }

}


void LeftPadString(char *string, uint32_t stringLength, char *inString, uint32_t paddingLength, char inChar)
{
    char *c;

    RemoveNull(string, stringLength);

    if ((strlen(inString) + paddingLength) < stringLength) 
    {

        memset(string, inChar, stringLength);

    }
    c = string + paddingLength;

    sprintf(c, "%s", inString);
}


//Written by Preston Garrison - Released to Public Domain
char *newftoa(float x, char *floatString, uint32_t count)
{
    char st[10];
    char right[10];

    if (count>9)
        return(floatString);

	 
    sprintf(floatString, "%d.", (int) x);
	if (((int)x == 0) && (x < 0))
		sprintf(floatString, "-0.");

    sprintf(right, "%lu", (uint32_t)ABS(((x - (uint32_t) x) * powl(10, count))));
    LeftPadString(st, 10, right, (count-strlen(right)), '0');
    strcat(floatString, st);
    
    return(floatString);
}


char *ftoa(float x, char *floatString)
{
    newftoa(x, floatString, 3);
    return(floatString);
}

char *ftoa6(float x, char *floatString)
{
    newftoa(x, floatString, 6);
    return(floatString);
}


void ValidateConfigSettings(void)
{
	int x;
	//if one PID value is wrong we reset them all
	for (x=PROFILE_COUNT-1;x>=0;x--)
	{
		if (
			(mainConfig.tuneProfile[x].pidConfig[YAW].kp > 100.0f)   || (mainConfig.tuneProfile[x].pidConfig[YAW].kp < 0.0f)   ||
			(mainConfig.tuneProfile[x].pidConfig[YAW].ki > 100.0f)   || (mainConfig.tuneProfile[x].pidConfig[YAW].ki < 0.0f)   ||
			(mainConfig.tuneProfile[x].pidConfig[YAW].kd > 100.0f)   || (mainConfig.tuneProfile[x].pidConfig[YAW].kd < 0.0f)   ||
			(mainConfig.tuneProfile[x].pidConfig[ROLL].kp > 100.0f)  || (mainConfig.tuneProfile[x].pidConfig[ROLL].kp < 0.0f)  ||
			(mainConfig.tuneProfile[x].pidConfig[ROLL].ki > 100.0f)  || (mainConfig.tuneProfile[x].pidConfig[ROLL].ki < 0.0f)  ||
			(mainConfig.tuneProfile[x].pidConfig[ROLL].kd > 100.0f)  || (mainConfig.tuneProfile[x].pidConfig[ROLL].kd < 0.0f)  ||
			(mainConfig.tuneProfile[x].pidConfig[PITCH].kp > 100.0f) || (mainConfig.tuneProfile[x].pidConfig[PITCH].kp < 0.0f) ||
			(mainConfig.tuneProfile[x].pidConfig[PITCH].ki > 100.0f) || (mainConfig.tuneProfile[x].pidConfig[PITCH].ki < 0.0f) ||
			(mainConfig.tuneProfile[x].pidConfig[PITCH].kd > 100.0f) || (mainConfig.tuneProfile[x].pidConfig[PITCH].kd < 0.0f)
		)
		{
			mainConfig.tuneProfile[x].pidConfig[YAW].kp   = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[YAW].ki   = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[YAW].kd   = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[ROLL].kp  = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[ROLL].ki  = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[ROLL].kd  = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[PITCH].kp = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[PITCH].ki = DEFAULT_PID_CONFIG_VALUE;
			mainConfig.tuneProfile[x].pidConfig[PITCH].kd = DEFAULT_PID_CONFIG_VALUE;
		}
	}
	for (x=configSize-1;x>=0;x--)
	{
		switch(valueTable[x].type)
		{
			case typeUINT:
				if ( ( *(uint32_t *)valueTable[x].ptr < (uint32_t)valueTable[x].Min ) || ( (*(uint32_t *)valueTable[x].ptr) > (uint32_t)valueTable[x].Max ) )
					(*(uint32_t *)valueTable[x].ptr) = (uint32_t)valueTable[x].Default;
				break;
			case typeINT:
				if ( ( (*(int32_t *)valueTable[x].ptr) < (int32_t)valueTable[x].Min ) || ( (*(int32_t *)valueTable[x].ptr) > (int32_t)valueTable[x].Max ) )
					(*(int32_t *)valueTable[x].ptr) = (int32_t)valueTable[x].Default;
				break;
			case typeFLOAT:
				if ( ( (*(float *)valueTable[x].ptr) < (float)valueTable[x].Min ) || ( (*(float *)valueTable[x].ptr) > (float)valueTable[x].Max ) )
					(*(float *)valueTable[x].ptr) = (float)valueTable[x].Default;
				break;
		}
	}
}

static void DoIdleStop(void)
{
	ZeroActuators( 1000 );
	SKIP_GYRO=0;
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me idlestop\n" );
	RfCustomReplyBuffer(rf_custom_out_buffer);
	taskIdleActuators[0]=0;
	taskIdleActuators[1]=0;
	taskIdleActuators[2]=0;
	taskIdleActuators[3]=0;
	taskIdleActuators[4]=0;
	taskIdleActuators[5]=0;
	taskIdleActuators[6]=0;
	taskIdleActuators[7]=0;
	taskDshotActuators = 0;
}

void GenerateConfig(void)
{
	uint32_t  x;
	char     *c;
	bzero(&mainConfig, sizeof(mainConfig));
	ResetTpaCurves();
	for (x=0;x<configSize;x++)
	{
		switch(valueTable[x].type)
		{
		case typeUINT:
			*(uint32_t *)valueTable[x].ptr = (uint32_t)valueTable[x].Default;
			break;
		case typeINT:
			*(int32_t *)valueTable[x].ptr  = (int32_t)valueTable[x].Default;
			break;
		case typeFLOAT:
			*(float *)valueTable[x].ptr    = (float)valueTable[x].Default;
			break;
		case typeSTRING:
			c = valueTable[x].ptr;
			strncpy(c,valueTable[x].strDefault,15);
			c[15] = 0;
			break;
		}
	}
	//used to find craft name in msd app

	mainConfig.marker1 = 0x43525549;
	mainConfig.marker2 = 0x53455221;
	mainConfig.marker3 = 0x574F4F46;
	mainConfig.craftName[0] = 'R';
	mainConfig.craftName[1] = 'a';
	mainConfig.craftName[2] = 'c';
	mainConfig.craftName[3] = 'e';
	mainConfig.craftName[4] = 'r';
	snprintf(&mainConfig.craftName[5], 10, "%lu", (uint32_t)ABS(rand()) );
	mainConfig.craftName[15] = 0;
}

void SaveConfig(uint32_t addresConfigStart)
{

	uint32_t addressOffset;

	DisarmBoard();
	SKIP_GYRO=1;
	if (resetBoard) {

		//This function does a shotgun startup of the flight code.
		InitFlight(mainConfig.mixerConfig.escProtocol, mainConfig.mixerConfig.escUpdateFrequency);

	}

	mainConfig.version  = CONFIG_VERSION;
	mainConfig.size     = sizeof(main_config);
	mainConfig.czechsum = GetChecksum8((const uint8_t *) &mainConfig, sizeof(main_config));

	EraseFlash(addresConfigStart, addresConfigStart+sizeof(main_config));
	PrepareFlash();
	for (addressOffset = 0; addressOffset < sizeof(main_config); addressOffset += 4) {
		WriteFlash(*(uint32_t *) ((char *) &mainConfig + addressOffset), addresConfigStart+addressOffset );
	}
	FinishFlash();
	SKIP_GYRO=0;
}

static int ValidateConfig (uint32_t addresConfigStart)
{

	const main_config *temp = (main_config *) addresConfigStart; //ADDRESS_FLASH_START;
	uint8_t czechsum = 0;

	if (temp->size != sizeof(main_config))
	    return (0);

	if (CONFIG_VERSION != temp->version)
		return (0);

	czechsum = GetChecksum8((const uint8_t *) temp, sizeof(main_config));
	if (czechsum != 0)
		return (1);

	return (1);

}

int LoadConfig (uint32_t addresConfigStart)
{
	configSize = (sizeof(valueTable)/sizeof(config_variables_rec));
	if (ValidateConfig(addresConfigStart) ) {
		memcpy(&mainConfig, (char *) addresConfigStart, sizeof(main_config));
		ValidateConfigSettings();
	} else {
		GenerateConfig();
		SaveConfig(addresConfigStart);
	}
	
	return(0);
}




//cleanup string // strip continuous spaces, first space, and non allowed characters
char *StripSpaces(char *inString)
{
	uint16_t head = 0;
	uint16_t position = 0;
	uint8_t inQuote = 0;
	uint16_t inStringLength = strlen(inString);

	for (position = 0; position < inStringLength; position++)
	{
		if (inString[position] == '"')
			inQuote = inQuote ^ 1;

		if ((inQuote) || (inString[position] != ' '))
			inString[head++] = inString[position];
	}


	inString[head] = 0;

	return (inString);
}

char *CleanupNumberString(char *inString)
{
	int firstRun = 1;
	uint16_t head = 0;
	uint16_t position = 0;
	uint16_t inStringLength = strlen(inString);

	for (position = 0; position < inStringLength; position++)
	{
		if (inString[position] == ' ') // removes multiple spaces in a row
			continue;

		if (isdigit((unsigned char)inString[position]) || (unsigned char)inString[position] == '.' || (unsigned char)inString[position] == '-' )
		{
				//only allow a negative sign on first pass
			if (firstRun && (unsigned char)inString[position] == '-')
				inString[head++] = inString[position];
			else if ((unsigned char)inString[position] != '-')
				inString[head++] = inString[position];
			firstRun=0;
		}
	}


	inString[head] = 0;

	return (inString);
}

char *CleanupString(char *inString)
{
	char last_char = ' ';
	uint16_t head = 0;
	uint16_t position = 0;
	uint16_t inStringLength = strlen(inString);

	for (position = 0; position < inStringLength; position++)
	{
		if ((last_char == ' ') && (inString[position] == ' ')) // removes multiple spaces in a row
			continue;

		if (isalnum((unsigned char)inString[position]) || (inString[position] == ' ') || (inString[position] == '=') || (inString[position] == '"') || (inString[position] == '.') || (inString[position] == '-') || (inString[position] == '_'))
		{
			inString[head++] = inString[position];
			last_char = inString[position];
		}
	}


	inString[head] = 0;

	return (inString);
}


static int GetValueFromString(char *string, const string_comp_rec thisStringCompTable[], uint32_t sizeOfArray)
{
	int x;

	//compare args with strings in stringCompTable
	for (x=(sizeOfArray/sizeof(string_comp_rec))-1;x>=0;x--)
	{
		if (!strcmp(thisStringCompTable[x].valueString, string))
		{
			return(thisStringCompTable[x].valueInt);
		}
	}

	return(-1);
}

void SetValueOrString(int position, char *value)
{
	uint32_t x;
	char stringBuffer[10];
	if (valueTable[position].type != typeSTRING)
	{
		value = CleanupNumberString(value);
		//compare args with strings in stringCompTable
		for (x=0;x<(sizeof(stringCompTable)/sizeof(string_comp_rec));x++)
		{
			if (!strcmp(stringCompTable[x].valueString, value))
			{
				//snprintf(buffer, 10, "%d", value);
				snprintf(stringBuffer, 10, "%ld", stringCompTable[x].valueInt);
				SetValue(position, stringBuffer);
				return;
			}
		}
	}
	SetValue(position, value);
}

void SetValue(int position, char *value)
{
	char *c;
		
	switch (valueTable[position].type) {
		//TODO used something better then atoi
		case typeUINT:
		case typeINT:
			*(uint32_t *)valueTable[position].ptr = atoi(value);
			break;
		case typeFLOAT:
			*(float *)valueTable[position].ptr = atof(value);
			break;
		case typeSTRING:
			c = valueTable[position].ptr;
			strncpy(c,value,15);
			c[15]=0;
			break;
	}
}

void SendStatusReport(char *inString)
{
	snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#ss %s\n", inString);
	RfCustomReplyBuffer(rf_custom_out_buffer);
}

static int SetVariable(char *inString)
{
	int   x;
	int   inStringLength;
	char *args = NULL;
	StripSpaces(inString);

	inStringLength = strlen(inString);

	for (x = 0; x < inStringLength; x++)
	{
		if (inString[x] == '=')
			break;
	}

	if (inStringLength > x)
	{
		args = inString + x + 1;
	}

	inString[x] = 0;

	for (x = strlen(inString); x >= 0; x--)
		inString[x] = tolower((unsigned char)inString[x]);

	for (x=configSize-1;x>=0;x--)
	{
		if (!strcmp(valueTable[x].name, inString))
		{
			SetValueOrString(x, args);
			if ( (!strcmp(valueTable[x].group, "telm")) || (!strcmp(valueTable[x].group, "mixr")) || (!strcmp(valueTable[x].group, "gyro")) || (!strcmp(valueTable[x].group, "filt")) || (!strcmp(valueTable[x].group, "rccf"))  || (!strcmp(valueTable[x].group, "rate")) )
			{
				resetBoard=1;
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me %s=%s\n", inString, args);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			return (1);
		}
	}
	return (0);
}




/**********************************************************************************************************/
void OutputVarSet(uint32_t position, int doDiff)
{
	char fString[20];
	char *c;
	switch (valueTable[position].type)
	{
		case typeUINT:
			if(doDiff)
			{
				if( (*(uint32_t *)valueTable[position].ptr) != (uint32_t)valueTable[position].Default ) 
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%d\n", valueTable[position].name, (int)*(uint32_t *)valueTable[position].ptr);
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}
			}
			else
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%d\n", valueTable[position].name, (int)*(uint32_t *)valueTable[position].ptr);
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			break;
		case typeINT:
			if(doDiff)
			{
				if( (*(int32_t *)valueTable[position].ptr) != (int32_t)valueTable[position].Default ) 
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%d\n", valueTable[position].name, (int)*(int32_t *)valueTable[position].ptr);
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}
			}
			else
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%d\n", valueTable[position].name, (int)*(int32_t *)valueTable[position].ptr);
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			break;
		case typeFLOAT:
			if(doDiff)
			{
				if( (*(float *)valueTable[position].ptr) != (float)valueTable[position].Default ) 
				{
					ftoa(*(float *)valueTable[position].ptr, fString);
					StripSpaces(fString);
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%s\n", valueTable[position].name, fString);
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}
			}
			else
			{
				ftoa(*(float *)valueTable[position].ptr, fString);
				StripSpaces(fString);
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%s\n", valueTable[position].name, fString);
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			break;
		case typeSTRING:
			c = valueTable[position].ptr;
			if(doDiff)
			{
				if( strcmp(valueTable[position].strDefault, c) ) 
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%s\n", valueTable[position].name, c);
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}
			}
			else
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "set %s=%s\n", valueTable[position].name, c);
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			break;
	}
}

void OutputVar(uint32_t position)
{
	char fString[20];
	char *c;
	switch (valueTable[position].type)
	{
		case typeUINT:
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "%s=%d\n", valueTable[position].name, (int)*(uint32_t *)valueTable[position].ptr);
			break;
		case typeINT:
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "%s=%d\n", valueTable[position].name, (int)*(int32_t *)valueTable[position].ptr);
			break;
		case typeFLOAT:
			ftoa(*(float *)valueTable[position].ptr, fString);
			StripSpaces(fString);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "%s=%s\n", valueTable[position].name, fString);
			break;
		case typeSTRING:
			c = valueTable[position].ptr;
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "%s=%s\n", valueTable[position].name, c);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			break;
	}
	RfCustomReplyBuffer(rf_custom_out_buffer);
}
/**********************************************************************************************************************/


int RfCustomReply(char *rf_custom_out_buffer)
{

	bzero((rfReplyBuffer+1), (sizeof(rfReplyBuffer)-1));
	rfReplyBuffer[0]=1;
	memcpy((char *)(rfReplyBuffer+1), rf_custom_out_buffer, RF_BUFFER_SIZE);

	USBD_HID_SendReport(&hUsbDeviceFS, rfReplyBuffer, HID_EPIN_SIZE);
	return(0);

}

void PrintVtxInfo(void)
{
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxDevice=%i\n",      vtxRecord.vtxDevice);      RfCustomReplyBuffer(rf_custom_out_buffer);
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxBand=%i\n",        vtxRecord.vtxBand);        RfCustomReplyBuffer(rf_custom_out_buffer);
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxChannel=%i\n",     vtxRecord.vtxChannel);     RfCustomReplyBuffer(rf_custom_out_buffer);
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxBandChannel=%s\n", vtxStringCompTable[vtxRecord.vtxBandChannel].valueString); RfCustomReplyBuffer(rf_custom_out_buffer);
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxPower=%i\n",       vtxRecord.vtxPower);       RfCustomReplyBuffer(rf_custom_out_buffer);
	if (vtxRecord.vtxPit == VTX_MODE_ACTIVE)
	{
		snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxPit=VTX Broadcasting\n");
		RfCustomReplyBuffer(rf_custom_out_buffer);
	}
	else
	{
		snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxPit=VTX in Pit Mode\n");
		RfCustomReplyBuffer(rf_custom_out_buffer);
	}
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxRegion=%i\n",      vtxRecord.vtxRegion);      RfCustomReplyBuffer(rf_custom_out_buffer);
	snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me vtx.vtxFrequency=%i\n",   vtxRecord.vtxFrequency);   RfCustomReplyBuffer(rf_custom_out_buffer);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////COMMANDS/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//process commands here.
void ProcessCommand(char *inString)
{

	//buffer_record *buffer = &flashInfo.buffer[flashInfo.bufferNum];
	uint32_t inStringLength;
	char *args = NULL;
	char *originalString = inString;
	uint32_t x;
	static uint32_t lastTimeMore = 0;
	static uint32_t firstTimeRunningTelem = 1;
	static uint32_t testModeAproved = 0;

	if (rfCustomReplyBufferPointerSent < rfCustomReplyBufferPointer)
	{
		//four second more timeout
		if (Micros() - lastTimeMore < 6000000)
		{
			SendRfCustomReplyBuffer();
			return;
		}
		else
		{
			rfCustomReplyBufferPointerSent = 0;
			rfCustomReplyBufferPointer = 0;
		}
	}

	if (originalString[1] == 77)
	{
		inString = originalString;
	}
	else
	{
		inString = CleanupString(inString);
	}

	inStringLength = strlen(inString);

	for (x = 0; x < inStringLength; x++)
	{
		if ( (inString[x] == ' ') || (inString[x] == 220) )
			break;
	}

	if (inStringLength > x)
	{
		args = inString + x + 1;
	}

	inString[x] = 0;


	args = CleanupString(args);

	inStringLength = strlen(args);

	for (x = 0; x < inStringLength; x++)
	{
		if ( (inString[x] == ' ') || (inString[x] == 220) )
		{
			args[x] = 0;
			break;
		}

	}


	for (x = 0; x < strlen(inString); x++)
		inString[x] = tolower((unsigned char)inString[x]);


	//ignore any string that starts with #
	if (inString[0] == '#')
		return;


	if (!strcmp("more", inString))
		{
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#nomore\n" );
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("polladc", inString))
		{
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Voltage: %lu, Current: %lu, mAh: %lu\n", (uint32_t)(adcVoltage*100) , (uint32_t)(adcCurrent*100), (uint32_t)(adcMAh));
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("esc_d_beep", inString))
		{
			dshotCommandHandler.motorCommMask     = 0x0F;
			dshotCommandHandler.requestActivation = 1;
			dshotCommandHandler.dshotCommandState = DSC_MODE_SEND;
			dshotCommandHandler.commandToSend     = DSHOT_CMD_BEEP4;
			RfCustomReplyBuffer("#me Sending command\n");
		}
	else if (!strcmp("esc_d_exit", inString))
		{
			dshotCommandHandler.requestActivation = 0;
			dshotCommandHandler.motorCommMask     = 0;
			RfCustomReplyBuffer("#me Exiting dshot command mode\n");
		}
	else if ( !strcmp("esc_d_reverse", inString) || !strcmp("esc_d_normal", inString))
		{
			switch (args[0])
			{
				case '0':
				case '1':
				case '2':
				case '3':
				case 'a':
					dshotCommandHandler.motorCommMask     = (args[0] == 'a' ? 0x0f : (1 << CONSTRAIN( atoi(args),0, MAX_MOTOR_NUMBER) ) );
					dshotCommandHandler.requestActivation = 1;
					dshotCommandHandler.dshotCommandState = DSC_MODE_SEND;
					dshotCommandHandler.commandToSend     = ( !strcmp("esc_d_normal", inString) ? DSHOT_CMD_SPIN_DIRECTION_1 : DSHOT_CMD_SPIN_DIRECTION_1);
					RfCustomReplyBuffer("#me Sending command\n");
					break;
				default:
					RfCustomReplyBuffer("#me Invalid ESC number\n");
					break;
			}
		}
	else if (!strcmp("idle", inString))
		{
			if (!strcmp("stop", args))
			{
				DoIdleStop();
			}
			else
			{
				uint32_t motorToSpin = CONSTRAIN( atoi(args),0, MAX_MOTOR_NUMBER);
				uint8_t serialOutBuffer[3];

				DisarmBoard();
				SKIP_GYRO=1;

				snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Spinning Motor %lu\n", motorToSpin );
				RfCustomReplyBuffer(rf_custom_out_buffer);

				//won't work if dshot command handler is active, or beep is active, or quopa is active
				if ( IsDshotEnabled() )
				{
					if(!taskDshotActuators)
					{
						for(uint32_t xxx=0;xxx<2000;xxx++)
						{
							ThrottleToDshot(serialOutBuffer, 0, 0, 0);
							OutputSerialDmaByte(serialOutBuffer, 2, board.motors[mainConfig.mixerConfig.motorOutput[0]], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
							OutputSerialDmaByte(serialOutBuffer, 2, board.motors[mainConfig.mixerConfig.motorOutput[1]], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
							OutputSerialDmaByte(serialOutBuffer, 2, board.motors[mainConfig.mixerConfig.motorOutput[2]], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
							OutputSerialDmaByte(serialOutBuffer, 2, board.motors[mainConfig.mixerConfig.motorOutput[3]], 1, 0, 1); //buffer with data, number of bytes, actuator to output on, msb, no serial frame
							DelayMs(1);
						}
						taskDshotActuators = 1;
					}
				}
				else
				{
					DelayMs(1500);
				}
				taskIdleActuators[motorToSpin]=1;
				IdleActuator( motorToSpin );
			}
		}
	else if (!strcmp("idlestop", inString))
		{
			DoIdleStop();
		}
	else if (!strcmp("dsbeep", inString))
		{
			EnableMode(M_BEEP);
			RfCustomReplyBuffer("#me Beep!\n");
		}
	else if (!strcmp("dsbeepstop", inString))
		{
			DisableMode(M_BEEP);
			RfCustomReplyBuffer("#me Boop!\n");
		}
	else if (!strcmp("error", inString))
		{

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%li %lu %lu\n", deviceWhoAmI, errorMask, failsafeHappend);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ba:%lu\n", armingStructure.boardArmed);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "lfa:%lu\n", armingStructure.latchFirstArm);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "ams:%lu\n", armingStructure.armModeSet);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "amm:%lu\n", armingStructure.armModeActive);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "rcc:%lu\n", armingStructure.rcCalibrated);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "bc:%lu\n", armingStructure.boardCalibrated);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "pm:%lu\n", armingStructure.progMode);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "tis:%lu\n", armingStructure.throttleIsSafe);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "rxt:%lu\n", armingStructure.rxTimeout);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "fh:%lu\n", armingStructure.failsafeHappend);RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "af:%lu\n", armingStructure.activeFailsafe);RfCustomReplyBuffer(rf_custom_out_buffer);

		}
	else if (!strcmp("set", inString))
		{
			if (!SetVariable(args))
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Setting Not Found:%s\n", inString);
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("rxraw", inString))
		{
			for (uint32_t xx = 0; xx < MAXCHANNELS;xx++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#rw %u=%u\n", (volatile unsigned int)xx+1, (volatile unsigned int)(rxDataRaw[xx]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("rxdata", inString))
		{
			for (uint32_t xx = 0; xx < MAXCHANNELS;xx++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#rx %u=%u\n", (volatile unsigned int)xx+1, (volatile unsigned int)(rxData[xx]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("rcdata", inString))
		{
			for (uint32_t xx = 0; xx < MAXCHANNELS;xx++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#rc %d=%d\n", (volatile int)xx+1, (volatile int)(trueRcCommandF[xx]*1000));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("rxrcdata", inString) || !strcmp("rcrxdata", inString))
		{
			for (uint32_t xx = 0; xx < MAXCHANNELS;xx++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#rb %u=%u:%d\n", (volatile unsigned int)(xx+1), (volatile unsigned int)(rxData[xx]), (volatile int)(trueRcCommandF[xx]*1000));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("telem", inString))
		{
			char pitchString[12];
			char rollString[12];
			char yawString[12];
			char ax[12];
			char ay[12];
			char az[12];
			char gx[12];
			char gy[12];
			char gz[12];
			char qx[12];
			char qy[12];
			char qz[12];
			char qw[12];

			if (firstTimeRunningTelem)
			{
				firstTimeRunningTelem = 0;
				ResetGyroCalibration();
			}
			ftoa(pitchAttitude, pitchString);
			ftoa(rollAttitude, rollString);
			ftoa(yawAttitude, yawString);
			StripSpaces(pitchString);
			StripSpaces(rollString);
			StripSpaces(yawString);

			ftoa(filteredAccData[ACCX], ax);
			ftoa(filteredAccData[ACCY], ay);
			ftoa(filteredAccData[ACCZ], az);
			StripSpaces(ax);
			StripSpaces(ay);
			StripSpaces(az);


			//X is pitch, y is roll, z is yaw
			ftoa(filteredGyroData[PITCH], gx);
			ftoa(filteredGyroData[ROLL], gy);
			ftoa(filteredGyroData[YAW], gz);
			StripSpaces(gx);
			StripSpaces(gy);
			StripSpaces(gz);

			//ftoa6(attitudeFrameQuat.w, qw);
			//ftoa6(attitudeFrameQuat.x, qx);
			//ftoa6(attitudeFrameQuat.y, qy);
			//ftoa6(attitudeFrameQuat.z, qz);
			ftoa(attitudeFrameQuat.w, qw);
			ftoa(attitudeFrameQuat.x, qx);
			ftoa(attitudeFrameQuat.y, qy);
			ftoa(attitudeFrameQuat.z, qz);
			StripSpaces(qw);
			StripSpaces(qx);
			StripSpaces(qy);
			StripSpaces(qz);

			//todo: make a way to combine strings
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm pitch=%s\n#tm roll=%s\n#tm heading=%s\n", pitchString,rollString,yawString);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm ax=%s\n#tm ay=%s\n#tm az=%s\n", ax,ay,az);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm gx=%s\n#tm gy=%s\n#tm gz=%s\n", gx,gy,gz);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm qx=%s\n", qx);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm qy=%s\n", qy);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm qz=%s\n", qz);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#tm qw=%s\n", qw);
			RfCustomReplyBuffer(rf_custom_out_buffer);

		}
	else if (!strncmp("sbus_", inString, 4))
		{
			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_SBUS_T;
					break;
				case 'r':
					protocol = USING_SBUS_R;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me SBUS Defaults\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strncmp("crsf_", inString, 4))
		{
			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_CRSF_T;
					break;
				case 'r':
					protocol = USING_CRSF_R;
					break;
				case 'b':
					protocol = USING_CRSF_B;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me CRSF Defaults\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strncmp("sumd_", inString, 4))
		{

			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_SUMD_T;
					break;
				case 'r':
					protocol = USING_SUMD_R;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me SUMD Defaults\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();

		}
	else if (!strncmp("ibus_", inString, 4))
		{

			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_IBUS_T;
					break;
				case 'r':
					protocol = USING_IBUS_R;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me IBUS Defaults\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strncmp("cppm_", inString, 4))
		{
			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_CPPM_T;
					break;
				case 'r':
					protocol = USING_CPPM_R;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			RfCustomReplyBuffer("#me CPPM Defaults\n");

			SaveAndSend();
		}
	else if (!strncmp("spek_", inString, 4))
		{
			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_SPEK_T;
					break;
				case 'r':
					protocol = USING_SPEK_R;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me SPEK Defaults\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strncmp("dsm2_", inString, 4))
		{
			uint32_t protocol;
			uint32_t usart;

			switch(inString[5])
			{
				case 't':
					protocol = USING_DSM2_T;
					break;
				case 'r':
					protocol = USING_DSM2_R;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}
			switch(inString[6])
			{
				case '1':
					usart = ENUM_USART1;
					break;
				case '2':
					usart = ENUM_USART2;
					break;
				case '3':
					usart = ENUM_USART3;
					break;
				case '4':
					usart = ENUM_USART4;
					break;
				case '5':
					usart = ENUM_USART5;
					break;
				case '6':
					usart = ENUM_USART6;
					break;
				default:
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Error\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
					break;
			}

			SetRxDefaults(protocol, usart);
			SetMode(M_ARMED, 4, 50, 100);
			resetBoard = 1;
			mainConfig.rcControlsConfig.rcCalibrated = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me DSM2 Defaults\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strcmp("vtxinfo", inString))
		{

			progMode=1;
			if (mainConfig.telemConfig.telemSmartAudio)
			{
				InitSmartAudio();
				DeInitSmartAudio();
			}
			else if (mainConfig.telemConfig.telemTramp)
			{
				TrampGetSettings();
			}

			vtxRequested.vtxBand = vtxRecord.vtxBand;
			vtxRequested.vtxChannel = vtxRecord.vtxChannel;
			vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
			vtxRequested.vtxPower = vtxRecord.vtxPower;
			vtxRequested.vtxPit = vtxRecord.vtxPit;
			vtxRequested.vtxFrequency = vtxRecord.vtxFrequency;
			vtxRequested.vtxRegion = vtxRecord.vtxRegion;
			vtxRequested.vtxTemp = vtxRecord.vtxTemp;
			PrintVtxInfo();
			progMode=0;
		}
	else if (!strcmp("vtxon", inString))
		{
			progMode=1;

			if (VtxTurnOn())
			{
				vtxRequested.vtxBand = vtxRecord.vtxBand;
				vtxRequested.vtxChannel = vtxRecord.vtxChannel;
				vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
				vtxRequested.vtxPower = vtxRecord.vtxPower;
				vtxRequested.vtxPit = vtxRecord.vtxPit;
				vtxRequested.vtxFrequency = vtxRecord.vtxFrequency;
				vtxRequested.vtxRegion = vtxRecord.vtxRegion;
				vtxRequested.vtxTemp = vtxRecord.vtxTemp;
				PrintVtxInfo();
			}
			else
			{
				RfCustomReplyBuffer("#me Error turning on VTX\n");
			}

			progMode=0;

		}
	else if (!strcmp("vtxpit", inString))
		{

			progMode=1;

			if (VtxTurnPit())
			{
				vtxRequested.vtxBand = vtxRecord.vtxBand;
				vtxRequested.vtxChannel = vtxRecord.vtxChannel;
				vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
				vtxRequested.vtxPower = vtxRecord.vtxPower;
				vtxRequested.vtxPit = vtxRecord.vtxPit;
				vtxRequested.vtxFrequency = vtxRecord.vtxFrequency;
				vtxRequested.vtxRegion = vtxRecord.vtxRegion;
				vtxRequested.vtxTemp = vtxRecord.vtxTemp;
				PrintVtxInfo();
			}
			else
			{
				RfCustomReplyBuffer("#me Error putting VTX into pit mode\n");
			}

			progMode=0;

		}
	else if (!strcmp("vtxpower", inString))
		{

			progMode=1;

			if (VtxPower(atoi(args)))
			{
				vtxRequested.vtxBand = vtxRecord.vtxBand;
				vtxRequested.vtxChannel = vtxRecord.vtxChannel;
				vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
				vtxRequested.vtxPower = vtxRecord.vtxPower;
				vtxRequested.vtxPit = vtxRecord.vtxPit;
				vtxRequested.vtxFrequency = vtxRecord.vtxFrequency;
				vtxRequested.vtxRegion = vtxRecord.vtxRegion;
				vtxRequested.vtxTemp = vtxRecord.vtxTemp;
				PrintVtxInfo();
			}
			else
			{
				RfCustomReplyBuffer("#me Error changing VTX power\n");
			}

			progMode=0;

		}
	else if (!strcmp("vtxbandchannel", inString))
		{

			progMode=1;

			if (VtxBandChannel( GetValueFromString(args, vtxStringCompTable, sizeof(vtxStringCompTable)) ))
			{
				vtxRequested.vtxBand = vtxRecord.vtxBand;
				vtxRequested.vtxChannel = vtxRecord.vtxChannel;
				vtxRequested.vtxBandChannel = vtxRecord.vtxBandChannel;
				vtxRequested.vtxPower = vtxRecord.vtxPower;
				vtxRequested.vtxPit = vtxRecord.vtxPit;
				vtxRequested.vtxFrequency = vtxRecord.vtxFrequency;
				vtxRequested.vtxRegion = vtxRecord.vtxRegion;
				vtxRequested.vtxTemp = vtxRecord.vtxTemp;
				PrintVtxInfo();
			}
			else
			{
				RfCustomReplyBuffer("#me Error changing VTX channel\n");
			}

			progMode=0;

		}
	else if (!strcmp("serial", inString))
		{
			DelayMs(100);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#serial: %lu%lu%lu\n", STM32_UUID[0], STM32_UUID[1], STM32_UUID[2] );
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("msmode", inString))
		{
			mainConfig.tuneProfile[activeProfile].filterConfig[YAW].ga   = 0;
			mainConfig.tuneProfile[activeProfile].filterConfig[ROLL].ga  = 0;
			mainConfig.tuneProfile[activeProfile].filterConfig[PITCH].ga = 0;
			mainConfig.mixerConfig.escProtocol               = ESC_MULTISHOT;
			mainConfig.mixerConfig.escUpdateFrequency        = 32000;
			resetBoard = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Digital Mode Enabled\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strcmp("dpmode", inString))
		{
			mainConfig.mixerConfig.escProtocol            = ESC_DSHOT600;
			mainConfig.mixerConfig.escUpdateFrequency     = 32000;
			resetBoard = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Digital Mode Enabled\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			SaveAndSend();
		}
	else if (!strcmp("resetcurves", inString))
		{
			ResetTpaCurves();
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me TPA and Throttle Curves have been reset to defaults.\n" );
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Do not forget to save.\n" );
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("testmotors", inString))
		{

			snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me WARNING! WARNING! WARNING! \n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me Your motors will be spun up to high throttle\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me You MUST remove your props to continue the test\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me Or serious injury or damage can occur during the test\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me To continue the test, please run propsarenowoff\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me before running this command\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
	
			if (!testModeAproved)
				testModeAproved = 1;

		}
	else if (!strcmp("dds", inString))
		{

			if (mainConfig.mixerConfig.escProtocol != ESC_DDSHOT)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Please enable DDSHOT like so first:\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me set esc_protocol=10\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			else
			{
				DisarmBoard();
				ZeroActuators( 1000 );
				SKIP_GYRO=1;
				args = StripSpaces(args);
				taskDdsActuators=atoi(args);

				if (taskDdsActuators == 0)
				{
					OutputDDShotDma(board.motors[0], 0, taskDdsActuators);
					OutputDDShotDma(board.motors[1], 0, taskDdsActuators);
					OutputDDShotDma(board.motors[2], 0, taskDdsActuators);
					OutputDDShotDma(board.motors[3], 0, taskDdsActuators);
					SKIP_GYRO=0;
				}
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me DDSHOTTING motors \n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}

		}
	else if (!strcmp("propsarenowoff", inString))
		{
			if (testModeAproved)
			{
				DelayMs(500);
				boardArmed=1;
				SKIP_GYRO=1;
				//run test here:
				motorOutput[0] = 0.0f;
				motorOutput[1] = 0.0f;
				motorOutput[2] = 0.0f;
				motorOutput[3] = 0.0f;

				DirectActuator(0, -1.0f);
				DirectActuator(1, -1.0f);
				DirectActuator(2, -1.0f);
				DirectActuator(3, -1.0f);
				for (uint32_t x=0;x<4;x++)
				{

					DirectActuator(x, 0);
					DelayMs(1000);
					boardArmed=1;
					SKIP_GYRO=1;
					for (uint32_t y=0;y<12;y++)
					{
						float stdDeviation[50];
						bzero(stdDeviation, sizeof(stdDeviation));

						if (y == 11)
						{
							DirectActuator(x, 0.0f);
						}
						else
						{
							DirectActuator(x, (y / 10.0f));
						}
						DelayMs(10);
						for (uint32_t simpleCouter=0;simpleCouter < 50;simpleCouter++)
						{
							stdDeviation[simpleCouter]   = ABS(geeForceAccArray[ACCX]) + ABS(geeForceAccArray[ACCY]) + ABS(geeForceAccArray[ACCZ]);
							if (y == 11)
							{
								delayUs(500);
							}
							else
							{
								DelayMs(3);
							}
						}
						if (y == 11)
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu noise during brake event: %lu\n", x, (uint32_t)(CalculateSDSize(stdDeviation, 50) * 1000.0f));RfCustomReplyBuffer(rf_custom_out_buffer);
						}
						else
						{
							snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "Motor %lu noise level at %lu percent: %lu\n", x, (uint32_t)(y * 10), (uint32_t)(CalculateSDSize(stdDeviation, 50) * 1000.0f));RfCustomReplyBuffer(rf_custom_out_buffer);
						}
					}
					DirectActuator(0, -1.0f);
					DirectActuator(1, -1.0f);
					DirectActuator(2, -1.0f);
					DirectActuator(3, -1.0f);
					DelayMs(500);
				}
				DirectActuator(0, -1.0f);
				DirectActuator(1, -1.0f);
				DirectActuator(2, -1.0f);
				DirectActuator(3, -1.0f);
				boardArmed      = 0;
				SKIP_GYRO       = 0;
				testModeAproved = 0;
			}
			else
			{
				testModeAproved = 0;
				snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me WARNING! WARNING! WARNING! \n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
				snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me Your motors will be spun up to high throttle\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
				snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me You MUST remove your props to continue the test\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
				snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me Or serious injury or damage can occur during the test\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
				snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me To continue the test, please run propsarenowoff\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);
				snprintf(rf_custom_out_buffer, (RF_BUFFER_SIZE-1), "#me before running this command\n");  rf_custom_out_buffer[RF_BUFFER_SIZE-1]=0;  RfCustomReplyBuffer(rf_custom_out_buffer);

			}
		}
	else if (!strcmp("realpids", inString))
		{
			char fString[20];

			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_YAW_KP) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me yaw_kp_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_ROLL_KP) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me roll_kp_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_PITCH_KP) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me pitch_kp_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);

			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_YAW_KI) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me yaw_ki_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_ROLL_KI) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me roll_ki_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_PITCH_KI) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me pitch_ki_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);

			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_YAW_KD) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me yaw_kd_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_ROLL_KD) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me roll_kd_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
			ftoa6( ( (DEFAULT_PID_CONFIG_VALUE / DEFAULT_PITCH_KD) ), fString); StripSpaces(fString);
			snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#me pitch_kd_real=%s \n", fString ); RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("flashstatus", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me flash status:%lu\n", flashInfo.status);
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("flashmsd", inString))
		{
			DeinitFlight();
			RfCustomReply("#me Rebooting to flash MSD\n");
			RtcWriteBackupRegister(RFBL_BKR_REBOOT_PENDING_REG, 222);
			RtcWriteBackupRegister(RFBL_BKR_BOOT_ADDRESSS_REG, 0x080E0000);
			DelayMs(500);
			SystemReset();
			return;
		}
	else if (!strcmp("flashmsd1", inString))
		{
			DeinitFlight();
			RfCustomReply("#me Rebooting to flash MSD\n");
			DelayMs(500);
			BootToAddress(0x080E0000);
			return;
		}
	else if (!strcmp("julian", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Goes Pro!! %i\n", julian);
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("cruiser", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me craft found is %i %i\n", (int)(learnedKiModel[0].m * 1000), (int)(learnedKiModel[0].b * 1000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me craft found is %i %i\n", (int)(learnedKiModel[1].m * 1000), (int)(learnedKiModel[1].b * 1000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me craft found is %i %i\n", (int)(learnedKiModel[2].m * 1000), (int)(learnedKiModel[2].b * 1000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("32bits", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me \n \n \n 32bitsofgil!\n \n  MM\n<@ \\___/|\n   \\____/\n     ><\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("forcecharmap", inString))
		{
			ForceUpdateCharMap();
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me char map updated\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("forceosdrefresh", inString))
		{
			ForceUpdateOsd();
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me osd updated\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("voltout", inString))
		{
			if (atoi(args) < 0)
			{
				StopSoftPwm();
			}
			else if( atoi(args) < (SPT_3_3+1) )
			{
				SoftPwmVoltage(USART4_TXPORT, USART4_TXPIN, (software_pwm_voltage_t)atoi(args));
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Voltage to %i\n", atoi(args) * 33);
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("logme", inString))
		{
			if ( !strcmp("1", args) )
			{
				logMe = 1;
				RfCustomReplyBuffer("#me LOGGING!\n");
			}
			else
			{
				logMe = 0;
				RfCustomReplyBuffer("#me NOT LOGGING!\n");
			}
		}
	else if (!strcmp("dump", inString) || !strcmp("diff", inString))
		{
			int doDiff = 0;
			uint32_t argsOutputted = 0;

			if(!strcmp("diff", inString))
				doDiff = 1;

			if ( (!strcmp("", args)) || (!strcmp("all", args)) )
			{

				RfCustomReplyBuffer(FULL_VERSION_STRING);
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#fc HARDWARE:%s\n", FC_NAME);
				RfCustomReplyBuffer(rf_custom_out_buffer);
				
				//snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%s", FULL_VERSION_STRING);
				//RfCustomReply(rf_custom_out_buffer);

				DlflStatusDump();
				PrintModes();
				PrintTpaCurves();

				for (x=0;x<configSize;x++)
				{
					OutputVarSet(x, doDiff);
					argsOutputted++;
				}

			}
			else
			{

				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%s\n", FULL_VERSION_STRING);
				RfCustomReplyBuffer(rf_custom_out_buffer);
				for (x=0;x<configSize;x++)
				{
					if (!strcmp(valueTable[x].group, args))
					{
						OutputVarSet(x, doDiff);
						argsOutputted++;
					}
				}

			}

			if (argsOutputted == 0)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me No Arguments Found For:%s\n", args);
				RfCustomReplyBuffer(rf_custom_out_buffer);

			}
			//RfCustomReplyBuffer("#RFEND");
			//snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#RFEND");
			//RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("tpaout", inString))
		{
			for (uint32_t x=0;x<1000;x++)
			{
				snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "%lu\n", (uint32_t)(throttleLookupKp[x] * 100) );
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("tpaoutt", inString))
		{
			for (uint32_t x=0;x<1000;x++)
			{
				snprintf( rf_custom_out_buffer, RF_BUFFER_SIZE, "%lu\n", (uint32_t)(throttleLookup[x] * 100) );
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("eraseallflash", inString))
		{

			if (flashInfo.enabled)
			{

				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Erasing Flash\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);

				if (MassEraseDataFlash(1))
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Erase Complete\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}
				else
				{
					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Erase Failed\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);
				}

			}
			else
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Chip Not Detected\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}

		}
	else if (!strcmp("eraseflash", inString))
		{
			if (flashInfo.enabled)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Erasing Flash\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);

				if (((float)(flashInfo.currentWriteAddress) / (float)flashInfo.totalSize) > 0.85) {
					if (MassEraseDataFlash(1))
					{
						snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Erase Complete\n");
						RfCustomReplyBuffer(rf_custom_out_buffer);
					}
					else
					{
						snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Erase Failed\n");
						RfCustomReplyBuffer(rf_custom_out_buffer);
					}
				} else {
					if (MassEraseDataFlashByPage(1, 0))
					{
						snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Erase Complete\n");
						RfCustomReplyBuffer(rf_custom_out_buffer);
					}
				}

			}
			else
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Chip Not Detected\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
		}
	else if (!strcmp("dlflstatusdump", inString))
		{
			DlflStatusDump();
		}
	else if (!strcmp("purse", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me purse v, g, i, v: %i, %i, %i, %i.%i\n",persistance.data.version,persistance.data.generation,persistance.data.itteration, FIRMWARE_VERSION_INT, CONFIG_VERSION);
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me t 1, 2, 3, 4: %i, %i, %i, %i\n",(int)(persistance.data.motorTrim[0] * 100), (int)(persistance.data.motorTrim[1] * 100), (int)(persistance.data.motorTrim[2] * 100), (int)(persistance.data.motorTrim[3] * 100));
			RfCustomReplyBuffer(rf_custom_out_buffer);

			//8bit storage
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me y ");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			for(uint32_t x=0;x<20;x++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%i,",(int)(persistance.data.yawKiTrim8[x]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me r ");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			for(uint32_t x=0;x<20;x++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%i,",(int)(persistance.data.rollKiTrim8[x]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me p ");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			for(uint32_t x=0;x<20;x++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%i,",(int)(persistance.data.pitchKiTrim8[x]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me g ");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			for(uint32_t x=0;x<20;x++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%i,",(int)(persistance.data.geeForce[x]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me r ");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			for(uint32_t x=0;x<20;x++)
			{
				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%lu,",(persistance.data.rememberence[x]));
				RfCustomReplyBuffer(rf_custom_out_buffer);
			}
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);

			/*
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me y 1, 2, 3, 4, 5: %i, %i, %i, %i, %i\n",(int)(persistance.data.yawKiTrim[0] * 1000000), (int)(persistance.data.yawKiTrim[1] * 1000000), (int)(persistance.data.yawKiTrim[2] * 1000000), (int)(persistance.data.yawKiTrim[3] * 1000000), (int)(persistance.data.yawKiTrim[5] * 1000000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me y 6, 7, 8, 9, 10: %i, %i, %i, %i, %i\n",(int)(persistance.data.yawKiTrim[5] * 1000000), (int)(persistance.data.yawKiTrim[6] * 1000000), (int)(persistance.data.yawKiTrim[7] * 1000000), (int)(persistance.data.yawKiTrim[8] * 1000000), (int)(persistance.data.yawKiTrim[9] * 1000000));
			RfCustomReplyBuffer(rf_custom_out_buffer);

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me r 1, 2, 3, 4, 5: %i, %i, %i, %i, %i\n",(int)(persistance.data.rollKiTrim[0] * 1000000), (int)(persistance.data.rollKiTrim[1] * 1000000), (int)(persistance.data.rollKiTrim[2] * 1000000), (int)(persistance.data.rollKiTrim[3] * 1000000), (int)(persistance.data.rollKiTrim[5] * 1000000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me r 6, 7, 8, 9, 10: %i, %i, %i, %i, %i\n",(int)(persistance.data.rollKiTrim[5] * 1000000), (int)(persistance.data.rollKiTrim[6] * 1000000), (int)(persistance.data.rollKiTrim[7] * 1000000), (int)(persistance.data.rollKiTrim[8] * 1000000), (int)(persistance.data.rollKiTrim[9] * 1000000));
			RfCustomReplyBuffer(rf_custom_out_buffer);

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me p 1, 2, 3, 4, 5: %i, %i, %i, %i, %i\n",(int)(persistance.data.pitchKiTrim[0] * 1000000), (int)(persistance.data.pitchKiTrim[1] * 1000000), (int)(persistance.data.pitchKiTrim[2] * 1000000), (int)(persistance.data.pitchKiTrim[3] * 1000000), (int)(persistance.data.pitchKiTrim[5] * 1000000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me p 6, 7, 8, 9, 10: %i, %i, %i, %i, %i\n",(int)(persistance.data.pitchKiTrim[5] * 1000000), (int)(persistance.data.pitchKiTrim[6] * 1000000), (int)(persistance.data.pitchKiTrim[7] * 1000000), (int)(persistance.data.pitchKiTrim[8] * 1000000), (int)(persistance.data.pitchKiTrim[9] * 1000000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
			*/
		}
	else if (!strcmp("iffy", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me apple 0, 1, 2: %i, %i, %i\n",(int)(kiTrim[0] * 1000),(int)(kiTrim[1] * 1000),(int)(kiTrim[2] * 1000));
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("amnesia", inString))
		{
			ResetPersistance(1);
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me and suddenly we were strangers again\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("quadwall", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " \\ /\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " / \\\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " \\ /\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " / \\\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " \\ /\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " / \\\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " \\ /\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, " / \\\n");
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "x   x\n");
		}
	else if (!strcmp("readflash", inString))
		{

			if (flashInfo.enabled) {

				args = StripSpaces(args);

				if ( M25p16ReadPage( atoi(args), flashInfo.buffer[0].txBuffer, flashInfo.buffer[0].rxBuffer) )
				{

					bzero(rf_custom_out_buffer,sizeof(rf_custom_out_buffer));
					for (uint32_t x=0;x<RF_BUFFER_SIZE;x++)
					{
						rf_custom_out_buffer[x] = flashInfo.buffer[0].rxBuffer[FLASH_CHIP_BUFFER_READ_DATA_START+x];
					}
					RfCustomReply(rf_custom_out_buffer);
					return;

				}
				else
				{

					snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Read Failed\n");
					RfCustomReplyBuffer(rf_custom_out_buffer);

				}

			}
			else
			{

				snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Flash Chip Not Detected\n");
				RfCustomReplyBuffer(rf_custom_out_buffer);

			}

		}
	else if (!strcmp("version", inString))
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "%s\n", FULL_VERSION_STRING);
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("wiz", inString))
		{
			args = StripSpaces(args);
			SetupWizard(args);
		}
	else if (!strcmp("modes", inString))
		{
			args = StripSpaces(args);
			SetupModes(args);
		}
	else if (!strcmp("throttlecurve1", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "throttlecurve1 ", mainConfig.tuneProfile[0].filterConfig[0].throttleCurve);
		}
	else if (!strcmp("throttlecurve2", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "throttlecurve2 ", mainConfig.tuneProfile[1].filterConfig[0].throttleCurve);
		}
	else if (!strcmp("throttlecurve3", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "throttlecurve3 ", mainConfig.tuneProfile[2].filterConfig[0].throttleCurve);
		}
	else if (!strcmp("tpakp1", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpakp1 ", mainConfig.tuneProfile[0].filterConfig[0].tpaKpCurve);
		}
	else if (!strcmp("tpaki1", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpaki1 ", mainConfig.tuneProfile[0].filterConfig[0].tpaKiCurve);
		}
	else if (!strcmp("tpakd1", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpakd1 ", mainConfig.tuneProfile[0].filterConfig[0].tpaKdCurve);
		}
	else if (!strcmp("tpakp2", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpakp2 ", mainConfig.tuneProfile[1].filterConfig[0].tpaKpCurve);
		}
	else if (!strcmp("tpaki2", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpaki2 ", mainConfig.tuneProfile[1].filterConfig[0].tpaKiCurve);
		}
	else if (!strcmp("tpakd2", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpakd2 ", mainConfig.tuneProfile[1].filterConfig[0].tpaKdCurve);
		}
	else if (!strcmp("tpakp3", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpakp3 ", mainConfig.tuneProfile[2].filterConfig[0].tpaKpCurve);
		}
	else if (!strcmp("tpaki3", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpaki3 ", mainConfig.tuneProfile[2].filterConfig[0].tpaKiCurve);
		}
	else if (!strcmp("tpakd3", inString))
		{
			args = StripSpaces(args);
			AdjustTpa(args, "tpakd3 ", mainConfig.tuneProfile[2].filterConfig[0].tpaKdCurve);
		}
	else if (!strcmp("save", inString))
		{
			SaveAndSend();
		}
	else if (!strcmp("reboot", inString) || !strcmp("reset", inString))
		{
			RfCustomReply("#me Rebooting\n");
			SystemReset();
			return;
		}
	else if (!strcmp("resetdfu", inString)  || !strcmp("rebootdfu", inString))
		{
			RfCustomReply("#me Rebooting Into DFU\n");
			SystemResetToDfuBootloader();
			return;
		}
	else if (!strcmp("resetconfig", inString))
		{

			resetBoard = 1;

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Resetting Config\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			GenerateConfig();

			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Config Reset\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
			SaveAndSend();

		}
	else if (!strcmp("binds", inString))
		{
			sendSpektrumBind();
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Binding Serial\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if ( (!strcmp("bind9", inString)) || (!strcmp("bind", inString)) )
		{
			mainConfig.rcControlsConfig.bind = 9;
			RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_SPEKTRUM9);
			RtcWriteBackupRegister(FC_STATUS_REG,BOOT_TO_SPEKTRUM9);
			SaveAndSend();
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#me Binding 9\n");
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}
	else if (!strcmp("bind5", inString))
		{
			mainConfig.rcControlsConfig.bind = 5;
			RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_SPEKTRUM9);
			RtcWriteBackupRegister(FC_STATUS_REG,BOOT_TO_SPEKTRUM9);
			SaveAndSend();
			RfCustomReplyBuffer("#me Binding 5\n");
		}
	else if (!strcmp("bind3", inString))
		{
			mainConfig.rcControlsConfig.bind = 3;
			RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_SPEKTRUM9);
			RtcWriteBackupRegister(FC_STATUS_REG,BOOT_TO_SPEKTRUM9);
			SaveAndSend();
			RfCustomReplyBuffer("#me Binding 3\n");
		}
	else if (!strcmp("rebootrfbl", inString) || !strcmp("resetrfbl", inString))
		{
			RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_RFBL_COMMAND);
			RfCustomReply("#me Rebooting Into RFBL\n");
			DelayMs(100);
			SystemReset();
			return;
		}
	else if (!strcmp("rebootrecovery", inString) || !strcmp("resetrecovery", inString))
		{
			RtcWriteBackupRegister(RFBL_BKR_BOOT_DIRECTION_REG,BOOT_TO_RECOVERY_COMMAND);
			RfCustomReply("#me Rebooting Into Recovery\n");
			DelayMs(100);
			SystemReset();
			return;
		}
	else if (!strcmp("1wire", inString))
		{
			args = StripSpaces(args);
			OneWire(args);
		}
	else
		{
			snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#ms Unknown Command:%s\n", inString);
			RfCustomReplyBuffer(rf_custom_out_buffer);
		}

	SendRfCustomReplyBuffer();
	lastTimeMore=Micros();

}

void SaveAndSend(void)
{
	snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#ms Saving\n");
	RfCustomReplyBuffer(rf_custom_out_buffer);
	SaveConfig(ADDRESS_CONFIG_START);
	snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE, "#ms Save Complete\n");
	RfCustomReplyBuffer(rf_custom_out_buffer);
}

void DlflStatusDump(void)
{
	snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE-1, "#fl size=%u\n#fl total=%u\n", (unsigned int)(flashInfo.currentWriteAddress), (unsigned int)(flashInfo.totalSize));
	RfCustomReplyBuffer(rf_custom_out_buffer);
}

int RfCustomReplyBuffer(char *rfCustomSendBufferAdder)
{

	uint32_t stringLength = strlen(rfCustomSendBufferAdder);

	if ( (stringLength+1+rfCustomReplyBufferPointer) <= LARGE_RF_BUFFER_SIZE)
	{
		memcpy(rfCustomSendBuffer+rfCustomReplyBufferPointer, rfCustomSendBufferAdder, stringLength);
		rfCustomReplyBufferPointer += stringLength;
		rfCustomSendBuffer[rfCustomReplyBufferPointer] = 0;
	}

	//add rfCustomSendBufferAdder to rfCustomSendBuffer and add a \n to the end
	//snprintf(rfCustomSendBuffer+rfCustomReplyBufferPointer, LARGE_RF_BUFFER_SIZE-rfCustomReplyBufferPointer, "%s", rfCustomSendBufferAdder);

//	rfCustomReplyBufferPointer += strlen(rfCustomSendBufferAdder) + 1; //adding a \n

	return(0);

}

int SendRfCustomReplyBuffer(void)
{

	if(rfCustomReplyBufferPointer > 0)
	{
		snprintf(rf_custom_out_buffer, RF_BUFFER_SIZE+1, "%s", rfCustomSendBuffer+rfCustomReplyBufferPointerSent);
		RfCustomReply(rf_custom_out_buffer);
		rfCustomReplyBufferPointerSent +=RF_BUFFER_SIZE;
	}
	else
	{
		rfCustomReplyBufferPointerSent=0;
		rfCustomReplyBufferPointer    =0;
	}

	if (rfCustomReplyBufferPointerSent >= rfCustomReplyBufferPointer)
	{
		rfCustomReplyBufferPointerSent=0;
		rfCustomReplyBufferPointer    =0;
	}

	return(0);

}
