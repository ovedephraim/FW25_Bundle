/*
/**************************************************************************//**
* @file cmd_task.c
* @brief command interpreter task
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/


#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <stddef.h>

#if defined(NOVALERT)
 #include "Application/NOVAlert/NOVAlert.h"
 #include "Application/NOVAlert/NOVAlert_External.h"
#endif


#include "Parser.h"
#include "a4frame.h"
#include "auxcmd.h"
#include "dispatcher_task.h"
#include "Freertos.h"


#define TRUE  1
#define FALSE 0

extern QueueHandle_t dispq;

typedef bool Bool;
typedef uint16_t UInt16;
typedef int16_t Int16;

static char PARSER_Response[100];
static Bool sendResponse = TRUE;


void PARSER_PrepareError(const char * verb_code)
{
	strcpy(PARSER_Response, "Error: ");
	strcat(PARSER_Response, verb_code);
	strcat(PARSER_Response, "\r");
}/* End of PARSER_PrepareError */

void PARSER_PrepareSystemParameterResponse(char *cmd, char *answer)
{
	memset(PARSER_Response,0,100);

	strcpy(PARSER_Response,cmd);
	strcat(PARSER_Response," = ");
	strcat(PARSER_Response, answer);
	strcat(PARSER_Response, " OK\r");
}

void PARSER_PrepareSystemCommandResponse(char *cmd)
{
	strcpy(PARSER_Response,cmd);
	strcat(PARSER_Response," ! OK\r");
}

void PARSER_PrepareChannelParameterResponse(char *cmd, char *strCh, char *answer)
{
//	char strCh[8];

//	sprintf(strCh,"%d",strCh);

	strcpy(PARSER_Response,cmd);
	strcat(PARSER_Response, " ");
	strcat(PARSER_Response, strCh);
	strcat(PARSER_Response," = ");
	strcat(PARSER_Response, answer);
	strcat(PARSER_Response, " OK\r");
}


//void PARSER_PrepareGlobalParameterResponse(char *param, char *strCh, char *answer)
void PARSER_PrepareGlobalParameterResponse(char *param, char *answer)
{
	strcpy(PARSER_Response,"G ");
	strcat(PARSER_Response,param);
	strcat(PARSER_Response, " = ");
//	strcat(PARSER_Response, strCh);
//	strcat(PARSER_Response, " ");
	strcat(PARSER_Response, answer);
	strcat(PARSER_Response, " OK\r");
}


Bool PARSER_Str2UInt32(char *str, uint32_t *ch)
{
	if (sscanf(str, "%u", ch)  == 1)	{	// only %d is supported in initial configuration. see: http://processors.wiki.ti.com/index.php/Printf_support_for_MSP430_CCSTUDIO_compiler
		return TRUE;
	}
	return FALSE;
}



Bool PARSER_Str2Int32(char *str, uint32_t *ch)
{
	if (sscanf(str, "%d", ch)  == 1)	{	// only %d is supported in initial configuration. see: http://processors.wiki.ti.com/index.php/Printf_support_for_MSP430_CCSTUDIO_compiler
		return TRUE;
	}
	return FALSE;
}


//Bool PARSER_SetParameter(UInt16 param, UInt16 ch, UInt16 val)
Bool PARSER_SetParameter(UInt16 param, UInt16 val)
{
	Bool ret = FALSE;

	switch(param & PARSER_PARAMETER_PREFIX_MASK)	{
	case PARSER_PARAMETER_PREFIX_NOVALERT:
	#if defined(NOVALERT)
	ret = NA_SetParameter((NA_Param_t)param , val);
	#endif
		break;
	case PARSER_PARAMETER_PREFIX_ADS131:

	#if 0
	//ret = ADS131_SetParameter((ADS131_Param_t)param , (ADS131_Channels_t)ch , val);
	ret = ADS131_SetParameter((ADS131_Param_t)param, val);
	#endif
	break;
	case PARSER_PARAMETER_PREFIX_3:
	break;
	case PARSER_PARAMETER_PREFIX_4:
	break;
	default:
		ret = FALSE;
	}
	return ret;
}

//Bool PARSER_GetParameter(UInt16 param, UInt16 ch, UInt16 *val)
Bool PARSER_GetParameter(UInt16 param, UInt16 *val)
{
	Bool ret = FALSE;

	switch(param & PARSER_PARAMETER_PREFIX_MASK)	{
	case PARSER_PARAMETER_PREFIX_NOVALERT:
#if defined(NOVALERT)
		ret = NA_GetParameter((NA_Param_t)param , val);
#endif
		break;
	case PARSER_PARAMETER_PREFIX_ADS131:
#if 0
//		ret = ADS131_GetParameter((ADS131_Param_t)param , (ADS131_Channels_t)ch , val);
		ret = ADS131_GetParameter((ADS131_Param_t)param, val);
#endif
		break;
	case PARSER_PARAMETER_PREFIX_3:
		break;
	case PARSER_PARAMETER_PREFIX_4:
		break;
	default:
		ret = FALSE;
	}
	return ret;
}

#define PARSER_SUBSTRINGS_MAX	8	// up to 8 substrings in command "U = 2017.04.28 11:14:50"

/**
 * @brief This function handles command strings
 */
bool PARSER_ParseCommand(char *cmd, PACKETBUF_HDR **resp, MEMBUF_POOL *pool)
{
	char answer[80] = {0};
	char *strs[PARSER_SUBSTRINGS_MAX];
	uint8_t i=0;
	bool ret=FALSE;

	const char *delimiters=" ,\t\r\n.:";
	char *err_verb = NULL;


	strs[i] = strtok (cmd,delimiters);
	while (strs[i] != NULL)	  {
		i++;
		if(i==PARSER_SUBSTRINGS_MAX)
			break;
	    strs[i] = strtok (NULL, delimiters);
	}

	if((strs[0] != NULL))
	{
		switch(*strs[0])
		{
		//===============================================================================================
		//	Per channel commands/ inqueries
		//===============================================================================================

			case 'C'://todo 1st priority ------------------------------------------------------------
			case 'c':
				if(PARSER_Str2UInt16(strs[1], &ch))
				{	// channel is valid
					switch(*strs[2])
					{
					case '=':
						if( !( PARSER_Str2UInt16(strs[3], &val) && CHAN_SetChannelActivation((CHAN_Channels_t)ch, (Bool)val)) )	{
							break;
						}
						// fall through...
					case '?':
						if(CHAN_GetChannelIsActiveStr((CHAN_Channels_t)ch, answer) )	{
							PARSER_PrepareChannelParameterResponse("C",strs[1],answer);
							ret = TRUE;
						}
					break;
					}
				}
			break;

		//===============================================================================================
		//	system parameters inquiry
		//===============================================================================================

			case 'N': //=================== Get Number of channels ========================//
			case 'n':{ // "N ?LF", "n ?LF"

				if((strs[1] != NULL) && (*strs[1]=='?'))
				{
#if 0
					CHAN_GetChannelNumberStr(answer);
#else
					memcpy(answer,"1", sizeof("1"));
#endif

					PARSER_PrepareSystemParameterResponse("N",answer);
					sendResponse = TRUE;
					ret=TRUE;
				}
				else
				{
					err_verb = "bad format";
				}
			}break;

			//===============================================================================================
			//	system commands
			//===============================================================================================


			case 'A': //=================== start dispatcher routine ======================//
			case 'a':{
				if( *strs[1]=='!')	{

					/* send command to the dispatcher task to start operational mode execution */
					if (pdPASS==sendToDispatcher(dispq, NULL, start_exec, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUXRX,MSG_TYPE_CMD), portMAX_DELAY))
					{
						sendResponse = FALSE;
						ret = TRUE;
					}
					else
					{
						sendResponse = TRUE;
						ret = FALSE;
					}
				}
			}break;

			case 'Z': //=================== stop dispatcher routine =======================//
			case 'z':{
				if( *strs[1]=='!')	{

					/* send command to the dispatcher task to stop operational mode execution */
					if (pdPASS==sendToDispatcher(dispq, NULL, abort_exec, MAKE_MSG_HDRTYPE(0,MSG_SRC_AUXRX,MSG_TYPE_CMD), portMAX_DELAY))
					{
						PARSER_PrepareSystemCommandResponse("Z");
						ret = TRUE;
					}
					sendResponse = TRUE;
				}
			}break;


#if 0
		case 'T'://todo 1st priority -------------------------------------------------------------
		case 't':
			if(*strs[1]=='?'){
				CHAN_GetTimeframeStr(answer);
				PARSER_PrepareSystemParameterResponse("T",answer);
				ret = TRUE;
			}
		break;//==================================================================================
#endif

#if 0
		case 'L'://todo 1st priority -------------------------------------------------------------
		case 'l':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelNameStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("L",strs[1],answer);
				ret = TRUE;
			}
		break;//==================================================================================
#endif

#if 0
		case 'X'://todo 1st priority -------------------------------------------------------------
		case 'x':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelXducerStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("X",strs[1],answer);
				ret = TRUE;
			}
		break;//==================================================================================
#endif

#if 0
		case 'P'://todo 1st priority -------------------------------------------------------------
		case 'p':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelPhysDimStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("P",strs[1],answer);
				ret = TRUE;
			}
		break;//==================================================================================
#endif

#if 0
		case 'Q'://todo 1st priority -------------------------------------------------------------
		case 'q':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelPhysMinStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("Q",strs[1],answer);
				ret = TRUE;
			}
		break;//==================================================================================
#endif

#if 0
		case 'R'://todo 1st priority -------------------------------------------------------------
		case 'r':
				if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelPhysMaxStr((CHAN_Channels_t)ch, answer)) )	{
					PARSER_PrepareChannelParameterResponse("R",strs[1],answer);
					ret = TRUE;
				}
		break;//==================================================================================
#endif

#if 0
		case 'D'://todo 1st priority -------------------------------------------------------------
		case 'd':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelDigMinStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("D",strs[1],answer);
				ret = TRUE;
			}
		break;//==================================================================================
#endif

#if 0
		case 'E'://todo 1st priority -------------------------------------------------------------
		case 'e':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelDigMaxStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("E",strs[1],answer);
				ret = TRUE;
			}
		break;//=================================================================================
#endif

#if 0
		case 'F'://todo 1st priority ------------------------------------------------------------
		case 'f':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelFilterStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("F",strs[1],answer);
				ret = TRUE;
			}
		break;//=================================================================================
#endif

#if 0
		case 'S'://todo 1st priority ------------------------------------------------------------
		case 's':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelSampleRateStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("S",strs[1],answer);
				ret = TRUE;
			}
		break;//=================================================================================
#endif

#if 0
		case 'Y'://todo 1st priority ------------------------------------------------------------
		case 'y':
			if( (PARSER_Str2UInt16(strs[1], &ch)) && (*strs[2]=='?') && (CHAN_GetChannelIsSyncStr((CHAN_Channels_t)ch, answer)) )	{
				PARSER_PrepareChannelParameterResponse("Y",strs[1],answer);
				ret = TRUE;
			}
		break;//================================================================================
#endif





#if 0
		case 'I'://set get global system parameters priority 2 --------------------------------------
		case 'i':
			if(PARSER_Str2UInt16(strs[1], &ch))	{	// channel is valid

				switch(*strs[2])	{
				case '=':
					if(!( PARSER_Str2UInt16(strs[3], &val) && ADS131_SetChannelInput((ADS131_Channels_t)ch, (ADS131_Input_t)val) ) ){
						break;
					}
					// fall through...
				case '?':
					if(CHAN_GetChannelInputStr((CHAN_Channels_t)ch, answer) )	{
						PARSER_PrepareChannelParameterResponse("I",strs[1],answer);
						ret = TRUE;
					}
					break;
				}
			}
		break;
#endif

#if 0
		case 'G'://set get global system parameters priority 2 --------------------------------------
		case 'g':
			if(PARSER_Str2UInt16(strs[1], &param))	{
				switch(*strs[2])	{
				case '=':
					if( !(PARSER_Str2UInt16(strs[3], &ch) && PARSER_Str2UInt16(strs[4], &val) && PARSER_SetParameter(param, ch, val)) )	{
					if( !(PARSER_Str2UInt16(strs[3], &val) && PARSER_SetParameter(param, val)) )	{
						break;
					}
					// fall through...
				case '?':
					if( PARSER_Str2UInt16(strs[3], &ch) && PARSER_GetParameter(param, ch, &val) ){
					if( PARSER_GetParameter(param, &val) ){
						sprintf(answer, "%u", val);
                        PARSER_PrepareGlobalParameterResponse(strs[1], strs[3], answer);
						PARSER_PrepareGlobalParameterResponse(strs[1], answer);
						ret = TRUE;
					}
					break;
				}
			}
		break;//=====================================================================================
#endif

#if 0
		case 'V'://---------------------------------------------------------------------------------
		case 'v':// priority 2
		{
			if(PARSER_Str2UInt16(strs[1], &val) && PARSER_Str2UInt16(strs[2], &on) && \
					PARSER_Str2UInt16(strs[3], &off) && *strs[4]=='!')	{
				if(val == 0)	{
					VIB_KillVibration();
				}
				else	{

					vib.VibrationsNum = val;
					vib.VibrationsOnTime = on;
					vib.VibrationsOffTime = off;

					VIB_SetVibration(vib);
				}
				char response[25];
				sprintf(response,"V %s %s %s",strs[1], strs[2], strs[3]);
				PARSER_PrepareSystemCommandResponse(response);
				ret = TRUE;
			}
		}
		break;//====================================================================================
#endif

#if 0
		case 'M':
		case 'm':// priority 2
			switch(*strs[1])	{
			case '=':
				if(!PARSER_Str2UInt16(strs[2], &val))	{
					break;
				}
				VIB_SetActivation(val);	// value is valid
				// fall through...
				// no break
			case '?':
				sprintf(answer, "%d", VIB_GetActivation());
				PARSER_PrepareSystemParameterResponse("M",answer);
				ret = TRUE;
				break;
			}
		break;//==================================================================================
#endif

#if 0
		case '0'://watch dog priority 2 -------------------------------------------------------------
			if( *strs[1]=='!')	{

				WDTCTL = 0xDEAD;	// Force PUC

			}
		break;//=====================================================================================
#endif

#if 0
			case 'U':
			case 'u':
				switch(*strs[1])	{
					struct tm timeStruct;
					time_t	t;

					case '=':
						if( !(PARSER_Str2Int16(strs[2], &timeStruct.tm_year) && (timeStruct.tm_year>=1900) 	&&
							  PARSER_Str2Int16(strs[3], &timeStruct.tm_mon)  && (timeStruct.tm_mon>=1)		&& (timeStruct.tm_mon<=12)	&&
							  PARSER_Str2Int16(strs[4], &timeStruct.tm_mday) && (timeStruct.tm_mday>=1)    	&& (timeStruct.tm_mday<=31) &&
							  PARSER_Str2Int16(strs[5], &timeStruct.tm_hour) && (timeStruct.tm_hour>=0) 	&& (timeStruct.tm_hour<=23)	&&
							  PARSER_Str2Int16(strs[6], &timeStruct.tm_min)  && (timeStruct.tm_min>=0) 		&& (timeStruct.tm_min<=59)	&&
							  PARSER_Str2Int16(strs[7], &timeStruct.tm_sec)  && (timeStruct.tm_sec>=0) 		&& (timeStruct.tm_sec<=59) ) ) {
							break;
						}
						timeStruct.tm_year-=1900;
						timeStruct.tm_mon-=1;
						timeStruct.tm_wday=0;
						timeStruct.tm_yday=0;
						timeStruct.tm_isdst=0;
						t = mktime(&timeStruct);	// std c library - converts on 1.1.1900 base
						if(t == (time_t)-1)	{
							break;
						}

						t -= 2208988800;// Seconds diff between 1.1.1900 and 1.1.1970 (works on 1.1.1970 base) - https://www.epochconverter.com/date-difference
						Seconds_set(t);

						//no break
					case '?':
						t=time(NULL);
						strftime(answer, 40, "%Y.%m.%d %H:%M:%S", localtime(&t));
						//sprintf(answer, "tzname=%s dstname=%s daylight=%d timezone=%ld", _tz.tzname, _tz.dstname, _tz.daylight, _tz.timezone);
						PARSER_PrepareSystemParameterResponse("U",answer);
						ret = TRUE;
						break;
				}
				break;
#endif
				default:
					ret = FALSE;
				break;
		}
	}

    /* build common for every case error string */
	if(ret == FALSE)	{
		PARSER_PrepareError((const char *)err_verb);
	}

	if((sendResponse) && (resp != NULL))
	{
		PACKETBUF_HDR *buf=NULL;

		/* Allocate memory buffer for response */
		if (NULL == (buf = getPacketBufferWithWait(pool,FIRST_PACKET_SEGMENT|LAST_PACKET_SEGMENT,MT_PACKET_FROM_AUX_CMD, A4_PROTO_FORMAT, 0, portMAX_DELAY))){
			return FALSE;
		}

		/* copy response payload string in the buffer */
		memcpy(PACKETBUF_DATA(buf),PARSER_Response, strlen(PARSER_Response));

		/* assign output value */
		*resp = (PACKETBUF_HDR *)buf;
	}
	return ret;
}/* End of PARSER_ParseCommand */


#if 0 //todo remove
#if 0 //Anton
		case 'A':
		case 'a':
			if( *strs[1]=='!')	{


				if(Semaphore_pend(mutexPARSER_RecordingActive, 0))	{	// mutex Available, no recording in place
					SuspendHeartBeatTask();
					CHAN_Reset();
					CHAN_PrepareChannels();
					#if (SD_STORE_EDF==1)
					EDFFile_CreateNewFile();
					#endif
					PARSER_PrepareSystemCommandResponse("A");

					#if defined(NOVALERT)
					Semaphore_post(semNA_StartAlgorithm);	// For NOVAlert algo control
					Semaphore_post(hSemNA_StartAlgorithm);	// For NOVAlert algo control
					#endif
					ret = TRUE;

					ADS131_Start();

				}
				else	{	// already recording
					ret = FALSE;
				}
				sendResponse = FALSE;
			}
			break;

		case 'Z':
		case 'z':
			if( *strs[1]=='!')	{
			//	ADS131_Stop();
			//09.04.22 Anton	Task_sleep(CHANNEL_TIMEFRAME_MS); //let time for ADS131 to stop 100MS
			//	CHAN_Reset();
	//			ResumeHeartBeatTask();
				#if (SD_STORE_EDF==1)
				int i = 2;
				while ( (Mailbox_getNumPendingMsgs(mbxCHAN_DataStore) != 0) && i--)	{
					Task_sleep(CHANNEL_TIMEFRAME_MS);
				}
				if(i == 0){
					System_printf("mbxCHAN_DataStore did not handles in %d[ms]\n",CHANNEL_TIMEFRAME_MS*2);
					System_flush();
				}
				EDFFile_CloseCurrentFile();
				#endif
				i = 2;

	//			while ( (Mailbox_getNumPendingMsgs(mbxCHAN_DataXmit) != 0) && i--)	{
	//				Task_sleep(CHANNEL_TIMEFRAME_MS);
	//
	//			}
				if(i == 0){
			//		System_printf("mbxCHAN_DataXmit did not handles in %d[ms]\n",CHANNEL_TIMEFRAME_MS*2);
		//			System_flush();
				}
				PARSER_PrepareSystemCommandResponse("Z");
			//	Semaphore_post(mutexPARSER_RecordingActive);
				sendResponse = TRUE;
				ret = TRUE;
			}
			break;
#endif


#endif

