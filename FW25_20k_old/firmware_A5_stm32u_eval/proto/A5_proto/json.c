/**
* @file  json.c
* @brief  json protocol
* @author Anton Kanaev
* @version 0.0.1
* @date 30.04.2022
*
* @section License
* <b>(C) Copyright 2022 Atlasense Ltd., http://www.atlasesne.com </b>
*/

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "cJSON.h"
#include "json.h"
#include "auxcmd.h"
#include "ver.h"
#include "channel_manager_task.h"
#include "hw_tests.h"
#include "rtc.h"
#include "sys_errno.h"
#include "main.h"

#define GET_VER_JCMD      "v"
#define LMD_JCMD_KEY      "l.m"
#define RTC_JCMD_KEY      "t"
#define CFG_JCMD_KEY      "c"

extern QueueHandle_t chanmngq;

char pbuff[100]={0};

static bool proc_SET_code(cJSON *root, char *pkey, void *pval, cJSON *resp);
bool proc_SET_code(cJSON *root, char *pkey, void *pval, cJSON *resp)
{
	bool ret_b=false;

	//==========================================================================================
	if(0 == strcmp(pkey,LMD_JCMD_KEY))//=====================   "l.m"  live.mode enable/disable |
	{
		if(pval && *(uint32_t*)pval==0)
		{
			/* send command to the channel manager task to stop operational mode execution */
			if (pdPASS==sendToChannelManager(chanmngq, NULL, CHAN_CMD_STOP_ALL,
					MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD), portMAX_DELAY))
			{
				/* create json number item */
				pval = cJSON_CreateNumber(0);
				/* add sting to the given key */
				cJSON_AddItemToObject(resp, LMD_JCMD_KEY, pval);
				/* set return value */
				ret_b=true;
			}
		}
		else
		{
			if(0 == CHAN_GetActiveChannelsNumber())
			{
				/* create json number item */
				pval = cJSON_CreateNumber(-1);
				/* add sting to the given key */
				cJSON_AddItemToObject(resp, LMD_JCMD_KEY, pval);
				/* set return value */
				ret_b=true;
			}
			else
			{
				/* send command to the channel manager task to stop operational mode execution */
				if (pdPASS==sendToChannelManager(chanmngq, NULL, CHAN_CMD_START_ALL,
						MAKE_MSG_HDRTYPE(0,MSG_SRC_AUX_DBRX,MSG_TYPE_CMD), portMAX_DELAY))
				{
//					time_t	t=0;
//					/*get time */
//					RTC_get(&t);
//					/* store in backup register */
//					RTC_save_time_stamp(t);
					/* create json number item */
					pval = cJSON_CreateNumber(1);
					/* add sting to the given key */
					cJSON_AddItemToObject(resp, LMD_JCMD_KEY, pval);
					/* set return value */
					ret_b=true;
				}
			}
		}
	}
	else  //==================================================================================
	if(0 == strcmp(pkey,RTC_JCMD_KEY))//=================================   "t" set rtc clock |
	{
		time_t	t=0;
		/* set rtc clock value */
		RTC_set(*(uint32_t*)pval);
		/* read rtc clock value */
		RTC_get(&t);
		/* create json number item */
		pval = cJSON_CreateNumber(t);
		/* add sting to the given key */
		cJSON_AddItemToObject(resp, RTC_JCMD_KEY, pval);
		/* set return value */
		ret_b=true;
	}
	else //===================================================================================
	if(0 == strcmp(pkey,CFG_JCMD_KEY))//====== set all channels configuration ================|
	{
		cJSON *prob=NULL;
		cJSON *cnf_arr=NULL;
		cJSON *outlist=NULL;
		cJSON *outch_item=NULL;
		int i = 0;

		/* fetch array with all configurations */
		cnf_arr=cJSON_GetObjectItem(root,pkey);

		/* create outgoing configuration list */
		outlist = cJSON_CreateArray();
		if (outlist == NULL) {
			Error_Handler((uint8_t *)__FILE__, __LINE__);
		}

		/* set activation for each channel */
		cJSON_ArrayForEach(prob, cnf_arr)
		{
			cJSON * mode=cJSON_GetObjectItem(prob,"m");
			cJSON * rate=cJSON_GetObjectItem(prob,"r");

			if(mode || rate)
			{
				outch_item = cJSON_CreateObject();
				if (outch_item == NULL)
				{
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}
			}

			if(mode)//= mode enable/disable
			{
				int rv=CHAN_SetChannelActivation((CHAN_Channels_t)i,
						(uint32_t) mode->valueint);

				if(rv>=0){//success
					/* add mode to the given key */
					cJSON_AddNumberToObject(outch_item, "m",
							mode->valueint);
				}
				else{//failure
					/* add error code to the given key */
					cJSON_AddNumberToObject(outch_item, "m", rv);
				}

#ifdef MODULE_DEBUG

				sprintf(pbuff,"[JSON \"c\" [%d] m[%d]rv[%d]\n",i,(int)mode->valueint,rv);
				/* debug log output printout */
				aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);

#endif

			}

			if(rate)//= sample rate TODO not supported yet
			{
				int32_t ch_rate=0;

				/* get channel sample rate */
				CHAN_GetChannelRate(i,&ch_rate);

				/* add sting to the given key */
				cJSON_AddNumberToObject(outch_item, "r", ch_rate);

#ifdef MODULE_DEBUG

				sprintf(pbuff,"[JSON \"c\" [%d] r[%d]\n",i,(int)rate->valueint);
				/* debug log output printout */
				aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);

#endif
			}

			if(mode || rate)
			{
				/* add item to the outgoing list */
				cJSON_AddItemToArray(outlist,outch_item);
			}

			i++;
		}/* cJSON_ArrayForEach(prob, cnf_arr) */


		/* add sting to the given key */
		cJSON_AddItemToObject(resp, pkey, outlist);

		/* set return value */
		ret_b=true;
	}
	else
	{
		sprintf(pbuff,"[JSON SET][%s] NOT SUPPORTED KEY \n",pkey);
		/* debug log output printout */
		aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);
	}

	return ret_b;
}/* proc_SET_code */

static bool json_handle_SET_cmg(cJSON *root, char * json_resp);
bool json_handle_SET_cmg(cJSON *root, char * json_resp)
{
	cJSON *temp=root;
	bool ret_b=false;
	char *str = NULL;

#ifdef MODULE_DEBUG
	sprintf(pbuff,"[JSON SET][%s][%d] NOT SUPPORTED \n",
			temp->child->string, temp->child->valueint);
	/* debug log output printout */
	aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);
#endif

	cJSON *resp = cJSON_CreateObject();
	if (resp != NULL)
	{
	    /* process json key */
		ret_b=proc_SET_code(root,
				temp->child->string,
				&temp->child->valueint,
				resp);

		temp = temp->child->next;
		while (temp != NULL)
		{
#ifdef MODULE_DEBUG
			sprintf(pbuff,"[JSON SET][%s][%d] NOT SUPPORTED \n",temp->string,temp->valueint);
			/* debug log output printout */
			aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);
#endif

			ret_b=proc_SET_code(root, temp->string,&temp->valueint,resp);
			temp = temp->next;
		}

		/* object serialization */
		str=cJSON_PrintUnformatted(resp);

		if(json_resp && str)
		{
			/* fill the output string */
			memcpy(json_resp, str,strlen(str)+1);
		}

#ifdef MODULE_DEBUG
		/* Useful debug printout */
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
#endif

		/* free memory */
		cJSON_free(str);
		cJSON_Delete(resp);
	}

	return ret_b;
}/* end of json_handle_SET_cmg */

static bool json_handle_GET_cmg(cJSON *root, char * json_resp);
bool json_handle_GET_cmg(cJSON *root, char * json_resp)
{
	cJSON *prob = NULL;
	char *str = NULL;
	bool ret_b = false;

	cJSON *resp = cJSON_CreateObject();
	if (resp != NULL)
	{
		/* string arrays used for get commands */
		cJSON_ArrayForEach(prob, root)
		{
			cJSON *pval=NULL;

			if(str!=NULL)
			{
				cJSON_free(str);
			}

			if(!strcmp(prob->valuestring,GET_VER_JCMD)){// ============= get version ============|

				/* create json string item */
				pval = cJSON_CreateString(get_CF_SoftwareVersion_str());
				/* add sting to the given key */
				cJSON_AddItemToObject(resp, prob->valuestring, pval);
				/* object serialization */
				str=cJSON_PrintUnformatted(resp);
				/* set return value */
				ret_b=true;
			}
			else //==============================================================================|
			if(!strcmp(prob->valuestring,RTC_JCMD_KEY)){// ============== get rtc ===============|

				time_t	t=0;
				/* read rtc clock value */
				RTC_get(&t);
				/* create json number item */
				pval = cJSON_CreateNumber(t);
				/* add sting to the given key */
				cJSON_AddItemToObject(resp, prob->valuestring, pval);
				/* object serialization */
				str=cJSON_PrintUnformatted(resp);
				/* set return value */
				ret_b=true;

			}//==================================================================================|
			if(!strcmp(prob->valuestring,CFG_JCMD_KEY)){// ===== get all channels configuration =|

				uint32_t i=0,num=0;
				cJSON *ch_item=NULL;

				cJSON *list = cJSON_CreateArray();
				if (list == NULL) {
					Error_Handler((uint8_t *)__FILE__, __LINE__);
				}

				CHAN_GetallChannelNumber(&num);
				for(i=0;i<num;i++)
				{
					uint32_t mode=0;
					int32_t rate=0;

					CHAN_GetChannelActivation(i,&mode);
					CHAN_GetChannelRate(i,&rate);

					ch_item = cJSON_CreateObject();
					if (ch_item != NULL)
					{
						/* add sting to the given key */
						cJSON_AddNumberToObject(ch_item, "m", mode);

						/* add sting to the given key */
						cJSON_AddNumberToObject(ch_item, "r", rate);

						/* add item to the list */
						cJSON_AddItemToArray(list,ch_item);
					}
					else
					{
						Error_Handler((uint8_t *)__FILE__, __LINE__);
					}
				}

				/* add sting to the given key */
				cJSON_AddItemToObject(resp, prob->valuestring, list);
				/* object serialization */
				str=cJSON_PrintUnformatted(resp);

				/* set return value */
				ret_b=true;
			}

			else // =================================================== not supported key ======|
			{
#ifdef MODULE_DEBUG
				sprintf(pbuff,"[JSON][%s][%d] NOT SUPPORTED \n",
				prob->valuestring,
				prob->valueint);
				/* debug log output printout */
				aux_sendToAux(pbuff,strlen(pbuff),0,1,DBG_AUX);
#endif
			}
		}

		if(json_resp && str)
		{
			/* fill the output string */
			memcpy(json_resp, str,strlen(str)+1);
		}

#ifdef MODULE_DEBUG
		/* Useful debug printout */
		aux_sendToAux(str,strlen(str),0,1,DBG_AUX);
#endif


		/* free memory */
		cJSON_Delete(resp);
		cJSON_free(str);
	}

	return ret_b;
}/* end of json_handle_GET_cmg */


bool prob_json_key(char *json_cmd, char * json_resp)
{
	cJSON *root = NULL;
	bool ret_b = false;

	root = cJSON_Parse(json_cmd);
	if(root)
	{
		if(cJSON_IsArray(root))
		{//GET command
			ret_b=json_handle_GET_cmg(root,json_resp);
		}
		else if(cJSON_IsObject(root))
		{//SET command
			ret_b=json_handle_SET_cmg(root,json_resp);
		}

		/* free memory */
		cJSON_Delete(root);
	}
	else
	{
		aux_sendToAux("[JSON] Invalid structure",strlen("[JSON] Invalid structure"),0,1,DBG_AUX);
	}


	return ret_b;
}/* end of prob_json_key */


