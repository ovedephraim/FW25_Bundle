/**
* @file cmd.h
* @brief General command processing definitions
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 29.07.2022
*/
#ifndef _CMD_H
#define _CMD_H


#ifdef __cplusplus
extern "C"
{
#endif


typedef int (*CB_CMD_COMPLETION_FN_t)(void *cmd, void *arg);
typedef int (*CB_CMD_COMPLETION_SYNC_FN_t)(void *arg);

struct sSyncCmdBuffer
{
	void *userInData;
	void *userOutData;
	CB_CMD_COMPLETION_SYNC_FN_t userCallback;
	void *userCallbackArg;
};


#ifdef __cplusplus
}
#endif

#endif
