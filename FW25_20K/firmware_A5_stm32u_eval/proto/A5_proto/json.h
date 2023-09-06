/**
* @file  json.h
* @brief A5 protocol record codes
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 26.05.2022
*
*/

#ifndef _JSON_H
#define _JSON_H

//Get Version
extern const char *GET_VER_JCMD;
extern const char *GET_VER_JRSP;

bool prob_json_key(char *json_cmd, char * json_resp);

#endif /* _JSON_H */
