/*
 * Parser.h
 *
 *  Created on: 06-Nov-2015
 *      Author: Eyal
 */

#ifndef PARSER_H_
#define PARSER_H_

#include "Parser_External.h"
#include "membuf.h"
#include "packetbuf.h"

/**
 * @brief PARSER_ParseCommand
 * @brief This function handles command strings
 * @param[in]  cmd - command string
 * @param[out] resp - response string
 * @param[in]  pool - memory pool
 * @returns true -sucess
 */
bool PARSER_ParseCommand(char *cmd, PACKETBUF_HDR **resp, MEMBUF_POOL *pool);

#endif /* PARSER_H_ */
