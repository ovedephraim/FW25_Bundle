/*
 * Parser_External.h
 *
 *  Created on: 19-May-2016
 *      Author: Eyal
 */

#if !defined(PARSER_EXTERNAL_H_)
#define PARSER_EXTERNAL_H_

// This file is shared with external apps that used SAMRTrode API
// DO NOT (!!) modify comments starting with // C2CS*
// See http://www.codeproject.com/Articles/800111/Passing-C-Cplusplus-Constants-e


// C2CS_Set_ClassName ParserConstants
#define PARSER_PARAMETER_PREFIX_NOVALERT	0x0100
#define PARSER_PARAMETER_PREFIX_ADS131		0x0200
#define PARSER_PARAMETER_PREFIX_3			0x0300
#define PARSER_PARAMETER_PREFIX_4			0x0400
#define PARSER_PARAMETER_PREFIX_LAST		0xF000

#define PARSER_PARAMETER_PREFIX_MASK		0xFF00
#define PARSER_PARAMETER_VALUE_MASK			0x00FF



#endif /* PARSER_EXTERNAL_H_ */
