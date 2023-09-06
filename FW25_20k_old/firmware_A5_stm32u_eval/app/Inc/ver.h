/**
* @file ver.h
* @brief Software version definition
*
* @author Anton Kanaev
*
* @version 0.0.1
* @date 22.04.2022
*/
#ifndef _VER_H
#define _VER_H



#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/* Go to the Project
 * Properties-> C/C++ Build -> Build variables
 * to setup version number . This number will be
 * attached to the output file name.
 */


#define get_sw_ver_banner \
"\r\n========| " STR(PROD_NAME) "_"STR(VER_MAJOR) "."STR(VER_MINOR) "."STR(VER_SUB_MINOR)"."STR(VER_BRANCH) " |=========\r\n"

/* get customer facing numbers */
#define get_CF_SoftwareVersion_str()  STR(PROD_NAME) "_"STR(VER_MAJOR) "."STR(VER_MINOR) "."STR(VER_SUB_MINOR)


//#define getSoftwareVersion_str()  ((((unsigned long)(SW_VER_MAJOR)+(SW_VER_MINOR))&0xFFUL)<<16)|((((unsigned long)(SW_VER_SUB_MINOR))&0xFFUL)<<8)|(((unsigned long)(SW_VER_PATCH))&0xFFUL)
//#define getSoftwareVersion()  ((((unsigned long)(SW_VER_MAJOR))&0xFFUL)<<24)|((((unsigned long)(SW_VER_MINOR))&0xFFUL)<<16)|((((unsigned long)(SW_VER_SUB_MINOR))&0xFFUL)<<8)|(((unsigned long)(SW_VER_PATCH))&0xFFUL)

#endif


