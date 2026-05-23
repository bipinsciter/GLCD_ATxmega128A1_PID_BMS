#ifndef __LOCKED_PRINTF_H__
#define __LOCKED_PRINTF_H__

#include "os_api.h"

void InitLockedPrintf();

int locked_sprintf_P( char * dataStr, const char * format, ...);
int locked_printf_P( const char * format, ...);
int locked_sprintf( char * dataStr, char * format, ...);
int locked_printf( char * format, ...);

#endif
