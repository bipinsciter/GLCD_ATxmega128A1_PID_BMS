#include <stdio.h>
#include "locked_printf.h"

static OSSemaMutex PrintfMutex;

void InitLockedPrintf()
{
   OSSemaCreateMutex( PrintfMutex );
   OSSemaGive(PrintfMutex);
}



int locked_sprintf_P( char * dataStr, const char * format, ...)
{ 
   va_list args;   // Pointer To List Of Arguments
   int ret;

   OSSemaTakeEver(PrintfMutex); 
   va_start(args, format);				// Parses The String For Variables
   ret = vsprintf_P(dataStr, format, args);	// And Converts Symbols To Actual Numbers
   va_end(args);	
   OSSemaGive(PrintfMutex); 
   return ret;
}

int locked_printf_P( const char * format, ...)
{ 
   va_list args;   // Pointer To List Of Arguments
   int ret;

   OSSemaTakeEver(PrintfMutex); 
   va_start(args, format);			// Parses The String For Variables
   ret = vfprintf_P( stdout, format, args ); // And Converts Symbols To Actual Numbers
   va_end(args);	
   OSSemaGive(PrintfMutex); 
   return ret;
}

int locked_sprintf( char * dataStr, char * format, ...)
{ 
   va_list args;   // Pointer To List Of Arguments
   int ret;

   OSSemaTakeEver(PrintfMutex); 
   va_start(args, format);			// Parses The String For Variables
   ret = vsprintf(dataStr, format, args);	// And Converts Symbols To Actual Numbers
   va_end(args);	
   OSSemaGive(PrintfMutex); 
   return ret;
}

int locked_printf( char * format, ...)
{ 
   va_list args;   // Pointer To List Of Arguments
   int ret;

   OSSemaTakeEver(PrintfMutex); 
   va_start(args, format);									// Parses The String For Variables
   ret = vprintf(format, args);						// And Converts Symbols To Actual Numbers
   va_end(args);	
   OSSemaGive(PrintfMutex); 
   return ret;
}

