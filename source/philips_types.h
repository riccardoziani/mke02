/**
@brief          this file contains all global defined datatypes to be used in the project
@copyright      Copyright(C) 2016 Koninklijke Philips N.V., All Rights Reserved.
                This source code and any compilation or derivative thereof is the
                information of Koninklijke Philips N.V.
                is confidential in nature.
                no circumstances is this software to be exposed to or placed
                an Open Source License of any type without the expressed
                permission of Koninklijke Philips N.V.
 */

 #ifndef PHILIPS_TYPES_H
 #define PHILIPS_TYPES_H

#include "fsl_common.h"

 #ifdef    FALSE
 #undef    FALSE
 #endif
 #define   FALSE     0     /**< FALSE redefined to ensure portability */

 #ifdef    TRUE
 #undef    TRUE
 #endif
 #define   TRUE      1     /**< TRUE redefined to ensure portability */

 #ifdef    NULL
 #undef    NULL
 #endif
 #define   NULL      0     /**< NULL redefined to ensure portability */

#define   ASSIGN_SECTION(id) __attribute__ ((section(id)))      /**< use this macro to specify where to link the variable/fucntion in the MCU */

#define _INT8_T_DECLARED
#define _INT32_T_DECLARED
#define _UINT32_T_DECLARED
#define _INT64_T_DECLARED
#define _UINT64_T_DECLARED
#define _FLOAT32_T_DECLARED
#define _FLOAT64_T_DECLARED

//typedef signed char         int8_t;      /** 8 bit signed integer signed char */
//typedef unsigned char       uint8_t;     /** 8 bit unsigned integer */
//typedef short int           int16_t;     /** 16 bit signed integer */
//typedef unsigned short int  uint16_t;    /** 16 bit unsigned integer*/
//typedef signed int          int32_t;     /** 32 bit signed integer */
//typedef unsigned int        uint32_t;    /** 32 bit unsigned integer */
//typedef long int            int64_t;     /** 64 bit signed integer long int */
//typedef unsigned long int   uint64_t;    /** 64 bit unsigned integer unsigned long int */
typedef float               float_t;   /** 32 bit single precision float */
typedef double              float64_t;   /** 64 bit single precision float */
//typedef long double       float128_t;  /** 128 bit single precision float */
typedef unsigned char       bool_t;      /** boolean datatype */
typedef void                void_t;      /** void datatype */
typedef char                char_t;      /** plain char type (only used for ascii chars) */
typedef unsigned short int  uchar_t;     /** a unicode char */

typedef union Type_Word_tag
{
    struct
    {
        uint8_t   LSB;
        uint8_t   MSB;
    }
    byte;

    uint16_t word;
}   Type_Word;

#endif /*PHILIPS_TYPES_H*/

