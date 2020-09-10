
/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/


/*
 *      PROJECT:   STxxxx firmware
 *      $Revision: $
 *      LANGUAGE:  ISO C99
 */

/*! \file st_types.h
 *
 *  \author
 *
 *  \brief Basic datatypes
 *
 */

#ifndef ST_TYPES_H
#define ST_TYPES_H

/*!
 * Basic datatypes are mapped to ST datatypes
 */
typedef unsigned char u8;      /*!< represents an unsigned 8bit-wide type */
typedef signed char s8;        /*!< represents a signed 8bit-wide type */
typedef unsigned short u16;    /*!< represents an unsigned 16bit-wide type */
typedef signed short s16;      /*!< represents a signed 16bit-wide type */
typedef unsigned long u32;     /*!< represents an unsigned 32bit-wide type */
typedef unsigned long long u64;/*!< represents an unsigned 64bit-wide type */
typedef signed long long s64;  /*!< represents n signed 64bit-wide type */
typedef signed long s32;       /*!< represents a signed 32bit-wide type */
typedef u16 umword;            /*!< USE WITH CARE!!! unsigned machine word: 8 bit on 8bit machines, 16 bit on 16 bit machines... */
typedef s16 smword;            /*!< USE WITH CARE!!! signed machine word: 8 bit on 8bit machines, 16 bit on 16 bit machines... */
typedef unsigned int uint;     /*!< type for unsigned integer types, useful as indices for arrays and loop variables */
typedef signed int sint;       /*!< type for integer types, useful as indices for arrays and loop variables */

extern u8  invalid_sizeof_u8 [(sizeof(u8 ) == 1)?(1):(-1)]; /*!< Ensures that the u8  type has expected size */
extern s8  invalid_sizeof_s8 [(sizeof(s8 ) == 1)?(1):(-1)]; /*!< Ensures that the s8  type has expected size */
extern u16 invalid_sizeof_u16[(sizeof(u16) == 2)?(1):(-1)]; /*!< Ensures that the u16 type has expected size */
extern s16 invalid_sizeof_s16[(sizeof(s16) == 2)?(1):(-1)]; /*!< Ensures that the s16 type has expected size */
extern u32 invalid_sizeof_u32[(sizeof(u32) == 4)?(1):(-1)]; /*!< Ensures that the u32 type has expected size */
extern s32 invalid_sizeof_s32[(sizeof(s32) == 4)?(1):(-1)]; /*!< Ensures that the s32 type has expected size */
extern u64 invalid_sizeof_u64[(sizeof(u64) == 8)?(1):(-1)]; /*!< Ensures that the u64 type has expected size */
extern s64 invalid_sizeof_s64[(sizeof(s64) == 8)?(1):(-1)]; /*!< Ensures that the s64 type has expected size */

#define U8_C(x)     (x)     /*!< Define a constant of type u8 */
#define S8_C(x)     (x)     /*!< Define a constant of type s8 */
#define U16_C(x)    (x)     /*!< Define a constant of type u16 */
#define S16_C(x)    (x)     /*!< Define a constant of type s16 */
#define U32_C(x)    (x##UL) /*!< Define a constant of type u32 */
#define S32_C(x)    (x##L)  /*!< Define a constant of type s32 */
#define U64_C(x)    (x##ULL)/*!< Define a constant of type u64 */
#define S64_C(x)    (x##LL) /*!< Define a constant of type s64 */
#define UMWORD_C(x) (x)     /*!< Define a constant of type umword */
#define MWORD_C(x)  (x)     /*!< Define a constant of type mword */

#if 1
typedef umword bool_t; /*!< represents a boolean type */

#ifndef TRUE
#define TRUE 1 /*!< used for the #bool_t type */
#endif /* !TRUE */
    
#ifndef FALSE
#define FALSE 0 /*!< used for the #bool_t type */
#endif /* !FALSE */

#else 
typedef BOOL bool_t;
#endif /* 1 */


#ifndef NULL
#define NULL (void*)0 /*!< represents a NULL pointer */
#endif /* !NULL */


#endif /* ST_TYPES_H */

