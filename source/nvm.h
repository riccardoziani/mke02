/*******************************************************************************
Filename: nvm.h
Author: Riccardo Ziani
Created: 26/11/2014

Description:
File header for Nvm.c
< place here any information concerning the module purpose >

Usage:
< place here info for module users >
******************************************************************************/

#ifndef nvm_H
#define nvm_H
//*****************************************************************************
#include "hal.h"
#include "saeco.h"
#include "grinder.h"
#include "autodose.h"

// Include files **************************************************************

// Public definitions *********************************************************

// Public typedefs ************************************************************
typedef struct nvm_tag                                          
{                                                       //  description
/*  0  */   uint16_t                head_check_byte;    //  reg di controllo iniziale
/*  2  */   uint8_t                 nvm_map_version;    //  nvm_map_version;
/*  3  */   Type_NVM_STATUS_FLAGS   status_flag;        //  flag di stato della macchina
/*  4  */   uint8_t                 bu_status;          //  flag dello stato del gruppo
/*  5  */   uint8_t                 shut_down_temp;     //  Caldaia: temperatura della caldaia allo spegnimento  
/*  6  */   uint8_t                 dregCounter;        //  number of dregs left before the drawer is full */
/*  7  */   uint8_t                 _8bit_7;            //  number of dregs left before the drawer is full */
/*  8  */   Type_nvm_sas            sas;                //  autodose system structure. 40 bytes long */
/*  48  */  uint8_t                 _8bit_48;           //  
/*  49  */  uint8_t                 _8bit_49;           //  
/*  50  */  uint8_t                 _8bit_50;           //  
/*  51  */  uint8_t                 _8bit_51;           //  
/*  52  */  uint8_t                 _8bit_52;           //  
/*  53  */  uint8_t                 _8bit_53;           //  
/*  54  */  uint8_t                 _8bit_54;           //  
/*  55  */  uint8_t                 _8bit_55;           //  
/*  56  */  uint8_t                 _8bit_56;           //  
/*  57  */  uint8_t                 _8bit_57;           //  
/*  58  */  uint8_t                 _8bit_58;           //  
/*  59  */  uint8_t                 _8bit_59;           //  
/*  60  */  uint8_t                 _8bit_60;           //  
/*  61  */  uint8_t                 _8bit_61;           //  
/*  62  */  uint8_t                 _8bit_62;           //  
/*  63  */  uint8_t                 last_error_logged;  //  Sistema: ultimo errore verificatosi

                        // 16 bit                                   

/*  64 */   uint16_t                _16bit_64;          //  
/*  66 */   uint16_t                _16bit_66;          //  
/*  68 */   Type_nvm_gr             gr;                 //  24 bytes
/*  92 */   uint16_t                _16bit_92;          //  
/*  94 */   uint16_t                _16bit_94;          //  
/*  96 */   uint16_t                _16bit_96;          //  
/*  98 */   uint16_t                _16bit_98;          //  
/* 100 */   uint16_t                _16bit_arr[30];     //  60 bytes
/* 162 */   uint16_t                FirmwareVersion;    //  Sistema: versione firmware

                        // 32 bit                                   

/* 164 */   uint32_t                _32bit_arr[16];     //  64 bytes
/* 228 */   uint32_t                _32bit_228;         //  
/* 232 */   uint32_t                _32bit_232;         //  
/* 236 */   uint32_t                _32bit_236;         //  
/* 240 */   uint32_t                last_standby_time;  //  
/* 244 */   uint32_t                total_standby_time; //  

/* 248 */   uint16_t                machine_model;      //  
/* 250 */   uint16_t                total_nvm_writes;   //   Sistema: n.ro totale di scritture in Flash
/* 252 */   uint16_t                checksum;           //   Sistema: checksum
/* 254 */   uint16_t                Tail_Check_Byte;    //  
}Type_NVM;                                              


// Public data ****************************************************************
extern	Type_NVM	nvm;

// Public functions ***********************************************************

extern  void_t                      Nvm_Enable                          (void_t);
//extern  void_t                      Nvm_Disable                         (void_t);
extern  void_t                      Nvm_Format                          (void_t);



extern  void_t                      Nvm_CheckIn                         (void_t);
extern  void_t                      Nvm_ChechOut                        (void_t);
extern  void_t                      Nvm_Check_Erase_Page                (void_t);
//extern  void_t                      Nvm_Reload_Factory_Defaults         (void_t);
//extern  void_t                      Nvm_Reload_Menu_Factory_Defaults    (void_t);
extern  void_t                      Nvm_Init                            (void_t);
extern  void_t                      Nvm_Load_Network_Cntr               (void_t);
extern  uint8_t                     Nvm_Save                            (void_t);
extern  void_t                      Nvm_Check_And_Save                  (void_t);
extern  __ramfunc void_t    inline  Nvm_Wait_Busy                       (void_t);


//*****************************************************************************
#endif /* ifndef nvm_H */
