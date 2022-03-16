/*******************************************************************************
Filename: nvm_cgf.h.h
Author: Riccardo Ziani
Created: 22/03/2018

Description:
File header for nvm_cgf.h.c
< place here any information concerning the module purpose >

Usage:
< place here info for module users >
******************************************************************************/

#ifndef nvm_cgf_H
#define nvm_cgf_H
//*****************************************************************************

// Include files **************************************************************

// Public definitions *********************************************************

// Public typedefs ************************************************************

// Public data ****************************************************************

// Public functions ***********************************************************

#define FLASH_PAGE_SIZE             (512)                               // Flash page size in byte
#define FLASH_SECTORS               (128)                               // Numbers of flash sectors
#define NVM_BANK_SIZE               (256)                               // Dimension of one bank of the nvm, in bytes
#define END_MEMORY                  (uint8_t*)(FLASH_PAGE_SIZE * FLASH_SECTORS)

#define PAGE0_ADDR                  (uint8_t*)(END_MEMORY - 1024)       // Adderss 0 of the first page of the nvm
#define PAGE1_ADDR                  (uint8_t*)(END_MEMORY - 512)        // Adderss 0 of the second page of the nvm

#define SIZE_PAGE0                  (PAGE1_ADDR - PAGE0_ADDR)           // Dimension of the first nvm page
#define SIZE_PAGE1                  (END_MEMORY - PAGE1_ADDR)           // Dimension of the second nvm page
#define SIZE_PAGE                   (SIZE_PAGE0)                        

#define NUMBER_OF_BANKS_IN_PAGE0    (SIZE_PAGE0/NVM_BANK_SIZE)          // 2(four) banks in page 1
#define NUMBER_OF_BANKS_IN_PAGE1    (SIZE_PAGE1/NVM_BANK_SIZE)          // 2(four) banks in page 2


#define NUMBER_OF_BANKS             (NUMBER_OF_BANKS_IN_PAGE0\
                                    + NUMBER_OF_BANKS_IN_PAGE1)         // 4
#define NVM_SIZE                    (4*NVM_BANK_SIZE)                   

#define NVM_MEMORY                  ((volatile uint16_t *)FLASH_ADDR)   

#define TIMEOUT_WRITE               (3)                                 // Number of zx lost, after that the system saves buffer page in nvm...
#define ZC_FILTER                   (50)                                // Number of zx to wait from a write to another.
#define HEAD_CHECK_BYTE             (0xAABB)                            // First half word of the nvm buffer, use by nvm algorithm 
#define TAIL_CHECK_BYTE             (0xCCDD)                            // Last half word of the nvm buffer, use by nvm algorithm


#define NVM_ALLOW_WRITE             (0xAA)                              // Key to allow nvm writes
#define NVM_DONT_ALLOW_WRITE        (0x55)                              // Key to not allow nvm writes on nvm

#define NVM_MAP_VERSION_0x1         (0x01)                              // Nunmer of Memory Map version
#define NVM_MAP_VERSION             (NVM_MAP_VERSION_0x1)

#define GetChkFlashBank(buff)       (*(uint16_t*)(uint8_t*)(buff + NVM_BANK_SIZE - 4))

#if NVM_SIZE%256
  #error nvm size does not supported by module.
#endif






//*****************************************************************************
#endif /* ifndef nvm_cgf_H */


