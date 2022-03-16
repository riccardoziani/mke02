/*******************************************************************************
Filename: Nvm_lmr.c
Author: Riccardo Ziani
Created: 26/11/2014
Description:

File di implemantazione dalla NVM.
La scrittura avviena su flash, in pagine predisposte e con un numero di banchi
per pagina precentemente scelti.
Le scritture avvengono sempre allo spegnimento della macchina, catturando il
momento con l'ausilio della frquenza di rete che è il primo segnale che viene
a mancare nel momento dello spegnimento.
Nei banchi non è presente un numero progressivo per indicare quale è stato
l'ultimo    scritto. Questo è possibile perche la ricerca dell'ultimo banco scritto
si basa sul fatto che esso deve avere davanti un banco effettivanete vuoto,
se cio avviene sapendo il meccanismo di scrittura utilizzato si puo affermare
che è sicuramente stato l'ultimo scritto.
Questa affermazione si puo fare perche appena viene trovato un banco buono
in una pagina l'altra verra sempre pulita. In questo modo si garantisce
sempre una scrittura con almeno un banco pulito davanti, facendo funzionare
sempre il meccanismo di ricerca.

*******************************************************************************/
// include files
#include "hal.h"
#include "nvm.h"
#include "crc.h"
#include "flash.h"
#include "nvm_cfg.h"

// private definitions *********************************************************

//#if defined(CPU_MKE02Z64VQH4)
//#else
//#error Unsupported device.
//#endif



// private typedefs ************************************************************
typedef struct
{
    uint8_t NVM_enable;
    uint8_t enable_write;       // Flag per abilitare la scrittura
    uint8_t read_OK;
    uint8_t *ptr_read;          // Contiene il puntatore del banco attualmente letto
    uint8_t *ptr_write;         // Contiene il puntatore in flash
    uint8_t timer;              // Contatore che si decrementa ogni 10ms
    uint8_t zero_crossing_cntr; // Contatore che si decrementa ogni zc
}Type_nvm_control;

// global data *****************************************************************

__attribute__((__aligned__(4)))
Type_NVM    nvm;

// private data ****************************************************************

static  volatile    Type_nvm_control    nvm_control;
static  volatile    uint32_t            nvm_flash_bits;
static  volatile    uint32_t            write_timer;
static  volatile    uint32_t            erase_timer;

static  volatile    uint32_t            size_nvm = sizeof(nvm);

static              uint8_t*            page0_addr;
static              uint8_t*            page1_addr;
static              uint32_t            flash_size;
static  volatile    uint32_t            flash_address;
static              uint32_t*           p_flash;
static              uint32_t*           p_nvm;

static __root  const Type_NVM nvm_defaults =
{
/*  0  */   HEAD_CHECK_BYTE,                    //  reg di controllo iniziale
/*  2  */   NVM_MAP_VERSION,                    //  nvm_map_version;
/*  3  */   STATUSFLAG,                         //  flag di stato della macchina
/*  4  */   0,                                  //  flag dello stato del gruppo
/*  5  */   25,                                 //  Caldaia: temperatura della caldaia allo spegnimento
/*  6  */   MAX_DREG_NUMBER,                    //  dreg number to fill the drawer
/*  7  */   0,                                  //  
/*  8  */   DEFAULT_NVM_SAS_STRUCTURE,          //  autodose system structure. 40 bytes
/* 48  */   0,                                  //  
/* 49  */   0,                                  //  
/* 50  */   0,                                  //  
/* 51  */   0,                                  //  
/* 52  */   0,                                  //  
/* 53  */   0,                                  //  
/* 54  */   0,                                  //  
/* 55  */   0,                                  //  
/* 56  */   0,                                  //
/* 57  */   0,                                  //
/* 58  */   0,                                  //
/* 59  */   0,                                  //
/* 60  */   0,                                  //
/* 61  */   0,                                  //
/* 62  */   0,                                  //
/* 63  */   0,                                  //

                    // 16 bit                   

/* 64  */   0,                                  //  
/* 66  */   0,                                  //  
/* 68  */   DEFAULT_NVM_GRINDER_STRUCTURE,      //  24 bytes
/* 92  */   0,                                  
/* 94  */   0,                                  //  
/* 96  */   0,                                  //  
/* 98  */   0,                                  //  
/* 100 */   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,    // 16 bytes
            0,0,0,0,0,0,0,0,0,0,0,0,0,0,        // 14 Bytes
/* 162 */   0,                                  // Sistema: versione firmware

                    // 32 bit                   

/* 164 */   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,    // 64 bytes    

/* 228 */   0,                                  //
/* 232 */   0,                                  //
/* 236 */   0,                                  //
/* 240 */   0,                                  //
/* 244 */   0,                                  //

                    // 16 bit

/* 248 */   MACHINE_MODEL,                      // Modello macchina
/* 250 */   0,                                  // Sistema: n.ro totale di scritture in Flash
/* 252 */   0,                                  // Sistema: checksum
/* 254 */   TAIL_CHECK_BYTE,                    // Coda di fine struttura
};                                              


static __root volatile  const  uint16_t nvm_address_buffer[][2] =
{                                                                                                           //  address size 
/*  0 */    {(uint8_t*)&    nvm.head_check_byte     - (uint8_t*)&nvm,   sizeof( nvm.head_check_byte     )}, //   0      2    
/*  1 */    {(uint8_t*)&    nvm.nvm_map_version     - (uint8_t*)&nvm,   sizeof( nvm.nvm_map_version     )}, //   2      1    
/*  2 */    {(uint8_t*)&    nvm.status_flag         - (uint8_t*)&nvm,   sizeof( nvm.status_flag         )}, //   3      1    
/*  3 */    {(uint8_t*)&    nvm.bu_status           - (uint8_t*)&nvm,   sizeof( nvm.bu_status           )}, //   4      1    
/*  4 */    {(uint8_t*)&    nvm.shut_down_temp      - (uint8_t*)&nvm,   sizeof( nvm.shut_down_temp      )}, //   5      1    
/*  5 */    {(uint8_t*)&    nvm.dregCounter         - (uint8_t*)&nvm,   sizeof( nvm.dregCounter         )}, //   6      1    
/*  6 */    {(uint8_t*)&    nvm._8bit_7             - (uint8_t*)&nvm,   sizeof( nvm._8bit_7             )}, //   7      1    
/*  7 */    {(uint8_t*)&    nvm.sas                 - (uint8_t*)&nvm,   sizeof( nvm.sas                 )}, //   8      24   
/*  8 */    {(uint8_t*)&    nvm._8bit_48            - (uint8_t*)&nvm,   sizeof( nvm._8bit_48            )}, //  48      1    
/*  9 */    {(uint8_t*)&    nvm._8bit_49            - (uint8_t*)&nvm,   sizeof( nvm._8bit_49            )}, //  49      1    
/* 10 */    {(uint8_t*)&    nvm._8bit_50            - (uint8_t*)&nvm,   sizeof( nvm._8bit_50            )}, //  50      1    
/* 11 */    {(uint8_t*)&    nvm._8bit_51            - (uint8_t*)&nvm,   sizeof( nvm._8bit_51            )}, //  51      1    
/* 12 */    {(uint8_t*)&    nvm._8bit_52            - (uint8_t*)&nvm,   sizeof( nvm._8bit_52            )}, //  52      1    
/* 13 */    {(uint8_t*)&    nvm._8bit_53            - (uint8_t*)&nvm,   sizeof( nvm._8bit_53            )}, //  53      1    
/* 14 */    {(uint8_t*)&    nvm._8bit_54            - (uint8_t*)&nvm,   sizeof( nvm._8bit_54            )}, //  54      1    
/* 15 */    {(uint8_t*)&    nvm._8bit_55            - (uint8_t*)&nvm,   sizeof( nvm._8bit_55            )}, //  55      1    
/* 16 */    {(uint8_t*)&    nvm._8bit_56            - (uint8_t*)&nvm,   sizeof( nvm._8bit_56            )}, //  56      1    
/* 17 */    {(uint8_t*)&    nvm._8bit_57            - (uint8_t*)&nvm,   sizeof( nvm._8bit_57            )}, //  57      1    
/* 18 */    {(uint8_t*)&    nvm._8bit_58            - (uint8_t*)&nvm,   sizeof( nvm._8bit_58            )}, //  58      1    
/* 19 */    {(uint8_t*)&    nvm._8bit_59            - (uint8_t*)&nvm,   sizeof( nvm._8bit_59            )}, //  59      1    
/* 20 */    {(uint8_t*)&    nvm._8bit_60            - (uint8_t*)&nvm,   sizeof( nvm._8bit_60            )}, //  60      1   
/* 21 */    {(uint8_t*)&    nvm._8bit_61            - (uint8_t*)&nvm,   sizeof( nvm._8bit_61            )}, //  61      1    
/* 22 */    {(uint8_t*)&    nvm._8bit_62            - (uint8_t*)&nvm,   sizeof( nvm._8bit_62            )}, //  62      1    
/* 23 */    {(uint8_t*)&    nvm.last_error_logged   - (uint8_t*)&nvm,   sizeof( nvm.last_error_logged   )}, //  63      1    
/* 24 */    {(uint8_t*)&    nvm._16bit_64           - (uint8_t*)&nvm,   sizeof( nvm._16bit_64           )}, //  64      2    
/* 25 */    {(uint8_t*)&    nvm._16bit_66           - (uint8_t*)&nvm,   sizeof( nvm._16bit_66           )}, //  66      2    
/* 26 */    {(uint8_t*)&    nvm.gr                  - (uint8_t*)&nvm,   sizeof( nvm.gr                  )}, //  68      40    
/* 27 */    {(uint8_t*)&    nvm._16bit_92           - (uint8_t*)&nvm,   sizeof( nvm._16bit_92           )}, //  92      2    
/* 28 */    {(uint8_t*)&    nvm._16bit_94           - (uint8_t*)&nvm,   sizeof( nvm._16bit_94           )}, //  94      2    
/* 29 */    {(uint8_t*)&    nvm._16bit_96           - (uint8_t*)&nvm,   sizeof( nvm._16bit_96           )}, //  96      2    
/* 30 */    {(uint8_t*)&    nvm._16bit_98           - (uint8_t*)&nvm,   sizeof( nvm._16bit_98           )}, //  98      2    
/* 31 */    {(uint8_t*)&    nvm._16bit_arr          - (uint8_t*)&nvm,   sizeof( nvm._16bit_arr          )}, //  100     60    
/* 32 */    {(uint8_t*)&    nvm.FirmwareVersion     - (uint8_t*)&nvm,   sizeof( nvm.FirmwareVersion     )}, //  162     2   
/* 33 */    {(uint8_t*)&    nvm._32bit_arr          - (uint8_t*)&nvm,   sizeof( nvm._32bit_arr          )}, //  164     64    
/* 34 */    {(uint8_t*)&    nvm._32bit_228          - (uint8_t*)&nvm,   sizeof( nvm._32bit_228          )}, //  228     4    
/* 35 */    {(uint8_t*)&    nvm._32bit_232          - (uint8_t*)&nvm,   sizeof( nvm._32bit_232          )}, //  232     4    
/* 36 */    {(uint8_t*)&    nvm._32bit_236          - (uint8_t*)&nvm,   sizeof( nvm._32bit_236          )}, //  236     4    
/* 37 */    {(uint8_t*)&    nvm.last_standby_time   - (uint8_t*)&nvm,   sizeof( nvm.last_standby_time   )}, //  240     4    
/* 38 */    {(uint8_t*)&    nvm.total_standby_time  - (uint8_t*)&nvm,   sizeof( nvm.total_standby_time  )}, //  244     4    
/* 39 */    {(uint8_t*)&    nvm.machine_model       - (uint8_t*)&nvm,   sizeof( nvm.machine_model       )}, //  248     2    
/* 40 */    {(uint8_t*)&    nvm.total_nvm_writes    - (uint8_t*)&nvm,   sizeof( nvm.total_nvm_writes    )}, //  250     2    
/* 41 */    {(uint8_t*)&    nvm.checksum            - (uint8_t*)&nvm,   sizeof( nvm.checksum            )}, //  252     2    
/* 42 */    {(uint8_t*)&    nvm.Tail_Check_Byte     - (uint8_t*)&nvm,   sizeof( nvm.Tail_Check_Byte     )}, //  254     2    
};                                                                                                                 




// private functions ***********************************************************                   
static              void_t      Nvm_Page_Erase          (uint8_t* pFlashPage);     
static              void_t      Nvm_Read                (uint8_t* pFlashBank);     
static              uint16_t    Nvm_Make_Chk            (uint8_t* buff);
static              uint8_t     Nvm_Empty_Buffer_Check  (uint8_t* buff, uint8_t value, uint16_t size);
static              void_t      Nvm_Check_Boundary      (volatile uint8_t** ptr);
static              uint8_t     Nvm_Check_Head_And_Tail (uint16_t* ptr);

//static __ramfunc    void_t      Nvm_Page_Erase_Row      (void_t);
//static __ramfunc    void_t      nvm_Write_bank         (void_t);



//******************************************************************************
// Implementation
//******************************************************************************




////******************************************************************************
//// Function: Nvm_Page_Erase_Row
//// Created: 30/12/2014
//// Parameters:   None
//// Returns:      None
//// Description:  < place here any information concerning the function purpose >
////
////******************************************************************************
//static __ramfunc void_t Nvm_Page_Erase_Row (void_t)
//{
//    Nvm_Wait_Busy();
//
//    /* Clear error flags */
//    //nvm_module->STATUS.reg |= NVMCTRL_STATUS_MASK;
//
//    /* Set address and command */
//    //nvm_module->ADDR.reg  = flash_address >> 1;
//
//    /* Set command */
//    //nvm_module->CTRLA.reg = NVMCTRL_CTRLA_CMD_ER | NVMCTRL_CTRLA_CMDEX_KEY;
//
//    Nvm_Wait_Busy();
//
//} // end of function Nvm_Page_Erase_Row

//******************************************************************************
// Function: nvm_Write_bank
// Created: 30/12/2014
// Parameters:   None
// Returns:      None
// Description:  < place here any information concerning the function purpose >
//
//******************************************************************************
static __ramfunc void_t nvm_Write_bank (void_t)
{
    uint8_t jj;
    volatile uint32_t primask;
    uint32_t register u32NVMTargetAddress;
    uint32_t register u32DwData0;
    uint32_t register u32DwData1;



    jj = 32;
    while(jj--)
    {
        u32NVMTargetAddress = (uint32_t)p_flash;
        u32DwData0 = *p_nvm++;
        u32DwData1 =  *p_nvm;

        // Clear error flags
        FTMRH->FSTAT = 0x30;

        // Write index to specify the command code to be loaded
        FTMRH->FCCOBIX = 0x0;
        // Write command code and memory address bits[23:16]	
        FTMRH->FCCOBHI = FLASH_CMD_PROGRAM;// program FLASH command
        FTMRH->FCCOBLO = u32NVMTargetAddress>>16;// memory address bits[23:16]
        // Write index to specify the lower byte memory address bits[15:0] to be loaded
        FTMRH->FCCOBIX = 0x1;
        // Write the lower byte memory address bits[15:0]
        FTMRH->FCCOBLO = u32NVMTargetAddress;
        FTMRH->FCCOBHI = u32NVMTargetAddress>>8;

        // Write index to specify the word0 (MSB word) to be programmed
        FTMRH->FCCOBIX = 0x2;
        // Write the word 0
        FTMRH->FCCOBHI = (u32DwData0) >>8;
        FTMRH->FCCOBLO = (u32DwData0);

        // Write index to specify the word1 (LSB word) to be programmed
        FTMRH->FCCOBIX = 0x3;
        // Write word 1
        FTMRH->FCCOBHI = (u32DwData0>>16)>>8;
        FTMRH->FCCOBLO = (u32DwData0>>16);

        // Write index to specify the word0 (MSB word) to be programmed
        FTMRH->FCCOBIX = 0x4;
        // Write the word2
        FTMRH->FCCOBHI = (u32DwData1) >>8;
        FTMRH->FCCOBLO = (u32DwData1);        

        // Write index to specify the word1 (LSB word) to be programmed
        FTMRH->FCCOBIX = 0x5;
        // Write word 3
        FTMRH->FCCOBHI = (u32DwData1>>16)>>8;
        FTMRH->FCCOBLO = (u32DwData1>>16);

        primask = DisableGlobalIRQ();   // Save interrupt status and disable it.
        write_timer  = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1);


        MCM->PLACR |= MCM_PLACR_ESFC_MASK;  // enable stalling flash controller when flash is busy
        FTMRH->FSTAT = 0x80;    
        while (!(FTMRH->FSTAT & FTMRH_FSTAT_CCIF_MASK));  // Wait till command is completed


        write_timer = (write_timer - PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1)) >> 4; // useconds..
        EnableGlobalIRQ(primask);   // Restaore interrupts status.
        
        p_flash += 2;
        p_nvm++;
    }

} // end of function nvm_Write_bank

////******************************************************************************
//// Function: Nvm_Get_Flash_App_Dim
//// Created: 03/04/2015
//// Parameters:   None
//// Returns:      None
//// Description:  < place here any information concerning the function purpose >
////
////******************************************************************************
//uint32_t Nvm_Get_Flash_App_Dim (void_t)
//{
//    return (uint32_t)page0_addr;
//} // end of function Nvm_Get_Flash_App_Dim

//******************************************************************************
// Function: Nvm_Enable
// Created: 28/06/2013
// Parameters:
// Returns:
// Description:
//
//******************************************************************************
void_t Nvm_Enable (void_t)
{
    nvm_control.NVM_enable = NVM_ALLOW_WRITE;
} // end of function Nvm_Enable


////******************************************************************************
//// Function: Nvm_Disable
//// Created: 28/06/2013
//// Parameters:
//// Returns:
//// Description:
////
////******************************************************************************
//void_t Nvm_Disable (void_t)
//{
//    nvm_control.NVM_enable = NVM_DONT_ALLOW_WRITE;
//} // end of function Nvm_Disable


//******************************************************************************
// Function: Nvm_Page_Erase
// Created: 30/12/2014
// Parameters:   None
// Returns:      None
// Description:  < place here any information concerning the function purpose >
//
//******************************************************************************
void_t Nvm_Page_Erase (uint8_t* pFlashPage)
{
	uint16_t flash_err;
    volatile uint32_t primask;

//  primask = DisableGlobalIRQ();   // Save interrupt status and disable it.
//
//
//  write_timer  = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1);
//  flash_err  = FLASH_Program((uint32_t)pFlashPage, (uint8_t*)0x00001000, 0xfff);
//  write_timer = (write_timer - PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1)) >> 4; // useconds..
//
//  EnableGlobalIRQ(primask);   // Restaore interrupts status.

    if (!Nvm_Empty_Buffer_Check (pFlashPage, 0xFF, SIZE_PAGE))
    {
        if (pFlashPage == page0_addr || pFlashPage == page1_addr )
        {
            flash_address = (uint32_t)pFlashPage;

            primask = DisableGlobalIRQ();   // Save interrupt status and disable it.

            erase_timer  = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1);
            flash_err  = FLASH_EraseSector(flash_address);
            erase_timer = (erase_timer - PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1)) >> 4; // useconds..

            EnableGlobalIRQ(primask);   // Restaore interrupts status.



            // ...Se ci sono interrupt pendenti qui vengono serviti...



            flash_address += FLASH_PAGE_SIZE;

            primask = DisableGlobalIRQ();   // Save interrupt status and disable it.

            erase_timer  = PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1);
            flash_err  = FLASH_EraseSector(flash_address);
            erase_timer = (erase_timer - PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1)) >> 4; // useconds..

            EnableGlobalIRQ(primask);   // Restaore interrupts status.
        }	// Con una flash nuova impiega circa 1700 u secondi...
    }
} // end of function Nvm_Page_Erase


//=============================================================================
// Name : Nvm_Check_Erase_Page
//      =====================================================
// Purpose: Se ho prelevato da una pagina che contiene almeno un banco
//          coerente, allora posso cancellare quella opposta.
//          La Nvm_Page_Erase cancellera soltanto se la pagina non è
//              gia pulita, altrimenti ritornera senza interventi
// Inputs:  none
// Returns: none
//=============================================================================
void_t Nvm_Check_Erase_Page (void_t)
{
    if (nvm_control.ptr_read >= page1_addr)
    {
        // Se sono in pagina 1 con almeno un banco coerente...
        Nvm_Page_Erase(page0_addr);         // Se necessario, cancello pagina 0
    }
    else
    {
        Nvm_Page_Erase(page1_addr);         // Se necessario, cancello pagina 1
    }
}

//=============================================================================
// Name :  Nvm_Read
//      =====================================================
// Description:
// carica la struttura eeprombuffer partendo dall'indirizzo
// pFlashBank per la lunghezza della costante NVM_BANK_SIZE
//=============================================================================
void_t Nvm_Read (uint8_t* pFlashBank)
{
    uint16_t ii;
    for (ii = 0 ; ii < NVM_BANK_SIZE; ii++ )
    {
        // carica ram con flash!
        *((uint8_t*)&nvm + ii) = *(pFlashBank + ii);
    }
}

//=============================================================================
// Name :  Nvm_Make_Chk
//      =====================================================
// Description:
// Calcola la chechsum del buffer passato in argomento
//
//=============================================================================
static uint16_t Nvm_Make_Chk (uint8_t* buff)
{
    return (CRC_Cal16(0xFFFF, buff, NVM_BANK_SIZE - 5));    
}

//=============================================================================
// Name :  Nvm_Empty_Buffer_Check
//      =====================================================
// Description:
// Effettua la comparazione di un buffer con un valore dato
// per una lunghezza size.
//
//=============================================================================
uint8_t Nvm_Empty_Buffer_Check (uint8_t* buff, uint8_t value, uint16_t size)
{
    for (;size;size-- )
    {
        if (*buff++ != value)
        {
            return(false);
        }
    }
    return (true);
}

//=============================================================================
// Name :  Nvm_Check_Boundary
//      =====================================================
// Description:
// Controlla il puntatore passato, se necessario rolla.
// Il uint8_t** è necessario perche devo avanzare un puntatore(Okkiiooo!!)
//=============================================================================
void_t Nvm_Check_Boundary (volatile uint8_t**  ptr)
{
    if (*ptr >= (uint8_t*)flash_size)
    {
        *ptr = page0_addr;              // Rolla...
    }
}

//=============================================================================
// Name :  Nvm_Check_Head_And_Tail
//      =====================================================
// Description:
//=============================================================================
uint8_t Nvm_Check_Head_And_Tail (uint16_t* ptr)
{
    if (*ptr == HEAD_CHECK_BYTE && *(ptr + sizeof(nvm)/2 - 1) == TAIL_CHECK_BYTE)
    {
        return (true);
    }
    else
    {
        return (false);
    }
}




// Public functions ***********************************************************


////******************************************************************************
//// Function: Nvm_Wait_Busy
//// Created: 26/03/2015
//// Parameters:   None
//// Returns:      None
//// Description:  < place here any information concerning the function purpose >
////
////******************************************************************************
//__ramfunc void_t  inline Nvm_Wait_Busy (void_t)
//{
//    /* Check if the module is busy */
//    //while(!nvm_module->INTFLAG.reg & NVMCTRL_INTFLAG_READY);
//    __asm("nop");
//
//} // end of function Nvm_Wait_Busy


//=======================================================================================
// Name : Nvm_CheckIn
//=======================================================================================
// Description: This func saves several vars on virtual e2
//
//=======================================================================================
void_t  Nvm_CheckIn   (void_t)
{
//  nvm.bu_status                       = (uint8_t) BU_Status();
//  nvm.shut_down_temp                  = Get_CoffeeHeater_Temperature_x10_C()/10;
//  nvm.status_flag.bit.alarm_refill    = Is_Pmp_AlarmRefill();
}


//=======================================================================================
// Name : Nvm_ChechOut
//=======================================================================================
// Description: This func recovers several variables from virtual e2
//
//=======================================================================================
void_t  Nvm_ChechOut   (void_t)
{
    if (nvm.nvm_map_version < NVM_MAP_VERSION)
    {
        //Gr_Format_Parameter();
        //Gr_Reset_mills_counter();
        nvm.nvm_map_version = NVM_MAP_VERSION;
    }

//  Set_BU_Status(nvm.bu_status);
//  Set_Brewing_Unit_State(nvm.status_flag.bit.Brwing_Unit_Warmed == true);
//  Set_Pmp_AlarmRefill(nvm.status_flag.bit.alarm_refill);
    //aroma_selection     = nvm.aroma;
}

////=======================================================================================
//// Name : Nvm_Reload_Menu_Factory_Defaults
////=======================================================================================
//// Description:
////
////=======================================================================================
//void_t  Nvm_Reload_Menu_Factory_Defaults   (void_t)
//{
//    //nvm.espresso_qty                    =   DEFAULT_PULSES_ESPRESSO;
//    //nvm.espresso_lungo_qty              =   DEFAULT_PULSES_ESPRESSO_LUNGO;
//    //nvm.caffe_crema_qty                       =  DEFAULT_PULSES_CAFFE_CREMA;
//    //nvm.coffee_intense_qty              = DEFAULT_PULSES_COFFEE_INTENSE; // Eliminato dalla lista prodotti...
//    //nvm.ristretto_qty                   =   DEFAULT_PULSES_RISTRETTO;
//    //nvm.espresso_double_qty             =   DEFAULT_PULSES_ESPRESSO_DOUBLE;
//    //nvm.cappuccino_qty                  =   DEFAULT_PULSES_CAPPUCCINO;
//    //nvm.cappuccino_seconds              =   DEFAULT_SECONDS_CAPPUCCINO;
//    //nvm.latte_macchiato_qty             =   DEFAULT_PULSES_LATTE_MACCHIATO;
//    //nvm.latte_macchiato_seconds         =   DEFAULT_SECONDS_LATTE_MACCHIATO;
//    //nvm.baby_cappuccino_qty             =   DEFAULT_PULSES_BABY_CAPPUCCINO;
//    //nvm.baby_cappuccino_seconds         =   DEFAULT_SECONDS_BABY_CAPPUCCINO;
//    //nvm.flat_white_qty                    =   DEFAULT_PULSES_FLAT_WHITE;
//    //nvm.flat_white_seconds              =   DEFAULT_SECONDS_FLAT_WHITE;
//    //nvm.milk_froth_seconds              =   DEFAULT_SECONDS_MILK_FROTH;
//    //nvm.steam_seconds                   =   DEFAULT_SECONDS_STEAM;
//    //nvm.water_qty                     =   DEFAULT_PULSES_HOT_WATER;
//
//
//    //nvm.aroma                           =   DEFAULT_e_AROMA ;
//    //aroma_selection                     =   DEFAULT_e_AROMA ;
//
//    //nvm.water_treatment.water_hardness  =   DEFAULT_WATER_HARDNESS;
//    //nvm.time_sleep                      =   DEFAULT_s_TIME_SLEEP;
//
//    //nvm.display_contrast                =   VOLUME_NOVATEK;
//
//    //nvm.user_coffee_temperature         =   DEFAULT_COFFEE_TEMPERATURE;
//    Nvm_CheckIn();
//}

////=======================================================================================
//// Name : Nvm_Reload_Factory_Defaults
////=======================================================================================
//// Description:
////
////=======================================================================================
//void_t  Nvm_Reload_Factory_Defaults   (void_t)
//{
//    nvm = nvm_defaults;                             // Load default parameters
//}

//=======================================================================================
// Name : Nvm_Format
//=======================================================================================
// Description: carica i default e poi faomatta la e2prom
//
//=======================================================================================
void_t Nvm_Format (void_t)
{
    Nvm_Page_Erase(page0_addr); // Pulisco tutto a prescindere
    Nvm_Page_Erase(page1_addr); // Pulisco tutto a prescindere
    nvm_control.ptr_write = page0_addr; // Inizializzazioni
    Nvm_Enable();
    (void_t)Nvm_Save();
    //Nvm_Disable();
    nvm_control.NVM_enable = NVM_DONT_ALLOW_WRITE;
}

//=============================================================================
// Name :  Nvm_Init
//      =====================================================
// Description:
//=============================================================================
void_t Nvm_Init (void_t)
{
    uint8_t* local_ptr_read;
    uint8_t banks = NUMBER_OF_BANKS + 1;


    (void_t)nvm_address_buffer;
    if (size_nvm != NVM_BANK_SIZE)
    {
        asm ("BKPT #0");
    }

    flash_size = FLASH_PAGE_SIZE*FLASH_SECTORS;
    page0_addr = (uint8_t*)(flash_size - (SIZE_PAGE0 + SIZE_PAGE1));
    page1_addr = (uint8_t*)(flash_size - SIZE_PAGE1);


    local_ptr_read = page0_addr;
    nvm_control.ptr_write = page0_addr;
    nvm_control.read_OK = false;

    //if (FLASH_Init(CLOCK_GetBusClkFreq()) != FLASH_ERR_SUCCESS)
    if (FLASH_Init(SystemCoreClock) != FLASH_ERR_SUCCESS)
    {
        asm ("BKPT #0");
    }                       


    // Inizio ricerca..
    for (;banks ;banks-- )
    {
        if (Nvm_Check_Head_And_Tail((uint16_t*)local_ptr_read))
        {
            // Se il banco è scritto...
            if (Nvm_Make_Chk(local_ptr_read) == GetChkFlashBank(local_ptr_read))
            {
                // controllo di coerenza...
                // se coerente leggo il banco e setto che è buono
                Nvm_Read(local_ptr_read);
                nvm_control.ptr_read = local_ptr_read;    // Memorizzo puntatore banco buono...
                nvm_control.read_OK = true;   // Setto banco buono letto.
            }
        }
        else
        {
            if (Nvm_Empty_Buffer_Check (local_ptr_read, 0xFF, NVM_BANK_SIZE))
            {
                // Banco effettivamete vuoto...
                nvm_control.ptr_write = local_ptr_read;   // Preparazione puntatore per prox scrittura.
                if (nvm_control.read_OK)
                {
                    // Se avevo già prelevato un banco buono...
                    // Adesso sono sicuro che l'ultimo banco prelevato è stato
                    // sicuramente anche l'ultimo scritto precentemente.
                    // Ora controllo se ho un cambio di memory map oppure un cambio di macchina(ENC o TOP)
                    if (nvm.machine_model != MACHINE_MODEL)
                    {   // Banchi coerenti..E' solo un cambio di memoria, oppure tastiera...
                        nvm = nvm_defaults;         // Load default parameters
                        //nvm.machine_model = MACHINE_MODEL;     // NVM type Top or enhanced, Hw dependent
                        break;  // Esci dal for e vai a formattare con parametri di default...
                    }
                    else
                    {
                        Nvm_Check_Erase_Page(); // Controlla se è possibile pulire una pagina
                        Nvm_ChechOut();
                        return;                 // Esci con successo.
                    }
                }
            }
        }

        local_ptr_read += NVM_BANK_SIZE;
        Nvm_Check_Boundary((volatile uint8_t**)&local_ptr_read);
    }

    if (!nvm_control.read_OK)
    {
        // ...Neanche un banco buono!
        nvm = nvm_defaults; // Load default parameters
        //nvm.machine_model = MACHINE_MODEL;
    }
    Nvm_Format();
    Nvm_ChechOut();
}


//=============================================================================
// Name : Nvm_Load_Network_Cntr
//      =====================================================
// Purpose:
// Controlla se è attiva l'abilitazione a scrivere su flash, se si,
// carica il timer con n° TIMEOUT_WRITE eventi, se no aspetta ZC_FILTER
// eventi prima di riabilitare la scrittura su flash.
// Alla scadenza del filtro di z_crossing lancia la Nvm_Page_Erase.
//
// Inputs:  none
//
// Returns: none
//
// Notes:  ATTENZIONE!!!! Questa funzione va chiamata all'interno
// dell'interrupt di zero crossing!!!
//
//=============================================================================
void_t Nvm_Load_Network_Cntr (void_t)
{
    nvm_control.timer = TIMEOUT_WRITE;        // ricarico sempre timer!!
    if (nvm_control.enable_write == false)
    {
        // se sto aspettando di cancellare...
        if (nvm_control.zero_crossing_cntr > ZC_FILTER)
        {
            // Ok filtro superato ora puoi scrivere
            nvm_control.enable_write = true;
        }
        else
        {
            ++nvm_control.zero_crossing_cntr;
        }
    }
}

//=============================================================================
// Name : Nvm_Save
//      =====================================================
//    Dapprima controlla l'integrita del banco in ram, poi salva ptr_write,
//      genera la chk del banco e lo scrive in flash. Poi lo rilegge e se
//      tutto ok ritorna vero. Prima di ritornare posiziona ptr_write per
//      una eventuale prossima scrittura.
//
// Returns: True se la scrittura è andata a buon fine False altrimenti
//=============================================================================
uint8_t Nvm_Save (void_t)
{
    uint8_t jj = 64;    // okkio 64 word 256 byte!!!
    uint8_t result = false;
    register uint32_t tmp;

    if (nvm_control.NVM_enable == NVM_ALLOW_WRITE)
    {
		tmp = (uint32_t)nvm_control.ptr_write;
        if (Nvm_Check_Head_And_Tail((uint16_t*)&nvm)    &&  // Testa e coda integri
            tmp%NVM_BANK_SIZE == 0                      &&  // Puntatore in write a multiplo di banco...
            nvm_control.ptr_write >= page0_addr         &&  // Puntatore dentro gli addresses della NVM
            nvm_control.ptr_write < (uint8_t*)flash_size)
        {
            // ...Per scrivere pretendo le condizioni sopraelencate...
            nvm_control.ptr_read = nvm_control.ptr_write;
            nvm.total_nvm_writes++;                      // Incremento il contatore delle scritture(solo curiosita)
            nvm.checksum = Nvm_Make_Chk((uint8_t*)&nvm);// Deve stare come ultima operazione prima della scrittura!!!

            p_flash = (uint32_t*)nvm_control.ptr_write;
            p_nvm = (uint32_t*)&nvm;

            nvm_Write_bank();  // Eseguo scrittura

            result = true;
            while(jj--)
            {
                if (*(--p_nvm) != *(--p_flash))
                {
                    if (HAL_Debugger_Present())
                    {
                        asm ("BKPT #0");
                    }
                    result = false;
                    break;
                }
            }

            nvm_control.ptr_write += NVM_BANK_SIZE;
            Nvm_Check_Boundary((volatile uint8_t**)&nvm_control.ptr_write);    //Preparo per eventuale prox scrittura

            // Disabilito scrittura, verra riabilitata al momento opportuno dalla Nvm_Load_Network_Cntr()
            nvm_control.enable_write = false;
            nvm_control.zero_crossing_cntr = 0;
        }
    }
    return (result);
}


//******************************************************************************
// Function: Nvm_Check_And_Save
// Created: 29/05/2015
// Parameters:   None  
// Returns:      None
// Description:  ATTENZIONE!!!! Con questi parametri questa funzione va chiamata
//               ogni 10 millisecondi!!! Se non possibile modificare i parametri
//******************************************************************************
void_t Nvm_Check_And_Save (void_t)
{
	if (nvm_control.timer == 0)
	{
		// Tre zero cross li ho persi andiamo avanti...
        if (nvm_control.enable_write == false)
        {
            // se non ho abilitazione a scrivere resetto antirimbalzo!!
            nvm_control.zero_crossing_cntr = 0;  // Riparti a contare!!
        }
        else
        {
            // Ok persi N° TIMEOUT_WRITE zeri di rete consecutivi, dopo un periodo di
            // ZC_FILTER zero cross stabili, mi sto spegnendo davvero, scrivi array!!!!
            Nvm_CheckIn();
            if (Nvm_Save())
            {
                // Se la scrittura è andata a buon fine allora...
                Nvm_Check_Erase_Page();             // cancello pagina opposta se ce ne bisogno.
            }
        }
    }
    else
    {
    }
} // end of function Nvm_Check_And_Save