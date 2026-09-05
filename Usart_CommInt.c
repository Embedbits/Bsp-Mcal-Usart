/**
 * \author Mr.Nobody
 * \file Usart_CommInt.h
 * \ingroup Usart
 * \brief CommInt module common functionality
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart_CommInt.h"                    /* Self include                   */
#include "Usart_Port.h"             /* Own port file include          */
#include "Usart_Types.h"            /* Module types definitions       */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define _MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define _MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define _PATCH_VERSION           ( 0u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
usart_ModuleVersion_t Usart_CommInt_Get_ModuleVersion( void )
{
    usart_ModuleVersion_t retVersion;

    retVersion.Major = _MAJOR_VERSION;
    retVersion.Minor = _MINOR_VERSION;
    retVersion.Patch = _PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes module CommInt
 *
 * This function shall call every necessary sub-module initialization function 
 * and set up all the necessary resources for the module to work. In case of
 * failure, the function shall handle it by itself and shall not be transferred
 * to AppMain layer.
 */
void Usart_CommInt_Init( void )
{

}


/**
 * \brief Deinitializes module CommInt
 *
 * This function shall call every necessary sub-module deinitialization function 
 * and free all the resources allocated by the module. In case of failure, the 
 * function shall handle it by itself and shall not be transferred to AppMain 
 * layer.
 */
void Usart_CommInt_Deinit( void )
{

}


/**
 * \brief Main task of module Usart
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's 
 * requirements.
 */
void Usart_CommInt_Task( void )
{

}

/* =========================== LOCAL FUNCTIONS ============================== */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
