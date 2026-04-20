/**
 * \author Mr.Nobody
 * \file Usart_CommDma.h
 * \ingroup Usart
 * \brief Usart module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef USART_USART_COMMDMA_H
#define USART_USART_COMMDMA_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usart_Types.h"                    /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

usart_RequestState_t        Usart_CommDma_Init                  ( usart_PeriphId_t usartId, usart_XferConfig_t * const xferConfig );
usart_RequestState_t        Usart_CommDma_Deinit                ( usart_PeriphId_t usartId );
void                        Usart_CommDma_Task                  ( void );

usart_RequestState_t        Usart_CommDma_Set_TxStart           ( usart_PeriphId_t usartId );
usart_RequestState_t        Usart_CommDma_Set_TxStop            ( usart_PeriphId_t usartId );

usart_RequestState_t        Usart_CommDma_Set_RxStart           ( usart_PeriphId_t usartId );
usart_RequestState_t        Usart_CommDma_Set_RxStop            ( usart_PeriphId_t usartId );

usart_RequestState_t        Usart_CommDma_Set_TxDataAddr        ( usart_PeriphId_t usartId, usart_TxData_t rxDataAddr );
usart_RequestState_t        Usart_CommDma_Set_RxDataAddr        ( usart_PeriphId_t usartId, usart_TxData_t rxDataAddr );

usart_RequestState_t        Usart_CommDma_Set_TxDataLen         ( usart_PeriphId_t usartId, usart_TxDataLen_t txDataLen );
usart_RequestState_t        Usart_CommDma_Set_RxDataLen         ( usart_PeriphId_t usartId, usart_RxDataLen_t rxDataLen );

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_COMMDMA_H */
