/**
 * \author Mr.Nobody
 * \file Usart_Poll.h
 * \ingroup Usart
 * \brief Usart module polling data transfer handler - private interface
 *
 */
#ifndef USART_USART_POLL_H
#define USART_USART_POLL_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usart_Types.h"                    /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

usart_RequestState_t Usart_Poll_Check_Config ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
usart_RequestState_t Usart_Poll_XferInit     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Poll_XferDeinit   ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Poll_TxStart      ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Poll_TxStop       ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Poll_RxStart      ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Poll_RxStop       ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Poll_Task         ( usart_PeriphId_t usartId );

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_POLL_H */
