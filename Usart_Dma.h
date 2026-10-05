/**
 * \author Mr.Nobody
 * \file Usart_Dma.h
 * \ingroup Usart
 * \brief Usart module DMA data transfer handler - private interface
 *
 */
#ifndef USART_USART_DMA_H
#define USART_USART_DMA_H

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

usart_RequestState_t Usart_Dma_Check_TxConfig ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
usart_RequestState_t Usart_Dma_TxInit         ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Dma_TxDeinit       ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Dma_TxStart        ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Dma_TxStop         ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Dma_Check_RxConfig ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
usart_RequestState_t Usart_Dma_RxInit         ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Dma_RxDeinit       ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Dma_RxStart        ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Dma_RxStop         ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Dma_Get_RxCount    ( usart_PeriphId_t usartId, usart_RxDataCnt_t * const rxCnt );

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_DMA_H */
