/**
 * \author Mr.Nobody
 * \file Usart_Isr.h
 * \ingroup Usart
 * \brief Usart module USART interrupt data transfer handler - private interface
 *
 */
#ifndef USART_USART_ISR_H
#define USART_USART_ISR_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usart_Types.h"                    /* Module types definitions       */
/* ============================= TYPEDEFS =================================== */

/** \brief USART interrupt sources used by the data transfer handlers (bit mask items) */
typedef enum
{
    USART_ISR_IT_NONE = 0x00u, /**< No interrupt source                                   */
    USART_ISR_IT_TXE  = 0x01u, /**< Transmit data register empty (TXEIE)                  */
    USART_ISR_IT_TC   = 0x02u, /**< Transmission complete (TCIE)                          */
    USART_ISR_IT_RXNE = 0x04u, /**< Receive data register not empty (RXNEIE)              */
    USART_ISR_IT_IDLE = 0x08u, /**< Idle line detected (IDLEIE)                           */
    USART_ISR_IT_RTO  = 0x10u, /**< Receiver timeout (RTOIE)                              */
    USART_ISR_IT_ERR  = 0x20u, /**< Framing / noise / overrun error (EIE)                 */
    USART_ISR_IT_PE   = 0x40u, /**< Parity error (PEIE)                                   */
    USART_ISR_IT_ALL  = 0x7Fu  /**< All interrupt sources used by the data handlers       */
}   usart_IsrIt_t;


/** \brief Combination of \ref usart_IsrIt_t items */
typedef uint32_t usart_IsrItMask_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

usart_RequestState_t Usart_Isr_Init            ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Isr_Deinit          ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Isr_Check_Config    ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
usart_RequestState_t Usart_Isr_XferInit        ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Isr_XferDeinit      ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Isr_TxStart         ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Isr_TxStop          ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Isr_RxStart         ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Isr_RxStop          ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Isr_Get_RxEventMask ( usart_PeriphId_t usartId, usart_IsrItMask_t * const itMask );
usart_RequestState_t Usart_Isr_Set_ItActive    ( usart_PeriphId_t usartId, usart_IsrItMask_t itMask );
usart_RequestState_t Usart_Isr_Set_ItInactive  ( usart_PeriphId_t usartId, usart_IsrItMask_t itMask );

usart_RequestState_t Usart_Isr_Handler         ( usart_PeriphId_t usartId );

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_ISR_H */
