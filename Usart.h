/**
 * \author Mr.Nobody
 * \file Usart.h
 * \ingroup Usart
 * \brief Universal Synchronous/Asynchronous Receiver-Transmitter (USART) MCAL
 *        module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 */

#ifndef USART_USART_H
#define USART_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usart_Types.h"                    /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

/** \brief Type representing iteration count of busy-wait loops (register read-back) */
typedef uint32_t usart_TimeoutCnt_t;


/** \brief Runtime context of data handling (one per USART/UART peripheral) */
typedef struct
{
    usart_DataConfig_t              Config;    /**< Copy of user data handling configuration          */
    usart_FunctionState_t           InitState; /**< Data handling is initialized                      */
    const usart_TxData_t * volatile TxData;    /**< Data of the running transmission                  */
    volatile usart_TxDataCnt_t      TxSize;    /**< Count of bytes of the running transmission        */
    volatile usart_TxDataCnt_t      TxIdx;     /**< Index of the next byte to be transmitted          */
    volatile usart_FunctionState_t  TxState;   /**< Transmission is running (start - last stop bit)   */
    volatile usart_RxDataCnt_t      RxIdx;     /**< Index of the next received byte (ISR / POLL mode) */
    volatile usart_FunctionState_t  RxState;   /**< Reception is running                              */
}   usart_XferContext_t;


/** \brief Data transfer mode handler interface (one per direction and mode) */
typedef struct
{
    usart_RequestState_t ( *CheckConfig )( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig ); /**< Mode specific configuration check */
    usart_RequestState_t ( *Init        )( usart_PeriphId_t usartId );                                              /**< Mode resources initialization     */
    usart_RequestState_t ( *Deinit      )( usart_PeriphId_t usartId );                                              /**< Mode resources deinitialization   */
    usart_RequestState_t ( *Start       )( usart_PeriphId_t usartId );                                              /**< Data transfer start               */
    usart_RequestState_t ( *Stop        )( usart_PeriphId_t usartId );                                              /**< Data transfer stop                */
}   usart_XferModeIf_t;


/* ========================= SYMBOLIC CONSTANTS ============================= */

/** Busy-wait iteration budget used for register read-back (common for all Usart module files) */
#define USART_TIMEOUT_RAW                       ( (usart_TimeoutCnt_t)0x84FCBu )

/** Maximal value of Driver Enable assertion / de-assertion time fields (DEAT / DEDT) */
#define USART_DE_ASSERT_MAX_VALUE               ( 31u )

/** Count of microseconds in one second */
#define USART_US_IN_SECOND                      ( 1000000u )

#if defined(USART_CR1_FIFOEN)
/** Transmit data register empty / TX FIFO not full flag (STM32L4+ USART with FIFO, FIFO not enabled) */
#define USART_ISR_FLAG_TXE                      ( LL_USART_ISR_TXE_TXFNF )

/** Read data register not empty / RX FIFO not empty flag (STM32L4+ USART with FIFO, FIFO not enabled) */
#define USART_ISR_FLAG_RXNE                     ( LL_USART_ISR_RXNE_RXFNE )
#else
/** Transmit data register empty flag (STM32L4 USART without FIFO) */
#define USART_ISR_FLAG_TXE                      ( LL_USART_ISR_TXE )

/** Read data register not empty flag (STM32L4 USART without FIFO) */
#define USART_ISR_FLAG_RXNE                     ( LL_USART_ISR_RXNE )
#endif /* USART_CR1_FIFOEN */

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Usart.c - common services for data transfer handlers */
usart_RequestState_t Usart_Get_PeriphReg      ( usart_PeriphId_t usartId, USART_TypeDef ** const periphReg );
usart_RequestState_t Usart_Get_PeriphDmaReq   ( usart_PeriphId_t usartId, dma_PeriphReqId_t * const txRequest, dma_PeriphReqId_t * const rxRequest );
usart_RequestState_t Usart_Get_XferContext    ( usart_PeriphId_t usartId, usart_XferContext_t ** const xferContext );
usart_RequestState_t Usart_Set_RxFlagsClear   ( usart_PeriphId_t usartId );

usart_RequestState_t Usart_Get_XferTxData     ( usart_PeriphId_t usartId, usart_TxData_t * const txData, usart_FunctionState_t * const dataValid );
usart_RequestState_t Usart_Set_XferTxDone     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferRxData     ( usart_PeriphId_t usartId, usart_RxData_t rxData );
usart_RequestState_t Usart_Set_XferRxHalf     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferRxDone     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferRxEnd      ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferError      ( usart_PeriphId_t usartId, usart_XferErrorId_t errorId );
usart_RequestState_t Usart_Set_XferRxErrors   ( usart_PeriphId_t usartId, uint32_t isrFlags );

/**
 * \brief Calculation of assertion and de-assertion values of Driver Enable (DE) pin
 *
 * The time is expressed in DE time units - sample time (1/16 or 1/8 bit time) for USART,
 * kernel clock cycle for LPUART. The product is calculated with 64-bit arithmetic (no overflow
 * for high baud-rates).
 *
 * \param targetTime [in]: Required time in us
 * \param unitFreq   [in]: Frequency of DE time units in Hz (baud-rate x over-sampling for USART,
 *                         kernel clock for LPUART)
 * \return Register value of assertion/de-assertion feature (limited to 31)
 */
static inline uint8_t Usart_Get_AssertDeassertRegValues( uint32_t targetTime,
                                                         uint32_t unitFreq )
{
    uint8_t        returnValue = 0u;
    const uint64_t time        = ( (uint64_t)targetTime * (uint64_t)unitFreq ) / (uint64_t)( USART_US_IN_SECOND - 1u );

    if( (uint64_t)USART_DE_ASSERT_MAX_VALUE < time )
    {
        returnValue = USART_DE_ASSERT_MAX_VALUE;
    }
    else
    {
        returnValue = (uint8_t)time;
    }

    return ( returnValue );
}


/**
 * \brief Calculation of assertion and de-assertion times of Driver Enable (DE) pin
 *
 * \param regValue [in]: Register value of assertion/de-assertion feature
 * \param unitFreq [in]: Frequency of DE time units in Hz (baud-rate x over-sampling for USART,
 *                       kernel clock for LPUART). Must not be 0.
 * \return Time in us
 */
static inline uint32_t Usart_Get_AssertDeassertTime( uint8_t regValue,
                                                     uint32_t unitFreq )
{
    return ( (uint32_t)( ( ( (uint64_t)regValue + 1u ) * (uint64_t)USART_US_IN_SECOND ) / (uint64_t)unitFreq ) );
}


#ifdef __cplusplus
}
#endif

#endif /* USART_USART_H */
