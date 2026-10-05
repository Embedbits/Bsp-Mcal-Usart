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

/** Minimal value of USARTDIV mantissa (BRR register) */
#define USART_BRR_MANTISSA_MIN                  ( 1u )

/** Maximal value of USARTDIV mantissa (BRR register) */
#define USART_BRR_MANTISSA_MAX                  ( 0x0FFFu )

/** Bit-mask of all possible reception errors in status register */
#define USART_SR_ERROR_MASK                     ( LL_USART_SR_PE | LL_USART_SR_FE | LL_USART_SR_NE | LL_USART_SR_ORE )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Usart.c - common services for data transfer handlers */
usart_RequestState_t Usart_Get_PeriphReg      ( usart_PeriphId_t usartId, USART_TypeDef ** const periphReg );
usart_RequestState_t Usart_Get_XferContext    ( usart_PeriphId_t usartId, usart_XferContext_t ** const xferContext );
usart_RequestState_t Usart_Set_RxFlagsClear   ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Get_XferTxData     ( usart_PeriphId_t usartId, usart_TxData_t * const txData, usart_FunctionState_t * const dataValid );
usart_RequestState_t Usart_Set_XferTxDone     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferRxData     ( usart_PeriphId_t usartId, usart_RxData_t rxData );
usart_RequestState_t Usart_Set_XferRxHalf     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferRxDone     ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferRxEnd      ( usart_PeriphId_t usartId );
usart_RequestState_t Usart_Set_XferError      ( usart_PeriphId_t usartId, usart_XferErrorId_t errorId );
usart_RequestState_t Usart_Set_XferRxErrors   ( usart_PeriphId_t usartId, uint32_t srFlags );


/**
 * \brief Calculates expected content of baud-rate register (BRR) for USART/UART
 *        peripheral.
 *
 * USARTDIV = periphClock / ( 8 * ( 2 - OVER8 ) * baudrate ) is stored in BRR as
 * 12 bit mantissa and 4 bit fraction (3 bits with over-sampling by 8). Mantissa
 * shall be in range 1 - 4095.
 *
 * \param periphClock  [in]: Peripheral clock in Hz
 * \param oversampling [in]: Peripheral over-sampling configuration
 * \param baudrate     [in]: Required peripheral baud-rate
 * \param brrValue    [out]: Calculated content of BRR register
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error (baud-rate not reachable with given clock).
 */
static inline usart_RequestState_t Usart_Get_ExpectedBrr( usart_FreqHz_t periphClock,
                                                          usart_Oversampling_t oversampling,
                                                          usart_Baudrate_t baudrate,
                                                          uint32_t *brrValue )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;
    uint32_t             mantissa = 0u;

    if( ( USART_NULL_PTR != brrValue ) &&
        ( 0u             != baudrate )    )
    {
        if( USART_OVERSAMPLING_8 == oversampling )
        {
            mantissa  = periphClock / ( baudrate * 8u );
            *brrValue = (uint16_t)__LL_USART_DIV_SAMPLING8( periphClock, baudrate );
        }
        else
        {
            mantissa  = periphClock / ( baudrate * 16u );
            *brrValue = (uint16_t)__LL_USART_DIV_SAMPLING16( periphClock, baudrate );
        }

        if( ( USART_BRR_MANTISSA_MIN <= mantissa ) &&
            ( USART_BRR_MANTISSA_MAX >= mantissa )    )
        {
            retState = USART_REQUEST_OK;
        }
        else
        {
            retState = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_H */
