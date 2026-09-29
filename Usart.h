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

#define USART_DE_ASSERT_MAX_VALUE               ( 31u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/* Implemented in Usart.c - common services for data transfer handlers */
usart_RequestState_t Usart_Get_PeriphReg      ( usart_PeriphId_t usartId, USART_TypeDef ** const periphReg );
usart_RequestState_t Usart_Get_PeriphDmaReq   ( usart_PeriphId_t usartId, gpdma_PeriphReqId_t * const txRequest, gpdma_PeriphReqId_t * const rxRequest );
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
 * \param targetTime   [in]: Required time in us
 * \param baudrate     [in]: Actual baudrate
 * \param oversampling [in]: Oversampling configuration
 * \return Register value of assertion/de-assertion feature
 */
static inline uint8_t Usart_Get_AssertDeassertRegValues( uint32_t targetTime,
                                                         usart_Baudrate_t baudrate,
                                                         usart_Oversampling_t oversampling )
{
    uint8_t returnValue          = 0u;
    uint8_t oversampleMultiplier = 0u;

    if( USART_OVERSAMPLING_8 == oversampling )
    {
        oversampleMultiplier = 8u;
    }
    else
    {
        oversampleMultiplier = 16u;
    }

    uint32_t time = ( targetTime * (uint32_t)baudrate * (uint32_t)oversampleMultiplier ) / ( 1000000u - 1u );

    if( USART_DE_ASSERT_MAX_VALUE < time )
    {
        returnValue = USART_DE_ASSERT_MAX_VALUE;
    }
    else
    {
        returnValue = time;
    }

    return ( returnValue );
}


/**
 * \brief Calculation of assertion and de-assertion values of Driver Enable (DE) pin
 *
 * \param targetTime   [in]: Required time in us
 * \param baudrate     [in]: Actual baudrate
 * \param oversampling [in]: Oversampling configuration
 * \return Register value of assertion/de-assertion feature
 */
static inline uint32_t Usart_Get_AssertDeassertTime( uint8_t regValue,
                                                     usart_Baudrate_t baudrate,
                                                     usart_Oversampling_t oversampling )
{
    uint32_t returnValue          = 0u;
    uint8_t  oversampleMultiplier = 0u;

    if( USART_OVERSAMPLING_8 == oversampling )
    {
        oversampleMultiplier = 8u;
    }
    else
    {
        oversampleMultiplier = 16u;
    }

    returnValue = ( ( regValue + 1u ) * 1000000u ) / ( baudrate * oversampleMultiplier );

    return ( returnValue );
}


#ifdef __cplusplus
}
#endif

#endif /* USART_USART_H */
