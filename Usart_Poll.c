/**
 * \author Mr.Nobody
 * \file Usart_Poll.c
 * \ingroup Usart
 * \brief Usart module polling data transfer handler
 *
 * USART_XFER_MODE_POLL - no interrupt is used. Usart_Task() calls Usart_Poll_Task() which
 * polls USART flags of a running transmission / reception:
 * - reception: errors (PE / FE / NE / ORE), received byte (RXNE), end of message (IDLE / RTO)
 * - transmission: TXE (next byte is written), TC after the last byte (end of transmission)
 *
 * Buffer handling and user callbacks are implemented in Usart.c (Usart_Set_Xfer* services).
 *
 * \note  One byte per direction is moved per call - Usart_Task() has to be called at least once
 *        per frame time, otherwise received bytes are lost (overrun is reported).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart_Poll.h"                     /* Self include                   */
#include "Usart.h"                          /* Module private interface       */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
/* ============================== TYPEDEFS ================================== */

/* ======================== FORWARD DECLARATIONS ============================ */

static usart_RequestState_t Usart_Poll_Rx ( usart_PeriphId_t usartId, USART_TypeDef * const periphReg, uint32_t isrFlags );
static usart_RequestState_t Usart_Poll_Tx ( usart_PeriphId_t usartId, USART_TypeDef * const periphReg, uint32_t isrFlags );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks polling related part of the data handling configuration
 *
 * \note  Polling mode has no mode specific resources - every peripheral is supported.
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_Check_Config( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes polling data transfer of one direction (no HW resource is needed)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_XferInit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Deinitializes polling data transfer of one direction (no HW resource is used)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_XferDeinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts polling transmission (bytes are written by Usart_Task())
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_TxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops polling transmission (Usart_Task() stops writing bytes when the transmission
 *        state in Usart.c is inactive, no HW resource is used)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_TxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts polling reception - stale data and flags are cleared (bytes are read by
 *        Usart_Task())
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_RxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Set_RxFlagsClear( usartId );

    return ( retState );
}


/**
 * \brief Stops polling reception (Usart_Task() stops reading bytes when the reception state in
 *        Usart.c is inactive, no HW resource is used)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_RxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Polls USART flags of one peripheral and moves data of running polling transfers
 *        (called from Usart_Task())
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Poll_Task( usart_PeriphId_t usartId )
{
    usart_RequestState_t  retState  = USART_REQUEST_ERROR;
    USART_TypeDef *       periphReg = USART_NULL_PTR;
    usart_XferContext_t * xferCtx   = USART_NULL_PTR;

    retState = Usart_Get_PeriphReg( usartId, &periphReg );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Get_XferContext( usartId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( USART_REQUEST_OK == retState )
    {
        const uint32_t isrFlags = LL_USART_ReadReg( periphReg, ISR );

        if( ( USART_XFER_MODE_POLL  == xferCtx->Config.RxMode ) &&
            ( USART_FUNCTION_ACTIVE == xferCtx->RxState       )    )
        {
            retState = Usart_Poll_Rx( usartId, periphReg, isrFlags );
        }
        else
        {
            /* No polling reception is running */
        }

        if( ( USART_XFER_MODE_POLL  == xferCtx->Config.TxMode ) &&
            ( USART_FUNCTION_ACTIVE == xferCtx->TxState       )    )
        {
            retState = Usart_Poll_Tx( usartId, periphReg, isrFlags );
        }
        else
        {
            /* No polling transmission is running */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Reception polling step: errors, received byte and end of message
 *
 * \param usartId   [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param periphReg [in]: USART peripheral registers
 * \param isrFlags  [in]: Snapshot of USART ISR register
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Poll_Rx( usart_PeriphId_t usartId, USART_TypeDef * const periphReg, uint32_t isrFlags )
{
    usart_RequestState_t  retState = USART_REQUEST_ERROR;
    usart_XferContext_t * xferCtx  = USART_NULL_PTR;

    retState = Usart_Get_XferContext( usartId, &xferCtx );

    if( USART_REQUEST_OK == retState )
    {
        /* Reception errors are cleared and reported */
        retState = Usart_Set_XferRxErrors( usartId, isrFlags );

        /* Received byte (reading of data register clears RXNE) */
        if( 0u != ( LL_USART_ISR_RXNE & isrFlags ) )
        {
            const usart_RxData_t rxData = LL_USART_ReceiveData8( periphReg );

            retState = Usart_Set_XferRxData( usartId, rxData );
        }
        else
        {
            /* No received byte */
        }

        /* End of received message */
        if( ( USART_RX_END_IDLE == xferCtx->Config.RxEndMode ) &&
            ( 0u               != ( LL_USART_ISR_IDLE & isrFlags ) ) )
        {
            LL_USART_ClearFlag_IDLE( periphReg );

            retState = Usart_Set_XferRxEnd( usartId );
        }
        else if( ( USART_RX_END_TIMEOUT == xferCtx->Config.RxEndMode ) &&
                 ( 0u                  != ( LL_USART_ISR_RTOF & isrFlags ) ) )
        {
            LL_USART_ClearFlag_RTO( periphReg );

            retState = Usart_Set_XferRxEnd( usartId );
        }
        else
        {
            /* End of message is not detected or not used */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Transmission polling step: next byte is written when TDR is empty, after the last
 *        byte the end of transmission is detected by TC flag
 *
 * \param usartId   [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param periphReg [in]: USART peripheral registers
 * \param isrFlags  [in]: Snapshot of USART ISR register
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Poll_Tx( usart_PeriphId_t usartId, USART_TypeDef * const periphReg, uint32_t isrFlags )
{
    usart_RequestState_t retState = USART_REQUEST_OK;

    if( 0u != ( LL_USART_ISR_TXE & isrFlags ) )
    {
        usart_TxData_t        txData    = 0u;
        usart_FunctionState_t dataValid = USART_FUNCTION_INACTIVE;

        retState = Usart_Get_XferTxData( usartId, &txData, &dataValid );

        if( ( USART_REQUEST_OK      == retState  ) &&
            ( USART_FUNCTION_ACTIVE == dataValid )    )
        {
            /* Write of data register clears TC */
            LL_USART_TransmitData8( periphReg, txData );
        }
        else if( ( USART_REQUEST_OK == retState                       ) &&
                 ( 0u               != ( LL_USART_ISR_TC & isrFlags ) )    )
        {
            /* All bytes were written and the last frame was sent */
            retState = Usart_Set_XferTxDone( usartId );
        }
        else
        {
            /* Last frame is still being sent */
        }
    }
    else
    {
        /* Transmit data register is full */
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
