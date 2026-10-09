/**
 * \author Mr.Nobody
 * \file Usart_Isr.c
 * \ingroup Usart
 * \brief Usart module USART interrupt data transfer handler
 *
 * The USART interrupt is used by ISR and DMA data transfer modes:
 * - USART_XFER_MODE_ISR transmission: bytes are written in TXE interrupt, the end of the
 *   transmission is detected by TC interrupt.
 * - USART_XFER_MODE_ISR reception: bytes are read in RXNE interrupt.
 * - USART_XFER_MODE_DMA: TC interrupt is used for the end of the transmission (enabled after
 *   the last byte was moved by DMA, see Usart_Dma.c).
 * - Reception errors (PE / FE / NE / ORE) and end of received message (IDLE / RTO) are
 *   reported from the USART interrupt in both modes.
 *
 * Buffer handling and user callbacks are implemented in Usart.c (Usart_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart_Isr.h"                      /* Self include                   */
#include "Usart.h"                          /* Module private interface       */
#include "Usart_Port.h"                     /* Module public interface        */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
/* ============================== TYPEDEFS ================================== */

/** \brief USART interrupt source control (LL functions of one interrupt enable bit) */
typedef struct
{
    usart_IsrItMask_t ItMask;                                     /**< Interrupt source (\ref usart_IsrIt_t)  */
    void              ( *Enable    )( USART_TypeDef *USARTx );       /**< LL function enabling the interrupt     */
    void              ( *Disable   )( USART_TypeDef *USARTx );       /**< LL function disabling the interrupt    */
    uint32_t          ( *IsEnabled )( const USART_TypeDef *USARTx ); /**< LL function reading the enable bit     */
}   usart_IsrItControl_t;


/** \brief Index of \ref usart_IsrItControl_t items */
typedef enum
{
    USART_ISR_IT_IDX_TXE = 0u, /**< Transmit data register empty       */
    USART_ISR_IT_IDX_TC,       /**< Transmission complete              */
    USART_ISR_IT_IDX_RXNE,     /**< Receive data register not empty    */
    USART_ISR_IT_IDX_IDLE,     /**< Idle line detected                 */
    USART_ISR_IT_IDX_RTO,      /**< Receiver timeout                   */
    USART_ISR_IT_IDX_ERR,      /**< Framing / noise / overrun error    */
    USART_ISR_IT_IDX_PE,       /**< Parity error                       */
    USART_ISR_IT_IDX_CNT       /**< Count of interrupt sources         */
}   usart_IsrItIdx_t;

/* ======================== FORWARD DECLARATIONS ============================ */

static usart_RequestState_t Usart_Isr_Get_ItActiveMask( USART_TypeDef * const periphReg, usart_IsrItMask_t * const itMask );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Interrupt sources reporting reception errors */
#define USART_ISR_IT_ERRORS          ( (usart_IsrItMask_t)USART_ISR_IT_ERR | (usart_IsrItMask_t)USART_ISR_IT_PE )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief USART interrupt sources and their LL control functions */
static const usart_IsrItControl_t usart_IsrItControl[ USART_ISR_IT_IDX_CNT ] =
{
    [USART_ISR_IT_IDX_TXE]  = { .ItMask = USART_ISR_IT_TXE,  .Enable = LL_USART_EnableIT_TXE_TXFNF,  .Disable = LL_USART_DisableIT_TXE_TXFNF,  .IsEnabled = LL_USART_IsEnabledIT_TXE_TXFNF  },
    [USART_ISR_IT_IDX_TC]   = { .ItMask = USART_ISR_IT_TC,   .Enable = LL_USART_EnableIT_TC,         .Disable = LL_USART_DisableIT_TC,         .IsEnabled = LL_USART_IsEnabledIT_TC         },
    [USART_ISR_IT_IDX_RXNE] = { .ItMask = USART_ISR_IT_RXNE, .Enable = LL_USART_EnableIT_RXNE_RXFNE, .Disable = LL_USART_DisableIT_RXNE_RXFNE, .IsEnabled = LL_USART_IsEnabledIT_RXNE_RXFNE },
    [USART_ISR_IT_IDX_IDLE] = { .ItMask = USART_ISR_IT_IDLE, .Enable = LL_USART_EnableIT_IDLE,       .Disable = LL_USART_DisableIT_IDLE,       .IsEnabled = LL_USART_IsEnabledIT_IDLE       },
    [USART_ISR_IT_IDX_RTO]  = { .ItMask = USART_ISR_IT_RTO,  .Enable = LL_USART_EnableIT_RTO,        .Disable = LL_USART_DisableIT_RTO,        .IsEnabled = LL_USART_IsEnabledIT_RTO        },
    [USART_ISR_IT_IDX_ERR]  = { .ItMask = USART_ISR_IT_ERR,  .Enable = LL_USART_EnableIT_ERROR,      .Disable = LL_USART_DisableIT_ERROR,      .IsEnabled = LL_USART_IsEnabledIT_ERROR      },
    [USART_ISR_IT_IDX_PE]   = { .ItMask = USART_ISR_IT_PE,   .Enable = LL_USART_EnableIT_PE,         .Disable = LL_USART_DisableIT_PE,         .IsEnabled = LL_USART_IsEnabledIT_PE         },
};


/** \brief usart_RxEndMode_t -> interrupt source detecting the end of received message */
static const usart_IsrItMask_t usart_IsrRxEndItLut[ USART_RX_END_CNT ] =
{
    [USART_RX_END_NONE]    = USART_ISR_IT_NONE,
    [USART_RX_END_IDLE]    = USART_ISR_IT_IDLE,
    [USART_RX_END_TIMEOUT] = USART_ISR_IT_RTO,
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Connects the USART interrupt to NVIC (handler, priority, enable)
 *
 * \pre   Transfer context of the peripheral contains the data handling configuration.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Init( usart_PeriphId_t usartId )
{
    usart_RequestState_t  retState = USART_REQUEST_ERROR;
    usart_XferContext_t * xferCtx  = USART_NULL_PTR;

    retState = Usart_Get_XferContext( usartId, &xferCtx );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_IrqPriority( usartId, xferCtx->Config.IrqPriority );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_InterruptsActive( usartId );
    }
    else
    {
        /* Priority configuration failed, interrupt is not enabled */
    }

    return ( retState );
}


/**
 * \brief Disables all USART interrupt sources used by the data handlers and the USART
 *        interrupt in NVIC
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Deinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Isr_Set_ItInactive( usartId, USART_ISR_IT_ALL );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_InterruptsInactive( usartId );
    }
    else
    {
        /* Interrupt sources could not be disabled */
    }

    return ( retState );
}


/**
 * \brief Checks ISR related part of the data handling configuration
 *
 * \note  The USART interrupt is available for every peripheral - only the parameters are
 *        validated.
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Check_Config( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
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
 * \brief Initializes ISR data transfer of one direction
 *
 * \note  The USART interrupt is connected by Usart_Isr_Init() (common for both directions).
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_XferInit( usart_PeriphId_t usartId )
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
 * \brief Deinitializes ISR data transfer of one direction
 *
 * \note  The USART interrupt is disconnected by Usart_Isr_Deinit() (common for both directions).
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_XferDeinit( usart_PeriphId_t usartId )
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
 * \brief Starts ISR transmission - TXE interrupt is enabled (TDR is empty, the first byte is
 *        written in the interrupt)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_TxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Isr_Set_ItActive( usartId, USART_ISR_IT_TXE );

    return ( retState );
}


/**
 * \brief Stops ISR transmission - TXE and TC interrupts are disabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_TxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Isr_Set_ItInactive( usartId, (usart_IsrItMask_t)USART_ISR_IT_TXE | (usart_IsrItMask_t)USART_ISR_IT_TC );

    return ( retState );
}


/**
 * \brief Starts ISR reception - stale data and flags are cleared, RXNE, error and end of
 *        message interrupts are enabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_RxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;
    usart_IsrItMask_t    itMask   = USART_ISR_IT_NONE;

    retState = Usart_Isr_Get_RxEventMask( usartId, &itMask );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_RxFlagsClear( usartId );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Isr_Set_ItActive( usartId, itMask | (usart_IsrItMask_t)USART_ISR_IT_RXNE );
    }
    else
    {
        /* Flags could not be cleared */
    }

    return ( retState );
}


/**
 * \brief Stops ISR reception - RXNE, error and end of message interrupts are disabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_RxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Isr_Set_ItInactive( usartId, (usart_IsrItMask_t)USART_ISR_IT_RXNE |
                                                  (usart_IsrItMask_t)USART_ISR_IT_IDLE |
                                                  (usart_IsrItMask_t)USART_ISR_IT_RTO  |
                                                  USART_ISR_IT_ERRORS                   );

    return ( retState );
}


/**
 * \brief Returns interrupt sources reporting reception events common for ISR and DMA mode:
 *        errors (EIE, PEIE) and end of received message (IDLE / RTO by RxEndMode)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param itMask [out]: Pointer to store the interrupt sources. Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Get_RxEventMask( usart_PeriphId_t usartId, usart_IsrItMask_t * const itMask )
{
    usart_RequestState_t  retState = USART_REQUEST_ERROR;
    usart_XferContext_t * xferCtx  = USART_NULL_PTR;

    retState = Usart_Get_XferContext( usartId, &xferCtx );

    if( ( USART_REQUEST_OK  == retState                     ) &&
        ( USART_NULL_PTR   != itMask                        ) &&
        ( USART_RX_END_CNT  > xferCtx->Config.RxEndMode     )    )
    {
        *itMask = USART_ISR_IT_ERRORS | usart_IsrRxEndItLut[ xferCtx->Config.RxEndMode ];
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Enables USART interrupt sources
 *
 * \note  Pending flags are not cleared - an already set flag triggers the interrupt immediately
 *        (required for TC after the last byte was already sent).
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param itMask  [in]: Combination of \ref usart_IsrIt_t items
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Set_ItActive( usart_PeriphId_t usartId, usart_IsrItMask_t itMask )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    USART_TypeDef *      periphReg = USART_NULL_PTR;

    retState = Usart_Get_PeriphReg( usartId, &periphReg );

    if( ( USART_REQUEST_OK == retState ) &&
        ( itMask == ( itMask & (usart_IsrItMask_t)USART_ISR_IT_ALL ) )    )
    {
        usart_IsrItMask_t activeMask = USART_ISR_IT_NONE;

        for( usart_IsrItIdx_t itIdx = USART_ISR_IT_IDX_TXE; USART_ISR_IT_IDX_CNT > itIdx; itIdx ++ )
        {
            if( 0u != ( itMask & usart_IsrItControl[ itIdx ].ItMask ) )
            {
                usart_IsrItControl[ itIdx ].Enable( periphReg );
            }
            else
            {
                /* Interrupt source is not requested */
            }
        }

        for( usart_TimeoutCnt_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            retState = Usart_Isr_Get_ItActiveMask( periphReg, &activeMask );

            if( ( USART_REQUEST_OK == retState ) &&
                ( itMask == ( activeMask & itMask ) )    )
            {
                break;
            }
            else
            {
                /* Interrupt enable has not yet been applied, keep return state as error */
                retState = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables USART interrupt sources
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param itMask  [in]: Combination of \ref usart_IsrIt_t items
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Set_ItInactive( usart_PeriphId_t usartId, usart_IsrItMask_t itMask )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    USART_TypeDef *      periphReg = USART_NULL_PTR;

    retState = Usart_Get_PeriphReg( usartId, &periphReg );

    if( ( USART_REQUEST_OK == retState ) &&
        ( itMask == ( itMask & (usart_IsrItMask_t)USART_ISR_IT_ALL ) )    )
    {
        usart_IsrItMask_t activeMask = USART_ISR_IT_NONE;

        for( usart_IsrItIdx_t itIdx = USART_ISR_IT_IDX_TXE; USART_ISR_IT_IDX_CNT > itIdx; itIdx ++ )
        {
            if( 0u != ( itMask & usart_IsrItControl[ itIdx ].ItMask ) )
            {
                usart_IsrItControl[ itIdx ].Disable( periphReg );
            }
            else
            {
                /* Interrupt source is not requested */
            }
        }

        for( usart_TimeoutCnt_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            retState = Usart_Isr_Get_ItActiveMask( periphReg, &activeMask );

            if( ( USART_REQUEST_OK == retState ) &&
                ( 0u == ( activeMask & itMask ) )    )
            {
                break;
            }
            else
            {
                /* Interrupt disable has not yet been applied, keep return state as error */
                retState = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Common USART interrupt handler of the data transfer handlers - processes enabled and
 *        pending events (called from USART interrupt service routine in Usart.c)
 *
 * Status register and enabled interrupt sources are read once at handler entry:
 * - errors (EIE / PEIE enabled): flags are cleared, every error is reported
 * - RXNE (ISR reception): received byte is stored
 * - IDLE / RTO (end of message): flag is cleared, end of message is reported
 * - TXE (ISR transmission): next byte is written, after the last byte TXE interrupt is replaced
 *   by TC interrupt
 * - TC (ISR / DMA transmission): transmission is finished (DMA transmit request disabled)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Isr_Handler( usart_PeriphId_t usartId )
{
    usart_RequestState_t  retState   = USART_REQUEST_ERROR;
    USART_TypeDef *       periphReg  = USART_NULL_PTR;
    usart_XferContext_t * xferCtx    = USART_NULL_PTR;
    usart_IsrItMask_t     activeMask = USART_ISR_IT_NONE;

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
        retState = Usart_Isr_Get_ItActiveMask( periphReg, &activeMask );
    }
    else
    {
        /* Transfer context is not available */
    }

    if( USART_REQUEST_OK == retState )
    {
        const uint32_t isrFlags = LL_USART_ReadReg( periphReg, ISR );

        /*------------------------- Reception errors -------------------------*/
        if( 0u != ( USART_ISR_IT_ERRORS & activeMask ) )
        {
            retState = Usart_Set_XferRxErrors( usartId, isrFlags );
        }
        else
        {
            /* No reception error pending or reported */
        }

        /*-------------------- Received byte (ISR reception) -----------------*/
        if( ( 0u != ( LL_USART_ISR_RXNE_RXFNE & isrFlags ) ) &&
            ( 0u != ( USART_ISR_IT_RXNE & activeMask     ) )    )
        {
            /* Reading of data register clears RXNE */
            const usart_RxData_t rxData = LL_USART_ReceiveData8( periphReg );

            retState = Usart_Set_XferRxData( usartId, rxData );
        }
        else
        {
            /* No received byte pending */
        }

        /*--------------------- End of received message ----------------------*/
        if( ( 0u != ( LL_USART_ISR_IDLE & isrFlags   ) ) &&
            ( 0u != ( USART_ISR_IT_IDLE & activeMask ) )    )
        {
            LL_USART_ClearFlag_IDLE( periphReg );

            retState = Usart_Set_XferRxEnd( usartId );
        }
        else
        {
            /* No idle line detected */
        }

        if( ( 0u != ( LL_USART_ISR_RTOF & isrFlags ) ) &&
            ( 0u != ( USART_ISR_IT_RTO  & activeMask ) )    )
        {
            LL_USART_ClearFlag_RTO( periphReg );

            retState = Usart_Set_XferRxEnd( usartId );
        }
        else
        {
            /* No receiver timeout detected */
        }

        /*----------------- Transmit data register empty ---------------------*/
        if( ( 0u != ( LL_USART_ISR_TXE_TXFNF & isrFlags ) ) &&
            ( 0u != ( USART_ISR_IT_TXE & activeMask )     )    )
        {
            usart_TxData_t        txData    = 0u;
            usart_FunctionState_t dataValid = USART_FUNCTION_INACTIVE;

            retState = Usart_Get_XferTxData( usartId, &txData, &dataValid );

            if( ( USART_REQUEST_OK == retState ) &&
                ( USART_FUNCTION_ACTIVE == dataValid )    )
            {
                LL_USART_TransmitData8( periphReg, txData );
            }
            else
            {
                /* All bytes were written - end of transmission is detected by TC */
                retState = Usart_Isr_Set_ItInactive( usartId, USART_ISR_IT_TXE );

                if( USART_REQUEST_OK == retState )
                {
                    retState = Usart_Isr_Set_ItActive( usartId, USART_ISR_IT_TC );
                }
                else
                {
                    /* TXE interrupt could not be disabled */
                }
            }
        }
        else
        {
            /* No byte to be written */
        }

        /*----------------------- Transmission complete ----------------------*/
        if( ( 0u != ( LL_USART_ISR_TC & isrFlags ) ) &&
            ( 0u != ( USART_ISR_IT_TC & activeMask ) )    )
        {
            LL_USART_ClearFlag_TC( periphReg );

            /* DMA mode: DMA transmit request stays enabled (errata ES0561 2.11.2, Usart_Dma.c) */
            retState = Usart_Isr_Set_ItInactive( usartId, USART_ISR_IT_TC );

            if( USART_REQUEST_OK == retState )
            {
                retState = Usart_Set_XferTxDone( usartId );
            }
            else
            {
                /* Transmission could not be finished */
            }
        }
        else
        {
            /* Transmission is not complete */
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
 * \brief Reads enabled USART interrupt sources used by the data handlers
 *
 * \param periphReg [in]: USART peripheral registers. Must not be NULL.
 * \param itMask   [out]: Pointer to store enabled sources (\ref usart_IsrIt_t items). Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Isr_Get_ItActiveMask( USART_TypeDef * const periphReg, usart_IsrItMask_t * const itMask )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_NULL_PTR != periphReg ) &&
        ( USART_NULL_PTR != itMask    )    )
    {
        usart_IsrItMask_t activeMask = USART_ISR_IT_NONE;

        for( usart_IsrItIdx_t itIdx = USART_ISR_IT_IDX_TXE; USART_ISR_IT_IDX_CNT > itIdx; itIdx ++ )
        {
            const uint32_t enableState = usart_IsrItControl[ itIdx ].IsEnabled( periphReg );

            if( 0u != enableState )
            {
                activeMask |= usart_IsrItControl[ itIdx ].ItMask;
            }
            else
            {
                /* Interrupt source is disabled */
            }
        }

        *itMask  = activeMask;
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
