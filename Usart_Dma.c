/**
 * \author Mr.Nobody
 * \file Usart_Dma.c
 * \ingroup Usart
 * \brief Usart module DMA data transfer handler
 *
 * USART_XFER_MODE_DMA - data are moved between the user buffer and USART data register by a
 * GPDMA channel (8-bit, one block per transfer):
 * - Transmission: GPDMA transfer complete means the last byte was written to TDR - the USART
 *   TC interrupt is enabled then and the end of the transmission is reported from the USART
 *   interrupt (Usart_Isr.c). DMA transmit request (DMAT) stays enabled from the first
 *   transmission until the de-initialization (errata ES0561 2.11.2 / ES0565 / ES0621: USART does
 *   not generate DMA requests after setting/clearing DMAT bit).
 * - Reception: half / full buffer is reported from GPDMA interrupt, errors and end of received
 *   message (IDLE / RTO) from the USART interrupt (Usart_Isr.c). In circular buffer mode the
 *   channel is re-armed in the transfer complete interrupt (GPDMA module does not support
 *   cyclic transfer lists for a single block yet).
 *
 * GPDMA callbacks have no parameter, so every peripheral has its own set of handlers generated
 * by USART_DMA_DEFINE_HANDLERS(). Buffer handling and user callbacks are implemented in Usart.c
 * (Usart_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart_Dma.h"                      /* Self include                   */
#include "Usart_Isr.h"                      /* USART interrupt handler        */
#include "Usart.h"                          /* Module private interface       */
#include "Usart_Port.h"                     /* Module public interface        */
#include "Gpdma_Port.h"                     /* GPDMA Mcal layer include       */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
/* ============================== TYPEDEFS ================================== */

/** \brief Data transfer direction handled by a GPDMA channel */
typedef enum
{
    USART_DMA_DIR_TX = 0u, /**< Transmission (memory to TDR) */
    USART_DMA_DIR_RX,      /**< Reception (RDR to memory)    */
    USART_DMA_DIR_CNT      /**< Count of directions          */
}   usart_DmaDir_t;


/** \brief GPDMA handlers of one peripheral (registered in GPDMA module) */
typedef struct
{
    gpdma_IsrCallback    *TxCpltIsr;  /**< Transmission transfer complete handler */
    gpdma_IsrErrCallback *TxErrorIsr; /**< Transmission transfer error handler    */
    gpdma_IsrCallback    *RxCpltIsr;  /**< Reception transfer complete handler    */
    gpdma_IsrCallback    *RxHalfIsr;  /**< Reception half transfer handler        */
    gpdma_IsrErrCallback *RxErrorIsr; /**< Reception transfer error handler       */
}   usart_DmaIsrConfig_t;


/** \brief GPDMA channel ownership (GPDMA channel can not be de-initialized separately) */
typedef struct
{
    usart_FunctionState_t Initialized; /**< GPDMA channel was initialized for the direction */
    usart_DmaPeriphId_t   PeriphId;    /**< Initialized GPDMA peripheral                    */
    usart_DmaChannelId_t  ChannelId;   /**< Initialized GPDMA channel                       */
}   usart_DmaChannelState_t;


/** \brief GPDMA error bit reporting */
typedef struct
{
    gpdma_ErrorMaskId_t DmaError; /**< GPDMA error bit             */
    usart_XferErrorId_t ErrorId;  /**< Error reported to the user  */
}   usart_DmaErrorConfig_t;


/** \brief Index of \ref usart_DmaErrorConfig_t items */
typedef enum
{
    USART_DMA_ERR_IDX_TRANSFER = 0u,  /**< Transfer error           */
    USART_DMA_ERR_IDX_CONFIG,         /**< Configuration error      */
    USART_DMA_ERR_IDX_CONFIG_UPDATE,  /**< Configuration update     */
    USART_DMA_ERR_IDX_TRIG_OVERRUN,   /**< Trigger overrun          */
    USART_DMA_ERR_IDX_CNT             /**< Count of reported errors */
}   usart_DmaErrIdx_t;

/* =============================== MACROS =================================== */

/**
 * \brief Declares GPDMA handlers of one USART/UART peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Usart1)
 */
#define USART_DMA_DECLARE_HANDLERS( name )                                              \
    static void Usart_Dma_##name##_TxCplt  ( void );                                    \
    static void Usart_Dma_##name##_TxError ( gpdma_ErrorMaskId_t errorMask );           \
    static void Usart_Dma_##name##_RxCplt  ( void );                                    \
    static void Usart_Dma_##name##_RxHalf  ( void );                                    \
    static void Usart_Dma_##name##_RxError ( gpdma_ErrorMaskId_t errorMask )

/**
 * \brief Defines GPDMA handlers of one USART/UART peripheral - every handler forwards the
 *        event with the peripheral identification to the common processing function
 *
 * \param name    [in]: Peripheral name used in handler names (e.g. Usart1)
 * \param usartId [in]: Peripheral identification, value from \ref usart_PeriphId_t
 */
#define USART_DMA_DEFINE_HANDLERS( name, usartId )                                                         \
    static void Usart_Dma_##name##_TxCplt( void )                          { (void)Usart_Dma_TxCplt( usartId ); }                          \
    static void Usart_Dma_##name##_TxError( gpdma_ErrorMaskId_t errorMask ) { (void)Usart_Dma_XferError( usartId, USART_DMA_DIR_TX, errorMask ); } \
    static void Usart_Dma_##name##_RxCplt( void )                          { (void)Usart_Dma_RxCplt( usartId ); }                          \
    static void Usart_Dma_##name##_RxHalf( void )                          { (void)Usart_Set_XferRxHalf( usartId ); }                      \
    static void Usart_Dma_##name##_RxError( gpdma_ErrorMaskId_t errorMask ) { (void)Usart_Dma_XferError( usartId, USART_DMA_DIR_RX, errorMask ); }

/**
 * \brief Handler table entry of one USART/UART peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Usart1)
 */
#define USART_DMA_ISR_CONFIG( name )                                                    \
    { .TxCpltIsr = Usart_Dma_##name##_TxCplt, .TxErrorIsr = Usart_Dma_##name##_TxError, \
      .RxCpltIsr = Usart_Dma_##name##_RxCplt, .RxHalfIsr  = Usart_Dma_##name##_RxHalf, \
      .RxErrorIsr = Usart_Dma_##name##_RxError }

/* ======================== FORWARD DECLARATIONS ============================ */

static usart_RequestState_t Usart_Dma_Set_ChannelInit ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_ChannelOff  ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_Transfer    ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_TxCplt          ( usart_PeriphId_t usartId );
static usart_RequestState_t Usart_Dma_RxCplt          ( usart_PeriphId_t usartId );
static usart_RequestState_t Usart_Dma_XferError       ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, gpdma_ErrorMaskId_t errorMask );

#ifdef USART1
USART_DMA_DECLARE_HANDLERS( Usart1 );
#endif /* USART1 */
#ifdef USART2
USART_DMA_DECLARE_HANDLERS( Usart2 );
#endif /* USART2 */
#ifdef USART3
USART_DMA_DECLARE_HANDLERS( Usart3 );
#endif /* USART3 */
#ifdef UART4
USART_DMA_DECLARE_HANDLERS( Uart4 );
#endif /* UART4 */
#ifdef UART5
USART_DMA_DECLARE_HANDLERS( Uart5 );
#endif /* UART5 */
#ifdef USART6
USART_DMA_DECLARE_HANDLERS( Usart6 );
#endif /* USART6 */
#ifdef UART7
USART_DMA_DECLARE_HANDLERS( Uart7 );
#endif /* UART7 */
#ifdef UART8
USART_DMA_DECLARE_HANDLERS( Uart8 );
#endif /* UART8 */
#ifdef UART9
USART_DMA_DECLARE_HANDLERS( Uart9 );
#endif /* UART9 */
#ifdef USART10
USART_DMA_DECLARE_HANDLERS( Usart10 );
#endif /* USART10 */
#ifdef USART11
USART_DMA_DECLARE_HANDLERS( Usart11 );
#endif /* USART11 */
#ifdef UART12
USART_DMA_DECLARE_HANDLERS( Uart12 );
#endif /* UART12 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Count of data items transferred per DMA request (single transfer) */
#define USART_DMA_BURST_LEN          ( 1u )

/** Count of transfers in GPDMA transfer list (one block) */
#define USART_DMA_TRANSFERS_CNT      ( 1u )

/** GPDMA errors reported to the user */
#define USART_DMA_ERROR_MASK         ( GPDMA_ERROR_TRANSFER      | \
                                       GPDMA_ERROR_CONFIG_UPDATE | \
                                       GPDMA_ERROR_CONFIG_ERROR  | \
                                       GPDMA_ERROR_TRIG_OVERRUN    )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief usart_PeriphId_t -> GPDMA handlers */
static const usart_DmaIsrConfig_t usart_DmaIsrConfig[ ] =
{
#ifdef USART1
    USART_DMA_ISR_CONFIG( Usart1 ),
#endif /* USART1 */
#ifdef USART2
    USART_DMA_ISR_CONFIG( Usart2 ),
#endif /* USART2 */
#ifdef USART3
    USART_DMA_ISR_CONFIG( Usart3 ),
#endif /* USART3 */
#ifdef UART4
    USART_DMA_ISR_CONFIG( Uart4 ),
#endif /* UART4 */
#ifdef UART5
    USART_DMA_ISR_CONFIG( Uart5 ),
#endif /* UART5 */
#ifdef USART6
    USART_DMA_ISR_CONFIG( Usart6 ),
#endif /* USART6 */
#ifdef UART7
    USART_DMA_ISR_CONFIG( Uart7 ),
#endif /* UART7 */
#ifdef UART8
    USART_DMA_ISR_CONFIG( Uart8 ),
#endif /* UART8 */
#ifdef UART9
    USART_DMA_ISR_CONFIG( Uart9 ),
#endif /* UART9 */
#ifdef USART10
    USART_DMA_ISR_CONFIG( Usart10 ),
#endif /* USART10 */
#ifdef USART11
    USART_DMA_ISR_CONFIG( Usart11 ),
#endif /* USART11 */
#ifdef UART12
    USART_DMA_ISR_CONFIG( Uart12 ),
#endif /* UART12 */
};

_Static_assert( USART_BUS_CNT == ( sizeof(usart_DmaIsrConfig) / sizeof(usart_DmaIsrConfig_t) ), "Usart: usart_DmaIsrConfig has incorrect size." );


/** \brief GPDMA error bits and reported errors */
static const usart_DmaErrorConfig_t usart_DmaErrorConfig[ USART_DMA_ERR_IDX_CNT ] =
{
    [USART_DMA_ERR_IDX_TRANSFER]      = { .DmaError = GPDMA_ERROR_TRANSFER,      .ErrorId = USART_XFER_ERROR_DMA_TRANSFER        },
    [USART_DMA_ERR_IDX_CONFIG]        = { .DmaError = GPDMA_ERROR_CONFIG_ERROR,  .ErrorId = USART_XFER_ERROR_DMA_CONFIG          },
    [USART_DMA_ERR_IDX_CONFIG_UPDATE] = { .DmaError = GPDMA_ERROR_CONFIG_UPDATE, .ErrorId = USART_XFER_ERROR_DMA_CONFIG_UPDATE   },
    [USART_DMA_ERR_IDX_TRIG_OVERRUN]  = { .DmaError = GPDMA_ERROR_TRIG_OVERRUN,  .ErrorId = USART_XFER_ERROR_DMA_TRIGGER_OVERRUN },
};


/** \brief GPDMA transfer lists (must be static, used by GPDMA HW) */
static gpdma_XferList_t usart_DmaXferList[ USART_BUS_CNT ][ USART_DMA_DIR_CNT ];


/** \brief GPDMA channel ownership per peripheral and direction */
static usart_DmaChannelState_t usart_DmaChannelState[ USART_BUS_CNT ][ USART_DMA_DIR_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the transmission configuration
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if DMA identifications and priority are valid and the
 *         channel differs from the reception DMA channel. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Check_TxConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        usart_FunctionState_t sameChannel = USART_FUNCTION_INACTIVE;

        /* Transmission and reception must not share one GPDMA channel */
        if( ( USART_XFER_MODE_DMA        == dataConfig->RxMode         ) &&
            ( dataConfig->TxDmaPeriphId  == dataConfig->RxDmaPeriphId  ) &&
            ( dataConfig->TxDmaChannelId == dataConfig->RxDmaChannelId )    )
        {
            sameChannel = USART_FUNCTION_ACTIVE;
        }
        else
        {
            sameChannel = USART_FUNCTION_INACTIVE;
        }

        if( ( USART_DMA_PERIPH_CNT          > dataConfig->TxDmaPeriphId           ) &&
            ( USART_DMA_CHANNEL_CNT         > dataConfig->TxDmaChannelId          ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT  > (uint32_t)dataConfig->TxDmaPriority ) &&
            ( USART_FUNCTION_INACTIVE      == sameChannel                         )    )
        {
            retState = USART_REQUEST_OK;
        }
        else
        {
            /* DMA configuration is invalid */
            retState = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA transmission - GPDMA channel (memory to TDR)
 *
 * \note  A GPDMA channel already initialized for the direction by a previous initialization is
 *        reused (GPDMA module does not support de-initialization of a single channel).
 *
 * \pre   Transfer context of the peripheral contains the data handling configuration.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxInit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Dma_Set_ChannelInit( usartId, USART_DMA_DIR_TX );

    return ( retState );
}


/**
 * \brief Deinitializes DMA transmission - GPDMA channel and its interrupt are disabled, USART DMA
 *        transmit request is disabled
 *
 * \note  Errata ES0561 2.11.2 (ES0565, ES0621): USART does not generate DMA requests after
 *        setting/clearing DMAT bit. Enabled peripheral is disabled and enabled again (UE) after
 *        DMAT is cleared, so a following DMA transmission (Usart_Set_DataConfig() without
 *        peripheral reset) gets the requests again. Reception is stopped by the caller - a byte
 *        received during the re-enable is lost.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxDeinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState    = USART_REQUEST_ERROR;
    usart_FlagState_t    dmaTxState  = USART_FLAG_INACTIVE;
    usart_FlagState_t    periphState = USART_FLAG_INACTIVE;

    retState = Usart_Dma_Set_ChannelOff( usartId, USART_DMA_DIR_TX );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Get_DmaTxReqState( usartId, &dmaTxState );
    }
    else
    {
        /* GPDMA channel could not be disabled */
    }

    if( ( USART_REQUEST_OK == retState ) && ( USART_FLAG_ACTIVE == dmaTxState ) )
    {
        retState = Usart_Set_DmaTxRequestInactive( usartId );

        if( USART_REQUEST_OK == retState )
        {
            retState = Usart_Get_PeriphState( usartId, &periphState );
        }
        else
        {
            /* DMA transmit request could not be disabled */
        }

        if( ( USART_REQUEST_OK == retState ) && ( USART_FLAG_ACTIVE == periphState ) )
        {
            retState = Usart_Set_PeriphInactive( usartId );

            if( USART_REQUEST_OK == retState )
            {
                retState = Usart_Set_PeriphActive( usartId );
            }
            else
            {
                /* Peripheral could not be disabled */
            }
        }
        else
        {
            /* Disabled peripheral - DMA request generation is restored by its enable */
        }
    }
    else
    {
        /* DMA transmit request was not used */
    }

    return ( retState );
}


/**
 * \brief Starts DMA transmission - GPDMA channel is armed for the data of the transmission and
 *        the USART DMA transmit request is enabled
 *
 * \note  DMA transmit request stays enabled after the first transmission (see
 *        Usart_Dma_TxDeinit()). The request pending since the last byte of the previous
 *        transmission (TXE) is served when the channel is enabled.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Dma_Set_Transfer( usartId, USART_DMA_DIR_TX );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_DmaTxRequestActive( usartId );
    }
    else
    {
        /* DMA channel could not be armed */
    }

    return ( retState );
}


/**
 * \brief Stops DMA transmission - GPDMA channel and TC interrupt are disabled
 *
 * \note  USART DMA transmit request stays enabled (see Usart_Dma_TxDeinit()).
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) && ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ USART_DMA_DIR_TX ].Initialized ) )
    {
        const usart_DmaChannelState_t * const chState  = &usart_DmaChannelState[ usartId ][ USART_DMA_DIR_TX ];
        const gpdma_RequestState_t            dmaState = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId );

        if( GPDMA_REQUEST_OK == dmaState )
        {
            retState = USART_REQUEST_OK;
        }
        else
        {
            retState = USART_REQUEST_ERROR;
        }

        if( USART_REQUEST_OK == retState )
        {
            retState = Usart_Isr_Set_ItInactive( usartId, USART_ISR_IT_TC );
        }
        else
        {
            /* DMA transmission could not be stopped */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Checks DMA related part of the reception configuration
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if DMA identifications and priority are valid (buffer is
 *         checked by Usart.c, GPDMA block size limit equals the usart_RxDataCnt_t range).
 *         Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Check_RxConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        if( ( USART_DMA_PERIPH_CNT         > dataConfig->RxDmaPeriphId           ) &&
            ( USART_DMA_CHANNEL_CNT        > dataConfig->RxDmaChannelId          ) &&
            ( (uint32_t)GPDMA_PRIORITY_CNT > (uint32_t)dataConfig->RxDmaPriority )    )
        {
            retState = USART_REQUEST_OK;
        }
        else
        {
            /* DMA configuration is invalid */
            retState = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA reception - GPDMA channel (RDR to memory)
 *
 * \note  A GPDMA channel already initialized for the direction by a previous initialization is
 *        reused (GPDMA module does not support de-initialization of a single channel).
 *
 * \pre   Transfer context of the peripheral contains the data handling configuration.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_RxInit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Dma_Set_ChannelInit( usartId, USART_DMA_DIR_RX );

    return ( retState );
}


/**
 * \brief Deinitializes DMA reception - GPDMA channel and its interrupt are disabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_RxDeinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Dma_Set_ChannelOff( usartId, USART_DMA_DIR_RX );

    return ( retState );
}


/**
 * \brief Starts DMA reception - stale data and flags are cleared, GPDMA channel is armed for the
 *        whole buffer, USART DMA receive request, error and end of message interrupts are enabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_RxStart( usart_PeriphId_t usartId )
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
        retState = Usart_Dma_Set_Transfer( usartId, USART_DMA_DIR_RX );
    }
    else
    {
        /* Flags could not be cleared */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_DmaRxRequestActive( usartId );
    }
    else
    {
        /* DMA channel could not be armed */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Isr_Set_ItActive( usartId, itMask );
    }
    else
    {
        /* DMA receive request could not be enabled */
    }

    return ( retState );
}


/**
 * \brief Stops DMA reception - GPDMA channel, USART DMA receive request, error and end of message
 *        interrupts are disabled
 *
 * \note  Remaining count of the channel is kept - Usart_Dma_Get_RxCount() returns the count of
 *        bytes received until the stop.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_RxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;
    usart_IsrItMask_t    itMask   = USART_ISR_IT_NONE;

    retState = Usart_Isr_Get_RxEventMask( usartId, &itMask );

    if( ( USART_REQUEST_OK == retState ) && ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ].Initialized ) )
    {
        const usart_DmaChannelState_t * const chState  = &usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ];
        const gpdma_RequestState_t            dmaState = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId );

        if( GPDMA_REQUEST_OK == dmaState )
        {
            retState = Usart_Set_DmaRxRequestInactive( usartId );
        }
        else
        {
            retState = USART_REQUEST_ERROR;
        }

        if( USART_REQUEST_OK == retState )
        {
            retState = Usart_Isr_Set_ItInactive( usartId, itMask );
        }
        else
        {
            /* DMA reception could not be stopped */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns count of bytes stored into RxBuffer by DMA (buffer size - remaining count)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param rxCnt  [out]: Pointer to store the count of received bytes. Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Get_RxCount( usart_PeriphId_t usartId, usart_RxDataCnt_t * const rxCnt )
{
    usart_RequestState_t  retState = USART_REQUEST_ERROR;
    usart_XferContext_t * xferCtx  = USART_NULL_PTR;

    retState = Usart_Get_XferContext( usartId, &xferCtx );

    if( ( USART_REQUEST_OK      == retState                                                        ) &&
        ( USART_NULL_PTR       != rxCnt                                                            ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ].Initialized )    )
    {
        const usart_DmaChannelState_t * const chState   = &usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ];
        gpdma_BlockSize_t                     remaining = 0u;
        const gpdma_RequestState_t            dmaState  = Gpdma_Get_BlockSize( (gpdma_PeriphId_t)chState->PeriphId, (gpdma_ChannelId_t)chState->ChannelId, &remaining );

        if( ( GPDMA_REQUEST_OK == dmaState ) && ( xferCtx->Config.RxBufferSize >= remaining ) )
        {
            *rxCnt   = (usart_RxDataCnt_t)( xferCtx->Config.RxBufferSize - remaining );
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

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Initializes (or reuses) the GPDMA channel of one direction and enables its interrupt
 *
 * - Transmission: memory (increment) -> TDR (static), transfer complete and error handler
 * - Reception: RDR (static) -> RxBuffer (increment), transfer complete, half transfer (only
 *   with RxHalfCallback) and error handler
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Set_ChannelInit( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t  retState  = USART_REQUEST_ERROR;
    USART_TypeDef *       periphReg = USART_NULL_PTR;
    usart_XferContext_t * xferCtx   = USART_NULL_PTR;
    gpdma_PeriphReqId_t   txRequest = GPDMA_REQ_USART1_TX; /* Overwritten by Usart_Get_PeriphDmaReq() */
    gpdma_PeriphReqId_t   rxRequest = GPDMA_REQ_USART1_RX; /* Overwritten by Usart_Get_PeriphDmaReq() */

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
        retState = Usart_Get_PeriphDmaReq( usartId, &txRequest, &rxRequest );
    }
    else
    {
        /* Transfer context is not available */
    }

    if( ( USART_REQUEST_OK == retState ) && ( USART_DMA_DIR_CNT > dmaDir ) )
    {
        usart_DmaChannelState_t * const    chState   = &usart_DmaChannelState[ usartId ][ dmaDir ];
        const usart_DmaIsrConfig_t * const isrConfig = &usart_DmaIsrConfig[ usartId ];
        gpdma_ConfigStruct_t               dmaConfig  = { 0u };
        gpdma_TransferConfig_t             xferConfig = { 0u };
        usart_DmaPeriphId_t                periphId   = xferCtx->Config.TxDmaPeriphId;
        usart_DmaChannelId_t               channelId  = xferCtx->Config.TxDmaChannelId;
        usart_DmaPriority_t                priority   = xferCtx->Config.TxDmaPriority;
        gpdma_RequestState_t               dmaState   = GPDMA_REQUEST_ERROR;

        /* --- Direction specific transfer parameters --- */
        xferConfig.EventMode             = GPDMA_TRANSFER_EVENT_BLOCK;
        xferConfig.TriggerType           = GPDMA_TRG_NOT_USED;
        xferConfig.TriggerMode           = GPDMA_TRIGGER_BLOCK;
        xferConfig.RequestMode           = GPDMA_PERIPH_REQ_SINGLE;
        xferConfig.BlockRepetitionCount  = 0u;
        xferConfig.SourceDataSize        = GPDMA_DATA_SIZE_8BITS;
        xferConfig.SourceBurstLength     = USART_DMA_BURST_LEN;
        xferConfig.SourcePortId          = GPDMA_PORT_DEFAULT;
        xferConfig.SourceDataOp          = GPDMA_SRC_DATA_PRESERVE;
        xferConfig.DestinationDataSize   = GPDMA_DATA_SIZE_8BITS;
        xferConfig.DestinationBurstLength= USART_DMA_BURST_LEN;
        xferConfig.DestinationPortId     = GPDMA_PORT_DEFAULT;
        xferConfig.DestinationDataOp     = GPDMA_DEST_DATA_PRESERVE;

        dmaState = Gpdma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.TransferExecMode   = GPDMA_XFER_EXEC_CONTINUOUS;
        dmaConfig.TransferConfig     = &xferConfig;
        dmaConfig.TransfersCount     = USART_DMA_TRANSFERS_CNT;
        dmaConfig.XferListAccessMode = GPDMA_TRANSFER_LIST_ACCESS_SINGLE;
        dmaConfig.XferList           = &usart_DmaXferList[ usartId ][ dmaDir ];
        dmaConfig.TransferLockState  = GPDMA_TRANSFER_LIST_LOCKED;
        dmaConfig.ErrorMask          = USART_DMA_ERROR_MASK;

        if( USART_DMA_DIR_TX == dmaDir )
        {
            /* Source address and block size are configured by every transmission start */
            xferConfig.Direction           = GPDMA_DIR_MEMORY_TO_PERIPH;
            xferConfig.RequestSource       = txRequest;
            xferConfig.BlockSize           = 0u;
            xferConfig.SourceAddr          = 0u;
            xferConfig.SourceAddrMode      = GPDMA_ADDR_INCREMENT;
            xferConfig.DestinationAddr     = (gpdma_DstAddr_t)LL_USART_DMA_GetRegAddr( periphReg, LL_USART_DMA_REG_DATA_TRANSMIT );
            xferConfig.DestinationAddrMode = GPDMA_ADDR_STATIC;

            dmaConfig.TransferCompleteIsr  = isrConfig->TxCpltIsr;
            dmaConfig.HalfTransferIsr      = GPDMA_NULL_PTR;
            dmaConfig.ErrorIsr             = isrConfig->TxErrorIsr;
        }
        else
        {
            periphId  = xferCtx->Config.RxDmaPeriphId;
            channelId = xferCtx->Config.RxDmaChannelId;
            priority  = xferCtx->Config.RxDmaPriority;

            xferConfig.Direction           = GPDMA_DIR_PERIPH_TO_MEMORY;
            xferConfig.RequestSource       = rxRequest;
            xferConfig.BlockSize           = (gpdma_BlockSize_t)xferCtx->Config.RxBufferSize;
            xferConfig.SourceAddr          = (gpdma_SrcAddr_t)LL_USART_DMA_GetRegAddr( periphReg, LL_USART_DMA_REG_DATA_RECEIVE );
            xferConfig.SourceAddrMode      = GPDMA_ADDR_STATIC;
            xferConfig.DestinationAddr     = (gpdma_DstAddr_t)xferCtx->Config.RxBuffer;
            xferConfig.DestinationAddrMode = GPDMA_ADDR_INCREMENT;

            dmaConfig.TransferCompleteIsr  = isrConfig->RxCpltIsr;
            dmaConfig.ErrorIsr             = isrConfig->RxErrorIsr;

            if( USART_NULL_PTR != xferCtx->Config.RxHalfCallback )
            {
                dmaConfig.HalfTransferIsr = isrConfig->RxHalfIsr;
            }
            else
            {
                dmaConfig.HalfTransferIsr = GPDMA_NULL_PTR;
            }
        }

        dmaConfig.PeriphId    = (gpdma_PeriphId_t)periphId;
        dmaConfig.ChannelId   = (gpdma_ChannelId_t)channelId;
        dmaConfig.ChannelPrio = (gpdma_Priority_t)priority;

        /* --- GPDMA channel initialization or reuse --- */
        if( ( USART_FUNCTION_ACTIVE == chState->Initialized ) &&
            ( periphId              == chState->PeriphId    ) &&
            ( channelId             == chState->ChannelId   )    )
        {
            /* Channel is already configured for this direction - priority and half transfer are updated */
            dmaState = Gpdma_Set_Priority( dmaConfig.PeriphId, dmaConfig.ChannelId, dmaConfig.ChannelPrio );

            if( ( GPDMA_REQUEST_OK == dmaState ) && ( GPDMA_NULL_PTR != dmaConfig.HalfTransferIsr ) )
            {
                dmaState = Gpdma_Set_HalfTransferIsrHandler( dmaConfig.PeriphId, dmaConfig.ChannelId, dmaConfig.HalfTransferIsr );

                if( GPDMA_REQUEST_OK == dmaState )
                {
                    dmaState = Gpdma_Set_HalfTransferIrqActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
                }
                else
                {
                    /* Handler registration failed, interrupt is not enabled */
                }
            }
            else if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_HalfTransferIrqInactive( dmaConfig.PeriphId, dmaConfig.ChannelId );
            }
            else
            {
                /* Priority configuration failed */
            }
        }
        else if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Init( &dmaConfig );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                chState->Initialized = USART_FUNCTION_ACTIVE;
                chState->PeriphId    = periphId;
                chState->ChannelId   = channelId;
            }
            else
            {
                /* GPDMA channel initialization failed */
            }
        }
        else
        {
            /* Default configuration is not available */
        }

        /* GPDMA module does not enable the channel interrupt in NVIC by itself */
        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_InterruptActive( dmaConfig.PeriphId, dmaConfig.ChannelId );
        }
        else
        {
            /* GPDMA channel configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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


/**
 * \brief Disables the GPDMA channel of one direction and its interrupt (channel configuration is
 *        kept, see Usart_Dma_Set_ChannelInit())
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems (also if no channel was initialized). Otherwise
 *         returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Set_ChannelOff( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) && ( USART_DMA_DIR_CNT > dmaDir ) )
    {
        const usart_DmaChannelState_t * const chState = &usart_DmaChannelState[ usartId ][ dmaDir ];

        if( USART_FUNCTION_ACTIVE == chState->Initialized )
        {
            const gpdma_PeriphId_t  dmaPeriph = (gpdma_PeriphId_t)chState->PeriphId;
            const gpdma_ChannelId_t dmaChan   = (gpdma_ChannelId_t)chState->ChannelId;
            gpdma_RequestState_t    dmaState  = Gpdma_Set_ChannelInactive( dmaPeriph, dmaChan );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_InterruptInactive( dmaPeriph, dmaChan );
            }
            else
            {
                /* Channel could not be disabled */
            }

            if( GPDMA_REQUEST_OK == dmaState )
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
            /* No GPDMA channel was initialized for the direction */
            retState = USART_REQUEST_OK;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Arms the GPDMA channel of one direction (block size, memory address, enable)
 *
 * - Transmission: TxSize bytes from TxData
 * - Reception: RxBufferSize bytes into RxBuffer
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Set_Transfer( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t  retState = USART_REQUEST_ERROR;
    usart_XferContext_t * xferCtx  = USART_NULL_PTR;

    retState = Usart_Get_XferContext( usartId, &xferCtx );

    if( ( USART_REQUEST_OK      == retState                                        ) &&
        ( USART_DMA_DIR_CNT      > dmaDir                                          ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ dmaDir ].Initialized ) )
    {
        const usart_DmaChannelState_t * const chState   = &usart_DmaChannelState[ usartId ][ dmaDir ];
        const gpdma_PeriphId_t                dmaPeriph = (gpdma_PeriphId_t)chState->PeriphId;
        const gpdma_ChannelId_t               dmaChan   = (gpdma_ChannelId_t)chState->ChannelId;
        gpdma_RequestState_t                  dmaState  = GPDMA_REQUEST_ERROR;

        if( USART_DMA_DIR_TX == dmaDir )
        {
            dmaState = Gpdma_Set_BlockSize( dmaPeriph, dmaChan, (gpdma_BlockSize_t)xferCtx->TxSize );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_SourceAddr( dmaPeriph, dmaChan, (gpdma_SrcAddr_t)xferCtx->TxData );
            }
            else
            {
                /* Block size configuration failed */
            }
        }
        else
        {
            dmaState = Gpdma_Set_BlockSize( dmaPeriph, dmaChan, (gpdma_BlockSize_t)xferCtx->Config.RxBufferSize );

            if( GPDMA_REQUEST_OK == dmaState )
            {
                dmaState = Gpdma_Set_DestinationAddr( dmaPeriph, dmaChan, (gpdma_DstAddr_t)xferCtx->Config.RxBuffer );
            }
            else
            {
                /* Block size configuration failed */
            }
        }

        if( GPDMA_REQUEST_OK == dmaState )
        {
            dmaState = Gpdma_Set_ChannelActive( dmaPeriph, dmaChan );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( GPDMA_REQUEST_OK == dmaState )
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


/**
 * \brief Transmission transfer complete processing - all bytes were written to TDR, the USART
 *        TC interrupt reports the end of the transmission (last stop bit sent)
 *
 * \note  DMA transmit request is not disabled - clearing and setting DMAT stops the generation
 *        of DMA requests (errata ES0561 2.11.2, see Usart_Dma_TxDeinit()). The request of the
 *        TXE event after the last byte stays pending with disabled GPDMA channel and it is served
 *        by the next transmission.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_TxCplt( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Isr_Set_ItActive( usartId, USART_ISR_IT_TC );

    if( USART_REQUEST_OK != retState )
    {
        /* End of transmission can not be detected - transmission is stopped */
        (void)Usart_Set_TxStop( usartId );
        (void)Usart_Set_XferError( usartId, USART_XFER_ERROR_DMA_TRANSFER );
    }
    else
    {
        /* End of transmission is reported by USART TC interrupt */
    }

    return ( retState );
}


/**
 * \brief Reception transfer complete processing - circular buffer is re-armed, full buffer is
 *        reported to Usart.c (one shot buffer stops the reception there)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_RxCplt( usart_PeriphId_t usartId )
{
    usart_RequestState_t  retState = USART_REQUEST_ERROR;
    usart_XferContext_t * xferCtx  = USART_NULL_PTR;

    retState = Usart_Get_XferContext( usartId, &xferCtx );

    if( ( USART_REQUEST_OK == retState ) && ( USART_BUFFER_MODE_CIRCULAR == xferCtx->Config.RxBufferMode ) )
    {
        /* Channel is disabled by HW at the end of the block - it is armed again immediately */
        retState = Usart_Dma_Set_Transfer( usartId, USART_DMA_DIR_RX );
    }
    else
    {
        /* One shot buffer - reception is stopped by Usart_Set_XferRxDone() */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_XferRxDone( usartId );
    }
    else
    {
        /* Re-arm failed, reception is stopped and reported as DMA transfer error */
        (void)Usart_Set_RxStop( usartId );
        (void)Usart_Set_XferError( usartId, USART_XFER_ERROR_DMA_TRANSFER );
    }

    return ( retState );
}


/**
 * \brief DMA error processing - transfer of the direction is stopped and every reported GPDMA
 *        error bit is forwarded as data transfer error
 *
 * \param usartId   [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir    [in]: Data transfer direction
 * \param errorMask [in]: GPDMA error bit mask
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_XferError( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, gpdma_ErrorMaskId_t errorMask )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_DMA_DIR_TX == dmaDir )
    {
        retState = Usart_Set_TxStop( usartId );
    }
    else
    {
        retState = Usart_Set_RxStop( usartId );
    }

    for( usart_DmaErrIdx_t errIdx = USART_DMA_ERR_IDX_TRANSFER; USART_DMA_ERR_IDX_CNT > errIdx; errIdx ++ )
    {
        if( 0u != ( (uint32_t)errorMask & (uint32_t)usart_DmaErrorConfig[ errIdx ].DmaError ) )
        {
            retState = Usart_Set_XferError( usartId, usart_DmaErrorConfig[ errIdx ].ErrorId );
        }
        else
        {
            /* Error bit is not reported */
        }
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

#ifdef USART1
USART_DMA_DEFINE_HANDLERS( Usart1, USART_BUS_1 )
#endif /* USART1 */
#ifdef USART2
USART_DMA_DEFINE_HANDLERS( Usart2, USART_BUS_2 )
#endif /* USART2 */
#ifdef USART3
USART_DMA_DEFINE_HANDLERS( Usart3, USART_BUS_3 )
#endif /* USART3 */
#ifdef UART4
USART_DMA_DEFINE_HANDLERS( Uart4, USART_BUS_4 )
#endif /* UART4 */
#ifdef UART5
USART_DMA_DEFINE_HANDLERS( Uart5, USART_BUS_5 )
#endif /* UART5 */
#ifdef USART6
USART_DMA_DEFINE_HANDLERS( Usart6, USART_BUS_6 )
#endif /* USART6 */
#ifdef UART7
USART_DMA_DEFINE_HANDLERS( Uart7, USART_BUS_7 )
#endif /* UART7 */
#ifdef UART8
USART_DMA_DEFINE_HANDLERS( Uart8, USART_BUS_8 )
#endif /* UART8 */
#ifdef UART9
USART_DMA_DEFINE_HANDLERS( Uart9, USART_BUS_9 )
#endif /* UART9 */
#ifdef USART10
USART_DMA_DEFINE_HANDLERS( Usart10, USART_BUS_10 )
#endif /* USART10 */
#ifdef USART11
USART_DMA_DEFINE_HANDLERS( Usart11, USART_BUS_11 )
#endif /* USART11 */
#ifdef UART12
USART_DMA_DEFINE_HANDLERS( Uart12, USART_BUS_12 )
#endif /* UART12 */

/* ================================ TASKS =================================== */
