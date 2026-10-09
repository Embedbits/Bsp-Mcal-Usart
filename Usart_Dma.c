/**
 * \author Mr.Nobody
 * \file Usart_Dma.c
 * \ingroup Usart
 * \brief Usart module DMA data transfer handler
 *
 * USART_XFER_MODE_DMA - data are moved between the user buffer and USART data registers (TDR /
 * RDR) by DMA channels (8-bit). STM32G4 DMA requests are routed by DMAMUX1 - any channel of DMA1
 * / DMA2 configured by the application serves the USART request, the request (DMAMUX1 input) is
 * selected by the handler:
 * - Transmission: TC flag is cleared and the channel is armed for the data of the transmission.
 *   DMA transfer complete means the last byte was written to TDR - the USART TC interrupt is
 *   enabled then and the end of the transmission is reported from the USART interrupt
 *   (Usart_Isr.c).
 * - Reception: the channel runs in normal (one shot buffer) or circular mode (circular buffer),
 *   half / full buffer is reported from DMA interrupt, errors and end of received message (IDLE /
 *   RTO) from the USART interrupt (Usart_Isr.c).
 *
 * DMA callbacks have no parameter, so every peripheral has its own set of handlers generated
 * by USART_DMA_DEFINE_HANDLERS(). Buffer handling and user callbacks are implemented in Usart.c
 * (Usart_Set_Xfer* services).
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart_Dma.h"                      /* Self include                   */
#include "Usart_Isr.h"                      /* USART interrupt handler        */
#include "Usart.h"                          /* Module private interface       */
#include "Usart_Port.h"                     /* Module public interface        */
#include "Dma_Port.h"                       /* DMA Mcal layer include         */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
/* ============================== TYPEDEFS ================================== */

/** \brief Data transfer direction handled by a DMA channel */
typedef enum
{
    USART_DMA_DIR_TX = 0u, /**< Transmission (memory to TDR) */
    USART_DMA_DIR_RX,      /**< Reception (RDR to memory)    */
    USART_DMA_DIR_CNT      /**< Count of directions          */
}   usart_DmaDir_t;


/** \brief DMA handlers of one USART/UART peripheral */
typedef struct
{
    dma_IsrCallback   TxCpltIsr;      /**< Transmission transfer complete handler */
    dma_IsrCallback   TxErrorIsr;     /**< Transmission transfer error handler    */
    dma_IsrCallback   RxCpltIsr;      /**< Reception transfer complete handler    */
    dma_IsrCallback   RxHalfIsr;      /**< Reception half transfer handler        */
    dma_IsrCallback   RxErrorIsr;     /**< Reception transfer error handler       */
}   usart_DmaPeriphConfig_t;


/** \brief Initialized DMA channel of one direction */
typedef struct
{
    usart_FunctionState_t Initialized; /**< DMA channel was initialized for the direction */
    dma_PeriphId_t        DmaId;       /**< Initialized DMA peripheral                    */
    dma_ChannelId_t       ChannelId;   /**< Initialized DMA channel                       */
}   usart_DmaChannelState_t;

/* =============================== MACROS =================================== */

/**
 * \brief Declares DMA handlers of one USART/UART peripheral
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Usart1)
 */
#define USART_DMA_DECLARE_HANDLERS( name )                                              \
    static void Usart_Dma_##name##_TxCplt  ( void );                                    \
    static void Usart_Dma_##name##_TxError ( void );                                    \
    static void Usart_Dma_##name##_RxCplt  ( void );                                    \
    static void Usart_Dma_##name##_RxHalf  ( void );                                    \
    static void Usart_Dma_##name##_RxError ( void )

/**
 * \brief Defines DMA handlers of one USART/UART peripheral - every handler forwards the
 *        event with the peripheral identification to the common processing function
 *
 * \param name    [in]: Peripheral name used in handler names (e.g. Usart1)
 * \param usartId [in]: Peripheral identification, value from \ref usart_PeriphId_t
 */
#define USART_DMA_DEFINE_HANDLERS( name, usartId )                                                                    \
    static void Usart_Dma_##name##_TxCplt( void )  { (void)Usart_Dma_TxCplt( usartId ); }                             \
    static void Usart_Dma_##name##_TxError( void ) { (void)Usart_Dma_XferError( usartId, USART_DMA_DIR_TX ); }        \
    static void Usart_Dma_##name##_RxCplt( void )  { (void)Usart_Set_XferRxDone( usartId ); }                         \
    static void Usart_Dma_##name##_RxHalf( void )  { (void)Usart_Set_XferRxHalf( usartId ); }                         \
    static void Usart_Dma_##name##_RxError( void ) { (void)Usart_Dma_XferError( usartId, USART_DMA_DIR_RX ); }

/**
 * \brief DMA handlers of one USART/UART peripheral (configuration table entry)
 *
 * \param name [in]: Peripheral name used in handler names (e.g. Usart1)
 */
#define USART_DMA_ISR_CONFIG( name )                                                    \
    .TxCpltIsr = Usart_Dma_##name##_TxCplt, .TxErrorIsr = Usart_Dma_##name##_TxError,   \
    .RxCpltIsr = Usart_Dma_##name##_RxCplt, .RxHalfIsr  = Usart_Dma_##name##_RxHalf,    \
    .RxErrorIsr = Usart_Dma_##name##_RxError

/* ======================== FORWARD DECLARATIONS ============================ */

static usart_RequestState_t Usart_Dma_Get_Request     ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t channelId, dma_PeriphReqId_t * const request );
static usart_RequestState_t Usart_Dma_Check_Channel   ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t channelId, usart_DmaPriority_t priority );
static usart_RequestState_t Usart_Dma_Set_ChannelInit ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_ChannelOff  ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_Transfer    ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_ChannelStop ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_TxCplt          ( usart_PeriphId_t usartId );
static usart_RequestState_t Usart_Dma_XferError       ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Get_BothOk      ( usart_RequestState_t firstState, usart_RequestState_t secondState );

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
#ifdef LPUART1
USART_DMA_DECLARE_HANDLERS( Lpuart1 );
#endif /* LPUART1 */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** \brief DMA handlers of USART/UART peripherals (order of \ref usart_PeriphId_t) */
static const usart_DmaPeriphConfig_t usart_DmaPeriphConfig[ ] =
{
#ifdef USART1
    { USART_DMA_ISR_CONFIG( Usart1 ) },
#endif /* USART1 */
#ifdef USART2
    { USART_DMA_ISR_CONFIG( Usart2 ) },
#endif /* USART2 */
#ifdef USART3
    { USART_DMA_ISR_CONFIG( Usart3 ) },
#endif /* USART3 */
#ifdef UART4
    { USART_DMA_ISR_CONFIG( Uart4 ) },
#endif /* UART4 */
#ifdef UART5
    { USART_DMA_ISR_CONFIG( Uart5 ) },
#endif /* UART5 */
#ifdef LPUART1
    { USART_DMA_ISR_CONFIG( Lpuart1 ) },
#endif /* LPUART1 */
};

_Static_assert( USART_BUS_CNT == ( sizeof(usart_DmaPeriphConfig) / sizeof(usart_DmaPeriphConfig_t) ), "Usart: usart_DmaPeriphConfig has incorrect size." );


/** \brief Initialized DMA channels per peripheral and direction */
static usart_DmaChannelState_t usart_DmaChannelState[ USART_BUS_CNT ][ USART_DMA_DIR_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the transmission configuration
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the DMA peripheral, channel and priority are valid and
 *         the channel differs from the reception DMA channel. Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Check_TxConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        retState = Usart_Dma_Check_Channel( usartId, USART_DMA_DIR_TX, dataConfig->TxDmaPeriphId, dataConfig->TxDmaChannelId, dataConfig->TxDmaPriority );

        /* Transmission and reception must not share one DMA channel */
        if( ( USART_XFER_MODE_DMA        == dataConfig->RxMode         ) &&
            ( dataConfig->TxDmaPeriphId  == dataConfig->RxDmaPeriphId  ) &&
            ( dataConfig->TxDmaChannelId == dataConfig->RxDmaChannelId )    )
        {
            retState = USART_REQUEST_ERROR;
        }
        else
        {
            /* Channels of the directions differ */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA transmission - DMA channel (memory to TDR)
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
 * \brief Deinitializes DMA transmission - DMA channel and its interrupts are released, USART DMA
 *        transmit request is disabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxDeinit( usart_PeriphId_t usartId )
{
    const usart_RequestState_t chState  = Usart_Dma_Set_ChannelOff( usartId, USART_DMA_DIR_TX );
    const usart_RequestState_t reqState = Usart_Set_DmaTxRequestInactive( usartId );

    return ( Usart_Dma_Get_BothOk( chState, reqState ) );
}


/**
 * \brief Starts DMA transmission - TC flag is cleared, DMA channel is armed for the data of the
 *        transmission and the USART DMA transmit request is enabled
 *
 * \note  TC is cleared before the transmission, so it signals the end of the last frame of this
 *        transmission (writes of DMA to TDR clear TC as well).
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    USART_TypeDef *      periphReg = USART_NULL_PTR;

    retState = Usart_Get_PeriphReg( usartId, &periphReg );

    if( USART_REQUEST_OK == retState )
    {
        LL_USART_ClearFlag_TC( periphReg );

        retState = Usart_Dma_Set_Transfer( usartId, USART_DMA_DIR_TX );
    }
    else
    {
        /* Invalid peripheral identification */
    }

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
 * \brief Stops DMA transmission - DMA channel, USART DMA transmit request and TC interrupt are
 *        disabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_TxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    retState = Usart_Dma_Set_ChannelStop( usartId, USART_DMA_DIR_TX );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_DmaTxRequestInactive( usartId );
    }
    else
    {
        /* DMA transmission could not be stopped */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Isr_Set_ItInactive( usartId, USART_ISR_IT_TC );
    }
    else
    {
        /* DMA transmit request could not be disabled */
    }

    return ( retState );
}


/**
 * \brief Checks DMA related part of the reception configuration
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the DMA peripheral, channel and priority are valid
 *         (buffer is checked by Usart.c, DMA data count limit equals the usart_RxDataCnt_t range).
 *         Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Check_RxConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        retState = Usart_Dma_Check_Channel( usartId, USART_DMA_DIR_RX, dataConfig->RxDmaPeriphId, dataConfig->RxDmaChannelId, dataConfig->RxDmaPriority );
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA reception - DMA channel (RDR to memory, normal mode for one shot buffer,
 *        circular mode for circular buffer)
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
 * \brief Deinitializes DMA reception - DMA channel and its interrupts are released, USART DMA
 *        receive request is disabled
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_RxDeinit( usart_PeriphId_t usartId )
{
    const usart_RequestState_t chState  = Usart_Dma_Set_ChannelOff( usartId, USART_DMA_DIR_RX );
    const usart_RequestState_t reqState = Usart_Set_DmaRxRequestInactive( usartId );

    return ( Usart_Dma_Get_BothOk( chState, reqState ) );
}


/**
 * \brief Starts DMA reception - stale data and flags are cleared, DMA channel is armed for the
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
 * \brief Stops DMA reception - DMA channel, USART DMA receive request, error and end of message
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

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Dma_Set_ChannelStop( usartId, USART_DMA_DIR_RX );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Set_DmaRxRequestInactive( usartId );
    }
    else
    {
        /* DMA reception could not be stopped */
    }

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Isr_Set_ItInactive( usartId, itMask );
    }
    else
    {
        /* DMA receive request could not be disabled */
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

    if( ( USART_REQUEST_OK      == retState                                                         ) &&
        ( USART_NULL_PTR        != rxCnt                                                            ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ].Initialized )    )
    {
        const usart_DmaChannelState_t * const chState   = &usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ];
        dma_DataCount_t                       remaining = 0u;
        const dma_RequestState_t              dmaState  = Dma_Get_DataCount( chState->DmaId, chState->ChannelId, &remaining );

        if( ( DMA_REQUEST_OK               == dmaState  ) &&
            ( xferCtx->Config.RxBufferSize >= remaining )    )
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
 * \brief Returns DMAMUX1 request of the USART direction for a DMA channel
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir     [in]: Data transfer direction
 * \param dmaId      [in]: DMA peripheral, value from \ref usart_DmaPeriphId_t
 * \param channelId  [in]: DMA channel, value from \ref usart_DmaChannelId_t
 * \param request   [out]: Pointer to store DMAMUX1 request of the direction. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the DMA peripheral and channel exist. Otherwise
 *         returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Get_Request( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t channelId, dma_PeriphReqId_t * const request )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    dma_PeriphReqId_t    txRequest = DMA_REQ_MEM2MEM;
    dma_PeriphReqId_t    rxRequest = DMA_REQ_MEM2MEM;

    if( ( USART_DMA_DIR_CNT      > dmaDir    ) &&
        ( USART_DMA_PERIPH_CNT   > dmaId     ) &&
        ( USART_DMA_CHANNEL_CNT  > channelId ) &&
        ( USART_NULL_PTR        != request   )    )
    {
        retState = Usart_Get_PeriphDmaReq( usartId, &txRequest, &rxRequest );
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    if( ( USART_REQUEST_OK == retState ) &&
        ( USART_DMA_DIR_TX == dmaDir   )    )
    {
        *request = txRequest;
    }
    else if( USART_REQUEST_OK == retState )
    {
        *request = rxRequest;
    }
    else
    {
        /* Invalid parameters */
    }

    return ( retState );
}


/**
 * \brief Checks DMA channel and priority of one direction
 *
 * \param usartId   [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir    [in]: Data transfer direction
 * \param dmaId     [in]: DMA peripheral, value from \ref usart_DmaPeriphId_t
 * \param channelId [in]: DMA channel, value from \ref usart_DmaChannelId_t
 * \param priority  [in]: DMA channel priority, value from \ref usart_DmaPriority_t
 *
 * \return Returns \ref USART_REQUEST_OK if the channel exists and the priority is valid.
 *         Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Check_Channel( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t channelId, usart_DmaPriority_t priority )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;
    dma_PeriphReqId_t    request  = DMA_REQ_MEM2MEM;

    if( (uint32_t)DMA_PRIORITY_CNT > (uint32_t)priority )
    {
        retState = Usart_Dma_Get_Request( usartId, dmaDir, dmaId, channelId, &request );
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA channel of one direction (DMAMUX1 request, priority, 8-bit data, TDR /
 *        RDR as peripheral address) and its interrupts
 *
 * - Transmission: memory (increment) -> TDR, normal mode, transfer complete and error handler
 * - Reception: RDR -> RxBuffer (increment), normal / circular mode by RxBufferMode, transfer
 *   complete, half transfer (only with RxHalfCallback) and error handler
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
    dma_PeriphReqId_t     request   = DMA_REQ_MEM2MEM;

    retState = Usart_Get_PeriphReg( usartId, &periphReg );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Get_XferContext( usartId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( ( USART_REQUEST_OK  == retState ) &&
        ( USART_DMA_DIR_CNT  > dmaDir   )    )
    {
        const usart_DmaPeriphConfig_t * const periphConf = &usart_DmaPeriphConfig[ usartId ];
        usart_DmaChannelState_t * const       chState    = &usart_DmaChannelState[ usartId ][ dmaDir ];
        dma_ConfigStruct_t                    dmaConfig;
        dma_RequestState_t                    dmaState   = DMA_REQUEST_ERROR;
        usart_DmaPeriphId_t                   dmaId      = xferCtx->Config.TxDmaPeriphId;
        usart_DmaChannelId_t                  channelId  = xferCtx->Config.TxDmaChannelId;
        usart_DmaPriority_t                   priority   = xferCtx->Config.TxDmaPriority;

        if( USART_DMA_DIR_RX == dmaDir )
        {
            dmaId     = xferCtx->Config.RxDmaPeriphId;
            channelId = xferCtx->Config.RxDmaChannelId;
            priority  = xferCtx->Config.RxDmaPriority;
        }
        else
        {
            /* Transmit channel */
        }

        retState = Usart_Dma_Get_Request( usartId, dmaDir, dmaId, channelId, &request );
        dmaState = Dma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.DmaPeriphId          = (dma_PeriphId_t)dmaId;
        dmaConfig.DmaChannel           = (dma_ChannelId_t)channelId;
        dmaConfig.PeripheralReqId      = request;
        dmaConfig.PeriphAddrIncrement  = DMA_PERIPH_ADDR_STATIC;
        dmaConfig.MemoryAddrIncrement  = DMA_MEMORY_ADDR_INCREMENT;
        dmaConfig.PeriphTransferSize   = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.MemoryTransferSize   = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.Priority             = (dma_Priority_t)priority;
        dmaConfig.HalfTransferCallback = DMA_NULL_PTR;

        if( USART_DMA_DIR_TX == dmaDir )
        {
            /* Memory address and count are configured by every transmission start */
            dmaConfig.Direction                = DMA_DIR_MEMORY_TO_PERIPH;
            dmaConfig.TransferMode             = DMA_TRANSFER_MODE_NORMAL;
            dmaConfig.PeriphAddress            = (dma_PeriphAddr_t)(uintptr_t)&periphReg->TDR;
            dmaConfig.MemoryAddress            = 0u;
            dmaConfig.DataCount                = 0u;
            dmaConfig.TransferCompleteCallback = periphConf->TxCpltIsr;
            dmaConfig.TransferErrorCallback    = periphConf->TxErrorIsr;
        }
        else
        {
            dmaConfig.Direction                = DMA_DIR_PERIPH_TO_MEMORY;
            dmaConfig.TransferMode             = DMA_TRANSFER_MODE_NORMAL;
            dmaConfig.PeriphAddress            = (dma_PeriphAddr_t)(uintptr_t)&periphReg->RDR;
            dmaConfig.MemoryAddress            = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Config.RxBuffer;
            dmaConfig.DataCount                = (dma_DataCount_t)xferCtx->Config.RxBufferSize;
            dmaConfig.TransferCompleteCallback = periphConf->RxCpltIsr;
            dmaConfig.TransferErrorCallback    = periphConf->RxErrorIsr;

            if( USART_BUFFER_MODE_CIRCULAR == xferCtx->Config.RxBufferMode )
            {
                dmaConfig.TransferMode = DMA_TRANSFER_MODE_CIRCULAR;
            }
            else
            {
                /* One shot buffer - normal mode */
            }

            if( USART_NULL_PTR != xferCtx->Config.RxHalfCallback )
            {
                dmaConfig.HalfTransferCallback = periphConf->RxHalfIsr;
            }
            else
            {
                /* Half transfer is not reported */
            }
        }

        if( ( USART_REQUEST_OK == retState ) &&
            ( DMA_REQUEST_OK   == dmaState )    )
        {
            dmaState = Dma_Init( &dmaConfig );
        }
        else
        {
            /* DMA channel does not exist */
            dmaState = DMA_REQUEST_ERROR;
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            chState->Initialized = USART_FUNCTION_ACTIVE;
            chState->DmaId       = dmaConfig.DmaPeriphId;
            chState->ChannelId   = dmaConfig.DmaChannel;

            dmaState = Dma_Set_TransferCompleteIrqActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Channel initialization failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferErrorIrqActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Transfer complete interrupt could not be enabled */
        }

        if( ( DMA_REQUEST_OK == dmaState                       ) &&
            ( DMA_NULL_PTR   != dmaConfig.HalfTransferCallback )    )
        {
            dmaState = Dma_Set_HalfTransferIrqActive( chState->DmaId, chState->ChannelId );
        }
        else if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_HalfTransferIrqInactive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Transfer error interrupt could not be enabled */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Half transfer interrupt could not be configured */
        }

        if( DMA_REQUEST_OK == dmaState )
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
 * \brief Releases DMA channel of one direction - channel is disabled, its interrupts and
 *        callbacks are released
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

    if( ( USART_BUS_CNT     > usartId ) &&
        ( USART_DMA_DIR_CNT > dmaDir  )    )
    {
        usart_DmaChannelState_t * const chState = &usart_DmaChannelState[ usartId ][ dmaDir ];

        if( USART_FUNCTION_ACTIVE == chState->Initialized )
        {
            const dma_RequestState_t xferState = Dma_Set_TransferInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t tcState   = Dma_Set_TransferCompleteIrqInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t htState   = Dma_Set_HalfTransferIrqInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t teState   = Dma_Set_TransferErrorIrqInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t nvicState = Dma_Set_InterruptInactive( chState->DmaId, chState->ChannelId );
            const dma_RequestState_t tcIsr     = Dma_Set_TransferCompleteIsrHandler( chState->DmaId, chState->ChannelId, DMA_NULL_PTR );
            const dma_RequestState_t htIsr     = Dma_Set_HalfTransferIsrHandler( chState->DmaId, chState->ChannelId, DMA_NULL_PTR );
            const dma_RequestState_t teIsr     = Dma_Set_TransferErrorIsrHandler( chState->DmaId, chState->ChannelId, DMA_NULL_PTR );

            chState->Initialized = USART_FUNCTION_INACTIVE;

            if( ( DMA_REQUEST_OK == xferState ) &&
                ( DMA_REQUEST_OK == tcState   ) &&
                ( DMA_REQUEST_OK == htState   ) &&
                ( DMA_REQUEST_OK == teState   ) &&
                ( DMA_REQUEST_OK == nvicState ) &&
                ( DMA_REQUEST_OK == tcIsr     ) &&
                ( DMA_REQUEST_OK == htIsr     ) &&
                ( DMA_REQUEST_OK == teIsr     )    )
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
            /* No DMA channel was initialized for the direction */
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
 * \brief Arms the DMA channel of one direction (memory address, count, enable)
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

    if( ( USART_REQUEST_OK      == retState                                               ) &&
        ( USART_DMA_DIR_CNT      > dmaDir                                                 ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ dmaDir ].Initialized )    )
    {
        const usart_DmaChannelState_t * const chState  = &usart_DmaChannelState[ usartId ][ dmaDir ];
        dma_MemoryAddr_t                      memAddr  = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Config.RxBuffer;
        dma_DataCount_t                       count    = (dma_DataCount_t)xferCtx->Config.RxBufferSize;
        dma_RequestState_t                    dmaState = DMA_REQUEST_ERROR;

        if( USART_DMA_DIR_TX == dmaDir )
        {
            memAddr = (dma_MemoryAddr_t)(uintptr_t)xferCtx->TxData;
            count   = (dma_DataCount_t)xferCtx->TxSize;
        }
        else
        {
            /* Whole receive buffer */
        }

        /* Channel is stopped (configuration is writable only with disabled channel) */
        dmaState = Dma_Set_TransferInactive( chState->DmaId, chState->ChannelId );

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddr( chState->DmaId, chState->ChannelId, memAddr );
        }
        else
        {
            /* Channel could not be stopped */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_DataCount( chState->DmaId, chState->ChannelId, count );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferActive( chState->DmaId, chState->ChannelId );
        }
        else
        {
            /* Count configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
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
 * \brief Disables the DMA channel of one direction (configuration and remaining count are kept)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR (also if
 *         no channel is initialized for the direction).
 */
static usart_RequestState_t Usart_Dma_Set_ChannelStop( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT          > usartId                                                ) &&
        ( USART_DMA_DIR_CNT      > dmaDir                                                 ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ dmaDir ].Initialized )    )
    {
        const usart_DmaChannelState_t * const chState  = &usart_DmaChannelState[ usartId ][ dmaDir ];
        const dma_RequestState_t              dmaState = Dma_Set_TransferInactive( chState->DmaId, chState->ChannelId );

        if( DMA_REQUEST_OK == dmaState )
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
 * \brief DMA error processing - transfer of the direction is stopped and the DMA transfer error
 *        is reported
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_XferError( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t stopState = USART_REQUEST_ERROR;

    if( USART_DMA_DIR_TX == dmaDir )
    {
        stopState = Usart_Set_TxStop( usartId );
    }
    else
    {
        stopState = Usart_Set_RxStop( usartId );
    }

    const usart_RequestState_t errState = Usart_Set_XferError( usartId, USART_XFER_ERROR_DMA_TRANSFER );

    return ( Usart_Dma_Get_BothOk( stopState, errState ) );
}


/**
 * \brief Combines results of two steps executed independently
 *
 * \param firstState  [in]: Result of the first step
 * \param secondState [in]: Result of the second step
 *
 * \return Returns \ref USART_REQUEST_OK if both steps were successful. Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Get_BothOk( usart_RequestState_t firstState, usart_RequestState_t secondState )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_REQUEST_OK == firstState  ) &&
        ( USART_REQUEST_OK == secondState )    )
    {
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
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
#ifdef LPUART1
USART_DMA_DEFINE_HANDLERS( Lpuart1, USART_BUS_LPUART1 )
#endif /* LPUART1 */

/* ================================ TASKS =================================== */
