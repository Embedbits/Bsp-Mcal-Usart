/**
 * \author Mr.Nobody
 * \file Usart_Dma.c
 * \ingroup Usart
 * \brief Usart module DMA data transfer handler
 *
 * USART_XFER_MODE_DMA - data are moved between the user buffer and USART data register (DR) by
 * DMA streams (8-bit). STM32F4 DMA has no request multiplexer - the stream configured by the
 * application has to be connected to the USART request (request mapping table of the reference
 * manual), its channel selection (CHSEL) is set by the handler:
 * - Transmission: TC flag is cleared and the stream is armed for the data of the transmission.
 *   DMA transfer complete means the last byte was written to DR - the USART TC interrupt is
 *   enabled then and the end of the transmission is reported from the USART interrupt
 *   (Usart_Isr.c).
 * - Reception: the stream runs in normal (one shot buffer) or circular mode (circular buffer),
 *   half / full buffer is reported from DMA interrupt, errors and end of received message (IDLE)
 *   from the USART interrupt (Usart_Isr.c).
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

/** \brief Data transfer direction handled by a DMA stream */
typedef enum
{
    USART_DMA_DIR_TX = 0u, /**< Transmission (memory to DR) */
    USART_DMA_DIR_RX,      /**< Reception (DR to memory)    */
    USART_DMA_DIR_CNT      /**< Count of directions         */
}   usart_DmaDir_t;


/** \brief DMA stream connected to USART request */
typedef struct
{
    dma_PeriphId_t    DmaId;      /**< DMA peripheral                        */
    dma_ChannelId_t   StreamId;   /**< DMA stream                            */
    dma_PeriphReqId_t ChannelSel; /**< Channel selection (request) of stream */
}   usart_DmaStream_t;


/** \brief DMA configuration of one USART/UART peripheral */
typedef struct
{
    usart_DmaStream_t TxStream[ 2u ]; /**< Streams of transmit request (twice if only one exists) */
    usart_DmaStream_t RxStream[ 2u ]; /**< Streams of receive request (twice if only one exists)  */
    dma_IsrCallback   TxCpltIsr;      /**< Transmission transfer complete handler                 */
    dma_IsrCallback   TxErrorIsr;     /**< Transmission transfer error handler                    */
    dma_IsrCallback   RxCpltIsr;      /**< Reception transfer complete handler                    */
    dma_IsrCallback   RxHalfIsr;      /**< Reception half transfer handler                        */
    dma_IsrCallback   RxErrorIsr;     /**< Reception transfer error handler                       */
}   usart_DmaPeriphConfig_t;


/** \brief Initialized DMA stream of one direction */
typedef struct
{
    usart_FunctionState_t Initialized; /**< DMA stream was initialized for the direction */
    dma_PeriphId_t        DmaId;       /**< Initialized DMA peripheral                   */
    dma_ChannelId_t       StreamId;    /**< Initialized DMA stream                       */
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

static usart_RequestState_t Usart_Dma_Get_Stream      ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t streamId, dma_PeriphReqId_t * const channelSel );
static usart_RequestState_t Usart_Dma_Check_Stream    ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t streamId, usart_DmaPriority_t priority );
static usart_RequestState_t Usart_Dma_Set_ChannelInit ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_ChannelOff  ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_Transfer    ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_Set_StreamStop  ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );
static usart_RequestState_t Usart_Dma_TxCplt          ( usart_PeriphId_t usartId );
static usart_RequestState_t Usart_Dma_XferError       ( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir );

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

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Count of streams connected to one USART request (\ref usart_DmaPeriphConfig_t) */
#define USART_DMA_STREAM_OPTIONS     ( 2u )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/**
 * \brief DMA streams of USART requests (RM0090 / RM0368 / RM0383 / RM0390 DMA request mapping,
 *        streams common for all STM32F4 devices with the peripheral)
 */
static const usart_DmaPeriphConfig_t usart_DmaPeriphConfig[ ] =
{
#ifdef USART1
    { .TxStream = { { DMA_PERIPH_2, DMA_STREAM_7, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_2, DMA_STREAM_7, DMA_REQ_CHANNEL_4 } },
      .RxStream = { { DMA_PERIPH_2, DMA_STREAM_2, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_2, DMA_STREAM_5, DMA_REQ_CHANNEL_4 } },
      USART_DMA_ISR_CONFIG( Usart1 ) },
#endif /* USART1 */
#ifdef USART2
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_6, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_6, DMA_REQ_CHANNEL_4 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_5, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_5, DMA_REQ_CHANNEL_4 } },
      USART_DMA_ISR_CONFIG( Usart2 ) },
#endif /* USART2 */
#ifdef USART3
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_3, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_4, DMA_REQ_CHANNEL_7 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_1, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_1, DMA_REQ_CHANNEL_4 } },
      USART_DMA_ISR_CONFIG( Usart3 ) },
#endif /* USART3 */
#ifdef UART4
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_4, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_4, DMA_REQ_CHANNEL_4 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_2, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_2, DMA_REQ_CHANNEL_4 } },
      USART_DMA_ISR_CONFIG( Uart4 ) },
#endif /* UART4 */
#ifdef UART5
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_7, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_7, DMA_REQ_CHANNEL_4 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_0, DMA_REQ_CHANNEL_4 }, { DMA_PERIPH_1, DMA_STREAM_0, DMA_REQ_CHANNEL_4 } },
      USART_DMA_ISR_CONFIG( Uart5 ) },
#endif /* UART5 */
#ifdef USART6
    { .TxStream = { { DMA_PERIPH_2, DMA_STREAM_6, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_2, DMA_STREAM_7, DMA_REQ_CHANNEL_5 } },
      .RxStream = { { DMA_PERIPH_2, DMA_STREAM_1, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_2, DMA_STREAM_2, DMA_REQ_CHANNEL_5 } },
      USART_DMA_ISR_CONFIG( Usart6 ) },
#endif /* USART6 */
#ifdef UART7
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_1, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_1, DMA_STREAM_1, DMA_REQ_CHANNEL_5 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_3, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_1, DMA_STREAM_3, DMA_REQ_CHANNEL_5 } },
      USART_DMA_ISR_CONFIG( Uart7 ) },
#endif /* UART7 */
#ifdef UART8
    { .TxStream = { { DMA_PERIPH_1, DMA_STREAM_0, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_1, DMA_STREAM_0, DMA_REQ_CHANNEL_5 } },
      .RxStream = { { DMA_PERIPH_1, DMA_STREAM_6, DMA_REQ_CHANNEL_5 }, { DMA_PERIPH_1, DMA_STREAM_6, DMA_REQ_CHANNEL_5 } },
      USART_DMA_ISR_CONFIG( Uart8 ) },
#endif /* UART8 */
};

_Static_assert( USART_BUS_CNT == ( sizeof(usart_DmaPeriphConfig) / sizeof(usart_DmaPeriphConfig_t) ), "Usart: usart_DmaPeriphConfig has incorrect size." );


/** \brief Initialized DMA streams per peripheral and direction */
static usart_DmaChannelState_t usart_DmaChannelState[ USART_BUS_CNT ][ USART_DMA_DIR_CNT ];

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Checks DMA related part of the transmission configuration
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the stream is connected to the transmit request, the
 *         priority is valid and the stream differs from the reception DMA stream. Otherwise
 *         returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Check_TxConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        retState = Usart_Dma_Check_Stream( usartId, USART_DMA_DIR_TX, dataConfig->TxDmaPeriphId, dataConfig->TxDmaChannelId, dataConfig->TxDmaPriority );

        /* Transmission and reception must not share one DMA stream */
        if( ( USART_XFER_MODE_DMA        == dataConfig->RxMode         ) &&
            ( dataConfig->TxDmaPeriphId  == dataConfig->RxDmaPeriphId  ) &&
            ( dataConfig->TxDmaChannelId == dataConfig->RxDmaChannelId )    )
        {
            retState = USART_REQUEST_ERROR;
        }
        else
        {
            /* Streams of the directions differ */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA transmission - DMA stream (memory to DR)
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
 * \brief Deinitializes DMA transmission - DMA stream and its interrupts are released, USART DMA
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

    return ( ( ( USART_REQUEST_OK == chState ) && ( USART_REQUEST_OK == reqState ) ) ? USART_REQUEST_OK : USART_REQUEST_ERROR );
}


/**
 * \brief Starts DMA transmission - TC flag is cleared, DMA stream is armed for the data of the
 *        transmission and the USART DMA transmit request is enabled
 *
 * \note  DMA writes to DR do not clear TC (no preceding read of SR) - TC is cleared before the
 *        transmission, so it signals the end of the last frame of this transmission.
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
        /* DMA stream could not be armed */
    }

    return ( retState );
}


/**
 * \brief Stops DMA transmission - DMA stream, USART DMA transmit request and TC interrupt are
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

    retState = Usart_Dma_Set_StreamStop( usartId, USART_DMA_DIR_TX );

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
 * \return Returns \ref USART_REQUEST_OK if the stream is connected to the receive request and
 *         the priority is valid (buffer is checked by Usart.c, DMA data count limit equals the
 *         usart_RxDataCnt_t range). Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Dma_Check_RxConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        retState = Usart_Dma_Check_Stream( usartId, USART_DMA_DIR_RX, dataConfig->RxDmaPeriphId, dataConfig->RxDmaChannelId, dataConfig->RxDmaPriority );
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA reception - DMA stream (DR to memory, normal mode for one shot buffer,
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
 * \brief Deinitializes DMA reception - DMA stream and its interrupts are released, USART DMA
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

    return ( ( ( USART_REQUEST_OK == chState ) && ( USART_REQUEST_OK == reqState ) ) ? USART_REQUEST_OK : USART_REQUEST_ERROR );
}


/**
 * \brief Starts DMA reception - stale data and flags are cleared, DMA stream is armed for the
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
        /* DMA stream could not be armed */
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
 * \brief Stops DMA reception - DMA stream, USART DMA receive request, error and end of message
 *        interrupts are disabled
 *
 * \note  Remaining count of the stream is kept - Usart_Dma_Get_RxCount() returns the count of
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
        retState = Usart_Dma_Set_StreamStop( usartId, USART_DMA_DIR_RX );
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

    if( ( USART_REQUEST_OK      == retState                                                        ) &&
        ( USART_NULL_PTR       != rxCnt                                                            ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ].Initialized )    )
    {
        const usart_DmaChannelState_t * const chState   = &usart_DmaChannelState[ usartId ][ USART_DMA_DIR_RX ];
        dma_DataCount_t                       remaining = 0u;
        const dma_RequestState_t              dmaState  = Dma_Get_DataCount( chState->DmaId, chState->StreamId, &remaining );

        if( ( DMA_REQUEST_OK == dmaState ) && ( xferCtx->Config.RxBufferSize >= remaining ) )
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
 * \brief Looks up DMA stream connected to the USART request
 *
 * \param usartId     [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir      [in]: Data transfer direction
 * \param dmaId       [in]: DMA peripheral, value from \ref usart_DmaPeriphId_t
 * \param streamId    [in]: DMA stream, value from \ref usart_DmaChannelId_t
 * \param channelSel [out]: Pointer to store channel selection of the stream. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK if the stream is connected to the request. Otherwise
 *         returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Get_Stream( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t streamId, dma_PeriphReqId_t * const channelSel )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT         > usartId    ) &&
        ( USART_DMA_DIR_CNT     > dmaDir     ) &&
        ( USART_DMA_PERIPH_CNT  > dmaId      ) &&
        ( USART_DMA_CHANNEL_CNT > streamId   ) &&
        ( USART_NULL_PTR       != channelSel )    )
    {
        const usart_DmaStream_t * const streams = ( USART_DMA_DIR_TX == dmaDir ) ? usart_DmaPeriphConfig[ usartId ].TxStream
                                                                                : usart_DmaPeriphConfig[ usartId ].RxStream;

        for( uint32_t streamIdx = 0u; USART_DMA_STREAM_OPTIONS > streamIdx; streamIdx ++ )
        {
            if( ( (dma_PeriphId_t)dmaId     == streams[ streamIdx ].DmaId    ) &&
                ( (dma_ChannelId_t)streamId == streams[ streamIdx ].StreamId )    )
            {
                *channelSel = streams[ streamIdx ].ChannelSel;
                retState    = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Stream does not match, keep searching */
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
 * \brief Checks DMA stream and priority of one direction
 *
 * \param usartId  [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir   [in]: Data transfer direction
 * \param dmaId    [in]: DMA peripheral, value from \ref usart_DmaPeriphId_t
 * \param streamId [in]: DMA stream, value from \ref usart_DmaChannelId_t
 * \param priority [in]: DMA stream priority, value from \ref usart_DmaPriority_t
 *
 * \return Returns \ref USART_REQUEST_OK if the stream is connected to the request and the
 *         priority is valid. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Check_Stream( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir, usart_DmaPeriphId_t dmaId, usart_DmaChannelId_t streamId, usart_DmaPriority_t priority )
{
    usart_RequestState_t retState   = USART_REQUEST_ERROR;
    dma_PeriphReqId_t    channelSel = DMA_REQ_CHANNEL_0;

    if( (uint32_t)DMA_PRIORITY_CNT > (uint32_t)priority )
    {
        retState = Usart_Dma_Get_Stream( usartId, dmaDir, dmaId, streamId, &channelSel );
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Initializes DMA stream of one direction (channel selection, priority, 8-bit data, DR as
 *        peripheral address) and its interrupts
 *
 * - Transmission: memory (increment) -> DR, normal mode, transfer complete and error handler
 * - Reception: DR -> RxBuffer (increment), normal / circular mode by RxBufferMode, transfer
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
    usart_RequestState_t  retState   = USART_REQUEST_ERROR;
    USART_TypeDef *       periphReg  = USART_NULL_PTR;
    usart_XferContext_t * xferCtx    = USART_NULL_PTR;
    dma_PeriphReqId_t     channelSel = DMA_REQ_CHANNEL_0;

    retState = Usart_Get_PeriphReg( usartId, &periphReg );

    if( USART_REQUEST_OK == retState )
    {
        retState = Usart_Get_XferContext( usartId, &xferCtx );
    }
    else
    {
        /* Invalid peripheral identification */
    }

    if( ( USART_REQUEST_OK == retState ) && ( USART_DMA_DIR_CNT > dmaDir ) )
    {
        const usart_DmaPeriphConfig_t * const periphConf = &usart_DmaPeriphConfig[ usartId ];
        usart_DmaChannelState_t * const       chState    = &usart_DmaChannelState[ usartId ][ dmaDir ];
        dma_ConfigStruct_t                    dmaConfig;
        dma_RequestState_t                    dmaState   = DMA_REQUEST_ERROR;
        usart_DmaPeriphId_t                   dmaId      = xferCtx->Config.TxDmaPeriphId;
        usart_DmaChannelId_t                  streamId   = xferCtx->Config.TxDmaChannelId;
        usart_DmaPriority_t                   priority   = xferCtx->Config.TxDmaPriority;

        if( USART_DMA_DIR_RX == dmaDir )
        {
            dmaId    = xferCtx->Config.RxDmaPeriphId;
            streamId = xferCtx->Config.RxDmaChannelId;
            priority = xferCtx->Config.RxDmaPriority;
        }
        else
        {
            /* Transmit stream */
        }

        retState = Usart_Dma_Get_Stream( usartId, dmaDir, dmaId, streamId, &channelSel );
        dmaState = Dma_Get_DefaultConfig( &dmaConfig );

        dmaConfig.DmaPeriphId          = (dma_PeriphId_t)dmaId;
        dmaConfig.DmaChannel           = (dma_ChannelId_t)streamId;
        dmaConfig.PeripheralReqId      = channelSel;
        dmaConfig.PeriphAddress        = (dma_PeriphAddr_t)(uintptr_t)&periphReg->DR;
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
            dmaConfig.MemoryAddress            = 0u;
            dmaConfig.DataCount                = 0u;
            dmaConfig.TransferCompleteCallback = periphConf->TxCpltIsr;
            dmaConfig.TransferErrorCallback    = periphConf->TxErrorIsr;
        }
        else
        {
            dmaConfig.Direction                = DMA_DIR_PERIPH_TO_MEMORY;
            dmaConfig.TransferMode             = ( USART_BUFFER_MODE_CIRCULAR == xferCtx->Config.RxBufferMode ) ? DMA_TRANSFER_MODE_CIRCULAR
                                                                                                                : DMA_TRANSFER_MODE_NORMAL;
            dmaConfig.MemoryAddress            = (dma_MemoryAddr_t)(uintptr_t)xferCtx->Config.RxBuffer;
            dmaConfig.DataCount                = (dma_DataCount_t)xferCtx->Config.RxBufferSize;
            dmaConfig.TransferCompleteCallback = periphConf->RxCpltIsr;
            dmaConfig.TransferErrorCallback    = periphConf->RxErrorIsr;

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
            /* Stream is not connected to the request */
            dmaState = DMA_REQUEST_ERROR;
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            chState->Initialized = USART_FUNCTION_ACTIVE;
            chState->DmaId       = dmaConfig.DmaPeriphId;
            chState->StreamId    = dmaConfig.DmaChannel;

            dmaState = Dma_Set_TransferCompleteIrqActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Stream initialization failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferErrorIrqActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Transfer complete interrupt could not be enabled */
        }

        if( ( DMA_REQUEST_OK == dmaState ) && ( DMA_NULL_PTR != dmaConfig.HalfTransferCallback ) )
        {
            dmaState = Dma_Set_HalfTransferIrqActive( chState->DmaId, chState->StreamId );
        }
        else if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_HalfTransferIrqInactive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Transfer error interrupt could not be enabled */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_InterruptActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Half transfer interrupt could not be configured */
        }

        retState = ( DMA_REQUEST_OK == dmaState ) ? USART_REQUEST_OK : USART_REQUEST_ERROR;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Releases DMA stream of one direction - stream is disabled, its interrupts and callbacks
 *        are released
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems (also if no stream was initialized). Otherwise
 *         returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Dma_Set_ChannelOff( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) && ( USART_DMA_DIR_CNT > dmaDir ) )
    {
        usart_DmaChannelState_t * const chState = &usart_DmaChannelState[ usartId ][ dmaDir ];

        if( USART_FUNCTION_ACTIVE == chState->Initialized )
        {
            const dma_RequestState_t xferState = Dma_Set_TransferInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t tcState   = Dma_Set_TransferCompleteIrqInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t htState   = Dma_Set_HalfTransferIrqInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t teState   = Dma_Set_TransferErrorIrqInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t nvicState = Dma_Set_InterruptInactive( chState->DmaId, chState->StreamId );
            const dma_RequestState_t tcIsr     = Dma_Set_TransferCompleteIsrHandler( chState->DmaId, chState->StreamId, DMA_NULL_PTR );
            const dma_RequestState_t htIsr     = Dma_Set_HalfTransferIsrHandler( chState->DmaId, chState->StreamId, DMA_NULL_PTR );
            const dma_RequestState_t teIsr     = Dma_Set_TransferErrorIsrHandler( chState->DmaId, chState->StreamId, DMA_NULL_PTR );

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
            /* No DMA stream was initialized for the direction */
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
 * \brief Arms the DMA stream of one direction (memory address, count, enable)
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

    if( ( USART_REQUEST_OK      == retState                                              ) &&
        ( USART_DMA_DIR_CNT      > dmaDir                                                ) &&
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

        /* Stream is stopped (configuration is writable only with disabled stream) */
        dmaState = Dma_Set_TransferInactive( chState->DmaId, chState->StreamId );

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_MemoryAddr( chState->DmaId, chState->StreamId, memAddr );
        }
        else
        {
            /* Stream could not be stopped */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_DataCount( chState->DmaId, chState->StreamId, count );
        }
        else
        {
            /* Memory address configuration failed */
        }

        if( DMA_REQUEST_OK == dmaState )
        {
            dmaState = Dma_Set_TransferActive( chState->DmaId, chState->StreamId );
        }
        else
        {
            /* Count configuration failed */
        }

        retState = ( DMA_REQUEST_OK == dmaState ) ? USART_REQUEST_OK : USART_REQUEST_ERROR;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Disables the DMA stream of one direction (configuration and remaining count are kept)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dmaDir  [in]: Data transfer direction
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR (also if
 *         no stream is initialized for the direction).
 */
static usart_RequestState_t Usart_Dma_Set_StreamStop( usart_PeriphId_t usartId, usart_DmaDir_t dmaDir )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT         > usartId                                                ) &&
        ( USART_DMA_DIR_CNT     > dmaDir                                                 ) &&
        ( USART_FUNCTION_ACTIVE == usart_DmaChannelState[ usartId ][ dmaDir ].Initialized )    )
    {
        const usart_DmaChannelState_t * const chState  = &usart_DmaChannelState[ usartId ][ dmaDir ];
        const dma_RequestState_t              dmaState = Dma_Set_TransferInactive( chState->DmaId, chState->StreamId );

        retState = ( DMA_REQUEST_OK == dmaState ) ? USART_REQUEST_OK : USART_REQUEST_ERROR;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Transmission transfer complete processing - all bytes were written to DR, the USART
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
    const usart_RequestState_t stopState = ( USART_DMA_DIR_TX == dmaDir ) ? Usart_Set_TxStop( usartId )
                                                                          : Usart_Set_RxStop( usartId );
    const usart_RequestState_t errState  = Usart_Set_XferError( usartId, USART_XFER_ERROR_DMA_TRANSFER );

    return ( ( ( USART_REQUEST_OK == stopState ) && ( USART_REQUEST_OK == errState ) ) ? USART_REQUEST_OK : USART_REQUEST_ERROR );
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

/* ================================ TASKS =================================== */
