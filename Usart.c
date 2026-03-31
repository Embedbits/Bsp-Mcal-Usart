/**
 * \author Mr.Nobody
 * \file Usart.h
 * \ingroup Usart
 * \brief Universal Synchronous/Asynchronous Receiver-Transmitter (USART) MCAL
 *        module common functionality
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart.h"                          /* Self include                   */
#include "Usart_Port.h"                     /* Own port file include          */
#include "Usart_Types.h"                    /* Module types definitions       */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
#include "Gpio_Port.h"                      /* GPIO handler functionality     */
#include "Gpdma_Port.h"                     /* DMA handler functionality      */
#include "Nvic_Port.h"                      /* NVIC handler functionality     */
#include "Rcc_Port.h"                       /* RCC handler functionality      */
/* ============================== TYPEDEFS ================================== */

/** Structure type used for USART/UART configuration array */
typedef struct
{
    USART_TypeDef*       PeriphReg;      /**< Peripheral configuration register        */
    rcc_PeriphId_t       PeriphRcc;      /**< Peripheral RCC configuration ID          */
    nvic_PeriphIrqList_t PeriphNvic;     /**< Peripheral NVIC configuration ID         */
    nvic_IsrCallback_t   PeriphIsr;      /**< Peripheral ISR callback routines         */
    gpdma_PeriphReqId_t  PeriphDmaTxReq; /**< DMA transfer request ID for transmission */
    gpdma_PeriphReqId_t  PeriphDmaRxReq; /**< DMA transfer request ID for reception    */
}   usart_PeriphConfigStruct_t;

/** Structure type used to store users USART/UART ISR callback pointers */
typedef struct
{
    usart_PeriphId_t              PeriphId;
    usart_TransferStyle_t         TransferStyle;
    usart_DmaPeriphId_t           DmaTxPeriphId;
    usart_DmaChannelId_t          DmaTxChannelId;
    usart_DmaPeriphId_t           DmaRxPeriphId;
    usart_DmaChannelId_t          DmaRxChannelId;
    usart_RxNeIrqCallback_t      *RxNotEmptyIsr;
    usart_ErrIrqCallback_t       *ErrorIsr;
    usart_TxeIrqCallback_t       *TxEmptyIsr;
    usart_TcIrqCallback_t        *TransferCompleteIsr;
    usart_IdleIrqCallback_t      *IdleIsr;
    usart_RxTimeoutIrqCallback_t *RxTimeoutIsr;

    gpdma_XferList_t              TxTransferList;
    gpdma_XferList_t              RxTransferList;
}   usart_RuntimeConfigStruct_t;

/* ======================== FORWARD DECLARATIONS ============================ */

#ifdef USART1
static void Usart_Usart1_IsrHandler(void);
#endif /* USART1 */
#ifdef USART2
static void Usart_Usart2_IsrHandler(void);
#endif /* USART2 */
#ifdef USART3
static void Usart_Usart3_IsrHandler(void);
#endif /* USART3 */
#ifdef UART4
static void Uart_Usart4_IsrHandler(void);
#endif /* UART4 */
#ifdef UART5
static void Uart_Usart5_IsrHandler(void);
#endif /* UART5 */
#ifdef USART6
static void Usart_Usart6_IsrHandler(void);
#endif /* USART6 */

static inline void Usart_GlobalIsrHandler( usart_PeriphId_t usartId );

static usart_RequestState_t Usart_Set_Prescaler( usart_PeriphId_t usartId, usart_Prescaler_t prescaler );
static usart_RequestState_t Usart_Get_Prescaler( usart_PeriphId_t usartId, usart_Prescaler_t *prescaler );
static usart_RequestState_t Usart_Get_ExpectedPrescaler( usart_PeriphId_t usartId,
                                                         usart_FreqHz_t periphClock,
                                                         usart_Oversampling_t oversampling,
                                                         usart_Baudrate_t baudrate,
                                                         usart_Prescaler_t *prescaler );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define USART_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define USART_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define USART_PATCH_VERSION           ( 0u )

/** Maximum timeout for system reaction to request */
#define USART_TIMEOUT_RAW             ( 0x84FCB )

/* =============================== MACROS =================================== */

/** Bit-mask of all possible errors in ISR register */
#define USART_ISR_ERROR_MASK          ( ( LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_NE | LL_USART_ISR_ORE ) )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** USART/UART peripherals runtime data array */
static volatile usart_RuntimeConfigStruct_t     usart_RuntimeData[ USART_BUS_CNT ] =
{
    { .PeriphId = USART_BUS_1, .TransferStyle = USART_TRANSFER_BLOCKING, .DmaTxPeriphId = USART_DMA_PERIPH_CNT, .DmaTxChannelId = USART_DMA_CHANNEL_CNT, .DmaRxPeriphId = USART_DMA_PERIPH_CNT, .DmaRxChannelId = USART_DMA_CHANNEL_CNT, .RxNotEmptyIsr = USART_NULL_PTR, .ErrorIsr = USART_NULL_PTR, .TxEmptyIsr = USART_NULL_PTR, .TransferCompleteIsr = USART_NULL_PTR, .IdleIsr = USART_NULL_PTR, .RxTimeoutIsr = USART_NULL_PTR },
    { .PeriphId = USART_BUS_2, .TransferStyle = USART_TRANSFER_BLOCKING, .DmaTxPeriphId = USART_DMA_PERIPH_CNT, .DmaTxChannelId = USART_DMA_CHANNEL_CNT, .DmaRxPeriphId = USART_DMA_PERIPH_CNT, .DmaRxChannelId = USART_DMA_CHANNEL_CNT, .RxNotEmptyIsr = USART_NULL_PTR, .ErrorIsr = USART_NULL_PTR, .TxEmptyIsr = USART_NULL_PTR, .TransferCompleteIsr = USART_NULL_PTR, .IdleIsr = USART_NULL_PTR, .RxTimeoutIsr = USART_NULL_PTR },
    { .PeriphId = USART_BUS_3, .TransferStyle = USART_TRANSFER_BLOCKING, .DmaTxPeriphId = USART_DMA_PERIPH_CNT, .DmaTxChannelId = USART_DMA_CHANNEL_CNT, .DmaRxPeriphId = USART_DMA_PERIPH_CNT, .DmaRxChannelId = USART_DMA_CHANNEL_CNT, .RxNotEmptyIsr = USART_NULL_PTR, .ErrorIsr = USART_NULL_PTR, .TxEmptyIsr = USART_NULL_PTR, .TransferCompleteIsr = USART_NULL_PTR, .IdleIsr = USART_NULL_PTR, .RxTimeoutIsr = USART_NULL_PTR },
    { .PeriphId = USART_BUS_4, .TransferStyle = USART_TRANSFER_BLOCKING, .DmaTxPeriphId = USART_DMA_PERIPH_CNT, .DmaTxChannelId = USART_DMA_CHANNEL_CNT, .DmaRxPeriphId = USART_DMA_PERIPH_CNT, .DmaRxChannelId = USART_DMA_CHANNEL_CNT, .RxNotEmptyIsr = USART_NULL_PTR, .ErrorIsr = USART_NULL_PTR, .TxEmptyIsr = USART_NULL_PTR, .TransferCompleteIsr = USART_NULL_PTR, .IdleIsr = USART_NULL_PTR, .RxTimeoutIsr = USART_NULL_PTR },
    { .PeriphId = USART_BUS_5, .TransferStyle = USART_TRANSFER_BLOCKING, .DmaTxPeriphId = USART_DMA_PERIPH_CNT, .DmaTxChannelId = USART_DMA_CHANNEL_CNT, .DmaRxPeriphId = USART_DMA_PERIPH_CNT, .DmaRxChannelId = USART_DMA_CHANNEL_CNT, .RxNotEmptyIsr = USART_NULL_PTR, .ErrorIsr = USART_NULL_PTR, .TxEmptyIsr = USART_NULL_PTR, .TransferCompleteIsr = USART_NULL_PTR, .IdleIsr = USART_NULL_PTR, .RxTimeoutIsr = USART_NULL_PTR },
};



/** USART/UART peripherals configuration array */
static usart_PeriphConfigStruct_t const         usart_PeriphConf[ USART_BUS_CNT ] =
{
#ifdef USART1
    { .PeriphReg = USART1, .PeriphRcc = RCC_PERIPH_USART1_APB2, .PeriphNvic = NVIC_PERIPH_IRQ_USART1, .PeriphIsr = Usart_Usart1_IsrHandler, .PeriphDmaTxReq = GPDMA_REQ_USART1_TX, .PeriphDmaRxReq = GPDMA_REQ_USART1_RX },
#endif
#ifdef USART2
    { .PeriphReg = USART2, .PeriphRcc = RCC_PERIPH_USART2_APB1, .PeriphNvic = NVIC_PERIPH_IRQ_USART2, .PeriphIsr = Usart_Usart2_IsrHandler, .PeriphDmaTxReq = GPDMA_REQ_USART2_TX, .PeriphDmaRxReq = GPDMA_REQ_USART2_RX },
#endif
#ifdef USART3
    { .PeriphReg = USART3, .PeriphRcc = RCC_PERIPH_USART3_APB1, .PeriphNvic = NVIC_PERIPH_IRQ_USART3, .PeriphIsr = Usart_Usart3_IsrHandler, .PeriphDmaTxReq = GPDMA_REQ_USART3_TX, .PeriphDmaRxReq = GPDMA_REQ_USART3_RX },
#endif
#ifdef UART4
    { .PeriphReg = UART4,  .PeriphRcc = RCC_PERIPH_UART4_PCLK1, .PeriphNvic = NVIC_PERIPH_IRQ_UART4 , .PeriphIsr = Uart_Usart4_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_UART4_TX , .PeriphDmaRxReq = GPDMA_REQ_UART4_RX  },
#endif
#ifdef UART5
    { .PeriphReg = UART5,  .PeriphRcc = RCC_PERIPH_UART5_PCLK1, .PeriphNvic = NVIC_PERIPH_IRQ_UART5 , .PeriphIsr = Uart_Usart5_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_UART5_TX , .PeriphDmaRxReq = GPDMA_REQ_UART5_RX  },
#endif
#ifdef USART6
    { .PeriphReg = USART6, .PeriphRcc = RCC_PERIPH_USART6_APB1, .PeriphNvic = NVIC_PERIPH_IRQ_USART6, .PeriphIsr = Usart_Usart6_IsrHandler, .PeriphDmaTxReq = GPDMA_REQ_USART6_TX, .PeriphDmaRxReq = GPDMA_REQ_USART6_RX },
#endif
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
usart_ModuleVersion_t Usart_Get_ModuleVersion( void )
{
    usart_ModuleVersion_t retVersion;

    retVersion.Major = USART_MAJOR_VERSION;
    retVersion.Minor = USART_MINOR_VERSION;
    retVersion.Patch = USART_PATCH_VERSION;

    return (retVersion);
}

/**
 * \brief USART/UART bus initialization through configuration structure
 *
 * \param usartConfig [in]: Pointer to configuration structure
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Init( usart_BusConfig_t * const usartConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_NULL_PTR != usartConfig )
    {
        rcc_FunctionState_t rccActivationState = RCC_FUNCTION_INACTIVE;
        rcc_RequestState_t  rccRequestState    = RCC_REQUEST_ERROR;

        /*------------- USART peripheral clock activation section ------------*/
        rccRequestState = Rcc_Get_PeriphState( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc, &rccActivationState );

        if( ( RCC_REQUEST_ERROR     != rccRequestState    ) &&
            ( RCC_FUNCTION_INACTIVE == rccActivationState )    )
        {
            rccRequestState = Rcc_Set_PeriphActive( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc );

            if( RCC_REQUEST_ERROR == rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
                return ( retState );
            }
        }
        else
        {
            /* No clock activation needed */
        }

        /*---------- USART peripheral GPIO initialization section ------------*/
        if( ( USART_BIT_MASK_DECODE_PERIPH( usartConfig->BusRxPin ) == usartConfig->PeriphId ) &&
            ( USART_RX_PIN_UNUSED                                   != usartConfig->BusRxPin )    )
        {
            Usart_InitRxGpio( usartConfig->BusRxPin );
        }
        else
        {
            /* RX pin configuration is not used */
        }

        if( ( USART_BIT_MASK_DECODE_PERIPH( usartConfig->BusTxPin ) == usartConfig->PeriphId ) &&
            ( USART_TX_PIN_UNUSED                                   != usartConfig->BusTxPin )    )
        {
            Usart_InitTxGpio( usartConfig->BusTxPin );
        }
        else
        {
            /* TX pin configuration is not used */
        }

        if( ( USART_BIT_MASK_DECODE_PERIPH( usartConfig->BusDePin ) == usartConfig->PeriphId ) &&
            ( USART_DE_PIN_UNUSED                                   != usartConfig->BusDePin )    )
        {
            Usart_InitDeGpio( usartConfig->BusDePin );
        }
        else
        {
            /* DE pin configuration is not used */
        }

        /*------------ USART peripheral initialization section ---------------*/
        rccRequestState = Rcc_Set_ResetActive( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc );
        if( RCC_REQUEST_OK != rccRequestState )
        {
            retState = USART_REQUEST_ERROR;
            return ( retState );
        }

        rccRequestState = Rcc_Set_ResetInactive( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc );
        if( RCC_REQUEST_OK != rccRequestState )
        {
            retState = USART_REQUEST_ERROR;
            return ( retState );
        }

        /* Over-sampling configuration must be executed before baud-rate configuration */
        retState = Usart_Set_Oversampling( usartConfig->PeriphId, usartConfig->Oversampling );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_Baudrate( usartConfig->PeriphId, usartConfig->BaudRate );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_DataWidth( usartConfig->PeriphId, usartConfig->DataWidth );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_StopBits( usartConfig->PeriphId, usartConfig->StopBits );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_Parity( usartConfig->PeriphId, usartConfig->Parity );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_TransferMode( usartConfig->PeriphId, usartConfig->TransferMode );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_FlowControl( usartConfig->PeriphId, usartConfig->HwFlowControl );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_HalfDuplexState( usartConfig->PeriphId, usartConfig->HalfDuplex );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_DriverEnableState( usartConfig->PeriphId, usartConfig->DriverEnableMode );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_DriverEnablePolarity( usartConfig->PeriphId, usartConfig->DriverEnablePolarity );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_PinLevels( usartConfig->PeriphId, usartConfig->RxPinOperationLevels, usartConfig->TxPinOperationLevels );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }


        if( USART_RX_TIMEOUT_MIN < usartConfig->RxTimeoutValue )
        {
            retState = Usart_Set_RxTimeoutActive( usartConfig->PeriphId, usartConfig->RxTimeoutValue );
        }
        else
        {
            retState = Usart_Set_RxTimeoutInactive( usartConfig->PeriphId );
        }

        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        if( USART_NULL_PTR != usartConfig->DmaConfiguration )
        {
            retState = Usart_Init_Dma( usartConfig->PeriphId, usartConfig->DmaConfiguration );

            if( USART_REQUEST_OK != retState )
            {
                return ( retState );
            }
        }

        Usart_Set_PeriphActive( usartConfig->PeriphId );

        Usart_Set_IrqPriority( usartConfig->PeriphId, usartConfig->IrqPriority );

        /* Return states of following functions are not checked. User do not need to strictly configure those options */
        ( void ) Usart_Set_RxNotEmptyIsrCallback( usartConfig->PeriphId, usartConfig->RxNotEmpty_ISR       );
        ( void ) Usart_Set_TxEmptyIsrCallback   ( usartConfig->PeriphId, usartConfig->TransmitEmpty_ISR    );
        ( void ) Usart_Set_TxCompleteIsrCallback( usartConfig->PeriphId, usartConfig->TransferComplete_ISR );
        ( void ) Usart_Set_IdleIsrCallback      ( usartConfig->PeriphId, usartConfig->Idle_ISR             );
        ( void ) Usart_Set_RxTimeoutIsrCallback ( usartConfig->PeriphId, usartConfig->RxTimeout_ISR        );
        ( void ) Usart_Set_ErrorIsrCallback     ( usartConfig->PeriphId, usartConfig->Error_ISR            );

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Component de-initialization function
 *
 * \param usartConfig [in]: Pointer to configuration structure
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Deinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t returnState = USART_REQUEST_ERROR;

    Rcc_Set_ResetActive( usart_PeriphConf[ usartId ].PeriphRcc );
    Rcc_Set_ResetInactive( usart_PeriphConf[ usartId ].PeriphRcc );

    Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartId ].PeriphNvic );

    LL_USART_Disable( usart_PeriphConf[ usartId ].PeriphReg );

    Usart_Set_InterruptsInactive( usartId );

    Rcc_Set_PeriphInactive( usart_PeriphConf[ usartId ].PeriphRcc );

    returnState = USART_REQUEST_OK;

    return ( returnState );
}


/**
 * \brief Main task of module Usart
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's 
 * requirements.
 */
void Usart_Task( void )
{

}


/**
 * \brief Initialization of configuration array to default values.
 *
 * \param usartConfig [out]: Pointer to configuration array
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DefaultConfig( usart_BusConfig_t* usartConfig )
{
    usart_RequestState_t returnState = USART_REQUEST_ERROR;

    if( USART_NULL_PTR != usartConfig )
    {
        usartConfig->PeriphId             = USART_BUS_1;
        usartConfig->BaudRate             = 115200u;
        usartConfig->DataWidth            = USART_DATA_WIDTH_8;
        usartConfig->StopBits             = USART_STOP_BITS_1;
        usartConfig->Parity               = USART_PARITY_NONE;
        usartConfig->TransferMode         = USART_TRANSFER_MODE_TX_RX;
        usartConfig->HwFlowControl        = USART_FLOW_CONTROL_NONE;
        usartConfig->DriverEnableMode     = USART_DE_DISABLED;
        usartConfig->DriverEnablePolarity = USART_DE_ACTIVE_HIGH;
        usartConfig->Oversampling         = USART_OVERSAMPLING_8;
        usartConfig->HalfDuplex           = USART_HALF_DUPLEX_INACTIVE;
        usartConfig->RxTimeoutValue       = 0u;
        usartConfig->RxPinOperationLevels = USART_RX_PIN_STANDARD;
        usartConfig->TxPinOperationLevels = USART_TX_PIN_STANDARD;
        usartConfig->OperationMode        = USART_TRANSFER_BLOCKING;
        usartConfig->DmaConfiguration     = USART_NULL_PTR;
        usartConfig->IrqPriority          = 10u;
        usartConfig->RxNotEmpty_ISR       = USART_NULL_PTR;
        usartConfig->TransmitEmpty_ISR    = USART_NULL_PTR;
        usartConfig->TransferComplete_ISR = USART_NULL_PTR;
        usartConfig->Error_ISR            = USART_NULL_PTR;
        usartConfig->Idle_ISR             = USART_NULL_PTR;
        usartConfig->RxTimeout_ISR        = USART_NULL_PTR;
        usartConfig->BusRxPin             = USART_RX_PIN_UNUSED;
        usartConfig->BusTxPin             = USART_TX_PIN_UNUSED;
        usartConfig->BusDePin             = USART_DE_PIN_UNUSED;

        returnState = USART_REQUEST_OK;
    }
    else
    {
        returnState = USART_REQUEST_ERROR;
    }

    return ( returnState );
}


/**
 * \brief Enables USART peripheral
 *
 * When peripheral is deactivated, the USART prescalers and outputs are stopped
 * immediately, and all current operations are discarded. The USART configuration
 * is kept, but all the USART_ISR status flags are reset. This bit is set and
 * cleared by software.
 * Note:
 * To enter low-power mode without generating errors on the line, the TE bit
 * must be previously reset and the software must wait for the TC bit in the
 * USART_ISR to be set before resetting the UE bit.
 * The DMA requests are also reset when UE = 0 so the DMA channel must be
 * disabled before resetting the UE bit.
 * In Smartcard mode, (SCEN = 1), the CK pin is always available when
 * CLKEN = 1, regardless of the UE bit value.
 *
 * \param usartId  [in]: USART/UART bus identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_PeriphActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0;

    if( USART_BUS_CNT > usartId )
    {
        regValue = LL_USART_IsEnabled( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u == regValue )
        {
            LL_USART_Enable( usart_PeriphConf[ usartId ].PeriphReg );

            for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                regValue = LL_USART_IsEnabled( usart_PeriphConf[ usartId ].PeriphReg );

                if( 0u != regValue )
                {
                    retValue = USART_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retValue = USART_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Peripheral is already active */
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        /* Required peripheral ID is incorrect */
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Disables USART peripheral
 *
 * When peripheral is deactivated, the USART prescalers and outputs are stopped
 * immediately, and all current operations are discarded. The USART configuration
 * is kept, but all the USART_ISR status flags are reset. This bit is set and
 * cleared by software.
 * Note:
 * To enter low-power mode without generating errors on the line, the TE bit
 * must be previously reset and the software must wait for the TC bit in the
 * USART_ISR to be set before resetting the UE bit.
 * The DMA requests are also reset when UE = 0 so the DMA channel must be
 * disabled before resetting the UE bit.
 * In Smartcard mode, (SCEN = 1), the CK pin is always available when
 * CLKEN = 1, regardless of the UE bit value.
 *
 * \param usartId  [in]: USART/UART bus identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_PeriphInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0;

    if( USART_BUS_CNT > usartId )
    {
        regValue = LL_USART_IsEnabled( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            LL_USART_Disable( usart_PeriphConf[ usartId ].PeriphReg );

            for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                regValue = LL_USART_IsEnabled( usart_PeriphConf[ usartId ].PeriphReg );

                if( 0u == regValue )
                {
                    retValue = USART_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retValue = USART_REQUEST_ERROR;
                }
            }
        }
        else
        {
            /* Peripheral is already active */
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        /* Required peripheral ID is incorrect */
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns USART peripheral activation state
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param reqState [out]: Actual activation state of peripheral
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_PeriphState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        regValue = LL_USART_IsEnabled( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        /* Required peripheral ID is incorrect */
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures baud-rate for selected USART/UART bus
 *
 * \note Configuration of baud-rate reads configured over-sampling configuration.
 *       Thus over-sampling configuration has to be executed before baud-rate
 *       configuration.
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param baudrate [in]: Value of required baud-rate
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Baudrate( usart_PeriphId_t usartId, usart_Baudrate_t baudrate )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_Oversampling_t oversampling   = USART_OVERSAMPLING_16;
    usart_FlagState_t    periphActState = USART_FLAG_INACTIVE;
    uint32_t             usartPeriphClk = 0;
    usart_Prescaler_t    usartPrescaler = 0u;

    retValue = Usart_Get_Oversampling( usartId, &oversampling );

    rcc_RequestState_t   rccRequestState    = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );
    usart_RequestState_t prescCalcState     = Usart_Get_ExpectedPrescaler( usartId, usartPeriphClk, oversampling, baudrate, &usartPrescaler);
    usart_RequestState_t prescCalcConfState = Usart_Set_Prescaler( usartId, usartPrescaler );

    if( ( 0u                  != baudrate           ) &&
        ( RCC_REQUEST_ERROR   != rccRequestState    ) &&
        ( USART_REQUEST_ERROR != prescCalcState     ) &&
        ( USART_REQUEST_ERROR != prescCalcConfState ) &&
        ( USART_REQUEST_ERROR != retValue           ) &&
        ( USART_BUS_CNT        > usartId            )    )
    {
        retValue = Usart_Get_PeriphState( usartId, &periphActState );

        if( USART_REQUEST_ERROR != retValue )
        {
            if( USART_FLAG_ACTIVE == periphActState )
            {
                retValue = Usart_Set_PeriphInactive( usartId );
            }
            else
            {
                /* Peripheral is already inactive */
            }


            if( USART_REQUEST_ERROR != retValue )
            {
                LL_USART_SetBaudRate( usart_PeriphConf[ usartId ].PeriphReg,
                                      usartPeriphClk,
                                      usartPrescaler,
                                      oversampling,
                                      baudrate );

                if( USART_FLAG_ACTIVE == periphActState )
                {
                    retValue = Usart_Set_PeriphActive( usartId );
                }
                else
                {
                    /* Peripheral was inactive before configuration. */
                }
            }
            else
            {
                /* Peripheral de-activation was not successful. Cannot proceded */
                retValue = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* De-activation of peripheral was unsuccessful. Configuration cannot be processed */
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the baud-rate configuration for selected USART/UART bus.
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param baudrate [out]: Value of required baud-rate
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Baudrate( usart_PeriphId_t usartId, usart_Baudrate_t * const baudrate )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_Oversampling_t oversampling   = USART_OVERSAMPLING_16;
    uint32_t             usartPeriphClk = 0;
    usart_Prescaler_t    usartPrescaler = 0u;

    usart_RequestState_t oversamplingState = Usart_Get_Oversampling( usartId, &oversampling );
    rcc_RequestState_t   rccRequestState   = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );
    usart_RequestState_t prescState        = Usart_Get_Prescaler( usartPeriphClk, &usartPrescaler);

    if( ( USART_NULL_PTR      != baudrate          ) &&
        ( RCC_REQUEST_ERROR   != rccRequestState   ) &&
        ( USART_REQUEST_ERROR != prescState        ) &&
        ( USART_REQUEST_ERROR != oversamplingState )    )
    {
        *baudrate = LL_USART_GetBaudRate( usart_PeriphConf[ usartId ].PeriphReg,
                                          usartPeriphClk,
                                          usartPrescaler,
                                          oversampling );

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the data width configuration for selected USART/UART bus
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param dataWidth [in]: Required data width configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DataWidth( usart_PeriphId_t usartId, usart_DataWidth_t dataWidth )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_FlagState_t    periphActState = USART_FLAG_INACTIVE;
    uint32_t             dataWidthReg   = 0u;

    if( USART_BUS_CNT > usartId )
    {
        retValue = Usart_Get_PeriphState( usartId, &periphActState );

        if( USART_REQUEST_ERROR != retValue )
        {
            if( USART_FLAG_INACTIVE == periphActState )
            {
                retValue = Usart_Set_PeriphInactive( usartId );
            }
            else
            {
                /* Peripheral is already inactive */
            }


            if( USART_REQUEST_ERROR != retValue )
            {
                LL_USART_SetDataWidth( usart_PeriphConf[ usartId ].PeriphReg, dataWidth );

                for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    dataWidthReg = LL_USART_GetDataWidth( usart_PeriphConf[ usartId ].PeriphReg );

                    if( dataWidth == dataWidthReg )
                    {
                        retValue = USART_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Clock source has not yet been changed, keep return state as error */
                        retValue = USART_REQUEST_ERROR;
                    }
                }

                if( USART_FLAG_INACTIVE != periphActState )
                {
                    retValue = Usart_Set_PeriphActive( usartId );
                }
                else
                {
                    /* Peripheral was inactive before configuration. */
                }
            }
            else
            {
                /* Peripheral de-activation was not successful. Cannot proceded */
                retValue = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* De-activation of peripheral was unsuccessful. Configuration cannot be processed */
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the data width configuration for selected USART/UART bus
 *
 * \param usartId    [in]: USART/UART bus identification.
 * \param dataWidth [out]: Value of data width configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DataWidth( usart_PeriphId_t usartId, usart_DataWidth_t * const dataWidth )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId   ) &&
        ( USART_NULL_PTR != dataWidth )    )
    {
        *dataWidth = LL_USART_GetDataWidth( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the stop-bits configuration for selected USART/UART bus
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param stopBits [in]: Required stop-bits configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_StopBits( usart_PeriphId_t usartId, usart_StopBits_t stopBits )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_FlagState_t    periphActState = USART_FLAG_INACTIVE;
    uint32_t             stopBitsReg    = 0u;

    if( USART_BUS_CNT > usartId )
    {
        retValue = Usart_Get_PeriphState( usartId, &periphActState );

        if( USART_REQUEST_ERROR != retValue )
        {
            if( USART_FLAG_INACTIVE == periphActState )
            {
                retValue = Usart_Set_PeriphInactive( usartId );
            }
            else
            {
                /* Peripheral is already inactive */
            }


            if( USART_REQUEST_ERROR != retValue )
            {
                LL_USART_SetStopBitsLength( usart_PeriphConf[ usartId ].PeriphReg, stopBits );

                for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    stopBitsReg = LL_USART_GetStopBitsLength( usart_PeriphConf[ usartId ].PeriphReg );

                    if( stopBits == stopBitsReg )
                    {
                        retValue = USART_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Clock source has not yet been changed, keep return state as error */
                        retValue = USART_REQUEST_ERROR;
                    }
                }

                if( USART_FLAG_INACTIVE != periphActState )
                {
                    retValue = Usart_Set_PeriphActive( usartId );
                }
                else
                {
                    /* Peripheral was inactive before configuration. */
                }
            }
            else
            {
                /* Peripheral de-activation was not successful. Cannot proceded */
                retValue = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* De-activation of peripheral was unsuccessful. Configuration cannot be processed */
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the stop-bits configuration for selected USART/UART bus
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param stopBits [out]: Value of stop-bits configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_StopBits( usart_PeriphId_t usartId, usart_StopBits_t * const stopBits )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != stopBits )    )
    {
        *stopBits = LL_USART_GetStopBitsLength( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the parity configuration for selected USART/UART bus
 *
 * \param usartId [in]: USART/UART bus identification.
 * \param parity  [in]: Required parity configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Parity( usart_PeriphId_t usartId, usart_Parity_t parity )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_FlagState_t    periphActState = USART_FLAG_INACTIVE;
    uint32_t             parityReg      = 0u;

    if( USART_BUS_CNT > usartId )
    {
        retValue = Usart_Get_PeriphState( usartId, &periphActState );

        if( USART_REQUEST_ERROR != retValue )
        {
            if( USART_FLAG_INACTIVE == periphActState )
            {
                retValue = Usart_Set_PeriphInactive( usartId );
            }
            else
            {
                /* Peripheral is already inactive */
            }


            if( USART_REQUEST_ERROR != retValue )
            {
                LL_USART_SetParity( usart_PeriphConf[ usartId ].PeriphReg, parity );

                for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
                {
                    parityReg = LL_USART_GetParity( usart_PeriphConf[ usartId ].PeriphReg );

                    if( parity == parityReg )
                    {
                        retValue = USART_REQUEST_OK;
                        break;
                    }
                    else
                    {
                        /* Clock source has not yet been changed, keep return state as error */
                        retValue = USART_REQUEST_ERROR;
                    }
                }

                if( USART_FLAG_INACTIVE != periphActState )
                {
                    retValue = Usart_Set_PeriphActive( usartId );
                }
                else
                {
                    /* Peripheral was inactive before configuration. */
                }
            }
            else
            {
                /* Peripheral de-activation was not successful. Cannot proceded */
                retValue = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* De-activation of peripheral was unsuccessful. Configuration cannot be processed */
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the parity configuration for selected USART/UART bus
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param parity  [out]: Value of parity configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Parity( usart_PeriphId_t usartId, usart_Parity_t * const parity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId ) &&
        ( USART_NULL_PTR != parity  )    )
    {
        *parity = LL_USART_GetParity( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the transfer mode configuration for selected USART/UART bus
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param transferMode [in]: Required transfer mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TransferMode( usart_PeriphId_t usartId, usart_TransferMode_t transferMode )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             transferModeReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetTransferDirection( usart_PeriphConf[ usartId ].PeriphReg, transferMode );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            transferModeReg = LL_USART_GetTransferDirection( usart_PeriphConf[ usartId ].PeriphReg );

            if( transferMode == transferModeReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the transfer mode configuration for selected USART/UART bus
 *
 * \param usartId       [in]: USART/UART bus identification.
 * \param transferMode [out]: Value of transfer mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TransferMode( usart_PeriphId_t usartId, usart_TransferMode_t * const transferMode )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId      ) &&
        ( USART_NULL_PTR != transferMode )    )
    {
        *transferMode = LL_USART_GetTransferDirection( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the hardware flow control configuration for selected USART/UART bus
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param flowControl [in]: Required hardware flow control configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_FlowControl( usart_PeriphId_t usartId, usart_FlowControl_t flowControl )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    uint32_t             flowControlReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetHWFlowCtrl( usart_PeriphConf[ usartId ].PeriphReg, flowControl );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            flowControlReg = LL_USART_GetHWFlowCtrl( usart_PeriphConf[ usartId ].PeriphReg );

            if( flowControl == flowControlReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the hardware flow control configuration for selected USART/UART bus
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param flowControl [out]: Value of hardware flow control configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_FlowControl( usart_PeriphId_t usartId, usart_FlowControl_t * const flowControl )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId     ) &&
        ( USART_NULL_PTR != flowControl )    )
    {
        *flowControl = LL_USART_GetHWFlowCtrl( usart_PeriphConf[ usartId ].PeriphReg );
        retValue     = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the Driver Enable (DE) feature activation state
 *
 * \param usartId [in]: USART/UART bus identification.
 * \param deState [in]: Required Driver Enable (DE) mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DriverEnableState( usart_PeriphId_t usartId, usart_DeFeatureState_t deState )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             driverEnableReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_DE_DISABLED != deState )
        {
            LL_USART_EnableDEMode( usart_PeriphConf[ usartId ].PeriphReg );
        }
        else
        {
            LL_USART_DisableDEMode( usart_PeriphConf[ usartId ].PeriphReg );
        }

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            driverEnableReg = LL_USART_IsEnabledDEMode( usart_PeriphConf[ usartId ].PeriphReg );

            if( deState == driverEnableReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the Driver Enable (DE) feature activation state
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param deState [out]: Value of Driver Enable (DE) mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DriverEnableState( usart_PeriphId_t usartId, usart_DeFeatureState_t * const deState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId ) &&
        ( USART_NULL_PTR != deState  )    )
    {
        *deState  = LL_USART_IsEnabledDEMode( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets the Driver Enable (DE) active logic level
 *
 * \param usartId    [in]: USART/UART bus identification.
 * \param dePolarity [in]: Logic level for active state of Driver Enable (DE) pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DriverEnablePolarity( usart_PeriphId_t usartId, usart_DePolarity_t dePolarity )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             driverEnableReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetDESignalPolarity( usart_PeriphConf[ usartId ].PeriphReg, dePolarity );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            driverEnableReg = LL_USART_GetDESignalPolarity( usart_PeriphConf[ usartId ].PeriphReg );

            if( dePolarity == driverEnableReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns the Driver Enable (DE) active logic level
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param dePolarity [out]: Logic level for active state of Driver Enable (DE) pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DriverEnablePolarity( usart_PeriphId_t usartId, usart_DePolarity_t * const dePolarity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId    ) &&
        ( USART_NULL_PTR != dePolarity )    )
    {
        *dePolarity  = LL_USART_GetDESignalPolarity( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * @brief Configures assertion and de-assertion times of Driver Enable (DE) pin.
 *
 * This table provides the DE assertion time for various baud rates
 * and oversampling settings (16x and 8x).
 *
 *  ------------------------------------------------------------------------------------------
 * | Baud Rate (bps) | Oversampling | Min DE Assertion Time (�s) | Max DE Assertion Time (�s) |
 * |-----------------|--------------|----------------------------|----------------------------|
 * | 9600            | 16x          | 6.51                       | 208.33                     |
 * | 9600            | 8x           | 13.02                      | 416.67                     |
 * | 115200          | 16x          | 0.54                       | 27.08                      |
 * | 115200          | 8x           | 1.09                       | 54.17                      |
 * | 1000000         | 16x          | 0.0625                     | 2                          |
 * | 1000000         | 8x           | 0.125                      | 4                          |
 *  ------------------------------------------------------------------------------------------
 *
 * DE Assertion Time Calculation Formulas:
 *
 * For 16x Oversampling (OVER8 = 0):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 16}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 16}
 *
 * For 8x Oversampling (OVER8 = 1):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 8}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 8}
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param assertTime   [in]: Pin Driver Enable (DE) assertion time in us
 * \param deassertTime [in]: Pin Driver Enable (DE) de-assertion time in us
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_AssertDeassertTimes( usart_PeriphId_t usartId, usart_AssertTime_us_t assertTime, usart_DeassertTime_us_t deassertTime )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_Baudrate_t     baudrateVal     = 0u;
        usart_Oversampling_t oversamplingVal = 0u;

        usart_RequestState_t baudRetValue       = Usart_Get_Baudrate( usartId, &baudrateVal );
        usart_RequestState_t oversampleRetValue = Usart_Get_Oversampling( usartId, &oversamplingVal );

        if( ( USART_REQUEST_ERROR != baudRetValue       ) &&
            ( USART_REQUEST_ERROR != oversampleRetValue )    )
        {
            retValue = USART_REQUEST_OK;

            const uint8_t assertTargetVal   = Usart_Get_AssertDeassertRegValues( assertTime, baudrateVal, oversamplingVal );
            const uint8_t deassertTargetVal = Usart_Get_AssertDeassertRegValues( deassertTime, baudrateVal, oversamplingVal );

            LL_USART_SetDEAssertionTime( usart_PeriphConf[ usartId ].PeriphReg, assertTargetVal );
            LL_USART_SetDEDeassertionTime( usart_PeriphConf[ usartId ].PeriphReg, deassertTargetVal );

            for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                uint8_t assertTimeReg   = LL_USART_GetDEAssertionTime( usart_PeriphConf[ usartId ].PeriphReg );
                uint8_t deassertTimeReg = LL_USART_GetDEDeassertionTime( usart_PeriphConf[ usartId ].PeriphReg );

                if( ( assertTargetVal   == assertTimeReg   ) &&
                    ( deassertTargetVal == deassertTimeReg )    )
                {
                    retValue = USART_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retValue = USART_REQUEST_ERROR;
                }
            }
        }
        else
        {
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * @brief Returns assertion and de-assertion times of Driver Enable (DE) pin.
 *
 * This table provides the DE assertion time for various baud rates
 * and oversampling settings (16x and 8x).
 *
 *  ------------------------------------------------------------------------------------------
 * | Baud Rate (bps) | Oversampling | Min DE Assertion Time (�s) | Max DE Assertion Time (�s) |
 * |-----------------|--------------|----------------------------|----------------------------|
 * | 9600            | 16x          | 6.51                       | 208.33                     |
 * | 9600            | 8x           | 13.02                      | 416.67                     |
 * | 115200          | 16x          | 0.54                       | 27.08                      |
 * | 115200          | 8x           | 1.09                       | 54.17                      |
 * | 1000000         | 16x          | 0.0625                     | 2                          |
 * | 1000000         | 8x           | 0.125                      | 4                          |
 *  ------------------------------------------------------------------------------------------
 *
 * DE Assertion Time Calculation Formulas:
 *
 * For 16x Oversampling (OVER8 = 0):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 16}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 16}
 *
 * For 8x Oversampling (OVER8 = 1):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 8}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 8}
 *
 * \param usartId       [in]: USART/UART bus identification.
 * \param assertTime   [out]: Pin Driver Enable (DE) assertion time in us
 * \param deassertTime [out]: Pin Driver Enable (DE) de-assertion time in us
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_AssertDeassertTimes( usart_PeriphId_t usartId, usart_AssertTime_us_t * const assertTime, usart_DeassertTime_us_t * const deassertTime )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId      ) &&
        ( USART_NULL_PTR != assertTime   ) &&
        ( USART_NULL_PTR != deassertTime )    )
    {
        usart_Baudrate_t     baudrateVal     = 0u;
        usart_Oversampling_t oversamplingVal = 0u;

        usart_RequestState_t baudRetValue       = Usart_Get_Baudrate( usartId, &baudrateVal );
        usart_RequestState_t oversampleRetValue = Usart_Get_Oversampling( usartId, &oversamplingVal );

        if( ( USART_REQUEST_ERROR != baudRetValue       ) &&
            ( USART_REQUEST_ERROR != oversampleRetValue )    )
        {
            retValue = USART_REQUEST_OK;

            uint8_t assertRegVal   = LL_USART_GetDEAssertionTime( usart_PeriphConf[ usartId ].PeriphReg );
            uint8_t deassertRegVal = LL_USART_GetDEDeassertionTime( usart_PeriphConf[ usartId ].PeriphReg );

            *assertTime   = Usart_Get_AssertDeassertTime( assertRegVal, baudrateVal, oversamplingVal );
            *deassertTime = Usart_Get_AssertDeassertTime( deassertRegVal, baudrateVal, oversamplingVal );

        }
        else
        {
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures over-sampling mode for required USART/UART bus
 *
 * \param usartId          [in]: USART/UART bus identification.
 * \param oversamplingMode [in]: Required over-sampling mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Oversampling( usart_PeriphId_t usartId, usart_Oversampling_t oversamplingMode )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             oversamplingReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetOverSampling( usart_PeriphConf[ usartId ].PeriphReg,
                                  oversamplingMode );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            oversamplingReg = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].PeriphReg );

            if( oversamplingMode == oversamplingReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns configured over-sampling mode for required USART/UART bus
 *
 * \param usartId           [in]: USART/UART bus identification.
 * \param oversamplingMode [out]: Configured over-sampling mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Oversampling( usart_PeriphId_t usartId, usart_Oversampling_t * const oversamplingMode )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        *oversamplingMode = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].PeriphReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures half-duplex state for required USART/UART bus
 *
 * \param usartId         [in]: USART/UART bus identification.
 * \param halfDuplexState [in]: Required half-duplex state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_HalfDuplexState( usart_PeriphId_t usartId, usart_HalfDuplex_t halfDuplexState )
{
    usart_RequestState_t retValue           = USART_REQUEST_ERROR;
    uint32_t             halfDuplexStateReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_HALF_DUPLEX_INACTIVE != halfDuplexState )
        {
            LL_USART_EnableHalfDuplex( usart_PeriphConf[ usartId ].PeriphReg );
        }
        else
        {
            LL_USART_DisableHalfDuplex( usart_PeriphConf[ usartId ].PeriphReg );
        }

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            halfDuplexStateReg = LL_USART_IsEnabledHalfDuplex( usart_PeriphConf[ usartId ].PeriphReg );

            if( halfDuplexState == halfDuplexStateReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns half-duplex state for required USART/UART bus
 *
 * \param usartId          [in]: USART/UART bus identification.
 * \param halfDuplexState [out]: Half-duplex state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_HalfDuplexState( usart_PeriphId_t usartId, usart_HalfDuplex_t * const halfDuplexState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        uint32_t halfDuplexStateReg = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != halfDuplexStateReg )
        {
            *halfDuplexState = USART_HALF_DUPLEX_ACTIVE;
        }
        else
        {
            *halfDuplexState = USART_HALF_DUPLEX_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures and activates receive timeout feature.
 *
 * This feature gives the Receiver timeout value in terms of number of bits
 * during which there is no activity on the RX line. In standard mode, the RTOF
 * flag is set if, after the last received character, no new start bit is
 * detected for more than the RTO value.
 *
 * \param usartId        [in]: USART/UART bus identification.
 * \param timeoutBitsCnt [in]: Count of bits triggering receiver timeout flag
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutActive( usart_PeriphId_t usartId, usart_RxTimeout_t timeoutBitsCnt )
{
    usart_RequestState_t retValue     = USART_REQUEST_ERROR;
    uint32_t             regValue     = 0u;
    uint32_t             timeoutValue = 0u;

    if( ( USART_BUS_CNT            > usartId        ) &&
        ( USART_RX_TIMEOUT_MAX > timeoutBitsCnt )    )
    {
        LL_USART_EnableRxTimeout( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_SetRxTimeout( usart_PeriphConf[ usartId ].PeriphReg, timeoutBitsCnt );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue     = LL_USART_IsEnabledRxTimeout( usart_PeriphConf[ usartId ].PeriphReg );
            timeoutValue = LL_USART_GetRxTimeout( usart_PeriphConf[ usartId ].PeriphReg );

            if( ( 0u              != regValue     ) &&
                ( timeoutBitsCnt  == timeoutValue )    )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief De-activates receive timeout feature.
 *
 * This feature gives the Receiver timeout value in terms of number of bits
 * during which there is no activity on the RX line. In standard mode, the RTOF
 * flag is set if, after the last received character, no new start bit is
 * detected for more than the RTO value.
 *
 * \param usartId [in]: USART/UART bus identification.
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableRxTimeout( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledRxTimeout( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of receiver timeout feature
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param reqState [out]: Receiver timeout feature activation status
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxTimeoutState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        uint32_t regValue = LL_USART_IsEnabledRxTimeout( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures pins operation modes
 *
 * Transmit and receive pins can be independently configured to use normal or
 * inverted mode.
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param rxPinLevels [in]: RX pin operation mode
 * \param txPinLevels [in]: TX pin operation mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_PinLevels( usart_PeriphId_t usartId, usart_RxPinLevel_t rxPinLevels, usart_TxPinLevel_t txPinLevels )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetRXPinLevel( usart_PeriphConf[ usartId ].PeriphReg, rxPinLevels );
        LL_USART_SetTXPinLevel( usart_PeriphConf[ usartId ].PeriphReg, txPinLevels );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t rxPinLevelReg = LL_USART_GetRXPinLevel( usart_PeriphConf[ usartId ].PeriphReg );
            uint32_t txPinLevelReg = LL_USART_GetTXPinLevel( usart_PeriphConf[ usartId ].PeriphReg );

            if( ( rxPinLevels == rxPinLevelReg ) &&
                ( txPinLevels == txPinLevelReg )    )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Receives pins operation modes
 *
 * Transmit and receive pins can be independently configured to use normal or
 * inverted mode.
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param rxPinLevels [out]: RX pin operation mode
 * \param txPinLevels [out]: TX pin operation mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_PinLevels( usart_PeriphId_t usartId, usart_RxPinLevel_t * const rxPinLevels, usart_TxPinLevel_t * const txPinLevels )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        *rxPinLevels = LL_USART_GetRXPinLevel( usart_PeriphConf[ usartId ].PeriphReg );
        *txPinLevels = LL_USART_GetTXPinLevel( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Reads transmission register address
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param regAddr [out]: Address of transmission register
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxRegisterAddr( usart_PeriphId_t usartId, usart_RxRegAddr_t * const regAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != regAddr )    )
    {
        *regAddr = usart_PeriphConf[ usartId ].PeriphReg->TDR;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Reads reception register address
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param regAddr [out]: Address of reception register
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxRegisterAddr( usart_PeriphId_t usartId, usart_RxRegAddr_t * const regAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != regAddr )    )
    {
        *regAddr = usart_PeriphConf[ usartId ].PeriphReg->RDR;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Writes data to transmission register
 *
 * \param usartId [in]: USART/UART bus identification.
 * \param txData  [in]: Data to be transmitted
 */
void Usart_SendData( usart_PeriphId_t usartId, usart_TxData_t txData )
{
    usart_PeriphConf[ usartId ].PeriphReg->TDR = txData;
}


/**
 * \brief Reads data from reception register
 *
 * \param usartId [in]: USART/UART bus identification.
 * \return Received data value
 */
usart_RxData_t Usart_ReadData( usart_PeriphId_t usartId )
{
    return ( usart_PeriphConf[ usartId ].PeriphReg->RDR );
}


/**
 * \brief Activates global Interrupt Requests (IRQ) for USART/UART peripheral
 *        and configures Interrupt Service Routine (ISR).
 *
 * Activation of global interrupt request is necessary for active any of
 * peripheral interrupt triggers (like TxE, RxNE ...).
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_InterruptsActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState               = USART_REQUEST_ERROR;
    nvic_RequestState_t  nvicActivationState    = NVIC_STATE_ERROR;
    nvic_RequestState_t  nvicHandlerConfigState = NVIC_STATE_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicHandlerConfigState = Nvic_Set_PeriphIrq_Handler( usart_PeriphConf[ usartId ].PeriphNvic,
                                                             usart_PeriphConf[ usartId ].PeriphIsr );

        nvicActivationState = Nvic_Set_PeriphIrq_Active( usart_PeriphConf[ usartId ].PeriphNvic );

        if( ( NVIC_STATE_OK != nvicActivationState    ) ||
            ( NVIC_STATE_OK != nvicHandlerConfigState )    )
        {
            retState  = USART_REQUEST_ERROR;
        }
        else
        {
            retState  = USART_REQUEST_OK;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief De-activates global Interrupt Requests (IRQ) for USART/UART peripheral
 *        and configures Interrupt Service Routine (ISR).
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_InterruptsInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    nvic_RequestState_t  nvicState = NVIC_STATE_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicState = Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartId ].PeriphNvic );

        if( NVIC_STATE_OK != nvicState )
        {
            retState  = USART_REQUEST_ERROR;
        }
        else
        {
            retState  = USART_REQUEST_OK;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Configures Interrupt Requests (IRQ) configured priority.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \param irqPrio [in]: Configured priority
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IrqPriority( usart_PeriphId_t usartId, usart_IrqPrio_t irqPrio )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    nvic_RequestState_t      nvicState = NVIC_STATE_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicState = Nvic_Set_PeriphIrq_Prio( usart_PeriphConf[ usartId ].PeriphNvic, irqPrio );

        if( NVIC_STATE_OK != nvicState )
        {
            retState  = USART_REQUEST_ERROR;
        }
        else
        {
            retState  = USART_REQUEST_OK;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reads Interrupt Requests (IRQ) configured priority.
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param irqPrio [out]: Configured priority
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_IrqPriority( usart_PeriphId_t usartId, usart_IrqPrio_t * const irqPrio )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    nvic_RequestState_t      nvicState = NVIC_STATE_ERROR;

    if( ( USART_BUS_CNT       > usartId ) &&
        ( USART_NULL_PTR != irqPrio )    )
    {
        nvicState = Nvic_Get_PeriphIrq_Prio( usart_PeriphConf[ usartId ].PeriphNvic, irqPrio );

        if( NVIC_STATE_OK != nvicState )
        {
            retState  = USART_REQUEST_ERROR;
        }
        else
        {
            retState  = USART_REQUEST_OK;
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Activates USART/UART DMA transfer requests for data transmission
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For transmission is this trigger activated by HW, when transmit data register
 * is empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaTxRequestActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief De-activates USART/UART DMA transfer requests for data transmission
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For transmission is this trigger activated by HW, when transmit data register
 * is empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaTxRequestInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation status of USART/UART DMA transfer request of data transmission
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For transmission is this trigger activated by HW, when transmit data register
 * is empty.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation status of transfer request state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DmaTxReqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates USART/UART DMA transfer requests for data reception
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaRxRequestActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief De-activates USART/UART DMA transfer requests for data reception
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaRxRequestInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation status of USART/UART DMA RX trigger request
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation of transfer request state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DmaRxReqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief De-activates USART/UART DMA transfer requests for data reception
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Init_Dma( usart_PeriphId_t usartId, usart_DmaConfig_t * const dmaConfig )
{
    usart_RequestState_t   retValue                = USART_REQUEST_ERROR;
    gpdma_ConfigStruct_t   dmaPeriphConfig         = { 0u };
    gpdma_TransferConfig_t dmaTransferConfig[ 2u ] = { 0u };

    usart_RuntimeData[ usartId ].TxTransferList;

    if( USART_NULL_PTR != dmaConfig )
    {
        if( ( USART_DMA_CHANNEL_CNT > dmaConfig->TxDmaChannelId ) &&
            ( USART_DMA_PERIPH_CNT  > dmaConfig->TxDmaPeriphId  )    )
        {
            dmaTransferConfig[ 0u ].Direction                   = GPDMA_DIR_MEMORY_TO_PERIPH;
            dmaTransferConfig[ 0u ].EventMode                   = GPDMA_TRANSFER_EVENT_BLOCK;

            dmaTransferConfig[ 0u ].TriggerType                 = GPDMA_TRG_NOT_USED;
            dmaTransferConfig[ 0u ].TriggerSource               = 0u;
            dmaTransferConfig[ 0u ].TriggerMode                 = GPDMA_TRIGGER_BLOCK;

            dmaTransferConfig[ 0u ].RequestSource               = usart_PeriphConf[ usartId ].PeriphDmaTxReq;
            dmaTransferConfig[ 0u ].RequestMode                 = GPDMA_PERIPH_REQ_SINGLE;

            dmaTransferConfig[ 0u ].BlockSize                   = 0u; /* This value will be configured by user before transfer start */
            dmaTransferConfig[ 0u ].BlockRepetitionCount        = 0u;

            dmaTransferConfig[ 0u ].SourceAddr                  = 0u; /* This value will be configured by user before transfer start */
            dmaTransferConfig[ 0u ].SourceDataSize              = GPDMA_DATA_SIZE_8BITS;
            dmaTransferConfig[ 0u ].SourceBurstLength           = 1u;
            dmaTransferConfig[ 0u ].SourceAddrMode              = GPDMA_ADDR_INCREMENT;
            dmaTransferConfig[ 0u ].SourcePortId                = GPDMA_PORT_DEFAULT;
            dmaTransferConfig[ 0u ].SourceDataOp                = GPDMA_SRC_DATA_PRESERVE;
            dmaTransferConfig[ 0u ].SourceBlockOffset2D         = 0u;
            dmaTransferConfig[ 0u ].SourceRepBlockOffset2D      = 0u;

            dmaTransferConfig[ 0u ].DestinationAddr             = (gpdma_DstAddr_t)&usart_PeriphConf[ usartId ].PeriphReg->TDR;
            dmaTransferConfig[ 0u ].DestinationDataSize         = GPDMA_DATA_SIZE_8BITS;
            dmaTransferConfig[ 0u ].DestinationBurstLength      = 1u;
            dmaTransferConfig[ 0u ].DestinationAddrMode         = GPDMA_ADDR_STATIC;
            dmaTransferConfig[ 0u ].DestinationPortId           = GPDMA_PORT_DEFAULT;
            dmaTransferConfig[ 0u ].DestinationDataOp           = GPDMA_DEST_DATA_PRESERVE;
            dmaTransferConfig[ 0u ].DestinationBlockOffset2D    = 0u;
            dmaTransferConfig[ 0u ].DestinationRepBlockOffset2D = 0u;

            usart_RuntimeData[ usartId ].DmaTxPeriphId  = dmaConfig->TxDmaPeriphId;
            usart_RuntimeData[ usartId ].DmaTxChannelId = dmaConfig->TxDmaChannelId;

            /* DMA configuration for transmit channel */
            Gpdma_Get_DefaultConfig( &dmaPeriphConfig );

            dmaPeriphConfig.PeriphId             = (gpdma_PeriphId_t)dmaConfig->TxDmaPeriphId;
            dmaPeriphConfig.ChannelId            = (gpdma_ChannelId_t)dmaConfig->TxDmaChannelId;
            dmaPeriphConfig.ChannelPrio          = (gpdma_Priority_t)dmaConfig->TxDmaPriority;

            dmaPeriphConfig.TransferExecMode     = GPDMA_XFER_EXEC_CONTINUOUS;
            dmaPeriphConfig.TransferConfig       = dmaTransferConfig;
            dmaPeriphConfig.TransfersCount       = 1u;
            dmaPeriphConfig.XferListAccessMode   = GPDMA_TRANSFER_LIST_ACCESS_APPEND;
            dmaPeriphConfig.XferList             = &usart_RuntimeData[ usartId ].TxTransferList;
            dmaPeriphConfig.TransferLockState    = GPDMA_TRANSFER_LIST_UNLOCKED;

            dmaPeriphConfig.TransferCompleteIsr  = (gpdma_IsrCallback*)dmaConfig->TxTransferCompleteCallback;
            dmaPeriphConfig.HalfTransferIsr      = (gpdma_IsrCallback*)dmaConfig->TxHalfTransferCallback;
            dmaPeriphConfig.ErrorIsr             = (gpdma_IsrErrCallback*)dmaConfig->TxErrorCallback;

            Gpdma_Init( &dmaPeriphConfig );
        }
        else
        {
            /* DMA for transmit is not required */
        }


        if( ( USART_DMA_CHANNEL_CNT > dmaConfig->RxDmaChannelId ) &&
            ( USART_DMA_PERIPH_CNT  > dmaConfig->RxDmaPeriphId  )    )
        {
            dmaTransferConfig[ 1u ].Direction                   = GPDMA_DIR_PERIPH_TO_MEMORY;
            dmaTransferConfig[ 1u ].EventMode                   = GPDMA_TRANSFER_EVENT_BLOCK;

            dmaTransferConfig[ 1u ].TriggerType                 = GPDMA_TRG_NOT_USED;
            dmaTransferConfig[ 1u ].TriggerSource               = 0u;
            dmaTransferConfig[ 1u ].TriggerMode                 = GPDMA_TRIGGER_BLOCK;

            dmaTransferConfig[ 1u ].RequestSource               = usart_PeriphConf[ usartId ].PeriphDmaRxReq;
            dmaTransferConfig[ 1u ].RequestMode                 = GPDMA_PERIPH_REQ_SINGLE;

            dmaTransferConfig[ 1u ].BlockSize                   = 0u; /* This value will be configured by user before transfer start */
            dmaTransferConfig[ 1u ].BlockRepetitionCount        = 0u;

            dmaTransferConfig[ 1u ].SourceAddr                  = 0u; /* This value will be configured by user before transfer start */
            dmaTransferConfig[ 1u ].SourceDataSize              = GPDMA_DATA_SIZE_8BITS;
            dmaTransferConfig[ 1u ].SourceBurstLength           = 1u;
            dmaTransferConfig[ 1u ].SourceAddrMode              = GPDMA_ADDR_STATIC;
            dmaTransferConfig[ 1u ].SourcePortId                = GPDMA_PORT_DEFAULT;
            dmaTransferConfig[ 1u ].SourceDataOp                = GPDMA_SRC_DATA_PRESERVE;
            dmaTransferConfig[ 1u ].SourceBlockOffset2D         = 0u;
            dmaTransferConfig[ 1u ].SourceRepBlockOffset2D      = 0u;

            dmaTransferConfig[ 1u ].DestinationAddr             = (gpdma_DstAddr_t)&usart_PeriphConf[ usartId ].PeriphReg->RDR;
            dmaTransferConfig[ 1u ].DestinationDataSize         = GPDMA_DATA_SIZE_8BITS;
            dmaTransferConfig[ 1u ].DestinationBurstLength      = 1u;
            dmaTransferConfig[ 1u ].DestinationAddrMode         = GPDMA_ADDR_INCREMENT;
            dmaTransferConfig[ 1u ].DestinationPortId           = GPDMA_PORT_DEFAULT;
            dmaTransferConfig[ 1u ].DestinationDataOp           = GPDMA_DEST_DATA_PRESERVE;
            dmaTransferConfig[ 1u ].DestinationBlockOffset2D    = 0u;
            dmaTransferConfig[ 1u ].DestinationRepBlockOffset2D = 0u;

            usart_RuntimeData[ usartId ].DmaRxPeriphId  = dmaConfig->RxDmaPeriphId;
            usart_RuntimeData[ usartId ].DmaRxChannelId = dmaConfig->RxDmaChannelId;


            /* DMA configuration for transmit channel */
            Gpdma_Get_DefaultConfig( &dmaPeriphConfig );

            dmaPeriphConfig.PeriphId             = (gpdma_PeriphId_t)dmaConfig->TxDmaPeriphId;
            dmaPeriphConfig.ChannelId            = (gpdma_ChannelId_t)dmaConfig->TxDmaChannelId;
            dmaPeriphConfig.ChannelPrio          = (gpdma_Priority_t)dmaConfig->TxDmaPriority;

            dmaPeriphConfig.TransferExecMode     = GPDMA_XFER_EXEC_CONTINUOUS;
            dmaPeriphConfig.TransferConfig       = dmaTransferConfig;
            dmaPeriphConfig.TransfersCount       = 1u;
            dmaPeriphConfig.XferListAccessMode   = GPDMA_TRANSFER_LIST_ACCESS_APPEND;
            dmaPeriphConfig.XferList             = &usart_RuntimeData[ usartId ].TxTransferList;
            dmaPeriphConfig.TransferLockState    = GPDMA_TRANSFER_LIST_UNLOCKED;

            dmaPeriphConfig.TransferCompleteIsr  = (gpdma_IsrCallback*)dmaConfig->TxTransferCompleteCallback;
            dmaPeriphConfig.HalfTransferIsr      = (gpdma_IsrCallback*)dmaConfig->TxHalfTransferCallback;
            dmaPeriphConfig.ErrorIsr             = (gpdma_IsrErrCallback*)dmaConfig->TxErrorCallback;

            Gpdma_Init( &dmaPeriphConfig );
        }
        else
        {
            /* DMA for receive is not required */
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


usart_RequestState_t Usart_Set_DmaTxStart( usart_PeriphId_t usartId, usart_TxDataCnt_t bytesCnt, usart_TxData_t * const dataBuff )
{
    usart_RequestState_t retValue    = USART_REQUEST_ERROR;
    gpdma_RequestState_t dmaRetValue = GPDMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaRetValue = Gpdma_Set_BlockSize( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaTxPeriphId,
                                           (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaTxChannelId,
                                           (gpdma_BlockSize_t)bytesCnt );

        if( GPDMA_REQUEST_ERROR != dmaRetValue )
        {

            dmaRetValue = Gpdma_Set_SourceAddr( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaTxPeriphId,
                                                (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaTxChannelId,
                                                (gpdma_SrcAddr_t)*dataBuff );

            if( GPDMA_REQUEST_ERROR != dmaRetValue )
            {

                dmaRetValue = Gpdma_Set_ChannelActive( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaTxPeriphId,
                                                     (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaTxChannelId );

                if( GPDMA_REQUEST_ERROR != dmaRetValue )
                {
                    retValue = Usart_Set_DmaTxRequestActive( usartId );
                }
                else
                {
                    /* Activation of DMA channel was unsuccessful */
                    retValue = USART_REQUEST_ERROR;
                }
            }
            else
            {
                /* Configuration of memory address was unsuccessful */
                retValue = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* Configuration of data bytes was unsuccessful */
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


usart_RequestState_t Usart_Set_DmaTxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        gpdma_RequestState_t dmaRetValue = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaTxPeriphId,
                                                                      (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaTxChannelId );

        if( GPDMA_REQUEST_ERROR == dmaRetValue )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = Usart_Set_DmaTxRequestInactive( usartId );
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


usart_RequestState_t Usart_Get_DmaTxRemainingCnt( usart_PeriphId_t usartId, usart_TxDataCnt_t * const bytesCnt )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != bytesCnt )    )
    {
        gpdma_RequestState_t dmaRetValue = Gpdma_Get_BlockSize( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaTxPeriphId,
                                                                (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaTxChannelId,
                                                                (gpdma_BlockSize_t*)bytesCnt );
        if( GPDMA_REQUEST_ERROR == dmaRetValue )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}



usart_RequestState_t Usart_Set_DmaRxStart( usart_PeriphId_t usartId, usart_TxDataCnt_t bytesCnt, usart_TxData_t * const dataBuff )
{
    usart_RequestState_t retValue    = USART_REQUEST_ERROR;
    gpdma_RequestState_t dmaRetValue = GPDMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaRetValue = Gpdma_Set_BlockSize( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaRxPeriphId,
                                           (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaRxChannelId,
                                           (gpdma_BlockSize_t)bytesCnt );

        if( GPDMA_REQUEST_ERROR != dmaRetValue )
        {

            dmaRetValue = Gpdma_Set_DestinationAddr( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaRxPeriphId,
                                                     (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaRxChannelId,
                                                     (gpdma_DstAddr_t)dataBuff );

            if( GPDMA_REQUEST_ERROR != dmaRetValue )
            {

                dmaRetValue = Gpdma_Set_ChannelActive( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaRxPeriphId,
                                                       (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaRxChannelId );

                if( GPDMA_REQUEST_ERROR != dmaRetValue )
                {
                    retValue = Usart_Set_DmaRxRequestActive( usartId );
                }
                else
                {
                    /* Activation of DMA channel was unsuccessful */
                    retValue = USART_REQUEST_ERROR;
                }
            }
            else
            {
                /* Configuration of memory address was unsuccessful */
                retValue = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* Configuration of data bytes was unsuccessful */
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


usart_RequestState_t Usart_Set_DmaRxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        gpdma_RequestState_t dmaRetValue = Gpdma_Set_ChannelInactive( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaRxPeriphId,
                                                                      (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaRxChannelId );

        if( GPDMA_REQUEST_ERROR == dmaRetValue )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = Usart_Set_DmaRxRequestInactive( usartId );
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}



usart_RequestState_t Usart_Get_DmaRxRemainingCnt( usart_PeriphId_t usartId, usart_RxDataCnt_t * const bytesCnt )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != bytesCnt )    )
    {
        gpdma_RequestState_t dmaRetValue = Gpdma_Get_BlockSize( (gpdma_PeriphId_t)usart_RuntimeData[ usartId ].DmaRxPeriphId,
                                                                (gpdma_ChannelId_t)usart_RuntimeData[ usartId ].DmaRxChannelId,
                                                                (gpdma_BlockSize_t*)bytesCnt );
        if( GPDMA_REQUEST_ERROR == dmaRetValue )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates Receive register Not Empty (RXNE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ReceiveData8(  usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_EnableIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;

                Usart_Set_InterruptsActive( usartId );

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Receive register Not Empty (RXNE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIrqInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ReceiveData8(  usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_DisableIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of Receive register Not Empty (RXNE) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of RX Not Empty (RXNE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxNotEmptyIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures Receive register Not Empty (RXNE) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIsrCallback( usart_PeriphId_t usartId, usart_RxNeIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].RxNotEmptyIsr = callback;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}



/**
 * \brief Activates Transmit register Empty (TXE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;

                Usart_Set_InterruptsActive( usartId );

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Transmit register Empty (TXE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIrqInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of Transmit Register Empty (TXE) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of TX Empty (TXE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxEmptyIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures Transmit Register Empty (TXE) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIsrCallback( usart_PeriphId_t usartId, usart_TxeIrqCallback_t * const callback )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].TxEmptyIsr = callback;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}



/**
 * \brief Activates Transmission Complete (TC) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_TC( usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_EnableIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;

                Usart_Set_InterruptsActive( usartId );

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Transmission Complete (TC) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIrqInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of Transmit Complete (TC) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of TX Complete (TC) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxCompleteIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures Transmit Complete (TC) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIsrCallback( usart_PeriphId_t usartId, usart_TcIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].TransferCompleteIsr = callback;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates Idle detection (IDLE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_EnableIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;

                Usart_Set_InterruptsActive( usartId );

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Idle detection (IDLE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIrqInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of Idle detection (IDLE) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of Idle detection (IDLE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_IdleIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures Idle detection (IDLE) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIsrCallback( usart_PeriphId_t usartId, usart_IdleIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].IdleIsr = callback;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates Receive Timeout (RTO) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retValue = Usart_Set_InterruptsActive( usartId );

        if( USART_REQUEST_ERROR != retValue )
        {
            LL_USART_ClearFlag_RTO( usart_PeriphConf[ usartId ].PeriphReg );

            LL_USART_EnableIT_RTO( usart_PeriphConf[ usartId ].PeriphReg );
        }
        else
        {
            retValue = USART_REQUEST_ERROR;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Receive Timeout (RTO) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIrqInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_RTO( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RTO( usart_PeriphConf[ usartId ].PeriphReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of Receive Timeout (RTO) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state Receive Timeout (RTO) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxTimeoutIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_RTO( usart_PeriphConf[ usartId ].PeriphReg );

        if( 0u != regValue )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures Receive Timeout (RTO) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIsrCallback( usart_PeriphId_t usartId, usart_RxTimeoutIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].RxTimeoutIsr = callback;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Activates Error interrupt request.
 *
 * When set, Error Interrupt Enable Bit is enabling interrupt generation in case
 * of a framing error, overrun error or noise flag. Activation of the parity
 * error interrupt is handled together with other errors.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_ErrorIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_FE( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_ORE( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_NE( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_PE( usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_EnableIT_ERROR( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_EnableIT_PE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t regErrValue = LL_USART_IsEnabledIT_ERROR( usart_PeriphConf[ usartId ].PeriphReg );
            uint32_t regPEValue  = LL_USART_IsEnabledIT_PE( usart_PeriphConf[ usartId ].PeriphReg );

            if( ( 0u != regErrValue ) &&
                ( 0u != regPEValue  )    )
            {
                retValue = USART_REQUEST_OK;

                Usart_Set_InterruptsActive( usartId );

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Deactivates Error interrupt request.
 *
 * When set, Error Interrupt Enable Bit is enabling interrupt generation in case
 * of a framing error, overrun error or noise flag. De-activation of the parity
 * error interrupt is handled together with other errors.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_ErrorIrqInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_ERROR( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_DisableIT_PE( usart_PeriphConf[ usartId ].PeriphReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t regErrValue = LL_USART_IsEnabledIT_ERROR( usart_PeriphConf[ usartId ].PeriphReg );
            uint32_t regPEValue  = LL_USART_IsEnabledIT_PE( usart_PeriphConf[ usartId ].PeriphReg );

            if( ( 0u == regErrValue ) &&
                ( 0u == regPEValue  )    )
            {
                retValue = USART_REQUEST_OK;

                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
            }
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Returns activation state of Error interrupt request.
 *
 * \note  When set, Error Interrupt Enable Bit is enabling interrupt generation
 *        in case of a framing error, overrun error or noise flag. Activation
 *        and de-activation of the parity error interrupt is handled together
 *        with other errors.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of TX Empty (TXE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_ErrorIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regErrValue = LL_USART_IsEnabledIT_ERROR( usart_PeriphConf[ usartId ].PeriphReg );
        uint32_t regPEValue  = LL_USART_IsEnabledIT_PE( usart_PeriphConf[ usartId ].PeriphReg );

        if( ( 0u != regErrValue ) &&
            ( 0u != regPEValue  )    )
        {
            *reqState = USART_FLAG_ACTIVE;
        }
        else
        {
            *reqState = USART_FLAG_INACTIVE;
        }

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures error interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_ErrorIsrCallback( usart_PeriphId_t usartId, usart_ErrIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].ErrorIsr = callback;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Initializes GPIO RX pin used by peripheral
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitRxGpio( usart_RxPin_t pinId )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        pinConfig;

    pinConfig.PortId         = USART_BIT_MASK_DECODE_PORT( pinId );
    pinConfig.PinId          = USART_BIT_MASK_DECODE_PIN( pinId );
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = USART_BIT_MASK_DECODE_AF( pinId );

    /* Initialize GPIO */
    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Initializes GPIO TX pin used by peripheral
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitTxGpio( usart_TxPin_t pinId )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        pinConfig;

    pinConfig.PortId         = USART_BIT_MASK_DECODE_PORT( pinId );
    pinConfig.PinId          = USART_BIT_MASK_DECODE_PIN( pinId );
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = USART_BIT_MASK_DECODE_AF( pinId );

    /* Initialize GPIO */
    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Initializes GPIO Driver Enable (DE) pin used by peripheral
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitDeGpio( usart_DePin_t pinId )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        pinConfig;

    pinConfig.PortId         = USART_BIT_MASK_DECODE_PORT( pinId );
    pinConfig.PinId          = USART_BIT_MASK_DECODE_PIN( pinId );
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = USART_BIT_MASK_DECODE_AF( pinId );

    /* Initialize GPIO */
    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Sets the prescaler value for the required USART/UART bus
 *
 * \param usartId   [in]: USART/UART bus identification
 * \param prescaler [in]: Value of prescaler
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static usart_RequestState_t Usart_Set_Prescaler( usart_PeriphId_t usartId, usart_Prescaler_t prescaler )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;
    uint32_t             prescReg = 0u;

    if( ( USART_BUS_CNT       > usartId   ) &&
        ( USART_PRESCALER_CNT > prescaler )    )
    {
        LL_USART_SetPrescaler( usart_PeriphConf[ usartId ].PeriphReg,
                               prescaler );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            prescReg = LL_USART_GetPrescaler( usart_PeriphConf[ usartId ].PeriphReg );

            if( prescaler == prescReg )
            {
                retState = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
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
 * \brief Reads the value of prescaler from required USART/UART bus
 *
 * \param usartId    [in]: USART/UART bus identification
 * \param prescaler [out]: Value of prescaler
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static usart_RequestState_t Usart_Get_Prescaler( usart_PeriphId_t usartId, usart_Prescaler_t *prescaler )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId   ) &&
        ( USART_NULL_PTR != prescaler )    )
    {
        *prescaler = LL_USART_GetPrescaler( usart_PeriphConf[ usartId ].PeriphReg );

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Calculate expected prescaler value for USART/UART peripheral
 *
 * \param periphClock  [in]: Peripheral clock in Hz
 * \param oversampling [in]: Peripheral over-sampling configuration
 * \param baudrate     [in]: Required peripheral baud-rate
 * \param prescaler   [out]: Calculated prescaler value
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static usart_RequestState_t Usart_Get_ExpectedPrescaler( usart_PeriphId_t usartId,
                                                         usart_FreqHz_t periphClock,
                                                         usart_Oversampling_t oversampling,
                                                         usart_Baudrate_t baudrate,
                                                         usart_Prescaler_t *prescaler )
{
    uint32_t             prescValue         = 0u;
    uint32_t             usartPeriphClk     = 0;
    usart_RequestState_t retState           = USART_REQUEST_ERROR;
    usart_Prescaler_t    lowerPrescId       = USART_PRESCALER_1;
    usart_Prescaler_t    higherPrescId      = USART_PRESCALER_1;
    uint32_t             dividerLower       = 0u;
    uint32_t             dividerHigher      = 0u;
    uint32_t             baudrateLower      = 0u;
    uint32_t             baudrateHigher     = 0u;
    uint32_t             baudrateDiffLower  = 0u;
    uint32_t             baudrateDiffHigher = 0u;

    if( USART_NULL_PTR != prescaler )
    {
        rcc_RequestState_t rccRequestState = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );

        if( RCC_REQUEST_ERROR != rccRequestState )
        {
            if( USART_OVERSAMPLING_8 == oversampling )
            {
                prescValue = periphClock / ( baudrate * 8u );
            }
            else
            {
                prescValue = periphClock / ( baudrate * 16u );
            }

            /* Find nearest possible value for prescaler */
            for( uint32_t prescIndex = 0u; USART_PRESCALER_CNT > prescIndex; prescIndex++ )
            {
                if( USART_PRESCALER_TAB[ prescIndex ] > prescValue )
                {
                    higherPrescId = prescIndex;
                    lowerPrescId  = prescIndex - 1u;
                    break;
                }
            }


            if( USART_OVERSAMPLING_8 == oversampling )
            {
                dividerLower = __LL_USART_DIV_SAMPLING8( usartPeriphClk,
                                                         lowerPrescId,
                                                         baudrate );

                dividerHigher = __LL_USART_DIV_SAMPLING8( usartPeriphClk,
                                                          higherPrescId,
                                                          baudrate );
            }
            else
            {
                dividerLower = __LL_USART_DIV_SAMPLING16( usartPeriphClk,
                                                          lowerPrescId,
                                                          baudrate );

                dividerHigher = __LL_USART_DIV_SAMPLING16( usartPeriphClk,
                                                           higherPrescId,
                                                           baudrate );

                baudrateLower = usartPeriphClk / ( ( USART_PRESCALER_TAB[ lowerPrescId ] + 1u ) * dividerLower );

                baudrateHigher = usartPeriphClk / ( ( USART_PRESCALER_TAB[ higherPrescId ] + 1u ) * dividerHigher );
            }


            if( baudrate > baudrateLower )
            {
                baudrateDiffLower = baudrate - baudrateLower;
            }
            else
            {
                baudrateDiffLower = baudrateLower - baudrate;
            }


            if( baudrate > baudrateHigher )
            {
                baudrateDiffHigher = baudrate - baudrateHigher;
            }
            else
            {
                baudrateDiffHigher = baudrateHigher - baudrate;
            }


            if( baudrateDiffHigher > baudrateDiffLower )
            {
                *prescaler = lowerPrescId;
            }
            else
            {
                *prescaler = higherPrescId;
            }

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

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief Global interrupt handler.
 *
 * \param usartId [in]: USART/UART bus identification
 */
inline void Usart_GlobalIsrHandler( usart_PeriphId_t usartId )
{
    /*----------------------------- Error interrupt --------------------------*/
    if( ( 0u != LL_USART_IsActiveFlag_PE( usart_PeriphConf[ usartId ].PeriphReg )  ) ||
        ( 0u != LL_USART_IsActiveFlag_FE( usart_PeriphConf[ usartId ].PeriphReg )  ) ||
        ( 0u != LL_USART_IsActiveFlag_NE( usart_PeriphConf[ usartId ].PeriphReg )  ) ||
        ( 0u != LL_USART_IsActiveFlag_ORE( usart_PeriphConf[ usartId ].PeriphReg ) )    )
    {
        /* Bit 3 ORE: Overrun error
         * This bit is set by hardware when the data currently being received in the shift register is
         * ready to be transferred into the USART_RDR register while RXNE = 1. It is cleared by a
         * software, writing 1 to the ORECF, in the USART_ICR register.
         * An interrupt is generated if RXNEIE = 1 or EIE = 1 in the USART_CR1 register.
         * 0: No overrun error
         * 1: Overrun error is detected
         * Note: When this bit is set, the USART_RDR register content is not lost but the shift register is
         * overwritten. An interrupt is generated if the ORE flag is set during multi buffer
         * communication if the EIE bit is set.
         * This bit is permanently forced to 0 (no overrun detection) when the bit OVRDIS is set in
         * the USART_CR3 register.
         *
         * Bit 2 NE: Noise detection flag
         * This bit is set by hardware when noise is detected on a received frame. It is cleared by
         * software, writing 1 to the NECF bit in the USART_ICR register.
         * 0: No noise is detected
         * 1: Noise is detected
         * Note: This bit does not generate an interrupt as it appears at the same time as the RXNE bit
         * which itself generates an interrupt. An interrupt is generated when the NE flag is set
         * during multi buffer communication if the EIE bit is set.
         * When the line is noise-free, the NE flag can be disabled by programming the ONEBIT
         * bit to 1 to increase the USART tolerance to deviations (Refer to Section 50.5.8:
         * Tolerance of the USART receiver to clock deviation on page 1731).
         *
         * Bit 1 FE: Framing error
         * This bit is set by hardware when a de-synchronization, excessive noise or a break character
         * is detected. It is cleared by software, writing 1 to the FECF bit in the USART_ICR register.
         * When transmitting data in Smartcard mode, this bit is set when the maximum number of
         * transmit attempts is reached without success (the card NACKs the data frame).
         * An interrupt is generated if EIE = 1 in the USART_CR1 register.
         * 0: No Framing error is detected
         * 1: Framing error or break character is detected
         *
         * Bit 0 PE: Parity error
         * This bit is set by hardware when a parity error occurs in receiver mode. It is cleared by
         * software, writing 1 to the PECF in the USART_ICR register.
         * An interrupt is generated if PEIE = 1 in the USART_CR1 register.
         * 0: No parity error
         * 1: Parity error */
        LL_USART_ReceiveData9( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_PE( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_FE( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_NE( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_ClearFlag_ORE( usart_PeriphConf[ usartId ].PeriphReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].ErrorIsr )
        {
            usart_RuntimeData[ usartId ].ErrorIsr( (uint16_t) usart_PeriphConf[ usartId ].PeriphReg->ISR & USART_ISR_ERROR_MASK );
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*------------------ Receive timeout (RTO) interrupt ---------------------*/
    if( 0u != LL_USART_IsActiveFlag_RTO( usart_PeriphConf[ usartId ].PeriphReg ) )
    {
        /* Bit 11 RTOF: Receiver timeout
         * This bit is set by hardware when the timeout value, programmed in the RTOR register has
         * lapsed, without any communication. It is cleared by software, writing 1 to the RTOCF bit in
         * the USART_ICR register.
         * An interrupt is generated if RTOIE = 1 in the USART_CR2 register.
         * In Smartcard mode, the timeout corresponds to the CWT or BWT timings.
         * 0: Timeout value not reached
         * 1: Timeout value reached without any data reception
         * Note: If a time equal to the value programmed in RTOR register separates 2 characters,
         * RTOF is not set. If this time exceeds this value + 2 sample times (2/16 or 2/8,
         * depending on the oversampling method), RTOF flag is set.
         * The counter counts even if RE = 0 but RTOF is set only when RE = 1. If the timeout has
         * already elapsed when RE is set, then RTOF is set.
         * If the USART does not support the Receiver timeout feature, this bit is reserved and
         * kept at reset value. */
        LL_USART_ClearFlag_RTO( usart_PeriphConf[ usartId ].PeriphReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].RxTimeoutIsr )
        {
            usart_RuntimeData[ usartId ].RxTimeoutIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*----------------------- IDLE Line detected interrupt -------------------*/
    if( 0u != LL_USART_IsActiveFlag_IDLE( usart_PeriphConf[ usartId ].PeriphReg ) )
    {
        /* Bit 4 IDLE: Idle line detected
         * This bit is set by hardware when an Idle Line is detected. An interrupt is generated if
         * IDLEIE = 1 in the USART_CR1 register. It is cleared by software, writing 1 to the IDLECF in
         * the USART_ICR register.
         * 0: No Idle line is detected
         * 1: Idle line is detected
         * Note: The IDLE bit is not set again until the RXNE bit has been set (i.e. a new idle line
         * occurs).
         * If Mute mode is enabled (MME = 1), IDLE is set if the USART is not mute (RWU = 0),
         * whatever the Mute mode selected by the WAKE bit. If RWU = 1, IDLE is not set. */
        LL_USART_ClearFlag_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].IdleIsr )
        {
            usart_RuntimeData[ usartId ].IdleIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }

    }

    /*-------------- Receiver buffer Not Empty (RXNE) interrupt --------------*/
    if( 0u != LL_USART_IsActiveFlag_RXNE( usart_PeriphConf[ usartId ].PeriphReg ) )
    {
        /* Bit 5 RXNE: Read data register not empty
         * RXNE bit is set by hardware when the content of the USART_RDR shift register has been
         * transferred to the USART_RDR register. It is cleared by reading from the USART_RDR
         * register. The RXNE flag can also be cleared by writing 1 to the RXFRQ in the USART_RQR
         * register.
         * An interrupt is generated if RXNEIE = 1 in the USART_CR1 register.
         * 0: Data is not received
         * 1: Received data is ready to be read. */
        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].RxNotEmptyIsr )
        {
            usart_RuntimeData[ usartId ].RxNotEmptyIsr( LL_USART_ReceiveData9( usart_PeriphConf[ usartId ].PeriphReg ) );
        }
    }

    /*-------------- Transmission buffer Empty (TXE) interrupt ---------------*/
    if( 0u != LL_USART_IsActiveFlag_TXE( usart_PeriphConf[ usartId ].PeriphReg ) )
    {
        /* Bit 7 TXE: Transmit data register empty
         * TXE is set by hardware when the content of the USART_TDR register has been transferred
         * into the shift register. It is cleared by writing to the USART_TDR register. The TXE flag can
         * also be set by writing 1 to the TXFRQ in the USART_RQR register, in order to discard the
         * data (only in Smartcard T = 0 mode, in case of transmission failure).
         * An interrupt is generated if the TXEIE bit = 1 in the USART_CR1 register.
         * 0: Data register full
         * 1: Data register not full */
        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].TxEmptyIsr )
        {
            usart_RuntimeData[ usartId ].TxEmptyIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*---------------- Transmission Complete (TC) interrupt ------------------*/
    if( 0u != LL_USART_IsActiveFlag_TC( usart_PeriphConf[ usartId ].PeriphReg ) )
    {
        /* Bit 6 TC: Transmission complete
         * This bit indicates that the last data written in the USART_TDR has been transmitted out of
         * the shift register.
         * It is set by hardware when the transmission of a frame containing data is complete and
         * when TXE is set.
         * An interrupt is generated if TCIE = 1 in the USART_CR1 register.
         * TC bit is is cleared by software, by writing 1 to the TCCF in the USART_ICR register or by a
         * write to the USART_TDR register.
         * 0: Transmission is not complete
         * 1: Transmission is complete
         * Note: If TE bit is reset and no transmission is on going, the TC bit is set immediately. */
        LL_USART_ClearFlag_TC( usart_PeriphConf[ usartId ].PeriphReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].TransferCompleteIsr )
        {
            usart_RuntimeData[ usartId ].TransferCompleteIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }
}


#ifdef USART1
/**
 * \brief USART1 Interrupt handler
 */
void Usart_Usart1_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_1 );
}
#endif /* USART1 */

#ifdef USART2
/**
 * \brief USART2 Interrupt handler
 */
void Usart_Usart2_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_2 );
}
#endif /* USART2 */

#ifdef USART3
/**
 * \brief USART3 Interrupt handler
 */
void Usart_Usart3_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_3 );
}
#endif /* USART3 */

#ifdef UART4
/**
 * \brief UART4 Interrupt handler
 */
void Uart_Usart4_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_4 );
}
#endif /* UART4 */

#ifdef UART5
/**
 * \brief UART5 Interrupt handler
 */
void Uart_Usart5_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_5 );
}
#endif /* UART5 */

#ifdef USART6
/**
 * \brief UART5 Interrupt handler
 */
void Usart_Usart6_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_6 );
}
#endif /* UART5 */

/* ================================ TASKS =================================== */

