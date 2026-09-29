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
#include "Usart_Dma.h"                      /* DMA data transfer handler      */
#include "Usart_Isr.h"                      /* ISR data transfer handler      */
#include "Usart_Poll.h"                     /* Polling data transfer handler  */
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
static void Usart_Uart4_IsrHandler(void);
#endif /* UART4 */
#ifdef UART5
static void Usart_Uart5_IsrHandler(void);
#endif /* UART5 */
#ifdef USART6
static void Usart_Usart6_IsrHandler(void);
#endif /* USART6 */
#ifdef UART7
static void Usart_Uart7_IsrHandler(void);
#endif /* UART7 */
#ifdef UART8
static void Usart_Uart8_IsrHandler(void);
#endif /* UART8 */
#ifdef UART9
static void Usart_Uart9_IsrHandler(void);
#endif /* UART9 */
#ifdef USART10
static void Usart_Usart10_IsrHandler(void);
#endif /* USART10 */
#ifdef USART11
static void Usart_Usart11_IsrHandler(void);
#endif /* USART11 */
#ifdef UART12
static void Usart_Uart12_IsrHandler(void);
#endif /* UART12 */

static inline void Usart_GlobalIsrHandler( usart_PeriphId_t usartId );

static usart_RequestState_t Usart_Set_Prescaler( usart_PeriphId_t usartId, usart_Prescaler_t prescaler );
static usart_RequestState_t Usart_Get_Prescaler( usart_PeriphId_t usartId, usart_Prescaler_t *prescaler );
static usart_RequestState_t Usart_Get_ExpectedPrescaler( usart_PeriphId_t usartId,
                                                         usart_FreqHz_t periphClock,
                                                         usart_Oversampling_t oversampling,
                                                         usart_Baudrate_t baudrate,
                                                         usart_Prescaler_t *prescaler );

static usart_RequestState_t Usart_Check_DataConfig   ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
static usart_RequestState_t Usart_Set_XferInit       ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
static usart_RequestState_t Usart_Set_XferDeinit     ( usart_PeriphId_t usartId );
static usart_FunctionState_t Usart_Get_IrqUsed       ( const usart_DataConfig_t * const dataConfig );

static usart_RequestState_t Usart_None_Check_Config  ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
static usart_RequestState_t Usart_None_XferInit      ( usart_PeriphId_t usartId );
static usart_RequestState_t Usart_None_XferStart     ( usart_PeriphId_t usartId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define USART_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define USART_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define USART_PATCH_VERSION           ( 0u )

/** Default baud-rate used by \ref Usart_Get_DefaultConfig */
#define USART_DEFAULT_BAUDRATE        ( 115200u )

/** Receive buffer half is reached after RxBufferSize / USART_BUFFER_HALF_DIVIDER bytes */
#define USART_BUFFER_HALF_DIVIDER     ( 2u )

/** Samples per bit with 8x oversampling */
#define USART_OVERSAMPLING_8_FACTOR   ( 8u )

/** Samples per bit with 16x oversampling */
#define USART_OVERSAMPLING_16_FACTOR  ( 16u )

/* =============================== MACROS =================================== */

/** Bit-mask of all possible errors in ISR register */
#define USART_ISR_ERROR_MASK          ( ( LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_NE | LL_USART_ISR_ORE ) )

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** USART/UART peripherals configuration array */
static usart_PeriphConfigStruct_t const         usart_PeriphConf[ ] =
{
#ifdef USART1
    { .PeriphReg = USART1 , .PeriphRcc = RCC_PERIPH_USART1_PCLK2 , .PeriphNvic = NVIC_PERIPH_IRQ_USART1 , .PeriphIsr = Usart_Usart1_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_USART1_TX , .PeriphDmaRxReq = GPDMA_REQ_USART1_RX  },
#endif
#ifdef USART2
    { .PeriphReg = USART2 , .PeriphRcc = RCC_PERIPH_USART2_PCLK1 , .PeriphNvic = NVIC_PERIPH_IRQ_USART2 , .PeriphIsr = Usart_Usart2_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_USART2_TX , .PeriphDmaRxReq = GPDMA_REQ_USART2_RX  },
#endif
#ifdef USART3
    { .PeriphReg = USART3 , .PeriphRcc = RCC_PERIPH_USART3_PCLK1 , .PeriphNvic = NVIC_PERIPH_IRQ_USART3 , .PeriphIsr = Usart_Usart3_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_USART3_TX , .PeriphDmaRxReq = GPDMA_REQ_USART3_RX  },
#endif
#ifdef UART4
    { .PeriphReg = UART4,   .PeriphRcc = RCC_PERIPH_UART4_PCLK1  , .PeriphNvic = NVIC_PERIPH_IRQ_UART4  , .PeriphIsr = Usart_Uart4_IsrHandler  , .PeriphDmaTxReq = GPDMA_REQ_UART4_TX  , .PeriphDmaRxReq = GPDMA_REQ_UART4_RX   },
#endif
#ifdef UART5
    { .PeriphReg = UART5,   .PeriphRcc = RCC_PERIPH_UART5_PCLK1  , .PeriphNvic = NVIC_PERIPH_IRQ_UART5  , .PeriphIsr = Usart_Uart5_IsrHandler  , .PeriphDmaTxReq = GPDMA_REQ_UART5_TX  , .PeriphDmaRxReq = GPDMA_REQ_UART5_RX   },
#endif
#ifdef USART6
    { .PeriphReg = USART6 , .PeriphRcc = RCC_PERIPH_USART6_PCLK1 , .PeriphNvic = NVIC_PERIPH_IRQ_USART6 , .PeriphIsr = Usart_Usart6_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_USART6_TX , .PeriphDmaRxReq = GPDMA_REQ_USART6_RX  },
#endif
#ifdef UART7
    { .PeriphReg = UART7  , .PeriphRcc = RCC_PERIPH_UART7_PCLK1  , .PeriphNvic = NVIC_PERIPH_IRQ_UART7  , .PeriphIsr = Usart_Uart7_IsrHandler  , .PeriphDmaTxReq = GPDMA_REQ_UART7_TX  , .PeriphDmaRxReq = GPDMA_REQ_UART7_RX   },
#endif
#ifdef UART8
    { .PeriphReg = UART8  , .PeriphRcc = RCC_PERIPH_UART8_PCLK1  , .PeriphNvic = NVIC_PERIPH_IRQ_UART8  , .PeriphIsr = Usart_Uart8_IsrHandler  , .PeriphDmaTxReq = GPDMA_REQ_UART8_TX  , .PeriphDmaRxReq = GPDMA_REQ_UART8_RX   },
#endif
#ifdef UART9
    { .PeriphReg = UART9  , .PeriphRcc = RCC_PERIPH_UART9_PCLK1  , .PeriphNvic = NVIC_PERIPH_IRQ_UART9  , .PeriphIsr = Usart_Uart9_IsrHandler  , .PeriphDmaTxReq = GPDMA_REQ_UART9_TX  , .PeriphDmaRxReq = GPDMA_REQ_UART9_RX   },
#endif
#ifdef USART10
    { .PeriphReg = USART10, .PeriphRcc = RCC_PERIPH_USART10_PCLK1, .PeriphNvic = NVIC_PERIPH_IRQ_USART10, .PeriphIsr = Usart_Usart10_IsrHandler, .PeriphDmaTxReq = GPDMA_REQ_USART10_TX, .PeriphDmaRxReq = GPDMA_REQ_USART10_RX },
#endif
#ifdef USART11
    { .PeriphReg = USART11, .PeriphRcc = RCC_PERIPH_USART11_PCLK1, .PeriphNvic = NVIC_PERIPH_IRQ_USART11, .PeriphIsr = Usart_Usart11_IsrHandler, .PeriphDmaTxReq = GPDMA_REQ_USART11_TX, .PeriphDmaRxReq = GPDMA_REQ_USART11_RX },
#endif
#ifdef UART12
    { .PeriphReg = UART12 , .PeriphRcc = RCC_PERIPH_UART12_PCLK1 , .PeriphNvic = NVIC_PERIPH_IRQ_UART12 , .PeriphIsr = Usart_Uart12_IsrHandler , .PeriphDmaTxReq = GPDMA_REQ_UART12_TX , .PeriphDmaRxReq = GPDMA_REQ_UART12_RX  },
#endif
};

_Static_assert( USART_BUS_CNT == ( (sizeof(usart_PeriphConf) / sizeof(usart_PeriphConfigStruct_t) ) ), "Usart: size of usart_PeriphConf is incorrect.");


/** \brief Data handling runtime context per peripheral */
static usart_XferContext_t usart_XferContext[ USART_BUS_CNT ];


/** \brief usart_XferMode_t -> transmission mode handler */
static const usart_XferModeIf_t usart_TxModeLut[ USART_XFER_MODE_CNT ] =
{
    [USART_XFER_MODE_NONE] = { .CheckConfig = Usart_None_Check_Config,  .Init = Usart_None_XferInit,  .Deinit = Usart_None_XferInit,   .Start = Usart_None_XferStart, .Stop = Usart_None_XferInit },
    [USART_XFER_MODE_DMA]  = { .CheckConfig = Usart_Dma_Check_TxConfig, .Init = Usart_Dma_TxInit,     .Deinit = Usart_Dma_TxDeinit,    .Start = Usart_Dma_TxStart,    .Stop = Usart_Dma_TxStop    },
    [USART_XFER_MODE_ISR]  = { .CheckConfig = Usart_Isr_Check_Config,   .Init = Usart_Isr_XferInit,   .Deinit = Usart_Isr_XferDeinit,  .Start = Usart_Isr_TxStart,    .Stop = Usart_Isr_TxStop    },
    [USART_XFER_MODE_POLL] = { .CheckConfig = Usart_Poll_Check_Config,  .Init = Usart_Poll_XferInit,  .Deinit = Usart_Poll_XferDeinit, .Start = Usart_Poll_TxStart,   .Stop = Usart_Poll_TxStop   },
};


/** \brief usart_XferMode_t -> reception mode handler */
static const usart_XferModeIf_t usart_RxModeLut[ USART_XFER_MODE_CNT ] =
{
    [USART_XFER_MODE_NONE] = { .CheckConfig = Usart_None_Check_Config,  .Init = Usart_None_XferInit,  .Deinit = Usart_None_XferInit,   .Start = Usart_None_XferStart, .Stop = Usart_None_XferInit },
    [USART_XFER_MODE_DMA]  = { .CheckConfig = Usart_Dma_Check_RxConfig, .Init = Usart_Dma_RxInit,     .Deinit = Usart_Dma_RxDeinit,    .Start = Usart_Dma_RxStart,    .Stop = Usart_Dma_RxStop    },
    [USART_XFER_MODE_ISR]  = { .CheckConfig = Usart_Isr_Check_Config,   .Init = Usart_Isr_XferInit,   .Deinit = Usart_Isr_XferDeinit,  .Start = Usart_Isr_RxStart,    .Stop = Usart_Isr_RxStop    },
    [USART_XFER_MODE_POLL] = { .CheckConfig = Usart_Poll_Check_Config,  .Init = Usart_Poll_XferInit,  .Deinit = Usart_Poll_XferDeinit, .Start = Usart_Poll_RxStart,   .Stop = Usart_Poll_RxStop   },
};


/** \brief usart_XferErrorId_t -> reception error flag in USART ISR register (DMA errors have no flag) */
static const usart_Error_t usart_RxErrorFlagLut[ USART_XFER_ERROR_CNT ] =
{
    [USART_XFER_ERROR_PARITY]              = USART_ERROR_PARITY_ERROR,
    [USART_XFER_ERROR_FRAMING]             = USART_ERROR_FRAMING_ERROR,
    [USART_XFER_ERROR_NOISE]               = USART_ERROR_NOISE_DETECTED,
    [USART_XFER_ERROR_OVERRUN]             = USART_ERROR_OVERRUN,
    [USART_XFER_ERROR_DMA_TRANSFER]        = USART_ERROR_NONE,
    [USART_XFER_ERROR_DMA_CONFIG]          = USART_ERROR_NONE,
    [USART_XFER_ERROR_DMA_CONFIG_UPDATE]   = USART_ERROR_NONE,
    [USART_XFER_ERROR_DMA_TRIGGER_OVERRUN] = USART_ERROR_NONE,
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
 * Data handling of a previous initialization is released, the peripheral is reset and
 * configured, enabled and the data handling is initialized (if DataConfig is set).
 *
 * \param usartConfig [in]: Pointer to configuration structure. Must not be NULL.
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Init( usart_BusConfig_t * const usartConfig )
{
    usart_RequestState_t retState = USART_REQUEST_OK;

    if( ( USART_NULL_PTR != usartConfig           ) &&
        ( USART_BUS_CNT   > usartConfig->PeriphId )    )
    {
        rcc_FunctionState_t rccActivationState = RCC_FUNCTION_INACTIVE;
        rcc_RequestState_t  rccRequestState    = RCC_REQUEST_ERROR;

        /*------------- Data handling of previous initialization -------------*/
        retState = Usart_Set_XferDeinit( usartConfig->PeriphId );

        /*------------- USART peripheral clock activation section ------------*/
        rccRequestState = Rcc_Get_PeriphState( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc, &rccActivationState );

        if( ( RCC_REQUEST_ERROR     != rccRequestState    ) &&
            ( RCC_FUNCTION_INACTIVE == rccActivationState )    )
        {
            rccRequestState = Rcc_Set_PeriphActive( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc );

            if( RCC_REQUEST_ERROR == rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
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

        if( USART_REQUEST_ERROR != retState )
        {
            rccRequestState = Rcc_Set_ResetActive( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc );
            if( RCC_REQUEST_OK != rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
            }
        }

        if( USART_REQUEST_ERROR != retState )
        {
            rccRequestState = Rcc_Set_ResetInactive( usart_PeriphConf[ usartConfig->PeriphId ].PeriphRcc );
            if( RCC_REQUEST_OK != rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
            }
        }

        if( USART_REQUEST_ERROR != retState )
        {
            /* Over-sampling configuration must be executed before baud-rate configuration */
            retState = Usart_Set_Oversampling( usartConfig->PeriphId, usartConfig->Oversampling );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_Baudrate( usartConfig->PeriphId, usartConfig->BaudRate );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_DataWidth( usartConfig->PeriphId, usartConfig->DataWidth );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_StopBits( usartConfig->PeriphId, usartConfig->StopBits );

        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_Parity( usartConfig->PeriphId, usartConfig->Parity );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_TransferMode( usartConfig->PeriphId, usartConfig->TransferMode );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_FlowControl( usartConfig->PeriphId, usartConfig->HwFlowControl );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_HalfDuplexState( usartConfig->PeriphId, usartConfig->HalfDuplex );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_DriverEnableState( usartConfig->PeriphId, usartConfig->DriverEnableMode );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_DriverEnablePolarity( usartConfig->PeriphId, usartConfig->DriverEnablePolarity );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_PinLevels( usartConfig->PeriphId, usartConfig->RxPinOperationLevels, usartConfig->TxPinOperationLevels );
        }


        if( USART_REQUEST_ERROR != retState )
        {
            if( USART_RX_TIMEOUT_MIN < usartConfig->RxTimeoutValue )
            {
                retState = Usart_Set_RxTimeoutActive( usartConfig->PeriphId, usartConfig->RxTimeoutValue );
            }
            else
            {
                retState = Usart_Set_RxTimeoutInactive( usartConfig->PeriphId );
            }
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_PeriphActive( usartConfig->PeriphId );
        }
        else
        {
            /* Error during initialization process */
        }

        /*------------------ Data handling initialization ------------------*/
        if( ( USART_REQUEST_ERROR != retState                ) &&
            ( USART_NULL_PTR      != usartConfig->DataConfig )    )
        {
            retState = Usart_Set_DataConfig( usartConfig->PeriphId, usartConfig->DataConfig );
        }
        else
        {
            /* Error during initialization process or data handling is not used */
        }
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
 * Stops the data handling and releases its resources (GPDMA channels, USART interrupt), disables
 * the peripheral interrupt in NVIC and the peripheral, then resets the peripheral and disables
 * its clock. All steps are executed, any failure is reported.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t.
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Deinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t returnState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        /* -1- Data handling (transfers stopped, GPDMA channels and USART interrupt released) */
        const usart_RequestState_t xferState = Usart_Set_XferDeinit( usartId );

        /* -2- Interrupt in NVIC */
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartId ].PeriphNvic );

        /* -3- Peripheral and its interrupts */
        LL_USART_Disable( usart_PeriphConf[ usartId ].PeriphReg );

        const usart_RequestState_t irqState = Usart_Set_InterruptsInactive( usartId );

        /* -4- Peripheral reset and clock */
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( usart_PeriphConf[ usartId ].PeriphRcc );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( usart_PeriphConf[ usartId ].PeriphRcc );
        const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( usart_PeriphConf[ usartId ].PeriphRcc );

        if( ( USART_REQUEST_OK == xferState     ) &&
            ( NVIC_REQUEST_OK  == nvicState     ) &&
            ( USART_REQUEST_OK == irqState      ) &&
            ( RCC_REQUEST_OK   == rstActState   ) &&
            ( RCC_REQUEST_OK   == rstInactState ) &&
            ( RCC_REQUEST_OK   == clkState      )    )
        {
            returnState = USART_REQUEST_OK;
        }
        else
        {
            returnState = USART_REQUEST_ERROR;
        }
    }
    else
    {
        returnState = USART_REQUEST_ERROR;
    }

    return ( returnState );
}


/**
 * \brief Main task of module Usart
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It moves data of running transfers in USART_XFER_MODE_POLL - it has to
 * be called at least once per frame time of the fastest polled peripheral.
 */
void Usart_Task( void )
{
    for( usart_PeriphId_t usartId = (usart_PeriphId_t)0u; USART_BUS_CNT > usartId; usartId ++ )
    {
        if( ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState      ) &&
            ( ( USART_XFER_MODE_POLL == usart_XferContext[ usartId ].Config.TxMode ) ||
              ( USART_XFER_MODE_POLL == usart_XferContext[ usartId ].Config.RxMode )    )    )
        {
            (void)Usart_Poll_Task( usartId );
        }
        else
        {
            /* No polling data handling on the peripheral */
        }
    }
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
        usartConfig->BaudRate             = USART_DEFAULT_BAUDRATE;
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
        usartConfig->DataConfig           = USART_NULL_PTR;
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != baudrate )    )
    {
        const usart_RequestState_t oversamplingState = Usart_Get_Oversampling( usartId, &oversampling );
        const rcc_RequestState_t   rccRequestState   = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );
        const usart_RequestState_t prescState        = Usart_Get_Prescaler( usartId, &usartPrescaler );

        if( ( RCC_REQUEST_ERROR   != rccRequestState   ) &&
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
 * \note  Invalid usartId is ignored (nothing is written).
 *
 * \param usartId [in]: USART/UART bus identification, value from \ref usart_PeriphId_t.
 * \param txData  [in]: Data to be transmitted
 */
void Usart_SendData( usart_PeriphId_t usartId, usart_TxData_t txData )
{
    if( USART_BUS_CNT > usartId )
    {
        usart_PeriphConf[ usartId ].PeriphReg->TDR = txData;
    }
    else
    {
        /* Invalid peripheral identification */
    }
}


/**
 * \brief Reads data from reception register
 *
 * \param usartId [in]: USART/UART bus identification, value from \ref usart_PeriphId_t.
 * \return Received data value, 0 for invalid usartId
 */
usart_RxData_t Usart_ReadData( usart_PeriphId_t usartId )
{
    usart_RxData_t rxData = 0u;

    if( USART_BUS_CNT > usartId )
    {
        rxData = (usart_RxData_t)usart_PeriphConf[ usartId ].PeriphReg->RDR;
    }
    else
    {
        /* Invalid peripheral identification */
    }

    return ( rxData );
}


/*------------------------------ Data handling -------------------------------*/

/**
 * \brief Configures data handling (transmission / reception mode, buffers, callbacks)
 *
 * The previous data handling is released (GPDMA channels, USART interrupt), the configuration
 * is copied and the mode handlers of both directions are initialized. Transfers are started by
 * Usart_Set_TxStart() / Usart_Set_RxStart().
 *
 * \pre   Transmission and reception must not be running (see Usart_Set_TxStop() /
 *        Usart_Set_RxStop()). Otherwise \ref USART_REQUEST_ERROR is returned and nothing is
 *        changed. RxEndMode USART_RX_END_TIMEOUT requires enabled receiver timeout
 *        (usart_BusConfig_t RxTimeoutValue).
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration \ref usart_DataConfig_t.
 *                         Must not be NULL. The configuration is copied.
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_DataConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        const usart_RequestState_t configState = Usart_Check_DataConfig( usartId, dataConfig );

        if( ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].TxState ) ||
            ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].RxState )    )
        {
            /* Transfer is running, Usart_Set_TxStop() / Usart_Set_RxStop() is required first */
            retState = USART_REQUEST_ERROR;
        }
        else if( USART_REQUEST_OK != configState )
        {
            /* New configuration is invalid */
            retState = USART_REQUEST_ERROR;
        }
        else
        {
            retState = Usart_Set_XferDeinit( usartId );

            if( USART_REQUEST_OK == retState )
            {
                retState = Usart_Set_XferInit( usartId, dataConfig );
            }
            else
            {
                /* Previous data handling resources could not be released */
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
 * \brief Returns the active data handling configuration
 *
 * \param usartId     [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [out]: Pointer to store the data handling configuration. Must not be NULL.
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was success,
 *         otherwise (also if data handling is not initialized) returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_DataConfig( usart_PeriphId_t usartId, usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        if( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState )
        {
            *dataConfig = usart_XferContext[ usartId ].Config;
            retState    = USART_REQUEST_OK;
        }
        else
        {
            /* Data handling is not initialized */
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
 * \brief Starts transmission of a data buffer in the configured transmission mode
 *
 * The end of the transmission (last stop bit sent) is reported by TxCompleteCallback and by
 * Usart_Get_TxState() returning \ref USART_FUNCTION_INACTIVE.
 *
 * \pre   Data handling is initialized with TxMode other than USART_XFER_MODE_NONE and no
 *        transmission is running. Otherwise \ref USART_REQUEST_ERROR is returned.
 *
 * \param usartId  [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param txData   [in]: Data to be transmitted. Must not be NULL and must stay valid until the
 *                       end of the transmission.
 * \param txSize   [in]: Count of bytes to be transmitted (> 0)
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_TxStart( usart_PeriphId_t usartId, const usart_TxData_t * const txData, usart_TxDataCnt_t txSize )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != txData  ) &&
        ( 0u              < txSize  )    )
    {
        if( ( USART_FUNCTION_ACTIVE   == usart_XferContext[ usartId ].InitState     ) &&
            ( USART_FUNCTION_INACTIVE == usart_XferContext[ usartId ].TxState       ) &&
            ( USART_XFER_MODE_NONE    != usart_XferContext[ usartId ].Config.TxMode )    )
        {
            usart_XferContext[ usartId ].TxData  = txData;
            usart_XferContext[ usartId ].TxSize  = txSize;
            usart_XferContext[ usartId ].TxIdx   = 0u;
            usart_XferContext[ usartId ].TxState = USART_FUNCTION_ACTIVE;

            retState = usart_TxModeLut[ usart_XferContext[ usartId ].Config.TxMode ].Start( usartId );

            if( USART_REQUEST_OK != retState )
            {
                usart_XferContext[ usartId ].TxState = USART_FUNCTION_INACTIVE;
            }
            else
            {
                /* Transmission is running */
            }
        }
        else
        {
            /* Data handling not initialized, transmission running or not used */
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
 * \brief Stops a running transmission (TxCompleteCallback is not called)
 *
 * \note  A byte already written to the data register is still sent by HW.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was success
 *         (also if no transmission is running), otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_TxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState ) &&
            ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].TxState   )    )
        {
            usart_XferContext[ usartId ].TxState = USART_FUNCTION_INACTIVE;

            retState = usart_TxModeLut[ usart_XferContext[ usartId ].Config.TxMode ].Stop( usartId );
        }
        else
        {
            /* Transmission is not running */
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
 * \brief Returns state of the transmission
 *
 * \param usartId  [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param txState [out]: Pointer to store the state - \ref USART_FUNCTION_ACTIVE from
 *                       Usart_Set_TxStart() until the last stop bit is sent. Must not be NULL.
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_TxState( usart_PeriphId_t usartId, usart_FunctionState_t * const txState )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != txState )    )
    {
        *txState = usart_XferContext[ usartId ].TxState;
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Starts reception into RxBuffer in the configured reception mode
 *
 * Stale received data and reception flags are cleared, the next received byte is stored to
 * RxBuffer[ 0 ].
 *
 * \pre   Data handling is initialized with RxMode other than USART_XFER_MODE_NONE and no
 *        reception is running. Otherwise \ref USART_REQUEST_ERROR is returned.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_RxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( ( USART_FUNCTION_ACTIVE   == usart_XferContext[ usartId ].InitState     ) &&
            ( USART_FUNCTION_INACTIVE == usart_XferContext[ usartId ].RxState       ) &&
            ( USART_XFER_MODE_NONE    != usart_XferContext[ usartId ].Config.RxMode )    )
        {
            usart_XferContext[ usartId ].RxIdx   = 0u;
            usart_XferContext[ usartId ].RxState = USART_FUNCTION_ACTIVE;

            retState = usart_RxModeLut[ usart_XferContext[ usartId ].Config.RxMode ].Start( usartId );

            if( USART_REQUEST_OK != retState )
            {
                usart_XferContext[ usartId ].RxState = USART_FUNCTION_INACTIVE;
            }
            else
            {
                /* Reception is running */
            }
        }
        else
        {
            /* Data handling not initialized, reception running or not used */
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
 * \brief Stops a running reception (received data stay in RxBuffer, see Usart_Get_RxCount())
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was success
 *         (also if no reception is running), otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_RxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState ) &&
            ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].RxState   )    )
        {
            usart_XferContext[ usartId ].RxState = USART_FUNCTION_INACTIVE;

            retState = usart_RxModeLut[ usart_XferContext[ usartId ].Config.RxMode ].Stop( usartId );
        }
        else
        {
            /* Reception is not running */
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
 * \brief Returns state of the reception
 *
 * \param usartId  [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param rxState [out]: Pointer to store the state - \ref USART_FUNCTION_ACTIVE from
 *                       Usart_Set_RxStart() until stop (one shot buffer full / end of message,
 *                       Usart_Set_RxStop()). Must not be NULL.
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was
 *         success, otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_RxState( usart_PeriphId_t usartId, usart_FunctionState_t * const rxState )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != rxState )    )
    {
        *rxState = usart_XferContext[ usartId ].RxState;
        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns count of bytes stored in RxBuffer since the reception start (one shot) or the
 *        write position in RxBuffer (circular)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param rxCnt  [out]: Pointer to store the count of received bytes. Must not be NULL.
 *
 * \return State of request execution. Returns \ref USART_REQUEST_OK if request was success,
 *         otherwise (also if reception is not used) returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_RxCount( usart_PeriphId_t usartId, usart_RxDataCnt_t * const rxCnt )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != rxCnt   )    )
    {
        if( ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState     ) &&
            ( USART_XFER_MODE_DMA   == usart_XferContext[ usartId ].Config.RxMode )    )
        {
            /* DMA moves the data - count is derived from the GPDMA remaining count */
            retState = Usart_Dma_Get_RxCount( usartId, rxCnt );
        }
        else if( ( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState     ) &&
                 ( USART_XFER_MODE_NONE  != usart_XferContext[ usartId ].Config.RxMode )    )
        {
            *rxCnt   = usart_XferContext[ usartId ].RxIdx;
            retState = USART_REQUEST_OK;
        }
        else
        {
            /* Reception is not used */
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
    nvic_RequestState_t  nvicActivationState    = NVIC_REQUEST_ERROR;
    nvic_RequestState_t  nvicHandlerConfigState = NVIC_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicHandlerConfigState = Nvic_Set_PeriphIrq_Handler( usart_PeriphConf[ usartId ].PeriphNvic,
                                                             usart_PeriphConf[ usartId ].PeriphIsr );

        nvicActivationState = Nvic_Set_PeriphIrq_Active( usart_PeriphConf[ usartId ].PeriphNvic );

        if( ( NVIC_REQUEST_OK != nvicActivationState    ) ||
            ( NVIC_REQUEST_OK != nvicHandlerConfigState )    )
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
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicState = Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartId ].PeriphNvic );

        if( NVIC_REQUEST_OK != nvicState )
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
    nvic_RequestState_t      nvicState = NVIC_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicState = Nvic_Set_PeriphIrq_Prio( usart_PeriphConf[ usartId ].PeriphNvic, irqPrio );

        if( NVIC_REQUEST_OK != nvicState )
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
    nvic_RequestState_t      nvicState = NVIC_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId ) &&
        ( USART_NULL_PTR != irqPrio )    )
    {
        nvicState = Nvic_Get_PeriphIrq_Prio( usart_PeriphConf[ usartId ].PeriphNvic, irqPrio );

        if( NVIC_REQUEST_OK != nvicState )
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
    gpio_Config_t        pinConfig     = { 0u };

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
    gpio_Config_t        pinConfig     = { 0u };

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
    gpio_Config_t        pinConfig     = { 0u };

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
 * \param usartId      [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t.
 * \param periphClock  [in]: Peripheral clock in Hz
 * \param oversampling [in]: Peripheral over-sampling configuration
 * \param baudrate     [in]: Required peripheral baud-rate
 * \param prescaler   [out]: Calculated prescaler value
 *
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

    if( ( USART_BUS_CNT   > usartId   ) &&
        ( 0u              < baudrate  ) &&
        ( USART_NULL_PTR != prescaler )    )
    {
        rcc_RequestState_t rccRequestState = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );

        if( RCC_REQUEST_ERROR != rccRequestState )
        {
            if( USART_OVERSAMPLING_8 == oversampling )
            {
                prescValue = periphClock / ( baudrate * USART_OVERSAMPLING_8_FACTOR );
            }
            else
            {
                prescValue = periphClock / ( baudrate * USART_OVERSAMPLING_16_FACTOR );
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

/*------------------------------ Data handling -------------------------------*/

/**
 * \brief Validates a data handling configuration
 *
 * - TxMode / RxMode must be valid.
 * - Reception used: RxBuffer, RxBufferSize > 0, valid RxBufferMode and RxEndMode, receiver
 *   timeout enabled for USART_RX_END_TIMEOUT.
 * - Mode specific rules are checked by the mode handlers (DMA: identifications, priority,
 *   different channels).
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration \ref usart_DataConfig_t
 *
 * \return Returns \ref USART_REQUEST_OK if the configuration is valid. Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Check_DataConfig( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT        > usartId            ) &&
        ( USART_NULL_PTR      != dataConfig         ) &&
        ( USART_XFER_MODE_CNT  > dataConfig->TxMode ) &&
        ( USART_XFER_MODE_CNT  > dataConfig->RxMode )    )
    {
        if( USART_XFER_MODE_NONE != dataConfig->RxMode )
        {
            usart_FlagState_t          rtoState = USART_FLAG_INACTIVE;
            const usart_RequestState_t rtoRead  = Usart_Get_RxTimeoutState( usartId, &rtoState );

            if( ( USART_NULL_PTR        != dataConfig->RxBuffer     ) &&
                ( 0u                     < dataConfig->RxBufferSize ) &&
                ( USART_BUFFER_MODE_CNT  > dataConfig->RxBufferMode ) &&
                ( USART_RX_END_CNT       > dataConfig->RxEndMode    ) &&
                ( USART_REQUEST_OK      == rtoRead                  )    )
            {
                retState = USART_REQUEST_OK;
            }
            else
            {
                /* Reception configuration is invalid */
                retState = USART_REQUEST_ERROR;
            }

            if( ( USART_REQUEST_OK     == retState              ) &&
                ( USART_RX_END_TIMEOUT == dataConfig->RxEndMode ) &&
                ( USART_FLAG_ACTIVE    != rtoState              )    )
            {
                /* End of message by receiver timeout requires enabled receiver timeout */
                retState = USART_REQUEST_ERROR;
            }
            else
            {
                /* End of message detection is available */
            }
        }
        else
        {
            /* Reception is not used */
            retState = USART_REQUEST_OK;
        }

        if( USART_REQUEST_OK == retState )
        {
            retState = usart_TxModeLut[ dataConfig->TxMode ].CheckConfig( usartId, dataConfig );
        }
        else
        {
            /* Common part of the configuration is invalid */
        }

        if( USART_REQUEST_OK == retState )
        {
            retState = usart_RxModeLut[ dataConfig->RxMode ].CheckConfig( usartId, dataConfig );
        }
        else
        {
            /* Transmission configuration is invalid */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stores the data handling configuration and initializes the mode handlers of both
 *        directions and the USART interrupt (if used)
 *
 * \note  Initialization state is set before the handlers are initialized, so partially
 *        initialized handlers are released by Usart_Set_XferDeinit().
 *
 * \pre   Configuration was validated, the previous data handling is released.
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration \ref usart_DataConfig_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Set_XferInit( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dataConfig )    )
    {
        /* Configuration is copied - user structure may be a temporary (stack) variable */
        usart_XferContext[ usartId ].Config    = *dataConfig;
        usart_XferContext[ usartId ].TxData    = USART_NULL_PTR;
        usart_XferContext[ usartId ].TxSize    = 0u;
        usart_XferContext[ usartId ].TxIdx     = 0u;
        usart_XferContext[ usartId ].TxState   = USART_FUNCTION_INACTIVE;
        usart_XferContext[ usartId ].RxIdx     = 0u;
        usart_XferContext[ usartId ].RxState   = USART_FUNCTION_INACTIVE;
        usart_XferContext[ usartId ].InitState = USART_FUNCTION_ACTIVE;

        retState = usart_TxModeLut[ usart_XferContext[ usartId ].Config.TxMode ].Init( usartId );

        if( USART_REQUEST_OK == retState )
        {
            retState = usart_RxModeLut[ usart_XferContext[ usartId ].Config.RxMode ].Init( usartId );
        }
        else
        {
            /* Transmission handler initialization failed */
        }

        const usart_FunctionState_t irqUsed = Usart_Get_IrqUsed( &usart_XferContext[ usartId ].Config );

        if( ( USART_REQUEST_OK == retState ) && ( USART_FUNCTION_ACTIVE == irqUsed ) )
        {
            retState = Usart_Isr_Init( usartId );
        }
        else
        {
            /* Initialization failed or USART interrupt is not used */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stops running transfers and releases resources of the mode handlers and the USART
 *        interrupt (all steps are executed, any failure is reported)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request was processed
 *         without problems (also if data handling is not initialized). Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_Set_XferDeinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].InitState )
        {
            const usart_RequestState_t txStopState   = Usart_Set_TxStop( usartId );
            const usart_RequestState_t rxStopState   = Usart_Set_RxStop( usartId );
            const usart_RequestState_t txDeinitState = usart_TxModeLut[ usart_XferContext[ usartId ].Config.TxMode ].Deinit( usartId );
            const usart_RequestState_t rxDeinitState = usart_RxModeLut[ usart_XferContext[ usartId ].Config.RxMode ].Deinit( usartId );
            const usart_FunctionState_t irqUsed      = Usart_Get_IrqUsed( &usart_XferContext[ usartId ].Config );
            usart_RequestState_t       irqState      = USART_REQUEST_OK;

            if( USART_FUNCTION_ACTIVE == irqUsed )
            {
                irqState = Usart_Isr_Deinit( usartId );
            }
            else
            {
                /* USART interrupt is not used */
            }

            usart_XferContext[ usartId ].InitState = USART_FUNCTION_INACTIVE;

            if( ( USART_REQUEST_OK == txStopState   ) &&
                ( USART_REQUEST_OK == rxStopState   ) &&
                ( USART_REQUEST_OK == txDeinitState ) &&
                ( USART_REQUEST_OK == rxDeinitState ) &&
                ( USART_REQUEST_OK == irqState      )    )
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
            /* Data handling is not initialized */
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
 * \brief Returns whether the USART interrupt is used by the data handling (any direction in
 *        DMA or ISR mode)
 *
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return \ref USART_FUNCTION_ACTIVE if the USART interrupt is used, otherwise
 *         \ref USART_FUNCTION_INACTIVE.
 */
static usart_FunctionState_t Usart_Get_IrqUsed( const usart_DataConfig_t * const dataConfig )
{
    usart_FunctionState_t irqUsed = USART_FUNCTION_INACTIVE;

    if( USART_NULL_PTR != dataConfig )
    {
        if( ( USART_XFER_MODE_DMA == dataConfig->TxMode ) ||
            ( USART_XFER_MODE_ISR == dataConfig->TxMode ) ||
            ( USART_XFER_MODE_DMA == dataConfig->RxMode ) ||
            ( USART_XFER_MODE_ISR == dataConfig->RxMode )    )
        {
            irqUsed = USART_FUNCTION_ACTIVE;
        }
        else
        {
            irqUsed = USART_FUNCTION_INACTIVE;
        }
    }
    else
    {
        irqUsed = USART_FUNCTION_INACTIVE;
    }

    return ( irqUsed );
}


/**
 * \brief Configuration check of unused direction (USART_XFER_MODE_NONE)
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param dataConfig [in]: Pointer to data handling configuration. Must not be NULL.
 *
 * \return Returns \ref USART_REQUEST_OK for valid parameters, otherwise \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_None_Check_Config( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig )
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
 * \brief Initialization / deinitialization / stop of unused direction (nothing to be done)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Returns \ref USART_REQUEST_OK for valid peripheral, otherwise \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_None_XferInit( usart_PeriphId_t usartId )
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
 * \brief Start of unused direction - not allowed
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Always returns \ref USART_REQUEST_ERROR.
 */
static usart_RequestState_t Usart_None_XferStart( usart_PeriphId_t usartId )
{
    (void)usartId;

    return ( USART_REQUEST_ERROR );
}

/* -------------------------------------------------------------------------- */
/* ------------------- Private interface (see Usart.h) ---------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief Returns CMSIS register pointer of a USART/UART peripheral (for data transfer handlers)
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param periphReg [out]: Pointer to store the register pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_PeriphReg( usart_PeriphId_t usartId, USART_TypeDef ** const periphReg )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId   ) &&
        ( USART_NULL_PTR != periphReg )    )
    {
        *periphReg = usart_PeriphConf[ usartId ].PeriphReg;
        retState   = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns GPDMA requests of a USART/UART peripheral (for DMA data transfer handler)
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param txRequest [out]: Pointer to store the transmission request. Must not be NULL.
 * \param rxRequest [out]: Pointer to store the reception request. Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_PeriphDmaReq( usart_PeriphId_t usartId, gpdma_PeriphReqId_t * const txRequest, gpdma_PeriphReqId_t * const rxRequest )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId   ) &&
        ( USART_NULL_PTR != txRequest ) &&
        ( USART_NULL_PTR != rxRequest )    )
    {
        *txRequest = usart_PeriphConf[ usartId ].PeriphDmaTxReq;
        *rxRequest = usart_PeriphConf[ usartId ].PeriphDmaRxReq;
        retState   = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns data handling context of a USART/UART peripheral (for data transfer handlers)
 *
 * \param usartId      [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param xferContext [out]: Pointer to store the context pointer. Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_XferContext( usart_PeriphId_t usartId, usart_XferContext_t ** const xferContext )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId     ) &&
        ( USART_NULL_PTR != xferContext )    )
    {
        *xferContext = &usart_XferContext[ usartId ];
        retState     = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Discards stale received data and clears reception flags (PE, FE, NE, ORE, IDLE, RTO)
 *        before a reception start
 *
 * \note  Flags are set by HW at any time, the clear is not verified by read-back.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_RxFlagsClear( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        USART_TypeDef * const periphReg = usart_PeriphConf[ usartId ].PeriphReg;

        (void)LL_USART_ReceiveData8( periphReg );
        LL_USART_ClearFlag_PE( periphReg );
        LL_USART_ClearFlag_FE( periphReg );
        LL_USART_ClearFlag_NE( periphReg );
        LL_USART_ClearFlag_ORE( periphReg );
        LL_USART_ClearFlag_IDLE( periphReg );
        LL_USART_ClearFlag_RTO( periphReg );

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Returns the next byte of the running transmission (ISR / POLL mode)
 *
 * \param usartId    [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param txData    [out]: Pointer to store the byte. Must not be NULL.
 * \param dataValid [out]: Pointer to store \ref USART_FUNCTION_ACTIVE if a byte was returned,
 *                         \ref USART_FUNCTION_INACTIVE if all bytes were already taken or no
 *                         transmission is running. Must not be NULL.
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Get_XferTxData( usart_PeriphId_t usartId, usart_TxData_t * const txData, usart_FunctionState_t * const dataValid )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId   ) &&
        ( USART_NULL_PTR != txData    ) &&
        ( USART_NULL_PTR != dataValid )    )
    {
        if( ( USART_FUNCTION_ACTIVE               == usart_XferContext[ usartId ].TxState ) &&
            ( USART_NULL_PTR                      != usart_XferContext[ usartId ].TxData  ) &&
            ( usart_XferContext[ usartId ].TxSize  > usart_XferContext[ usartId ].TxIdx   )    )
        {
            *txData    = usart_XferContext[ usartId ].TxData[ usart_XferContext[ usartId ].TxIdx ];
            *dataValid = USART_FUNCTION_ACTIVE;
            usart_XferContext[ usartId ].TxIdx ++;
        }
        else
        {
            /* No byte left to be transmitted */
            *dataValid = USART_FUNCTION_INACTIVE;
        }

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Finishes the transmission (last stop bit sent) and reports TxCompleteCallback
 *        (a new transmission can be started from the callback)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferTxDone( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].TxState )
        {
            usart_XferContext[ usartId ].TxState = USART_FUNCTION_INACTIVE;

            if( USART_NULL_PTR != usart_XferContext[ usartId ].Config.TxCompleteCallback )
            {
                usart_XferContext[ usartId ].Config.TxCompleteCallback();
            }
            else
            {
                /* Event is not reported */
            }
        }
        else
        {
            /* Transmission was stopped meanwhile */
        }

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Stores one received byte into RxBuffer (ISR / POLL mode) and reports half / full
 *        buffer events
 *
 * \note  Bytes received while the reception is not running are dropped.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param rxData  [in]: Received byte
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferRxData( usart_PeriphId_t usartId, usart_RxData_t rxData )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;

        if( ( USART_FUNCTION_ACTIVE                           == usart_XferContext[ usartId ].RxState         ) &&
            ( USART_NULL_PTR                                  != usart_XferContext[ usartId ].Config.RxBuffer ) &&
            ( usart_XferContext[ usartId ].Config.RxBufferSize > usart_XferContext[ usartId ].RxIdx           )    )
        {
            const usart_RxDataCnt_t halfSize = usart_XferContext[ usartId ].Config.RxBufferSize / USART_BUFFER_HALF_DIVIDER;

            usart_XferContext[ usartId ].Config.RxBuffer[ usart_XferContext[ usartId ].RxIdx ] = rxData;
            usart_XferContext[ usartId ].RxIdx ++;

            if( ( 0u != halfSize ) && 
            ( halfSize == usart_XferContext[ usartId ].RxIdx ) )
            {
                retState = Usart_Set_XferRxHalf( usartId );
            }
            else
            {
                /* Half of the buffer not reached in this step */
            }

            if( usart_XferContext[ usartId ].Config.RxBufferSize <= usart_XferContext[ usartId ].RxIdx )
            {
                retState = Usart_Set_XferRxDone( usartId );
            }
            else
            {
                /* Buffer is not full yet */
            }
        }
        else
        {
            /* Reception is not running - byte is dropped */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Reports half filled receive buffer (RxHalfCallback)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferRxHalf( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_NULL_PTR != usart_XferContext[ usartId ].Config.RxHalfCallback )
        {
            usart_XferContext[ usartId ].Config.RxHalfCallback();
        }
        else
        {
            /* Event is not reported */
        }

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Handles full receive buffer: circular buffer continues from RxBuffer[ 0 ], one shot
 *        buffer stops the reception; then RxCompleteCallback is called (the reception can be
 *        restarted from the callback)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferRxDone( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_BUFFER_MODE_CIRCULAR == usart_XferContext[ usartId ].Config.RxBufferMode )
        {
            /* Next byte is stored to the buffer start (DMA is re-armed by Usart_Dma.c) */
            usart_XferContext[ usartId ].RxIdx = 0u;
            retState = USART_REQUEST_OK;
        }
        else
        {
            /* One shot - reception is stopped */
            retState = Usart_Set_RxStop( usartId );
        }

        if( USART_NULL_PTR != usart_XferContext[ usartId ].Config.RxCompleteCallback )
        {
            usart_XferContext[ usartId ].Config.RxCompleteCallback();
        }
        else
        {
            /* Event is not reported */
        }
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Handles end of received message: one shot reception is stopped, RxEndCallback is
 *        called with the count of received bytes (the reception can be restarted from the
 *        callback)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferRxEnd( usart_PeriphId_t usartId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RxDataCnt_t rxCnt = 0u;

        if( USART_FUNCTION_ACTIVE == usart_XferContext[ usartId ].RxState )
        {
            /* Count is read before the reception is stopped */
            retState = Usart_Get_RxCount( usartId, &rxCnt );

            if( ( USART_REQUEST_OK           == retState                                         ) && 
                ( USART_BUFFER_MODE_ONE_SHOT == usart_XferContext[ usartId ].Config.RxBufferMode )    )
            {
                retState = Usart_Set_RxStop( usartId );
            }
            else
            {
                /* Circular buffer - reception continues */
            }

            if( ( USART_REQUEST_OK == retState                                          ) && 
                ( USART_NULL_PTR   != usart_XferContext[ usartId ].Config.RxEndCallback )    )
            {
                usart_XferContext[ usartId ].Config.RxEndCallback( rxCnt );
            }
            else
            {
                /* Event is not reported */
            }
        }
        else
        {
            /* Reception is not running */
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
 * \brief Reports a data transfer error (ErrorCallback)
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param errorId [in]: Error identification, value from \ref usart_XferErrorId_t
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems. Otherwise returns \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferError( usart_PeriphId_t usartId, usart_XferErrorId_t errorId )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT        > usartId ) && 
        ( USART_XFER_ERROR_CNT > errorId )    )
    {
        if( USART_NULL_PTR != usart_XferContext[ usartId ].Config.ErrorCallback )
        {
            usart_XferContext[ usartId ].Config.ErrorCallback( errorId );
        }
        else
        {
            /* Error is not reported */
        }

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}


/**
 * \brief Clears pending reception error flags (PE, FE, NE, ORE) and reports every pending error
 *
 * \param usartId  [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param isrFlags [in]: Snapshot of USART ISR register (flags read before clearing)
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems (also if no error is pending). Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferRxErrors( usart_PeriphId_t usartId, uint32_t isrFlags )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;

        if( 0u != ( USART_ISR_ERROR_MASK & isrFlags ) )
        {
            LL_USART_ClearFlag_PE( usart_PeriphConf[ usartId ].PeriphReg );
            LL_USART_ClearFlag_FE( usart_PeriphConf[ usartId ].PeriphReg );
            LL_USART_ClearFlag_NE( usart_PeriphConf[ usartId ].PeriphReg );
            LL_USART_ClearFlag_ORE( usart_PeriphConf[ usartId ].PeriphReg );

            for( usart_XferErrorId_t errorId = USART_XFER_ERROR_PARITY; USART_XFER_ERROR_CNT > errorId; errorId ++ )
            {
                const uint32_t errorFlag = (uint32_t)usart_RxErrorFlagLut[ errorId ];

                if( ( USART_ERROR_NONE != errorFlag                ) && 
                    ( 0u               != ( errorFlag & isrFlags ) )    )
                {
                    retState = Usart_Set_XferError( usartId, errorId );
                }
                else
                {
                    /* Error is not pending or has no flag */
                }
            }
        }
        else
        {
            /* No reception error pending */
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
 * \brief Global interrupt handler - the USART interrupt is used only by the data transfer
 *        handlers (see Usart_Isr.c)
 *
 * \param usartId [in]: USART/UART bus identification
 */
static inline void Usart_GlobalIsrHandler( usart_PeriphId_t usartId )
{
    (void)Usart_Isr_Handler( usartId );
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
void Usart_Uart4_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_4 );
}
#endif /* UART4 */

#ifdef UART5
/**
 * \brief UART5 Interrupt handler
 */
void Usart_Uart5_IsrHandler(void)
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

#ifdef UART7


/**
 * \brief UART7 interrupt service routine.
 */
static void Usart_Uart7_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_7 );
}
#endif /* UART7 */

#ifdef UART8


/**
 * \brief UART8 interrupt service routine.
 */
static void Usart_Uart8_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_8 );
}
#endif /* UART8 */

#ifdef UART9


/**
 * \brief UART9 interrupt service routine.
 */
static void Usart_Uart9_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_9 );
}
#endif /* UART9 */

#ifdef USART10


/**
 * \brief USART10 interrupt service routine.
 */
static void Usart_Usart10_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_10 );
}
#endif /* USART10 */

#ifdef USART11


/**
 * \brief USART11 interrupt service routine.
 */
static void Usart_Usart11_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_11 );
}
#endif /* USART11 */

#ifdef UART12


/**
 * \brief UART12 interrupt service routine.
 */
static void Usart_Uart12_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_12 );
}
#endif /* UART12 */

/* ================================ TASKS =================================== */

