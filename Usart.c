/**
 * \author Mr.Nobody
 * \file Usart.c
 * \ingroup Usart
 * \brief Universal Synchronous/Asynchronous Receiver-Transmitter (USART) MCAL
 *        module common functionality
 *
 * Implementation for STM32F4 family with the public interface of STM32H5 USART
 * module. STM32F4 USART has single status register (SR) and single data
 * register (DR) for transmission and reception. Flags PE, FE, NE, ORE and IDLE
 * are cleared by read of SR followed by read of DR, flag TC is cleared by write
 * of 0.
 *
 * Features of the public interface not available on STM32F4 (receiver timeout,
 * Driver Enable (DE), pin level inversion, 7 bit word length) accept only the
 * inactive / standard configuration, otherwise return error. UART4 / UART5
 * (UART7 / UART8) do not support hardware flow control and 0.5 / 1.5 stop bits.
 *
 * Data handling (transmission / reception of buffers in DMA / ISR / POLL mode)
 * is configured by \ref usart_DataConfig_t, the mode handlers are implemented in
 * Usart_Dma.c, Usart_Isr.c and Usart_Poll.c.
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
#include "Rcc_Port.h"                       /* RCC port functionality         */
#include "Nvic_Port.h"                      /* NVIC port functionality        */
#include "Gpio_Port.h"                      /* GPIO port functionality        */
/* ============================== TYPEDEFS ================================== */

/** Structure type used for USART/UART configuration array */
typedef struct
{
    USART_TypeDef*       PeriphReg;  /**< Peripheral configuration register */
    rcc_PeriphId_t       PeriphRcc;  /**< Peripheral RCC configuration ID   */
    nvic_PeriphIrqList_t PeriphNvic; /**< Peripheral NVIC configuration ID  */
    nvic_IsrCallback_t   PeriphIsr;  /**< Peripheral ISR callback routines  */
}   usart_PeriphConfigStruct_t;

/* ======================== FORWARD DECLARATIONS ============================ */

#ifdef USART1
static void Usart_Usart1_IsrHandler( void );
#endif /* USART1 */
#ifdef USART2
static void Usart_Usart2_IsrHandler( void );
#endif /* USART2 */
#ifdef USART3
static void Usart_Usart3_IsrHandler( void );
#endif /* USART3 */
#ifdef UART4
static void Usart_Uart4_IsrHandler( void );
#endif /* UART4 */
#ifdef UART5
static void Usart_Uart5_IsrHandler( void );
#endif /* UART5 */
#ifdef USART6
static void Usart_Usart6_IsrHandler( void );
#endif /* USART6 */
#ifdef UART7
static void Usart_Uart7_IsrHandler( void );
#endif /* UART7 */
#ifdef UART8
static void Usart_Uart8_IsrHandler( void );
#endif /* UART8 */

static inline void Usart_GlobalIsrHandler( usart_PeriphId_t usartId );

static usart_RequestState_t Usart_Verify_RegBits     ( volatile const uint32_t * const regAddr, uint32_t bitMask, uint32_t expectedValue );
static usart_RequestState_t Usart_InitGpioPin        ( uint32_t pinCode, gpio_PinOutputType_t outType, gpio_PinPullCfg_t pull );
static usart_RequestState_t Usart_Set_RtsOutput      ( usart_PeriphId_t usartId, uint32_t cr1Value );

static usart_RequestState_t Usart_Check_DataConfig   ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
static usart_RequestState_t Usart_Set_XferInit       ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
static usart_RequestState_t Usart_Set_XferDeinit     ( usart_PeriphId_t usartId );
static usart_FunctionState_t Usart_Get_IrqUsed       ( const usart_DataConfig_t * const dataConfig );

static usart_RequestState_t Usart_None_Check_Config  ( usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig );
static usart_RequestState_t Usart_None_XferInit      ( usart_PeriphId_t usartId );
static usart_RequestState_t Usart_None_XferStart     ( usart_PeriphId_t usartId );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define USART_MAJOR_VERSION           ( 2u )

/** Value of minor version of SW module */
#define USART_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define USART_PATCH_VERSION           ( 0u )

/** Default baud-rate used by \ref Usart_Get_DefaultConfig */
#define USART_DEFAULT_BAUDRATE        ( 115200u )

/** Receive buffer half is reached after RxBufferSize / USART_BUFFER_HALF_DIVIDER bytes */
#define USART_BUFFER_HALF_DIVIDER     ( 2u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** USART/UART peripherals configuration array */
static usart_PeriphConfigStruct_t const         usart_PeriphConf[ ] =
{
#ifdef USART1
    { .PeriphReg = USART1, .PeriphRcc = RCC_PERIPH_USART1, .PeriphNvic = NVIC_PERIPH_IRQ_USART1, .PeriphIsr = Usart_Usart1_IsrHandler },
#endif
#ifdef USART2
    { .PeriphReg = USART2, .PeriphRcc = RCC_PERIPH_USART2, .PeriphNvic = NVIC_PERIPH_IRQ_USART2, .PeriphIsr = Usart_Usart2_IsrHandler },
#endif
#ifdef USART3
    { .PeriphReg = USART3, .PeriphRcc = RCC_PERIPH_USART3, .PeriphNvic = NVIC_PERIPH_IRQ_USART3, .PeriphIsr = Usart_Usart3_IsrHandler },
#endif
#ifdef UART4
    { .PeriphReg = UART4,  .PeriphRcc = RCC_PERIPH_UART4,  .PeriphNvic = NVIC_PERIPH_IRQ_UART4,  .PeriphIsr = Usart_Uart4_IsrHandler  },
#endif
#ifdef UART5
    { .PeriphReg = UART5,  .PeriphRcc = RCC_PERIPH_UART5,  .PeriphNvic = NVIC_PERIPH_IRQ_UART5,  .PeriphIsr = Usart_Uart5_IsrHandler  },
#endif
#ifdef USART6
    { .PeriphReg = USART6, .PeriphRcc = RCC_PERIPH_USART6, .PeriphNvic = NVIC_PERIPH_IRQ_USART6, .PeriphIsr = Usart_Usart6_IsrHandler },
#endif
#ifdef UART7
    { .PeriphReg = UART7,  .PeriphRcc = RCC_PERIPH_UART7,  .PeriphNvic = NVIC_PERIPH_IRQ_UART7,  .PeriphIsr = Usart_Uart7_IsrHandler  },
#endif
#ifdef UART8
    { .PeriphReg = UART8,  .PeriphRcc = RCC_PERIPH_UART8,  .PeriphNvic = NVIC_PERIPH_IRQ_UART8,  .PeriphIsr = Usart_Uart8_IsrHandler  },
#endif
};

_Static_assert( USART_BUS_CNT == ( (sizeof(usart_PeriphConf) / sizeof(usart_PeriphConfigStruct_t) ) ), "Usart: size of usart_PeriphConf is incorrect.");


/** \brief Data handling runtime context per peripheral */
static usart_XferContext_t usart_XferContext[ USART_BUS_CNT ];


/** \brief RTS hardware flow control requested by \ref Usart_Set_FlowControl per peripheral - the
 *         RTS output (RTSE) is enabled only while the USART and its receiver are enabled (device
 *         errata "RTS is active while RE or UE = 0") */
static usart_FunctionState_t usart_RtsRequest[ USART_BUS_CNT ];


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


/** \brief usart_XferErrorId_t -> reception error flag in USART status register (DMA errors have no flag) */
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
 * Data handling of a previous initialization is released, peripheral clock is
 * activated (if inactive), the peripheral is reset, configured RX / TX pins are
 * initialized (pin of another peripheral or "_UNUSED" pin is not configured),
 * the peripheral is configured and enabled and the data handling is
 * initialized (if DataConfig is set).
 *
 * \note In half-duplex mode TX pin is configured as open-drain with pull-up
 *       (TX pin is released by peripheral when no data are transmitted).
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
        const usart_PeriphId_t usartId            = usartConfig->PeriphId;
        rcc_FunctionState_t    rccActivationState = RCC_FUNCTION_INACTIVE;
        rcc_RequestState_t     rccRequestState    = RCC_REQUEST_ERROR;

        /*------------- Data handling of previous initialization -------------*/
        retState = Usart_Set_XferDeinit( usartId );

        /*------------- USART peripheral clock activation section ------------*/
        rccRequestState = Rcc_Get_PeriphState( usart_PeriphConf[ usartId ].PeriphRcc, &rccActivationState );

        if( RCC_REQUEST_OK != rccRequestState )
        {
            retState = USART_REQUEST_ERROR;
        }
        else if( RCC_FUNCTION_INACTIVE == rccActivationState )
        {
            rccRequestState = Rcc_Set_PeriphActive( usart_PeriphConf[ usartId ].PeriphRcc );

            if( RCC_REQUEST_OK != rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
            }
        }
        else
        {
            /* No clock activation needed */
        }

        /*------------------ USART peripheral reset section ------------------*/
        if( USART_REQUEST_ERROR != retState )
        {
            rccRequestState = Rcc_Set_ResetActive( usart_PeriphConf[ usartId ].PeriphRcc );

            if( RCC_REQUEST_OK == rccRequestState )
            {
                rccRequestState = Rcc_Set_ResetInactive( usart_PeriphConf[ usartId ].PeriphRcc );
            }

            if( RCC_REQUEST_OK != rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
            }
        }

        /*---------- USART peripheral GPIO initialization section ------------*/
        if( ( USART_REQUEST_ERROR                                 != retState              ) &&
            ( USART_RX_PIN_UNUSED                                 != usartConfig->BusRxPin ) &&
            ( (uint32_t)USART_BIT_MASK_DECODE_PERIPH( usartConfig->BusRxPin ) == (uint32_t)usartId ) )
        {
            retState = Usart_InitRxGpio( usartConfig->BusRxPin );
        }
        else
        {
            /* RX pin configuration is not used */
        }

        if( ( USART_REQUEST_ERROR                                 != retState              ) &&
            ( USART_TX_PIN_UNUSED                                 != usartConfig->BusTxPin ) &&
            ( (uint32_t)USART_BIT_MASK_DECODE_PERIPH( usartConfig->BusTxPin ) == (uint32_t)usartId ) )
        {
            if( USART_HALF_DUPLEX_INACTIVE != usartConfig->HalfDuplex )
            {
                retState = Usart_InitGpioPin( (uint32_t)usartConfig->BusTxPin, GPIO_PIN_OUTPUT_OPENDRAIN, GPIO_PIN_PULL_UP );
            }
            else
            {
                retState = Usart_InitTxGpio( usartConfig->BusTxPin );
            }
        }
        else
        {
            /* TX pin configuration is not used */
        }

        /* STM32F4 has no Driver Enable (DE) pin - BusDePin is not used, DE mode is rejected below */

        /*------------ USART peripheral initialization section ---------------*/
        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_PeriphInactive( usartId );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            /* Over-sampling configuration must be executed before baud-rate configuration */
            retState = Usart_Set_Oversampling( usartId, usartConfig->Oversampling );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_Baudrate( usartId, usartConfig->BaudRate );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_DataWidth( usartId, usartConfig->DataWidth );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_StopBits( usartId, usartConfig->StopBits );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_Parity( usartId, usartConfig->Parity );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_TransferMode( usartId, usartConfig->TransferMode );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_FlowControl( usartId, usartConfig->HwFlowControl );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_HalfDuplexState( usartId, usartConfig->HalfDuplex );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_DriverEnableState( usartId, usartConfig->DriverEnableMode );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_DriverEnablePolarity( usartId, usartConfig->DriverEnablePolarity );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_PinLevels( usartId, usartConfig->RxPinOperationLevels, usartConfig->TxPinOperationLevels );
        }

        if( USART_REQUEST_ERROR != retState )
        {
            if( USART_RX_TIMEOUT_MIN < usartConfig->RxTimeoutValue )
            {
                retState = Usart_Set_RxTimeoutActive( usartId, usartConfig->RxTimeoutValue );
            }
            else
            {
                retState = Usart_Set_RxTimeoutInactive( usartId );
            }
        }

        if( USART_REQUEST_ERROR != retState )
        {
            retState = Usart_Set_PeriphActive( usartId );
        }
        else
        {
            /* Error during initialization process */
        }

        /*------------------ Data handling initialization ------------------*/
        if( ( USART_REQUEST_ERROR != retState                ) &&
            ( USART_NULL_PTR      != usartConfig->DataConfig )    )
        {
            retState = Usart_Set_DataConfig( usartId, usartConfig->DataConfig );
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
 * Stops the data handling and releases its resources (DMA streams, USART interrupt), disables
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
        /* -1- Data handling (transfers stopped, DMA streams and USART interrupt released) */
        const usart_RequestState_t xferState = Usart_Set_XferDeinit( usartId );

        /* -2- Interrupt in NVIC */
        const nvic_RequestState_t nvicState = Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartId ].PeriphNvic );

        /* -3- Peripheral */
        const usart_RequestState_t periphState = Usart_Set_PeriphInactive( usartId );

        /* -4- Peripheral reset and clock */
        const rcc_RequestState_t rstActState   = Rcc_Set_ResetActive( usart_PeriphConf[ usartId ].PeriphRcc );
        const rcc_RequestState_t rstInactState = Rcc_Set_ResetInactive( usart_PeriphConf[ usartId ].PeriphRcc );
        const rcc_RequestState_t clkState      = Rcc_Set_PeriphInactive( usart_PeriphConf[ usartId ].PeriphRcc );

        /* Flow control configuration is cleared by the reset */
        usart_RtsRequest[ usartId ] = USART_FUNCTION_INACTIVE;

        if( ( USART_REQUEST_OK == xferState     ) &&
            ( NVIC_REQUEST_OK  == nvicState     ) &&
            ( USART_REQUEST_OK == periphState   ) &&
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
 * Default configuration: USART1, 115200 Bd, 8 data bits, 1 stop bit, no parity,
 * transmitter and receiver enabled, no flow control, over-sampling by 8, no
 * pins configured, no data handling.
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
        usartConfig->RxTimeoutValue       = USART_RX_TIMEOUT_MIN;
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
 * \brief Enables USART peripheral (UE)
 *
 * \note  Requested RTS output (\ref Usart_Set_FlowControl) is enabled after UE if the receiver
 *        is enabled (device errata "RTS is active while RE or UE = 0").
 *
 * \param usartId  [in]: USART/UART bus identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_PeriphActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_Enable( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_UE, USART_CR1_UE );

        if( USART_REQUEST_OK == retValue )
        {
            retValue = Usart_Set_RtsOutput( usartId, usart_PeriphConf[ usartId ].PeriphReg->CR1 );
        }
        else
        {
            /* Peripheral was not enabled */
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
 * \brief Disables USART peripheral (UE)
 *
 * When peripheral is deactivated, the USART prescalers and outputs are stopped
 * at the end of the current byte transfer.
 *
 * \note  RTS output is disabled before UE (device errata "RTS is active while RE or UE = 0"),
 *        RTS flow control stays requested.
 *
 * \param usartId  [in]: USART/UART bus identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_PeriphInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retValue = Usart_Set_RtsOutput( usartId, usart_PeriphConf[ usartId ].PeriphReg->CR1 & ~USART_CR1_UE );

        LL_USART_Disable( usart_PeriphConf[ usartId ].PeriphReg );

        if( USART_REQUEST_OK == retValue )
        {
            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_UE, 0u );
        }
        else
        {
            /* RTS output was not disabled - error is kept */
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        const uint32_t regValue = LL_USART_IsEnabled( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
    }
    else
    {
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
 *         otherwise return error (also when baud-rate is not reachable with
 *         actual peripheral clock).
 */
usart_RequestState_t Usart_Set_Baudrate( usart_PeriphId_t usartId, usart_Baudrate_t baudrate )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_Oversampling_t oversampling   = USART_OVERSAMPLING_16;
    rcc_FreqHz_t         usartPeriphClk = 0u;
    uint32_t             brrValue       = 0u;

    if( ( 0u            != baudrate ) &&
        ( USART_BUS_CNT  > usartId  )    )
    {
        usart_RequestState_t oversamplingState = Usart_Get_Oversampling( usartId, &oversampling );
        rcc_RequestState_t   rccRequestState   = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );

        if( ( USART_REQUEST_OK == oversamplingState ) &&
            ( RCC_REQUEST_OK   == rccRequestState   )    )
        {
            retValue = Usart_Get_ExpectedBrr( usartPeriphClk, oversampling, baudrate, &brrValue );
        }

        if( USART_REQUEST_OK == retValue )
        {
            LL_USART_SetBaudRate( usart_PeriphConf[ usartId ].PeriphReg,
                                  usartPeriphClk,
                                  oversampling,
                                  baudrate );

            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->BRR, USART_BRR_DIV_Mantissa | USART_BRR_DIV_Fraction, brrValue );
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
 * Baud-rate is calculated from content of BRR register and actual peripheral
 * clock, it may differ from configured value by rounding of the divider.
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param baudrate [out]: Value of configured baud-rate
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Baudrate( usart_PeriphId_t usartId, usart_Baudrate_t * const baudrate )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_Oversampling_t oversampling   = USART_OVERSAMPLING_16;
    rcc_FreqHz_t         usartPeriphClk = 0u;

    if( ( USART_NULL_PTR != baudrate ) &&
        ( USART_BUS_CNT   > usartId  )    )
    {
        usart_RequestState_t oversamplingState = Usart_Get_Oversampling( usartId, &oversampling );
        rcc_RequestState_t   rccRequestState   = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].PeriphRcc, &usartPeriphClk );

        if( ( USART_REQUEST_OK == oversamplingState ) &&
            ( RCC_REQUEST_OK   == rccRequestState   )    )
        {
            *baudrate = LL_USART_GetBaudRate( usart_PeriphConf[ usartId ].PeriphReg,
                                              usartPeriphClk,
                                              oversampling );

            /* Baud-rate 0 - BRR register is not configured */
            retValue = ( 0u != *baudrate ) ? USART_REQUEST_OK : USART_REQUEST_ERROR;
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
 * \note STM32F4 supports 8 and 9 bit word length only.
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param dataWidth [in]: Required data width configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DataWidth( usart_PeriphId_t usartId, usart_DataWidth_t dataWidth )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) &&
        ( ( USART_DATA_WIDTH_8 == dataWidth ) ||
          ( USART_DATA_WIDTH_9 == dataWidth )    ) )
    {
        LL_USART_SetDataWidth( usart_PeriphConf[ usartId ].PeriphReg, dataWidth );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_M, dataWidth );
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

    if( ( USART_BUS_CNT   > usartId   ) &&
        ( USART_NULL_PTR != dataWidth )    )
    {
        *dataWidth = LL_USART_GetDataWidth( usart_PeriphConf[ usartId ].PeriphReg );
        retValue   = USART_REQUEST_OK;
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
 * \note 0.5 and 1.5 stop bits are not available on UART4 / UART5 / UART7 / UART8.
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param stopBits [in]: Required stop-bits configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_StopBits( usart_PeriphId_t usartId, usart_StopBits_t stopBits )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( ( USART_STOP_BITS_1 == stopBits ) ||
            ( USART_STOP_BITS_2 == stopBits )    )
        {
            retValue = USART_REQUEST_OK;
        }
        else if( ( ( USART_STOP_BITS_0_5 == stopBits )   ||
                   ( USART_STOP_BITS_1_5 == stopBits )      ) &&
                 ( IS_USART_INSTANCE( usart_PeriphConf[ usartId ].PeriphReg ) ) )
        {
            retValue = USART_REQUEST_OK;
        }
        else
        {
            /* Invalid configuration or fractional stop bits on UART */
            retValue = USART_REQUEST_ERROR;
        }

        if( USART_REQUEST_OK == retValue )
        {
            LL_USART_SetStopBitsLength( usart_PeriphConf[ usartId ].PeriphReg, stopBits );

            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR2, USART_CR2_STOP, stopBits );
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != stopBits )    )
    {
        *stopBits = LL_USART_GetStopBitsLength( usart_PeriphConf[ usartId ].PeriphReg );
        retValue  = USART_REQUEST_OK;
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
 * \note Parity bit is part of the data word - 8 bit word with parity carries
 *       7 data bits.
 *
 * \param usartId [in]: USART/UART bus identification.
 * \param parity  [in]: Required parity configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Parity( usart_PeriphId_t usartId, usart_Parity_t parity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) &&
        ( ( USART_PARITY_NONE == parity ) ||
          ( USART_PARITY_EVEN == parity ) ||
          ( USART_PARITY_ODD  == parity )    ) )
    {
        LL_USART_SetParity( usart_PeriphConf[ usartId ].PeriphReg, parity );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_PCE | USART_CR1_PS, parity );
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

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != parity  )    )
    {
        *parity  = LL_USART_GetParity( usart_PeriphConf[ usartId ].PeriphReg );
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
 * \note  RTS output follows the receiver (device errata "RTS is active while RE or UE = 0"):
 *        it is disabled before the receiver is disabled and enabled after the receiver is
 *        enabled (if RTS flow control is requested and the USART is enabled).
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param transferMode [in]: Required transfer mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TransferMode( usart_PeriphId_t usartId, usart_TransferMode_t transferMode )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) &&
        ( 0u == ( (uint32_t)transferMode & ~( USART_CR1_TE | USART_CR1_RE ) ) ) )
    {
        USART_TypeDef * const periphReg = usart_PeriphConf[ usartId ].PeriphReg;

        /* RTS stays enabled only if the receiver stays enabled */
        retValue = Usart_Set_RtsOutput( usartId, periphReg->CR1 & ( ~USART_CR1_RE | (uint32_t)transferMode ) );

        if( USART_REQUEST_OK == retValue )
        {
            LL_USART_SetTransferDirection( periphReg, transferMode );

            retValue = Usart_Verify_RegBits( &periphReg->CR1, USART_CR1_TE | USART_CR1_RE, transferMode );
        }
        else
        {
            /* RTS output was not disabled, receiver is not changed */
        }

        if( USART_REQUEST_OK == retValue )
        {
            retValue = Usart_Set_RtsOutput( usartId, periphReg->CR1 );
        }
        else
        {
            /* Transfer mode was not configured */
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

    if( ( USART_BUS_CNT   > usartId      ) &&
        ( USART_NULL_PTR != transferMode )    )
    {
        *transferMode = LL_USART_GetTransferDirection( usart_PeriphConf[ usartId ].PeriphReg );
        retValue      = USART_REQUEST_OK;
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
 * \note UART4 / UART5 / UART7 / UART8 have no hardware flow control.
 *
 * \note Device errata "RTS is active while RE or UE = 0": RTS is driven active as soon as RTSE
 *       is set. Requested RTS flow control is stored, RTSE is set only while the USART (UE)
 *       and its receiver (RE) are enabled - \ref Usart_Set_PeriphActive,
 *       \ref Usart_Set_PeriphInactive and \ref Usart_Set_TransferMode follow the request.
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param flowControl [in]: Required hardware flow control configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_FlowControl( usart_PeriphId_t usartId, usart_FlowControl_t flowControl )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_FLOW_CONTROL_NONE == flowControl )
        {
            retValue = USART_REQUEST_OK;
        }
        else if( ( 0u == ( (uint32_t)flowControl & ~( USART_CR3_RTSE | USART_CR3_CTSE ) ) ) &&
                 ( IS_UART_HWFLOW_INSTANCE( usart_PeriphConf[ usartId ].PeriphReg )        )    )
        {
            retValue = USART_REQUEST_OK;
        }
        else
        {
            /* Invalid configuration or flow control on UART */
            retValue = USART_REQUEST_ERROR;
        }

        if( USART_REQUEST_OK == retValue )
        {
            USART_TypeDef * const periphReg = usart_PeriphConf[ usartId ].PeriphReg;
            const uint32_t        ctsValue  = (uint32_t)flowControl & USART_CR3_CTSE;

            usart_RtsRequest[ usartId ] = ( 0u != ( (uint32_t)flowControl & USART_CR3_RTSE ) ) ? USART_FUNCTION_ACTIVE : USART_FUNCTION_INACTIVE;

            /* CTS is applied directly, RTS output follows the USART and receiver state */
            LL_USART_SetHWFlowCtrl( periphReg, ctsValue | ( periphReg->CR3 & USART_CR3_RTSE ) );

            retValue = Usart_Verify_RegBits( &periphReg->CR3, USART_CR3_CTSE, ctsValue );

            if( USART_REQUEST_OK == retValue )
            {
                retValue = Usart_Set_RtsOutput( usartId, periphReg->CR1 );
            }
            else
            {
                /* CTS was not configured */
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

    if( ( USART_BUS_CNT   > usartId     ) &&
        ( USART_NULL_PTR != flowControl )    )
    {
        /* RTS is reported as requested - RTSE is set only while UE and RE are set */
        const uint32_t rtsValue = ( USART_FUNCTION_ACTIVE == usart_RtsRequest[ usartId ] ) ? USART_CR3_RTSE : 0u;

        *flowControl = (usart_FlowControl_t)( ( LL_USART_GetHWFlowCtrl( usart_PeriphConf[ usartId ].PeriphReg ) & USART_CR3_CTSE ) | rtsValue );
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
 * \note STM32F4 USART has no Driver Enable (DE) output, only inactive state is
 *       accepted.
 *
 * \param usartId [in]: USART/UART bus identification.
 * \param deState [in]: Required Driver Enable (DE) mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DriverEnableState( usart_PeriphId_t usartId, usart_DeFeatureState_t deState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT     > usartId ) &&
        ( USART_DE_DISABLED == deState )    )
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
 * \brief Returns the Driver Enable (DE) feature activation state
 *
 * \note STM32F4 USART has no Driver Enable (DE) output, state is always inactive.
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param deState [out]: Value of Driver Enable (DE) mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DriverEnableState( usart_PeriphId_t usartId, usart_DeFeatureState_t * const deState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != deState )    )
    {
        *deState = USART_DE_DISABLED;
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
 * \note STM32F4 USART has no Driver Enable (DE) output, only default polarity
 *       (active high) is accepted.
 *
 * \param usartId    [in]: USART/UART bus identification.
 * \param dePolarity [in]: Logic level for active state of Driver Enable (DE) pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DriverEnablePolarity( usart_PeriphId_t usartId, usart_DePolarity_t dePolarity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT        > usartId    ) &&
        ( USART_DE_ACTIVE_HIGH == dePolarity )    )
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
 * \brief Returns the Driver Enable (DE) active logic level
 *
 * \note STM32F4 USART has no Driver Enable (DE) output, default polarity
 *       (active high) is returned.
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param dePolarity [out]: Logic level for active state of Driver Enable (DE) pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DriverEnablePolarity( usart_PeriphId_t usartId, usart_DePolarity_t * const dePolarity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId    ) &&
        ( USART_NULL_PTR != dePolarity )    )
    {
        *dePolarity = USART_DE_ACTIVE_HIGH;
        retValue    = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Configures assertion and de-assertion times of Driver Enable (DE) pin.
 *
 * \note STM32F4 USART has no Driver Enable (DE) output, only zero times are
 *       accepted.
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

    if( ( USART_BUS_CNT > usartId      ) &&
        ( 0u            == assertTime   ) &&
        ( 0u            == deassertTime )    )
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
 * \brief Returns assertion and de-assertion times of Driver Enable (DE) pin.
 *
 * \note STM32F4 USART has no Driver Enable (DE) output, zero times are returned.
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

    if( ( USART_BUS_CNT   > usartId      ) &&
        ( USART_NULL_PTR != assertTime   ) &&
        ( USART_NULL_PTR != deassertTime )    )
    {
        *assertTime   = 0u;
        *deassertTime = 0u;

        retValue = USART_REQUEST_OK;
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
 * \note Baud-rate register depends on over-sampling mode, baud-rate shall be
 *       configured again after change of over-sampling mode.
 *
 * \param usartId          [in]: USART/UART bus identification.
 * \param oversamplingMode [in]: Required over-sampling mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Oversampling( usart_PeriphId_t usartId, usart_Oversampling_t oversamplingMode )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT > usartId ) &&
        ( ( USART_OVERSAMPLING_16 == oversamplingMode ) ||
          ( USART_OVERSAMPLING_8  == oversamplingMode )    ) )
    {
        LL_USART_SetOverSampling( usart_PeriphConf[ usartId ].PeriphReg, oversamplingMode );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_OVER8, oversamplingMode );
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

    if( ( USART_BUS_CNT   > usartId          ) &&
        ( USART_NULL_PTR != oversamplingMode )    )
    {
        *oversamplingMode = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].PeriphReg );
        retValue          = USART_REQUEST_OK;
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
 * In single-wire half-duplex mode TX and RX lines are internally connected, RX
 * pin is not used. Transmitted data are received by own receiver.
 *
 * \param usartId         [in]: USART/UART bus identification.
 * \param halfDuplexState [in]: Required half-duplex state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_HalfDuplexState( usart_PeriphId_t usartId, usart_HalfDuplex_t halfDuplexState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_HALF_DUPLEX_INACTIVE == halfDuplexState )
        {
            LL_USART_DisableHalfDuplex( usart_PeriphConf[ usartId ].PeriphReg );

            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_HDSEL, 0u );
        }
        else if( ( USART_HALF_DUPLEX_ACTIVE == halfDuplexState                                 ) &&
                 ( IS_UART_HALFDUPLEX_INSTANCE( usart_PeriphConf[ usartId ].PeriphReg ) )    )
        {
            LL_USART_EnableHalfDuplex( usart_PeriphConf[ usartId ].PeriphReg );

            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_HDSEL, USART_CR3_HDSEL );
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

    if( ( USART_BUS_CNT   > usartId         ) &&
        ( USART_NULL_PTR != halfDuplexState )    )
    {
        uint32_t halfDuplexStateReg = LL_USART_IsEnabledHalfDuplex( usart_PeriphConf[ usartId ].PeriphReg );

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
 * \note STM32F4 USART has no receiver timeout, request always fails.
 *
 * \param usartId        [in]: USART/UART bus identification.
 * \param timeoutBitsCnt [in]: Count of bits triggering receiver timeout flag
 * \return Returns always error - feature is not supported.
 */
usart_RequestState_t Usart_Set_RxTimeoutActive( usart_PeriphId_t usartId, usart_RxTimeout_t timeoutBitsCnt )
{
    ( void ) usartId;
    ( void ) timeoutBitsCnt;

    return ( USART_REQUEST_ERROR );
}


/**
 * \brief De-activates receive timeout feature.
 *
 * \note STM32F4 USART has no receiver timeout, feature is always inactive.
 *
 * \param usartId [in]: USART/UART bus identification.
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutInactive( usart_PeriphId_t usartId )
{
    return ( ( USART_BUS_CNT > usartId ) ? USART_REQUEST_OK : USART_REQUEST_ERROR );
}


/**
 * \brief Returns activation state of receiver timeout feature
 *
 * \note STM32F4 USART has no receiver timeout, feature is always inactive.
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param reqState [out]: Receiver timeout feature activation status
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxTimeoutState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        *reqState = USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * \note STM32F4 USART does not support inverted pin levels, only standard
 *       levels are accepted.
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

    if( ( USART_BUS_CNT         > usartId     ) &&
        ( USART_RX_PIN_STANDARD == rxPinLevels ) &&
        ( USART_TX_PIN_STANDARD == txPinLevels )    )
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
 * \brief Returns pins operation modes
 *
 * \note STM32F4 USART does not support inverted pin levels, standard levels
 *       are returned.
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

    if( ( USART_BUS_CNT   > usartId     ) &&
        ( USART_NULL_PTR != rxPinLevels ) &&
        ( USART_NULL_PTR != txPinLevels )    )
    {
        *rxPinLevels = USART_RX_PIN_STANDARD;
        *txPinLevels = USART_TX_PIN_STANDARD;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Reads transmission register address (used as DMA destination)
 *
 * \note STM32F4 USART uses one data register (DR) for transmission and reception.
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param regAddr [out]: Address of transmission register
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxRegisterAddr( usart_PeriphId_t usartId, usart_TxRegAddr_t * const regAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != regAddr )    )
    {
        *regAddr = (usart_TxRegAddr_t)(uintptr_t)&usart_PeriphConf[ usartId ].PeriphReg->DR;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Reads reception register address (used as DMA source)
 *
 * \note STM32F4 USART uses one data register (DR) for transmission and reception.
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
        *regAddr = (usart_RxRegAddr_t)(uintptr_t)&usart_PeriphConf[ usartId ].PeriphReg->DR;

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
 * \param txData  [in]: Data to be transmitted (8 bits)
 */
void Usart_SendData( usart_PeriphId_t usartId, usart_TxData_t txData )
{
    if( USART_BUS_CNT > usartId )
    {
        LL_USART_TransmitData8( usart_PeriphConf[ usartId ].PeriphReg, txData );
    }
}


/**
 * \brief Reads data from reception register
 *
 * \note Read of data register clears flags RXNE, and with preceding read of
 *       status register also PE, FE, NE, ORE and IDLE.
 *
 * \param usartId [in]: USART/UART bus identification.
 * \return Received data value (0 for invalid bus identification)
 */
usart_RxData_t Usart_ReadData( usart_PeriphId_t usartId )
{
    usart_RxData_t rxData = 0u;

    if( USART_BUS_CNT > usartId )
    {
        rxData = LL_USART_ReceiveData8( usart_PeriphConf[ usartId ].PeriphReg );
    }

    return ( rxData );
}


/*------------------------------ Data handling -------------------------------*/

/**
 * \brief Configures data handling (transmission / reception mode, buffers, callbacks)
 *
 * The previous data handling is released (DMA streams, USART interrupt), the configuration
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
            /* DMA moves the data - count is derived from the DMA stream remaining count */
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
 * \brief De-activates global Interrupt Requests (IRQ) for USART/UART peripheral.
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
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

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
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
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

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_DMAT, USART_CR3_DMAT );
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
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaTxRequestInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_DMAT, 0u );
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
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation status of transfer request state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DmaTxReqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * For reception is the DMA trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaRxRequestActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_DMAR, USART_CR3_DMAR );
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
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaRxRequestInactive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_DMAR, 0u );
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
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation of transfer request state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DmaRxReqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * Stale data in receive register are discarded before activation.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        ( void ) LL_USART_ReceiveData9( usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_EnableIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_RXNEIE, USART_CR1_RXNEIE );
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

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_RXNEIE, 0u );
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * \note TXE interrupt is pending as long as transmit register is empty. The
 *       callback shall write next data or deactivate the interrupt. Without
 *       configured callback the interrupt is deactivated by the module.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_TXEIE, USART_CR1_TXEIE );
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

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_TXEIE, 0u );
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * Pending transmission complete flag is cleared before activation.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_TC( usart_PeriphConf[ usartId ].PeriphReg );

        LL_USART_EnableIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_TCIE, USART_CR1_TCIE );
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

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_TCIE, 0u );
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * Pending idle flag is cleared before activation (read of status and data
 * register - pending received data are discarded).
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIrqActive( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        if( 0u != LL_USART_IsActiveFlag_IDLE( usart_PeriphConf[ usartId ].PeriphReg ) )
        {
            LL_USART_ClearFlag_IDLE( usart_PeriphConf[ usartId ].PeriphReg );
        }

        LL_USART_EnableIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_IDLEIE, USART_CR1_IDLEIE );
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

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_IDLEIE, 0u );
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

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_IDLE( usart_PeriphConf[ usartId ].PeriphReg );

        *reqState = ( 0u != regValue ) ? USART_FLAG_ACTIVE : USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * \note STM32F4 USART has no receiver timeout, request always fails.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return Returns always error - feature is not supported.
 */
usart_RequestState_t Usart_Set_RxTimeoutIrqActive( usart_PeriphId_t usartId )
{
    ( void ) usartId;

    return ( USART_REQUEST_ERROR );
}


/**
 * \brief Deactivates Receive Timeout (RTO) interrupt request.
 *
 * \note STM32F4 USART has no receiver timeout, interrupt is always inactive.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIrqInactive( usart_PeriphId_t usartId )
{
    return ( ( USART_BUS_CNT > usartId ) ? USART_REQUEST_OK : USART_REQUEST_ERROR );
}


/**
 * \brief Returns activation state of Receive Timeout (RTO) interrupt request.
 *
 * \note STM32F4 USART has no receiver timeout, interrupt is always inactive.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state Receive Timeout (RTO) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxTimeoutIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        *reqState = USART_FLAG_INACTIVE;
        retValue  = USART_REQUEST_OK;
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
 * error interrupt is handled together with other errors. Pending errors are
 * cleared before activation (read of status and data register - pending
 * received data are discarded).
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
        if( 0u != ( usart_PeriphConf[ usartId ].PeriphReg->SR & USART_SR_ERROR_MASK ) )
        {
            ( void ) LL_USART_ReceiveData9( usart_PeriphConf[ usartId ].PeriphReg );
        }

        LL_USART_EnableIT_ERROR( usart_PeriphConf[ usartId ].PeriphReg );
        LL_USART_EnableIT_PE( usart_PeriphConf[ usartId ].PeriphReg );

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_EIE, USART_CR3_EIE );

        if( USART_REQUEST_OK == retValue )
        {
            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_PEIE, USART_CR1_PEIE );
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
 * De-activation of the parity error interrupt is handled together with other
 * errors.
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

        retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR3, USART_CR3_EIE, 0u );

        if( USART_REQUEST_OK == retValue )
        {
            retValue = Usart_Verify_RegBits( &usart_PeriphConf[ usartId ].PeriphReg->CR1, USART_CR1_PEIE, 0u );
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
 * \note  Error interrupt is active when both error interrupt (framing error,
 *        overrun error, noise) and parity error interrupt are active.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of error interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_ErrorIrqState( usart_PeriphId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId  ) &&
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
 * Pin is configured in alternate function mode with pull-up (idle level of
 * not connected line).
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitRxGpio( usart_RxPin_t pinId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_RX_PIN_UNUSED != pinId )
    {
        retValue = Usart_InitGpioPin( (uint32_t)pinId, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_UP );
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
 * Pin is configured in alternate function push-pull mode without pull resistor.
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitTxGpio( usart_TxPin_t pinId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_TX_PIN_UNUSED != pinId )
    {
        retValue = Usart_InitGpioPin( (uint32_t)pinId, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_NONE );
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
 * \note STM32F4 USART has no Driver Enable (DE) output, request always fails.
 *
 * \param pinId [in]: Pin identification
 * \return Returns always error - feature is not supported.
 */
usart_RequestState_t Usart_InitDeGpio( usart_DePin_t pinId )
{
    ( void ) pinId;

    return ( USART_REQUEST_ERROR );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Waits until configuration bits of register reach expected value.
 *
 * \param regAddr       [in]: Address of the register
 * \param bitMask       [in]: Mask of verified bits
 * \param expectedValue [in]: Expected value of masked bits
 * \return State of request execution. Returns "OK" if bits reached expected
 *         value, otherwise return error (timeout).
 */
static usart_RequestState_t Usart_Verify_RegBits( volatile const uint32_t * const regAddr, uint32_t bitMask, uint32_t expectedValue )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    for( usart_TimeoutCnt_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
    {
        if( ( *regAddr & bitMask ) == ( expectedValue & bitMask ) )
        {
            retValue = USART_REQUEST_OK;
            break;
        }
        else
        {
            /* Configuration has not yet been changed, keep return state as error */
            retValue = USART_REQUEST_ERROR;
        }
    }

    return ( retValue );
}


/**
 * \brief Enables RTS output (RTSE) only if RTS flow control is requested and the USART and its
 *        receiver are enabled, otherwise disables it
 *
 * Device errata "RTS is active while RE or UE = 0": the RTS line is driven active as soon as RTSE
 * is set, even if the USART (UE = 0) or the receiver (RE = 0) is disabled.
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param cr1Value [in]: CR1 value (UE, RE) the RTS output has to correspond to
 * \return State of request execution. Returns "OK" if RTSE reached the required
 *         state, otherwise return error.
 */
static usart_RequestState_t Usart_Set_RtsOutput( usart_PeriphId_t usartId, uint32_t cr1Value )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        USART_TypeDef * const periphReg  = usart_PeriphConf[ usartId ].PeriphReg;
        const uint32_t        rxOnMask   = USART_CR1_UE | USART_CR1_RE;
        const uint32_t        rtsValue   = ( ( USART_FUNCTION_ACTIVE == usart_RtsRequest[ usartId ] ) &&
                                             ( rxOnMask              == ( cr1Value & rxOnMask )    )    ) ? USART_CR3_RTSE : 0u;

        if( 0u != rtsValue )
        {
            LL_USART_EnableRTSHWFlowCtrl( periphReg );
        }
        else
        {
            LL_USART_DisableRTSHWFlowCtrl( periphReg );
        }

        retValue = Usart_Verify_RegBits( &periphReg->CR3, USART_CR3_RTSE, rtsValue );
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Initializes GPIO pin of USART/UART peripheral in alternate function mode
 *
 * \param pinCode [in]: Encoded pin (\ref USART_PIN_BIT_MASK_ENCODE - port, pin, alternate function)
 * \param outType [in]: Output type of the pin
 * \param pull    [in]: Pull resistor configuration of the pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static usart_RequestState_t Usart_InitGpioPin( uint32_t pinCode, gpio_PinOutputType_t outType, gpio_PinPullCfg_t pull )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        gpioConfig;

    gpioConfig.PortId         = (gpio_PortId_t)USART_BIT_MASK_DECODE_PORT( pinCode );
    gpioConfig.PinId          = (gpio_PinId_t)USART_BIT_MASK_DECODE_PIN( pinCode );
    gpioConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    gpioConfig.PinPull        = pull;
    gpioConfig.PinSpeed       = GPIO_PIN_SPEED_HIGH;
    gpioConfig.PinOutType     = outType;
    gpioConfig.PinAltFunction = (gpio_AltFunction_t)USART_BIT_MASK_DECODE_AF( pinCode );
    gpioConfig.PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    gpioInitState = Gpio_Init( &gpioConfig );

    if( GPIO_REQUEST_OK == gpioInitState )
    {
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
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
 * \brief Discards stale received data and clears reception flags (RXNE, PE, FE, NE, ORE, IDLE)
 *        before a reception start
 *
 * STM32F4: the flags are cleared by read of SR followed by read of DR.
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

        (void)LL_USART_ReadReg( periphReg, SR );
        (void)LL_USART_ReceiveData8( periphReg );

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

            if( ( 0u       != halfSize                           ) &&
                ( halfSize == usart_XferContext[ usartId ].RxIdx )    )
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
            /* Next byte is stored to the buffer start (DMA stream runs in circular mode) */
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
 * \brief Reports every pending reception error (PE, FE, NE, ORE) of a status register snapshot
 *
 * \note  STM32F4: the error flags are cleared by the read of SR (snapshot) followed by the read
 *        of DR - DR is read by the caller (received byte) or by the DMA stream.
 *
 * \param usartId [in]: USART/UART peripheral identification, value from \ref usart_PeriphId_t
 * \param srFlags [in]: Snapshot of USART status register (SR)
 *
 * \return Function processing state. Returns \ref USART_REQUEST_OK if request
 *         was processed without problems (also if no error is pending). Otherwise returns
 *         \ref USART_REQUEST_ERROR.
 */
usart_RequestState_t Usart_Set_XferRxErrors( usart_PeriphId_t usartId, uint32_t srFlags )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        retState = USART_REQUEST_OK;

        if( 0u != ( USART_SR_ERROR_MASK & srFlags ) )
        {
            for( usart_XferErrorId_t errorId = USART_XFER_ERROR_PARITY; USART_XFER_ERROR_CNT > errorId; errorId ++ )
            {
                const uint32_t errorFlag = (uint32_t)usart_RxErrorFlagLut[ errorId ];

                if( ( USART_ERROR_NONE != errorFlag               ) &&
                    ( 0u               != ( errorFlag & srFlags ) )    )
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
static void Usart_Usart1_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_1 );
}
#endif /* USART1 */

#ifdef USART2
/**
 * \brief USART2 Interrupt handler
 */
static void Usart_Usart2_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_2 );
}
#endif /* USART2 */

#ifdef USART3
/**
 * \brief USART3 Interrupt handler
 */
static void Usart_Usart3_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_3 );
}
#endif /* USART3 */

#ifdef UART4
/**
 * \brief UART4 Interrupt handler
 */
static void Usart_Uart4_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_4 );
}
#endif /* UART4 */

#ifdef UART5
/**
 * \brief UART5 Interrupt handler
 */
static void Usart_Uart5_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_5 );
}
#endif /* UART5 */

#ifdef USART6
/**
 * \brief USART6 Interrupt handler
 */
static void Usart_Usart6_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_6 );
}
#endif /* USART6 */

#ifdef UART7
/**
 * \brief UART7 Interrupt handler
 */
static void Usart_Uart7_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_7 );
}
#endif /* UART7 */

#ifdef UART8
/**
 * \brief UART8 Interrupt handler
 */
static void Usart_Uart8_IsrHandler( void )
{
    Usart_GlobalIsrHandler( USART_BUS_8 );
}
#endif /* UART8 */

/* ================================ TASKS =================================== */
