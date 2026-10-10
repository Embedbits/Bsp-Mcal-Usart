/**
 * \author Mr.Nobody
 * \file Test_Usart.c
 * \ingroup Usart
 * \brief Unit tests of Universal Synchronous/Asynchronous Receiver-Transmitter (USART) module.
 *
 * Usart.c, Usart_Isr.c, Usart_Poll.c and Usart_Dma.c are compiled unchanged with
 * real LL drivers. USART / UART registers of STM32F7 are emulated by RegMem, RCC, NVIC,
 * GPIO and DMA modules are mocked by CMock (DMA functions are replaced by recording stubs
 * in the data handling tests). DMA streams of the tests follow the STM32F7 DMA request
 * mapping (USART1: TX DMA2 stream 7, RX DMA2 stream 2 / 5, channel 4). USART ISR registered in NVIC is captured by stub and called
 * directly to test interrupt data handling.
 *
 * \note Emulated registers are plain memory. ISR flags are not cleared by
 *       ICR writes and data register reads, tests preset the flags the module
 *       waits for and check the written clear bits.
 *
 * \note Data handling context of the module is static - setUp() releases the
 *       data handling of the previous test (Usart_Deinit of all peripherals with
 *       ignored mocks) and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "UtCommon.h"                       /* Common test helpers            */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Usart_Port.h"                     /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_usart.h"                    /* USART registers definition     */
/* ============================= TYPEDEFS =================================== */

/** \brief Record of DMA calls of one channel */
typedef struct
{
    uint32_t         ActiveCnt;   /**< Count of Dma_Set_TransferActive() calls    */
    uint32_t         InactiveCnt; /**< Count of Dma_Set_TransferInactive() calls  */
    dma_MemoryAddr_t MemoryAddr;  /**< Last memory address                        */
    dma_DataCount_t  DataCount;   /**< Last data count                            */
    uint32_t         HtIrq;       /**< Half transfer interrupt state (1 enabled)  */
    uint32_t         TcIrq;       /**< Transfer complete interrupt state          */
    uint32_t         TeIrq;       /**< Transfer error interrupt state             */
    uint32_t         NvicIrq;     /**< Channel interrupt in NVIC state            */
}   ut_UsartDmaChannel_t;

/** Call of a GPIO function on the RTS pin with the USART register state at the call */
typedef struct
{
    uint32_t Event;               /**< UT_USART_RTS_EV_x                          */
    uint32_t Cr1;                 /**< CR1 at the call (UE, RE)                   */
    uint32_t Cr3;                 /**< CR3 at the call (RTSE)                     */
}   ut_UsartRtsEvent_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static nvic_RequestState_t  Ut_Usart_NvicSetHandlerStub ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static rcc_RequestState_t   Ut_Usart_RccGetClkStub      ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_Usart_RccGetStateStub    ( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt );
static void                 Ut_Usart_Ignore_PeriphMocks ( void );
static void                 Ut_Usart_Release            ( void );
static usart_BusConfig_t    Ut_Usart_Get_BusConfig      ( void );
static usart_DataConfig_t   Ut_Usart_Get_DataConfig     ( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize );
static void                 Ut_Usart_Init               ( usart_BusConfig_t * const busConfig );
static void                 Ut_Usart_Call_Isr           ( uint32_t isrFlags );
static void                 Ut_Usart_Stub_DmaMocks      ( void );
static void                 Ut_Usart_Set_DataConfig     ( const usart_DataConfig_t * const dataConfig );
static gpio_RequestState_t  Ut_Usart_GpioSetLevelStub   ( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_PinLevel_t pinLevel, int callCnt );
static gpio_RequestState_t  Ut_Usart_GpioSetModeStub    ( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_PinMode_t pinType, int callCnt );
static void                 Ut_Usart_Add_RtsEvent       ( uint32_t event );
static void                 Ut_Usart_Init_RtsPin        ( void );
static void                 Ut_Usart_Set_RtsActive      ( void );

static nvic_RequestState_t  Ut_Usart_NvicAnyHandlerStub ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaInitStub        ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaActiveStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaInactiveStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaMemoryAddrStub  ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaDataCountStub   ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaGetDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaHtIrqOnStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaHtIrqOffStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTcIrqOnStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTcIrqOffStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTeIrqOnStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTeIrqOffStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaNvicOnStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaNvicOffStub     ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static ut_UsartDmaChannel_t* Ut_Usart_Get_DmaChannel    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel );

static void                 Ut_Usart_TxCompleteCallback ( void );
static void                 Ut_Usart_RxHalfCallback     ( void );
static void                 Ut_Usart_RxCompleteCallback ( void );
static void                 Ut_Usart_RxEndCallback      ( usart_RxDataCnt_t rxCnt );
static void                 Ut_Usart_ErrorCallback      ( usart_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** USART peripheral used by tests (available on all supported MCUs) */
#define UT_USART_BUS                        ( USART_BUS_1 )
#define UT_USART_REG                        ( USART1 )
#define UT_USART_RCC                        ( RCC_PERIPH_USART1_PCLK2 )
#define UT_USART_NVIC                       ( NVIC_PERIPH_IRQ_USART1 )

/** DMA streams used by data handling tests (USART1 requests: DMA2 stream 7 TX, stream 2 RX, channel 4) */
#define UT_USART_DMA                        ( DMA_PERIPH_2 )
#define UT_USART_DMA_TX_CH                  ( DMA_STREAM_7 )
#define UT_USART_DMA_RX_CH                  ( DMA_STREAM_2 )

/** DMA channel selection of USART1 requests */
#define UT_USART_DMA_CHSEL                  ( DMA_REQ_CHANNEL_4 )

/** DMA stream list items of USART1 used by data handling tests (DMA2 stream 7 TX, stream 2 RX) */
#define UT_USART_TX_DMA                     ( USART_TX_DMA_BUS1_DMA2_STREAM7 )
#define UT_USART_RX_DMA                     ( USART_RX_DMA_BUS1_DMA2_STREAM2 )

/** Encoded DMA stream from the bus index, DMA peripheral index, stream number and channel selection number
 *  (bit-fields written independently of USART_DMA_ENCODE) */
#define UT_USART_DMA_CODE( BUS, DMA, STREAM, CHSEL )    ( ( (BUS) << 15u ) | ( (DMA) << 10u ) | ( (STREAM) << 5u ) | (CHSEL) )

/** Maximal count of captured DMA configurations */
#define UT_USART_DMA_CFG_CNT                ( 4u )

/** Maximal count of DMA streams of one DMA peripheral */
#define UT_USART_DMA_CHANNELS               ( 8u )

/** Default kernel clock returned by RCC mock [Hz] */
#define UT_USART_CLK_HZ                     ( 64000000u )

/** Baud rate of test configurations */
#define UT_USART_BAUDRATE                   ( 115200u )

/** Size of receive buffer */
#define UT_USART_RX_SIZE                    ( 8u )

/** Interrupt priority of test configurations */
#define UT_USART_PRIO                       ( 6u )

/** RTS / DE pin of USART1 used by the RTS tests (PA12) */
#define UT_USART_RTS_PIN                    ( USART_DE_PIN_BUS1_PA12 )
#define UT_USART_RTS_PORT                   ( GPIO_PORT_A )
#define UT_USART_RTS_PIN_ID                 ( GPIO_PIN_ID_12 )

/** Maximal count of recorded RTS pin events */
#define UT_USART_RTS_EV_CNT                 ( 8u )

/** RTS pin events recorded by the GPIO stubs */
#define UT_USART_RTS_EV_LEVEL_HIGH          ( 1u )
#define UT_USART_RTS_EV_MODE_OUTPUT         ( 2u )
#define UT_USART_RTS_EV_MODE_AF             ( 3u )
#define UT_USART_RTS_EV_OTHER               ( 4u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the USART */
static nvic_IsrCallback_t       utUsart_Isr;

/** ISR registered in NVIC for any USART/UART peripheral */
static nvic_IsrCallback_t       utUsart_AnyIsr;

/** NVIC line expected by the handler registration stub */
static nvic_PeriphIrqList_t     utUsart_Nvic;

/** Kernel clock returned by RCC stub [Hz] */
static rcc_FreqHz_t             utUsart_ClkHz;

/** DMA configurations passed to Dma_Init() */
static dma_ConfigStruct_t       utUsart_DmaConfig[ UT_USART_DMA_CFG_CNT ];

/** Count of Dma_Init() calls */
static uint32_t                 utUsart_DmaInitCnt;

/** Return state of Dma_Init() stub */
static dma_RequestState_t       utUsart_DmaInitState;

/** Remaining count returned by Dma_Get_DataCount() stub */
static dma_DataCount_t          utUsart_DmaRemaining;

/** Records of DMA channel calls */
static ut_UsartDmaChannel_t     utUsart_DmaChannel[ DMA_PERIPH_CNT ][ UT_USART_DMA_CHANNELS ];

/** Receive buffer */
static usart_RxData_t           utUsart_RxBuf[ UT_USART_RX_SIZE ];

/** Transmit data */
static const usart_TxData_t     utUsart_TxBuf[ 4u ] = { 0x11u, 0x22u, 0x33u, 0x44u };

/** Counts of callback calls */
static uint32_t                 utUsart_TxCompleteCnt;
static uint32_t                 utUsart_RxHalfCnt;
static uint32_t                 utUsart_RxCompleteCnt;
static uint32_t                 utUsart_RxEndCnt;
static uint32_t                 utUsart_ErrorCnt;

/** Parameters of the last callback calls */
static usart_RxDataCnt_t        utUsart_RxEndBytes;
static usart_XferErrorId_t      utUsart_LastError;

/** GPIO calls on the RTS pin */
static ut_UsartRtsEvent_t       utUsart_RtsEvent[ UT_USART_RTS_EV_CNT ];
static uint32_t                 utUsart_RtsEventCnt;

/** Return state of the RTS pin GPIO stubs */
static gpio_RequestState_t      utUsart_RtsGpioState;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    Ut_Usart_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utUsart_Isr           = NULL;
    utUsart_TxCompleteCnt = 0u;
    utUsart_RxHalfCnt     = 0u;
    utUsart_RxCompleteCnt = 0u;
    utUsart_RxEndCnt      = 0u;
    utUsart_ErrorCnt      = 0u;
    utUsart_RxEndBytes    = 0u;
    utUsart_LastError     = USART_XFER_ERROR_CNT;
    utUsart_AnyIsr        = NULL;
    utUsart_Nvic          = UT_USART_NVIC;
    utUsart_ClkHz         = UT_USART_CLK_HZ;
    utUsart_DmaInitCnt    = 0u;
    utUsart_DmaInitState  = DMA_REQUEST_OK;
    utUsart_DmaRemaining  = 0u;
    utUsart_RtsEventCnt   = 0u;
    utUsart_RtsGpioState  = GPIO_REQUEST_OK;

    for( uint32_t byteIdx = 0u; UT_USART_RX_SIZE > byteIdx; byteIdx++ )
    {
        utUsart_RxBuf[ byteIdx ] = 0u;
    }

    for( uint32_t dmaIdx = 0u; DMA_PERIPH_CNT > dmaIdx; dmaIdx++ )
    {
        for( uint32_t chIdx = 0u; UT_USART_DMA_CHANNELS > chIdx; chIdx++ )
        {
            utUsart_DmaChannel[ dmaIdx ][ chIdx ] = (ut_UsartDmaChannel_t){ 0u };
        }
    }
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Usart_Get_ModuleVersion() returns version of the module.
 *
 * \details Reads the module version structure.
 *
 * \par Expected results
 * - Version is 1.0.0 (Major 1, Minor 0, Patch 0).
 */
void Ut_Usart_Get_ModuleVersion_ReturnsVersion( void )
{
    const usart_ModuleVersion_t version = Usart_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 1u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* =========================== DEFAULT CONFIG =============================== */

/**
 * \brief   Usart_Get_DefaultConfig() fills default configuration.
 *
 * \details Reads default configuration.
 *
 * \par Expected results
 * - USART_REQUEST_OK, USART1, 115200 Bd, 8 data bits, 1 stop bit, no parity, TX + RX.
 * - No flow control, driver enable disabled, oversampling 8, half-duplex inactive,
 *   receive timeout 0.
 * - No data handling configuration, RX / TX / DE pins unused.
 */
void Ut_Usart_Get_DefaultConfig_ReturnsDefaults( void )
{
    usart_BusConfig_t config;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( USART_BUS_1,                config.PeriphId );
    TEST_ASSERT_EQUAL_UINT32( 115200u,             config.BaudRate );
    TEST_ASSERT_EQUAL( USART_DATA_WIDTH_8,         config.DataWidth );
    TEST_ASSERT_EQUAL( USART_STOP_BITS_1,          config.StopBits );
    TEST_ASSERT_EQUAL( USART_PARITY_NONE,          config.Parity );
    TEST_ASSERT_EQUAL( USART_TRANSFER_MODE_TX_RX,  config.TransferMode );
    TEST_ASSERT_EQUAL( USART_FLOW_CONTROL_NONE,    config.HwFlowControl );
    TEST_ASSERT_EQUAL( USART_DE_DISABLED,          config.DriverEnableMode );
    TEST_ASSERT_EQUAL( USART_OVERSAMPLING_8,       config.Oversampling );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_INACTIVE, config.HalfDuplex );
    TEST_ASSERT_EQUAL_UINT32( 0u,                  config.RxTimeoutValue );
    TEST_ASSERT_NULL( config.DataConfig );
    TEST_ASSERT_EQUAL( USART_RX_PIN_UNUSED,        config.BusRxPin );
    TEST_ASSERT_EQUAL( USART_TX_PIN_UNUSED,        config.BusTxPin );
    TEST_ASSERT_EQUAL( USART_DE_PIN_UNUSED,        config.BusDePin );
    TEST_ASSERT_EQUAL( USART_CTS_PIN_UNUSED,       config.BusCtsPin );
    TEST_ASSERT_EQUAL( USART_RTS_PIN_UNUSED,       config.BusRtsPin );
}


/**
 * \brief   Usart_Get_DefaultConfig() rejects NULL pointer.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Get_DefaultConfig_NullPtr_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DefaultConfig( NULL ) );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Usart_Init() rejects NULL configuration.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, no RCC / GPIO / NVIC call (strict mocks).
 */
void Ut_Usart_Init_NullConfig_ReturnsErrorWithoutAccess( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( NULL ) );
}


/**
 * \brief   Usart_Init() rejects peripheral out of range.
 *
 * \details Initializes configuration with USART_BUS_CNT.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, no RCC / GPIO / NVIC call (strict mocks).
 */
void Ut_Usart_Init_InvalidPeriph_ReturnsErrorWithoutAccess( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.PeriphId = USART_BUS_CNT;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
}


/**
 * \brief   Usart_Init() enables clock, resets and configures the peripheral.
 *
 * \details Initializes USART1 with default configuration, RCC mock reports inactive
 *          clock and 64 MHz kernel clock.
 *
 * \par Expected results
 * - Clock enabled, peripheral reset activated and released (in this order).
 * - USART_REQUEST_OK, CR1: UE, TE, RE, OVER8 set, 8 data bits, no parity.
 * - 1 stop bit, no half-duplex / driver enable / flow control, BRR is not 0.
 */
void Ut_Usart_Init_DefaultConfig_ClockResetAndRegisters( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );

    /* Enabled, transmitter and receiver, 8 data bits, no parity, 1 stop bit, oversampling 8 */
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8,
                             UT_USART_REG->CR1 & ( USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8 | USART_CR1_M | USART_CR1_PCE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR2 & USART_CR2_STOP );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_HDSEL | USART_CR3_DEM | USART_CR3_RTSE | USART_CR3_CTSE ) );
    TEST_ASSERT_NOT_EQUAL( 0u, UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Init() does not enable already active clock.
 *
 * \details RCC mock reports active clock of USART1, the module is initialized with
 *          default configuration.
 *
 * \par Expected results
 * - Rcc_Set_PeriphActive() is not called, peripheral reset activated and released.
 * - USART_REQUEST_OK.
 */
void Ut_Usart_Init_ClockAlreadyActive_ClockNotActivatedAgain( void )
{
    static rcc_FunctionState_t clockState = RCC_FUNCTION_ACTIVE;
    usart_BusConfig_t          config     = Ut_Usart_Get_BusConfig();

    Rcc_Get_PeriphState_ExpectAndReturn( UT_USART_RCC, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &clockState );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
}


/**
 * \brief   Usart_Init() stops on peripheral reset error.
 *
 * \details RCC mock returns error on reset activation.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, peripheral is not enabled (CR1.UE = 0).
 */
void Ut_Usart_Init_ResetError_ReturnsErrorPeripheralNotEnabled( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() reports GPIO pin initialization error.
 *
 * \details Gpio_Init() mock returns error for the RX pin, then the clock activation
 *          fails before the pins are configured.
 * \note    Bug AB#631: result of the pin initialization was ignored - OK was returned
 *          and the peripheral enabled without working pins.
 *
 * \par Expected results
 * - GPIO error: USART_REQUEST_ERROR, TX pin and peripheral reset not processed,
 *   peripheral not enabled.
 * - Clock error: USART_REQUEST_ERROR, Gpio_Init() not called.
 */
void Ut_Usart_Init_GpioError_ReturnsErrorPeripheralNotEnabled( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.BusTxPin = USART_TX_PIN_BUS1_PA9;
    config.BusRxPin = USART_RX_PIN_BUS1_PA10;

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() configures TX and RX pins.
 *
 * \details Initializes USART1 with TX pin PA9 and RX pin PA10.
 *
 * \par Expected results
 * - Gpio_Init() called twice (after clock enabling, before peripheral reset).
 * - USART_REQUEST_OK.
 */
void Ut_Usart_Init_TxRxPins_GpioConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.BusTxPin = USART_TX_PIN_BUS1_PA9;
    config.BusRxPin = USART_RX_PIN_BUS1_PA10;

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
}


/**
 * \brief   Clear To Send (CTS) pin is configured in alternate function mode.
 *
 * \details Initializes the CTS pin PA11 of USART1 (AF7), then Gpio_Init() returns error.
 *
 * \par Expected results
 * - Gpio_Init() called for PA11 with alternate function 7, push-pull, no pull, medium speed.
 * - USART_REQUEST_OK, then USART_REQUEST_ERROR on the GPIO error.
 */
void Ut_Usart_InitCtsGpio_ConfiguresAlternateFunction( void )
{
    static gpio_Config_t expected;

    expected                = (gpio_Config_t){ 0 };
    expected.PortId         = GPIO_PORT_A;
    expected.PinId          = GPIO_PIN_ID_11;
    expected.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expected.PinPull        = GPIO_PIN_PULL_NONE;
    expected.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    expected.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expected.PinAltFunction = GPIO_ALT_FUNC_7;

    Gpio_Init_ExpectAndReturn( &expected, GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitCtsGpio( USART_CTS_PIN_BUS1_PA11 ) );

    Gpio_Init_ExpectAndReturn( &expected, GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitCtsGpio( USART_CTS_PIN_BUS1_PA11 ) );
}


/**
 * \brief   Request To Send (RTS) pin is configured in alternate function mode.
 *
 * \details Initializes the RTS pin PA12 of USART1 (AF7), then Gpio_Init() returns error.
 *
 * \par Expected results
 * - Gpio_Init() called for PA12 with alternate function 7, push-pull, no pull, medium speed.
 * - USART_REQUEST_OK, then USART_REQUEST_ERROR on the GPIO error.
 */
void Ut_Usart_InitRtsGpio_ConfiguresAlternateFunction( void )
{
    static gpio_Config_t expected;

    expected                = (gpio_Config_t){ 0 };
    expected.PortId         = GPIO_PORT_A;
    expected.PinId          = GPIO_PIN_ID_12;
    expected.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expected.PinPull        = GPIO_PIN_PULL_NONE;
    expected.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    expected.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expected.PinAltFunction = GPIO_ALT_FUNC_7;

    Gpio_Init_ExpectAndReturn( &expected, GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitRtsGpio( USART_RTS_PIN_BUS1_PA12 ) );

    Gpio_Init_ExpectAndReturn( &expected, GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitRtsGpio( USART_RTS_PIN_BUS1_PA12 ) );
}


/**
 * \brief   Usart_Init() configures the CTS and RTS pins of the hardware flow control.
 *
 * \details Initializes USART1 with the CTS pin PA11 and the RTS pin PA12.
 *
 * \par Expected results
 * - Gpio_Init() called for the CTS pin and for the RTS pin (after clock enabling, before peripheral reset).
 * - USART_REQUEST_OK.
 */
void Ut_Usart_Init_FlowControlPins_GpioConfigured( void )
{
    static gpio_Config_t expectedCts;
    static gpio_Config_t expectedRts;
    usart_BusConfig_t    config = Ut_Usart_Get_BusConfig();

    expectedCts                = (gpio_Config_t){ 0 };
    expectedCts.PortId         = GPIO_PORT_A;
    expectedCts.PinId          = GPIO_PIN_ID_11;
    expectedCts.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expectedCts.PinPull        = GPIO_PIN_PULL_NONE;
    expectedCts.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    expectedCts.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expectedCts.PinAltFunction = GPIO_ALT_FUNC_7;

    expectedRts                = (gpio_Config_t){ 0 };
    expectedRts.PortId         = GPIO_PORT_A;
    expectedRts.PinId          = GPIO_PIN_ID_12;
    expectedRts.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expectedRts.PinPull        = GPIO_PIN_PULL_NONE;
    expectedRts.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    expectedRts.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    expectedRts.PinAltFunction = GPIO_ALT_FUNC_7;

    config.BusCtsPin = USART_CTS_PIN_BUS1_PA11;
    config.BusRtsPin = USART_RTS_PIN_BUS1_PA12;

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Gpio_Init_ExpectAndReturn( &expectedCts, GPIO_REQUEST_OK );
    Gpio_Init_ExpectAndReturn( &expectedRts, GPIO_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
}


/**
 * \brief   Usart_Init() ignores the CTS and RTS pins of another peripheral.
 *
 * \details Initializes USART1 with the CTS pin PA0 and the RTS pin PA1 of USART2.
 *
 * \par Expected results
 * - Gpio_Init() not called (the mock would report an unexpected call).
 * - USART_REQUEST_OK.
 */
void Ut_Usart_Init_FlowControlPinsOfOtherPeriph_GpioNotConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.BusCtsPin = USART_CTS_PIN_BUS2_PA0;
    config.BusRtsPin = USART_RTS_PIN_BUS2_PA1;

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
}


/**
 * \brief   Usart_Init() reports GPIO error of the flow control pins.
 *
 * \details Gpio_Init() mock returns error for the CTS pin, then for the RTS pin.
 *
 * \par Expected results
 * - CTS pin error: USART_REQUEST_ERROR, RTS pin and peripheral reset not processed, peripheral not enabled.
 * - RTS pin error: USART_REQUEST_ERROR, peripheral reset not processed, peripheral not enabled.
 */
void Ut_Usart_Init_FlowControlPinGpioError_ReturnsErrorPeripheralNotEnabled( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.BusCtsPin = USART_CTS_PIN_BUS1_PA11;
    config.BusRtsPin = USART_RTS_PIN_BUS1_PA12;

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );

    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() configures all frame and line options.
 *
 * \details Initializes USART1 with half-duplex, odd parity, 9 data bits, 2 stop bits,
 *          driver enable active low and receive timeout 100 bits.
 *
 * \par Expected results
 * - CR3: HDSEL, DEM and DEP set.
 * - CR1: PCE and PS set, M = 9 data bits.
 * - CR2: STOP = 2 stop bits, RTOEN set, RTOR.RTO = 99 (100 bits from the end of the first stop bit).
 *
 * \note  Device errata bug AB#1295 (ES0334 "Receiver timeout counter wrong start in two-stop-bit
 *        configuration")
 */
void Ut_Usart_Init_HalfDuplexParityDriverEnable_RegistersConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.HalfDuplex           = USART_HALF_DUPLEX_ACTIVE;
    config.Parity               = USART_PARITY_ODD;
    config.DataWidth            = USART_DATA_WIDTH_9;
    config.StopBits             = USART_STOP_BITS_2;
    config.DriverEnableMode     = USART_DE_ENABLED;
    config.DriverEnablePolarity = USART_DE_ACTIVE_LOW;
    config.RxTimeoutValue       = 100u;

    Ut_Usart_Init( &config );

    TEST_ASSERT_EQUAL_HEX32( USART_CR3_HDSEL, UT_USART_REG->CR3 & USART_CR3_HDSEL );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DEM | USART_CR3_DEP, UT_USART_REG->CR3 & ( USART_CR3_DEM | USART_CR3_DEP ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_PCE | USART_CR1_PS, UT_USART_REG->CR1 & ( USART_CR1_PCE | USART_CR1_PS ) );
    TEST_ASSERT_EQUAL_HEX32( LL_USART_DATAWIDTH_9B, UT_USART_REG->CR1 & USART_CR1_M );
    TEST_ASSERT_EQUAL_HEX32( LL_USART_STOPBITS_2, UT_USART_REG->CR2 & USART_CR2_STOP );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_RTOEN, UT_USART_REG->CR2 & USART_CR2_RTOEN );
    TEST_ASSERT_EQUAL_HEX32( 99u, UT_USART_REG->RTOR & USART_RTOR_RTO );
}

/* =========================== DEINITIALIZATION ============================= */

/**
 * \brief   Usart_Deinit() disables interrupts, resets the peripheral and disables
 *          its clock.
 *
 * \details Initializes USART1 and deinitializes it.
 *
 * \par Expected results
 * - NVIC line disabled, peripheral reset activated and released, clock disabled.
 * - USART_REQUEST_OK, peripheral is not enabled (CR1.UE = 0).
 */
void Ut_Usart_Deinit_InitializedBus_PeripheralResetAndClockDisabled( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    Ut_Usart_Init( &config );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_StopIgnore();
    Rcc_Set_ResetInactive_StopIgnore();
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Deinit() rejects peripheral out of range.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Deinit_InvalidPeriph_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Deinit( USART_BUS_CNT ) );
}

/* =========================== PERIPHERAL STATE ============================= */

/**
 * \brief   Peripheral is enabled, disabled and the state is reported.
 *
 * \details Enables USART1, reads the state, disables USART1 and reads the state.
 *
 * \par Expected results
 * - CR1.UE set, state USART_FLAG_ACTIVE.
 * - CR1.UE cleared, state USART_FLAG_INACTIVE.
 */
void Ut_Usart_Set_PeriphActive_PeripheralEnabledAndReported( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );
}


/**
 * \brief   Peripheral state functions reject invalid arguments.
 *
 * \details Calls enable, disable and state functions with peripheral out of range and
 *          state getter with NULL pointer.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases.
 */
void Ut_Usart_Set_PeriphActive_InvalidArgs_ReturnsError( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PeriphActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PeriphInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PeriphState( USART_BUS_CNT, &state ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PeriphState( UT_USART_BUS, NULL ) );
}

/* ============================== BAUD RATE ================================= */

/**
 * \brief   Baud rate with oversampling 16 is calculated and read back.
 *
 * \details Kernel clock 64 MHz, oversampling 16. Sets 115200 Bd and reads it back.
 *
 * \par Expected results
 * - BRR = 556 +- 1 (64 MHz / 115200, no prescaler on STM32F7).
 * - Baud rate reads back 115200 Bd +- 1 %.
 */
void Ut_Usart_Set_Baudrate_Oversampling16_Brr( void )
{
    usart_Baudrate_t baudrate = 0u;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );

    /* 64 MHz / 115200 = 555.6 -> BRR 556 */
    TEST_ASSERT_UINT32_WITHIN( 1u, 556u, UT_USART_REG->BRR );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( UT_USART_BUS, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( UT_USART_BAUDRATE / 100u, UT_USART_BAUDRATE, baudrate );
}


/**
 * \brief   Baud rate with oversampling 8 keeps USARTDIV at least 16.
 *
 * \details Kernel clock 64 MHz, oversampling 8. Sets 115200 Bd.
 *
 * \par Expected results
 * - BRR[15:4] >= 1 (USARTDIV >= 16 - reference manual requirement).
 */
void Ut_Usart_Set_Baudrate_Oversampling8_UsartDivAtLeast16( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_8 ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );

    /* BRR[15:4] = USARTDIV[15:4] >= 1 (USARTDIV >= 16) */
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32( 0x10u, UT_USART_REG->BRR & 0xFFF0u );
}


/**
 * \brief   Usart_Set_Baudrate() rejects peripheral out of range.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, no RCC call (strict mock).
 */
void Ut_Usart_Set_Baudrate_InvalidPeriph_ReturnsErrorWithoutAccess( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( USART_BUS_CNT, UT_USART_BAUDRATE ) );
}


/**
 * \brief   Usart_Set_Baudrate() rejects baud rate 0.
 *
 * \details BRR preset to 0x1234, sets baud rate 0.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, BRR unchanged.
 */
void Ut_Usart_Set_Baudrate_ZeroBaudrate_ReturnsErrorWithoutWrite( void )
{
    UT_USART_REG->BRR = 0x1234u;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, 0u ) );

    TEST_ASSERT_EQUAL_HEX32( 0x1234u, UT_USART_REG->BRR );
}


/**
 * \brief   Lowest baud rates are limited by the 16-bit BRR (no prescaler on STM32F7).
 *
 * \details Kernel clock 64 MHz, oversampling 16, BRR preset. Sets 1200 Bd and reads it back,
 *          then sets 600 Bd (divider 106667 above 16-bit BRR).
 *
 * \par Expected results
 * - 1200 Bd: USART_REQUEST_OK, BRR = 53333 +- 1, baud rate reads back 1200 Bd +- 1 %.
 * - 600 Bd: USART_REQUEST_ERROR, BRR unchanged.
 */
void Ut_Usart_Set_Baudrate_LowBaudrate_DividerLimitedByBrr( void )
{
    usart_Baudrate_t baudrate = 0u;

    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );

    /* 64 MHz / 1200 Bd = 53333 */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, 1200u ) );
    TEST_ASSERT_UINT32_WITHIN( 1u, 53333u, UT_USART_REG->BRR );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( UT_USART_BUS, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( 12u, 1200u, baudrate );

    /* 64 MHz / 600 Bd = 106667 > 0xFFFF */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, 600u ) );
    TEST_ASSERT_UINT32_WITHIN( 1u, 53333u, UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Set_Baudrate() rejects baud rate above maximum.
 *
 * \details Kernel clock 64 MHz, oversampling 16 (maximum 4 MBd), BRR preset. Sets 5 MBd.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, BRR unchanged.
 */
void Ut_Usart_Set_Baudrate_UnreachableBaudrate_ReturnsErrorWithoutWrite( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );

    UT_USART_REG->BRR = 0x1234u;

    /* 64 MHz / 16 = 4 MBd is the maximum with oversampling 16 */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, 5000000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0x1234u, UT_USART_REG->BRR );
}


/**
 * \brief   Baud rate of enabled peripheral is written and peripheral stays enabled.
 *
 * \details CR1.UE preset, sets 115200 Bd (BRR is writable only with disabled peripheral).
 *
 * \par Expected results
 * - USART_REQUEST_OK, BRR = 556 +- 1.
 * - Peripheral is enabled again (CR1.UE set).
 */
void Ut_Usart_Set_Baudrate_EnabledPeripheral_ConfiguredAndEnabledAgain( void )
{
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    UT_USART_REG->CR1 = USART_CR1_UE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );

    TEST_ASSERT_UINT32_WITHIN( 1u, 556u, UT_USART_REG->BRR );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Get_Baudrate() rejects invalid arguments.
 *
 * \details Calls the function with peripheral out of range and with NULL pointer.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in both cases.
 */
void Ut_Usart_Get_Baudrate_InvalidArgs_ReturnsError( void )
{
    usart_Baudrate_t baudrate = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Baudrate( USART_BUS_CNT, &baudrate ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Baudrate( UT_USART_BUS, NULL ) );
}

/* ============================ FRAME FORMAT ================================ */

/**
 * \brief   All data widths are written and read back.
 *
 * \details Sets data width 7, 8 and 9 bits and reads it back.
 *
 * \par Expected results
 * - CR1.M corresponds to the width, width reads back.
 */
void Ut_Usart_Set_DataWidth_AllWidths_RegisterAndReadBack( void )
{
    const usart_DataWidth_t widths[] = { USART_DATA_WIDTH_7, USART_DATA_WIDTH_8, USART_DATA_WIDTH_9 };
    usart_DataWidth_t       width    = USART_DATA_WIDTH_8;

    for( uint32_t idx = 0u; ( sizeof( widths ) / sizeof( widths[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataWidth( UT_USART_BUS, widths[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( widths[ idx ], UT_USART_REG->CR1 & USART_CR1_M );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataWidth( UT_USART_BUS, &width ) );
        TEST_ASSERT_EQUAL( widths[ idx ], width );
    }
}


/**
 * \brief   All stop bit options are written and read back.
 *
 * \details Sets 0.5, 1, 1.5 and 2 stop bits and reads them back.
 *
 * \par Expected results
 * - CR2.STOP corresponds to the option, option reads back.
 */
void Ut_Usart_Set_StopBits_AllOptions_RegisterAndReadBack( void )
{
    const usart_StopBits_t stopBits[] = { USART_STOP_BITS_0_5, USART_STOP_BITS_1, USART_STOP_BITS_1_5, USART_STOP_BITS_2 };
    usart_StopBits_t       stop       = USART_STOP_BITS_1;

    for( uint32_t idx = 0u; ( sizeof( stopBits ) / sizeof( stopBits[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, stopBits[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( stopBits[ idx ], UT_USART_REG->CR2 & USART_CR2_STOP );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_StopBits( UT_USART_BUS, &stop ) );
        TEST_ASSERT_EQUAL( stopBits[ idx ], stop );
    }
}


/**
 * \brief   All parity options are written and read back.
 *
 * \details Sets no, even and odd parity and reads it back.
 *
 * \par Expected results
 * - CR1.PCE / PS correspond to the option, option reads back.
 */
void Ut_Usart_Set_Parity_AllOptions_RegisterAndReadBack( void )
{
    const usart_Parity_t parities[] = { USART_PARITY_NONE, USART_PARITY_EVEN, USART_PARITY_ODD };
    usart_Parity_t       parity     = USART_PARITY_NONE;

    for( uint32_t idx = 0u; ( sizeof( parities ) / sizeof( parities[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Parity( UT_USART_BUS, parities[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( parities[ idx ], UT_USART_REG->CR1 & ( USART_CR1_PCE | USART_CR1_PS ) );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( UT_USART_BUS, &parity ) );
        TEST_ASSERT_EQUAL( parities[ idx ], parity );
    }
}


/**
 * \brief   Data width of enabled peripheral is written and peripheral stays enabled.
 *
 * \details CR1.UE preset, sets 9 data bits.
 *
 * \par Expected results
 * - USART_REQUEST_OK, CR1.M = 9 data bits, CR1.UE set.
 */
void Ut_Usart_Set_DataWidth_EnabledPeripheral_StaysEnabled( void )
{
    UT_USART_REG->CR1 = USART_CR1_UE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataWidth( UT_USART_BUS, USART_DATA_WIDTH_9 ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );
    TEST_ASSERT_EQUAL_HEX32( LL_USART_DATAWIDTH_9B, UT_USART_REG->CR1 & USART_CR1_M );
}


/**
 * \brief   Frame format functions reject invalid arguments.
 *
 * \details Calls data width, stop bits and parity setters / getters with peripheral
 *          out of range and data width getter with NULL pointer.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases.
 */
void Ut_Usart_Set_FrameFormat_InvalidPeriph_ReturnsError( void )
{
    usart_DataWidth_t width  = USART_DATA_WIDTH_8;
    usart_StopBits_t  stop   = USART_STOP_BITS_1;
    usart_Parity_t    parity = USART_PARITY_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataWidth( USART_BUS_CNT, USART_DATA_WIDTH_8 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_StopBits( USART_BUS_CNT, USART_STOP_BITS_1 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Parity( USART_BUS_CNT, USART_PARITY_NONE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataWidth( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_StopBits( USART_BUS_CNT, &stop ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Parity( USART_BUS_CNT, &parity ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataWidth( USART_BUS_CNT, &width ) );
}

/* ======================= TRANSFER MODE / FEATURES ========================= */

/**
 * \brief   All transfer modes are written and read back.
 *
 * \details Sets transfer mode none, RX, TX and TX + RX and reads it back.
 *
 * \par Expected results
 * - CR1.TE / RE correspond to the mode, mode reads back.
 */
void Ut_Usart_Set_TransferMode_AllModes_RegisterAndReadBack( void )
{
    const usart_TransferMode_t modes[] = { USART_TRANSFER_MODE_NONE, USART_TRANSFER_MODE_RX, USART_TRANSFER_MODE_TX, USART_TRANSFER_MODE_TX_RX };
    usart_TransferMode_t       mode    = USART_TRANSFER_MODE_NONE;

    for( uint32_t idx = 0u; ( sizeof( modes ) / sizeof( modes[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, modes[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( modes[ idx ], UT_USART_REG->CR1 & ( USART_CR1_TE | USART_CR1_RE ) );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TransferMode( UT_USART_BUS, &mode ) );
        TEST_ASSERT_EQUAL( modes[ idx ], mode );
    }
}


/**
 * \brief   RTS / CTS flow control is written and read back.
 *
 * \details Sets flow control RTS + CTS and reads it back.
 *
 * \par Expected results
 * - CR3.RTSE and CTSE set, flow control reads back RTS + CTS.
 */
void Ut_Usart_Set_FlowControl_RtsCts_RegisterAndReadBack( void )
{
    usart_FlowControl_t flowControl = USART_FLOW_CONTROL_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS_CTS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE | USART_CR3_CTSE, UT_USART_REG->CR3 & ( USART_CR3_RTSE | USART_CR3_CTSE ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_FlowControl( UT_USART_BUS, &flowControl ) );
    TEST_ASSERT_EQUAL( USART_FLOW_CONTROL_RTS_CTS, flowControl );
}


/**
 * \brief   Driver enable state and polarity are written and read back.
 *
 * \details Enables driver enable with active low polarity, reads both back, then
 *          disables driver enable.
 *
 * \par Expected results
 * - CR3.DEM and DEP set, state enabled and polarity active low read back.
 * - CR3.DEM cleared after disabling.
 */
void Ut_Usart_Set_DriverEnable_StateAndPolarity_RegisterAndReadBack( void )
{
    usart_DeFeatureState_t deState    = USART_DE_DISABLED;
    usart_DePolarity_t     dePolarity = USART_DE_ACTIVE_HIGH;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DriverEnableState( UT_USART_BUS, USART_DE_ENABLED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DriverEnablePolarity( UT_USART_BUS, USART_DE_ACTIVE_LOW ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DEM | USART_CR3_DEP, UT_USART_REG->CR3 & ( USART_CR3_DEM | USART_CR3_DEP ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DriverEnableState( UT_USART_BUS, &deState ) );
    TEST_ASSERT_EQUAL( USART_DE_ENABLED, deState );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DriverEnablePolarity( UT_USART_BUS, &dePolarity ) );
    TEST_ASSERT_EQUAL( USART_DE_ACTIVE_LOW, dePolarity );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DriverEnableState( UT_USART_BUS, USART_DE_DISABLED ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_DEM );
}


/**
 * \brief   Both oversampling options are written and read back.
 *
 * \details Sets oversampling 8 and 16 and reads it back.
 *
 * \par Expected results
 * - Oversampling 8: CR1.OVER8 set, reads back 8.
 * - Oversampling 16: CR1.OVER8 cleared, reads back 16.
 */
void Ut_Usart_Set_Oversampling_Both_RegisterAndReadBack( void )
{
    usart_Oversampling_t oversampling = USART_OVERSAMPLING_16;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_8 ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_OVER8, UT_USART_REG->CR1 & USART_CR1_OVER8 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Oversampling( UT_USART_BUS, &oversampling ) );
    TEST_ASSERT_EQUAL( USART_OVERSAMPLING_8, oversampling );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_OVER8 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Oversampling( UT_USART_BUS, &oversampling ) );
    TEST_ASSERT_EQUAL( USART_OVERSAMPLING_16, oversampling );
}


/**
 * \brief   Half-duplex mode is enabled and disabled.
 *
 * \details Activates and deactivates half-duplex mode.
 *
 * \par Expected results
 * - CR3.HDSEL set, then cleared.
 */
void Ut_Usart_Set_HalfDuplexState_ActiveInactive_Register( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_HalfDuplexState( UT_USART_BUS, USART_HALF_DUPLEX_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_HDSEL, UT_USART_REG->CR3 & USART_CR3_HDSEL );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_HalfDuplexState( UT_USART_BUS, USART_HALF_DUPLEX_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_HDSEL );
}


/**
 * \brief   Half-duplex state is read from HDSEL independently of oversampling.
 *
 * \details CR1 = 0 (oversampling 16), CR3.HDSEL set. Reads half-duplex state.
 *
 * \par Expected results
 * - USART_REQUEST_OK, state USART_HALF_DUPLEX_ACTIVE.
 */
void Ut_Usart_Get_HalfDuplexState_Oversampling16HalfDuplex_ReturnsActive( void )
{
    usart_HalfDuplex_t halfDuplex = USART_HALF_DUPLEX_INACTIVE;

    UT_USART_REG->CR1 = 0u;                 /* Oversampling 16 */
    UT_USART_REG->CR3 = USART_CR3_HDSEL;    /* Half-duplex */

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_HalfDuplexState( UT_USART_BUS, &halfDuplex ) );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_ACTIVE, halfDuplex );
}


/**
 * \brief   Receive timeout is enabled with value and disabled.
 *
 * \details Activates receive timeout 0x1234 bits, reads the state, deactivates it and
 *          reads the state.
 *
 * \par Expected results
 * - CR2.RTOEN set, RTOR.RTO = 0x1234, state active.
 * - CR2.RTOEN cleared, state inactive.
 */
void Ut_Usart_Set_RxTimeoutActive_ValueEnabledAndReadBack( void )
{
    usart_FlagState_t rtoState = USART_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutActive( UT_USART_BUS, 0x1234u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_RTOEN, UT_USART_REG->CR2 & USART_CR2_RTOEN );
    TEST_ASSERT_EQUAL_HEX32( 0x1234u, UT_USART_REG->RTOR & USART_RTOR_RTO );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxTimeoutState( UT_USART_BUS, &rtoState ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, rtoState );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR2 & USART_CR2_RTOEN );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxTimeoutState( UT_USART_BUS, &rtoState ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, rtoState );
}


/**
 * \brief   Receive timeout rejects value above 24-bit range.
 *
 * \details Activates receive timeout with value RTO mask + 1.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, CR2.RTOEN and RTOR stay 0.
 */
void Ut_Usart_Set_RxTimeoutActive_ValueAboveRange_ReturnsErrorWithoutWrite( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutActive( UT_USART_BUS, USART_RTOR_RTO + 1u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR2 & USART_CR2_RTOEN );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->RTOR );
}


/**
 * \brief   Receive timeout with two stop bits is programmed one bit shorter.
 *
 * \details Sets two stop bits and activates receive timeout 0x1234 bits, then receive timeout
 *          0 bits.
 *
 * \par Expected results
 * - USART_REQUEST_OK, CR2.RTOEN set, RTOR.RTO = 0x1233.
 * - Timeout 0 bits: USART_REQUEST_OK, RTOR.RTO = 0 (no underflow).
 *
 * \note  Device errata bug AB#1295 (ES0334 "Receiver timeout counter wrong start in two-stop-bit
 *        configuration")
 */
void Ut_Usart_Set_RxTimeoutActive_TwoStopBits_ValueOneBitShorter( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, USART_STOP_BITS_2 ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutActive( UT_USART_BUS, 0x1234u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_RTOEN, UT_USART_REG->CR2 & USART_CR2_RTOEN );
    TEST_ASSERT_EQUAL_HEX32( 0x1233u, UT_USART_REG->RTOR & USART_RTOR_RTO );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutActive( UT_USART_BUS, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->RTOR & USART_RTOR_RTO );
}


/**
 * \brief   Programmed receive timeout follows a stop bits change.
 *
 * \details Activates receive timeout 500 bits with one stop bit, changes the stop bits to 2,
 *          1.5 and back to 1.
 *
 * \par Expected results
 * - One stop bit: RTOR.RTO = 500.
 * - Two stop bits: RTOR.RTO = 499, CR2.RTOEN stays set.
 * - 1.5 and one stop bit: RTOR.RTO = 500.
 *
 * \note  Device errata bug AB#1295 (ES0334 "Receiver timeout counter wrong start in two-stop-bit
 *        configuration")
 */
void Ut_Usart_Set_StopBits_RxTimeoutActive_ValueFollowsStopBits( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, USART_STOP_BITS_1 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutActive( UT_USART_BUS, 500u ) );
    TEST_ASSERT_EQUAL_HEX32( 500u, UT_USART_REG->RTOR & USART_RTOR_RTO );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, USART_STOP_BITS_2 ) );
    TEST_ASSERT_EQUAL_HEX32( 499u, UT_USART_REG->RTOR & USART_RTOR_RTO );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_RTOEN, UT_USART_REG->CR2 & USART_CR2_RTOEN );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, USART_STOP_BITS_1_5 ) );
    TEST_ASSERT_EQUAL_HEX32( 500u, UT_USART_REG->RTOR & USART_RTOR_RTO );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, USART_STOP_BITS_1 ) );
    TEST_ASSERT_EQUAL_HEX32( 500u, UT_USART_REG->RTOR & USART_RTOR_RTO );
}


/**
 * \brief   Stop bits change without active receive timeout keeps RTOR.
 *
 * \details Writes RTOR directly with CR2.RTOEN cleared and changes the stop bits to 2.
 *
 * \par Expected results
 * - USART_REQUEST_OK, RTOR.RTO unchanged.
 *
 * \note  Device errata bug AB#1295 (ES0334 "Receiver timeout counter wrong start in two-stop-bit
 *        configuration")
 */
void Ut_Usart_Set_StopBits_RxTimeoutInactive_RtorUnchanged( void )
{
    UT_USART_REG->RTOR = 77u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, USART_STOP_BITS_2 ) );
    TEST_ASSERT_EQUAL_HEX32( 77u, UT_USART_REG->RTOR & USART_RTOR_RTO );
}


/**
 * \brief   Inverted pin levels are written and read back.
 *
 * \details Sets inverted RX and TX pin levels and reads them back.
 *
 * \par Expected results
 * - CR2.RXINV and TXINV set, both levels read back inverted.
 */
void Ut_Usart_Set_PinLevels_Inverted_RegisterAndReadBack( void )
{
    usart_RxPinLevel_t rxLevel = USART_RX_PIN_STANDARD;
    usart_TxPinLevel_t txLevel = USART_TX_PIN_STANDARD;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PinLevels( UT_USART_BUS, USART_RX_PIN_INVERTED, USART_TX_PIN_INVERTED ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_RXINV | USART_CR2_TXINV, UT_USART_REG->CR2 & ( USART_CR2_RXINV | USART_CR2_TXINV ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PinLevels( UT_USART_BUS, &rxLevel, &txLevel ) );
    TEST_ASSERT_EQUAL( USART_RX_PIN_INVERTED, rxLevel );
    TEST_ASSERT_EQUAL( USART_TX_PIN_INVERTED, txLevel );
}

/* ======================= DATA REGISTERS / ADDRESSES ======================= */

/**
 * \brief   Usart_SendData() writes byte to transmit data register.
 *
 * \details Sends 0xA5 by USART1, then 0x5A by peripheral out of range.
 *
 * \par Expected results
 * - TDR = 0xA5, invalid peripheral is ignored (TDR unchanged).
 */
void Ut_Usart_SendData_ByteWrittenToTdr( void )
{
    Usart_SendData( UT_USART_BUS, 0xA5u );
    TEST_ASSERT_EQUAL_HEX32( 0xA5u, UT_USART_REG->TDR );

    /* Invalid peripheral is ignored */
    Usart_SendData( USART_BUS_CNT, 0x5Au );
    TEST_ASSERT_EQUAL_HEX32( 0xA5u, UT_USART_REG->TDR );
}


/**
 * \brief   Usart_ReadData() reads byte from receive data register.
 *
 * \details RDR = 0x13C, reads data of USART1 and of peripheral out of range.
 *
 * \par Expected results
 * - USART1: 0x3C (lower 8 bits).
 * - Invalid peripheral: 0x00.
 */
void Ut_Usart_ReadData_ByteReadFromRdr( void )
{
    UT_USART_REG->RDR = 0x13Cu;

    TEST_ASSERT_EQUAL_HEX8( 0x3Cu, Usart_ReadData( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX8( 0x00u, Usart_ReadData( USART_BUS_CNT ) );
}


/**
 * \brief   Data register addresses are returned for DMA.
 *
 * \details Reads TX and RX register address of USART1, then calls the getters with
 *          peripheral out of range and with NULL pointer.
 *
 * \par Expected results
 * - TX address = &TDR, RX address = &RDR.
 * - Invalid arguments: USART_REQUEST_ERROR.
 */
void Ut_Usart_Get_RegisterAddr_TdrAndRdrAddresses( void )
{
    usart_RxRegAddr_t regAddr = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxRegisterAddr( UT_USART_BUS, &regAddr ) );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->TDR, regAddr );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxRegisterAddr( UT_USART_BUS, &regAddr ) );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->RDR, regAddr );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxRegisterAddr( USART_BUS_CNT, &regAddr ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxRegisterAddr( UT_USART_BUS, NULL ) );
}

/* ====================== INTERRUPTS / DMA REQUESTS ========================= */

/**
 * \brief   Usart_Set_IrqPriority() forwards priority to NVIC.
 *
 * \details Sets priority 6 (NVIC OK), 99 (NVIC error) and priority of peripheral out
 *          of range.
 *
 * \par Expected results
 * - Priority 6: Nvic_Set_PeriphIrq_Prio() called for USART1 line, USART_REQUEST_OK.
 * - Priority 99: NVIC error returned as USART_REQUEST_ERROR.
 * - Invalid peripheral: USART_REQUEST_ERROR without NVIC call.
 */
void Ut_Usart_Set_IrqPriority_ForwardedToNvic( void )
{
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, UT_USART_PRIO, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IrqPriority( UT_USART_BUS, UT_USART_PRIO ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, 99u, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IrqPriority( UT_USART_BUS, 99u ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IrqPriority( USART_BUS_CNT, UT_USART_PRIO ) );
}


/**
 * \brief   Usart_Get_IrqPriority() reads priority from NVIC.
 *
 * \details NVIC mock returns priority 6, then the getter is called with NULL pointer.
 *
 * \par Expected results
 * - USART_REQUEST_OK, priority 6.
 * - NULL pointer: USART_REQUEST_ERROR.
 */
void Ut_Usart_Get_IrqPriority_ReadFromNvic( void )
{
    static nvic_IrqPrio_t prio = UT_USART_PRIO;
    usart_IrqPrio_t       read = 0u;

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    Nvic_Get_PeriphIrq_Prio_ReturnThruPtr_irqPrio( &prio );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_IrqPriority( UT_USART_BUS, &read ) );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_PRIO, read );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_IrqPriority( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Interrupts of the USART are enabled and disabled in NVIC.
 *
 * \details Activates and deactivates interrupts of USART1.
 *
 * \par Expected results
 * - Activation: ISR registered in NVIC, NVIC line enabled, USART_REQUEST_OK.
 * - Deactivation: NVIC line disabled, USART_REQUEST_OK.
 */
void Ut_Usart_Set_InterruptsActive_HandlerRegisteredAndLineEnabled( void )
{
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_InterruptsActive( UT_USART_BUS ) );
    TEST_ASSERT_NOT_NULL( utUsart_Isr );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_InterruptsInactive( UT_USART_BUS ) );
}


/**
 * \brief   NVIC error of interrupt activation is reported.
 *
 * \details NVIC mock returns error on line enabling.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Set_InterruptsActive_NvicError_ReturnsError( void )
{
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsActive( UT_USART_BUS ) );
}


/**
 * \brief   DMA TX and RX requests are enabled, reported and disabled.
 *
 * \details Activates TX and RX DMA requests, reads their states and deactivates them.
 *
 * \par Expected results
 * - CR3.DMAT and DMAR set, both states active.
 * - CR3.DMAT and DMAR cleared after deactivation.
 */
void Ut_Usart_Set_DmaRequests_TxRx_RegisterAndState( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaTxRequestActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaRxRequestActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAT | USART_CR3_DMAR, UT_USART_REG->CR3 & ( USART_CR3_DMAT | USART_CR3_DMAR ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DmaTxReqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DmaRxReqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaTxRequestInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaRxRequestInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_DMAT | USART_CR3_DMAR ) );
}


/**
 * \brief   RX not empty interrupt is enabled, reported and disabled.
 *
 * \details Activates RXNE interrupt, reads its state and deactivates it.
 *
 * \par Expected results
 * - ISR registered and NVIC line enabled, CR1.RXNEIE set, state active.
 * - CR1.RXNEIE cleared after deactivation.
 */
void Ut_Usart_Set_RxNotEmptyIrqActive_EnableBitAndNvicLine( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxNotEmptyIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RXNEIE, UT_USART_REG->CR1 & USART_CR1_RXNEIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxNotEmptyIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxNotEmptyIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_RXNEIE );
}


/**
 * \brief   TX empty and TX complete interrupts are enabled, reported and disabled.
 *
 * \details Activates TXE and TC interrupts, reads their states and deactivates them.
 *
 * \par Expected results
 * - CR1.TXEIE and TCIE set, both states active.
 * - CR1.TXEIE and TCIE cleared after deactivation.
 */
void Ut_Usart_Set_TxIrqs_TxEmptyAndComplete_EnableBits( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxEmptyIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxCompleteIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TXEIE | USART_CR1_TCIE, UT_USART_REG->CR1 & ( USART_CR1_TXEIE | USART_CR1_TCIE ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxEmptyIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxCompleteIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxEmptyIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxCompleteIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_TXEIE | USART_CR1_TCIE ) );
}


/**
 * \brief   Idle, receive timeout and error interrupts are enabled, reported and
 *          disabled.
 *
 * \details Activates IDLE, RTO and error interrupts, reads their states and
 *          deactivates them.
 *
 * \par Expected results
 * - CR1.IDLEIE, CR1.RTOIE and CR3.EIE set, all states active.
 * - All enable bits cleared after deactivation.
 */
void Ut_Usart_Set_RxEventIrqs_IdleTimeoutError_EnableBits( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IdleIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_ErrorIrqActive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_IDLEIE | USART_CR1_RTOIE, UT_USART_REG->CR1 & ( USART_CR1_IDLEIE | USART_CR1_RTOIE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_EIE, UT_USART_REG->CR3 & USART_CR3_EIE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_IdleIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxTimeoutIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_ErrorIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IdleIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_ErrorIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_IDLEIE | USART_CR1_RTOIE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_EIE );
}

/* ============================ DATA HANDLING =============================== */

/**
 * \brief   Usart_Set_DataConfig() rejects invalid configurations.
 *
 * \details Calls the function with NULL configuration, peripheral out of range, NULL
 *          RX buffer, RX buffer size 0, TX mode out of range and RX buffer mode out
 *          of range.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases, no NVIC / DMA call (strict mocks).
 */
void Ut_Usart_Set_DataConfig_InvalidConfig_ReturnsErrorWithoutAccess( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( USART_BUS_CNT, &dataConfig ) );

    dataConfig.RxBuffer = NULL;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, 0u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    dataConfig.TxMode = USART_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    dataConfig.RxBufferMode = USART_BUFFER_MODE_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() rejects timeout end mode without receive timeout.
 *
 * \details RX end mode timeout, receive timeout of the peripheral is not enabled.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Set_DataConfig_TimeoutEndWithoutRto_ReturnsError( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    dataConfig.RxEndMode = USART_RX_END_TIMEOUT;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() rejects invalid DMA streams.
 *
 * \details DMA mode with the transmit stream in the receive member, then with the receive stream
 *          in the transmit member, with an unused stream and with TX stream out of range.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases.
 */
void Ut_Usart_Set_DataConfig_DmaStreamOfOtherDirection_ReturnsError( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    dataConfig.RxDma = (usart_RxDma_t)UT_USART_TX_DMA;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)UT_USART_RX_DMA;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = USART_TX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_CNT, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() in interrupt mode configures NVIC.
 *
 * \details Sets data handling in interrupt mode (RX buffer 8 bytes, priority 6) and
 *          reads the configuration back.
 *
 * \par Expected results
 * - NVIC priority 6 set, ISR registered, NVIC line enabled, USART_REQUEST_OK.
 * - Configuration reads back interrupt TX mode and the RX buffer.
 */
void Ut_Usart_Set_DataConfig_IsrMode_PriorityAndInterruptConfigured( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_DataConfig_t readConfig;

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, UT_USART_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_NOT_NULL( utUsart_Isr );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
    TEST_ASSERT_EQUAL( USART_XFER_MODE_ISR, readConfig.TxMode );
    TEST_ASSERT_EQUAL_PTR( utUsart_RxBuf, readConfig.RxBuffer );
}


/**
 * \brief   Usart_Get_DataConfig() reports not configured data handling.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Get_DataConfig_NotInitialized_ReturnsError( void )
{
    usart_DataConfig_t readConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
}


/**
 * \brief   Transmission / reception start rejects invalid state and arguments.
 *
 * \details Without data handling configuration starts transmission (valid data, NULL
 *          data, size 0, peripheral out of range) and reception.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases.
 */
void Ut_Usart_Set_TxStart_InvalidArgsOrNotInitialized_ReturnsError( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, NULL, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 0u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( USART_BUS_CNT, utUsart_TxBuf, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( UT_USART_BUS ) );
}

/* ---------------------------- Interrupt mode ------------------------------ */

/**
 * \brief   Transmission in interrupt mode writes bytes and calls complete callback.
 *
 * \details
 * 1. Starts transmission of 2 bytes, reads TX state, starts second transmission.
 * 2. Calls ISR with TXE 2x.
 * 3. Calls ISR with TXE (all bytes written).
 * 4. Calls ISR with TC.
 *
 * \par Expected results
 * 1. CR1.TXEIE set, TX state active, second start: USART_REQUEST_ERROR.
 * 2. TDR = 0x11, then 0x22.
 * 3. TXE interrupt disabled, TC interrupt enabled, no complete callback yet.
 * 4. ICR.TCCF written, TC interrupt disabled, complete callback 1x, TX state inactive.
 */
void Ut_Usart_Isr_Transmission_BytesWrittenAndCompleteCallback( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t txState    = USART_FUNCTION_INACTIVE;

    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TXEIE, UT_USART_REG->CR1 & USART_CR1_TXEIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, txState );

    /* Second transmission while the first one is running */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );

    Ut_Usart_Call_Isr( USART_ISR_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_USART_REG->TDR );
    Ut_Usart_Call_Isr( USART_ISR_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x22u, UT_USART_REG->TDR );

    /* All bytes written - TXE interrupt replaced by TC interrupt */
    Ut_Usart_Call_Isr( USART_ISR_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TXEIE );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );

    Ut_Usart_Call_Isr( USART_ISR_TC );
    TEST_ASSERT_EQUAL_HEX32( USART_ICR_TCCF, UT_USART_REG->ICR & USART_ICR_TCCF );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_TxCompleteCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
}


/**
 * \brief   Reception in interrupt mode fills one-shot buffer.
 *
 * \details Interrupt mode, one-shot RX buffer of 4 bytes. Starts reception and calls
 *          ISR with RXNE for 4 bytes 0xA0 - 0xA3.
 *
 * \par Expected results
 * - After start: CR1.RXNEIE, CR1.PEIE and CR3.EIE set.
 * - After 2nd byte: half callback 1x, RX count 2.
 * - Complete callback 1x, buffer contains 0xA0 ... 0xA3.
 * - Reception stopped (RX state inactive, RXNE interrupt disabled).
 */
void Ut_Usart_Isr_Reception_BufferFilledHalfAndCompleteCallbacks( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, 4u );
    usart_FunctionState_t rxState    = USART_FUNCTION_INACTIVE;
    usart_RxDataCnt_t     rxCnt      = 0u;

    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RXNEIE | USART_CR1_PEIE, UT_USART_REG->CR1 & ( USART_CR1_RXNEIE | USART_CR1_PEIE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_EIE, UT_USART_REG->CR3 & USART_CR3_EIE );

    for( uint32_t byteIdx = 0u; 4u > byteIdx; byteIdx++ )
    {
        UT_USART_REG->RDR = 0xA0u + byteIdx;
        Ut_Usart_Call_Isr( USART_ISR_RXNE );

        if( 1u == byteIdx )
        {
            TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxHalfCnt );
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
            TEST_ASSERT_EQUAL_UINT16( 2u, rxCnt );
        }
        else
        {
            /* No action required */
        }
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0xA0u, utUsart_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0xA3u, utUsart_RxBuf[ 3u ] );

    /* One shot buffer - reception stopped */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_RXNEIE );
}


/**
 * \brief   Circular reception in interrupt mode wraps to buffer start.
 *
 * \details Interrupt mode, circular RX buffer of 2 bytes. Starts reception and calls
 *          ISR with RXNE for 3 bytes 0x10 - 0x12.
 *
 * \par Expected results
 * - Complete callback 1x, buffer = { 0x12, 0x11 } (3rd byte at buffer start).
 * - Reception keeps running (RX state active).
 */
void Ut_Usart_Isr_CircularReception_WrapsToBufferStart( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, 2u );
    usart_FunctionState_t rxState    = USART_FUNCTION_INACTIVE;

    dataConfig.RxBufferMode = USART_BUFFER_MODE_CIRCULAR;

    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    for( uint32_t byteIdx = 0u; 3u > byteIdx; byteIdx++ )
    {
        UT_USART_REG->RDR = 0x10u + byteIdx;
        Ut_Usart_Call_Isr( USART_ISR_RXNE );
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x12u, utUsart_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x11u, utUsart_RxBuf[ 1u ] );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );
}


/**
 * \brief   Idle line ends the message in interrupt mode.
 *
 * \details Interrupt mode, RX end mode idle, buffer 8 bytes. Starts reception,
 *          receives 2 bytes and calls ISR with IDLE.
 *
 * \par Expected results
 * - CR1.IDLEIE set after start.
 * - ICR.IDLECF written, RX end callback 1x with 2 bytes, no complete callback.
 */
void Ut_Usart_Isr_IdleLine_RxEndCallbackWithCount( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    dataConfig.RxEndMode = USART_RX_END_IDLE;

    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_IDLEIE, UT_USART_REG->CR1 & USART_CR1_IDLEIE );

    UT_USART_REG->RDR = 0x55u;
    Ut_Usart_Call_Isr( USART_ISR_RXNE );
    UT_USART_REG->RDR = 0x66u;
    Ut_Usart_Call_Isr( USART_ISR_RXNE );

    Ut_Usart_Call_Isr( USART_ISR_IDLE );

    TEST_ASSERT_EQUAL_HEX32( USART_ICR_IDLECF, UT_USART_REG->ICR & USART_ICR_IDLECF );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 2u, utUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_RxCompleteCnt );
}


/**
 * \brief   Framing error calls error callback in interrupt mode.
 *
 * \details Interrupt mode, reception started. Calls ISR with FE flag.
 *
 * \par Expected results
 * - Error flags cleared by ICR writes (emulated ICR holds the last write ORECF).
 * - Error callback 1x with USART_XFER_ERROR_FRAMING.
 */
void Ut_Usart_Isr_FramingError_ErrorCallbackAndFlagCleared( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->ICR = 0u;
    Ut_Usart_Call_Isr( USART_ISR_FE );

    /* Error flags are cleared by separate ICR writes, emulated ICR holds the last one (ORECF) */
    TEST_ASSERT_EQUAL_HEX32( USART_ICR_ORECF, UT_USART_REG->ICR );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_FRAMING, utUsart_LastError );
}


/**
 * \brief   Usart_Set_RxStop() stops running reception.
 *
 * \details Interrupt mode. Starts reception, stops it and
 *          stops it again without running reception.
 *
 * \par Expected results
 * - CR1.RXNEIE, PEIE, IDLEIE and CR3.EIE cleared, RX state inactive.
 * - Second stop: USART_REQUEST_OK.
 */
void Ut_Usart_Set_RxStop_RunningReception_InterruptsDisabled( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_ACTIVE;

    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_EIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );

    /* Stop without running reception */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( UT_USART_BUS ) );
}


/**
 * \brief   Running transmission is stopped and data handling released in interrupt and
 *          polling mode.
 *
 * \details
 * 1. Interrupt mode: starts transmission of 2 bytes, stops it, de-initializes the USART.
 * 2. Polling mode: starts transmission of 1 byte, stops it, de-initializes the USART.
 * 3. Reception only (transmission mode NONE): starts transmission.
 *
 * \par Expected results
 * 1. Stop: USART_REQUEST_OK, CR1.TXEIE / TCIE cleared, TX state inactive, no complete
 *    callback; Usart_Deinit(): USART_REQUEST_OK.
 * 2. Stop: USART_REQUEST_OK, TX state inactive; Usart_Deinit(): USART_REQUEST_OK.
 * 3. USART_REQUEST_ERROR, TX state inactive.
 */
void Ut_Usart_IsrPoll_TxStopAndDeinit_Released( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t txState    = USART_FUNCTION_ACTIVE;

    /* 1. Interrupt mode */
    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TXEIE, UT_USART_REG->CR1 & USART_CR1_TXEIE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_TXEIE | USART_CR1_TCIE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );

    Ut_Usart_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    /* 2. Polling mode */
    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );
    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );

    Ut_Usart_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    /* 3. Reception only */
    dataConfig        = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );
    dataConfig.TxMode = USART_XFER_MODE_NONE;
    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
}

/* ---------------------------- Polling mode -------------------------------- */

/**
 * \brief   Usart_Task() transmits and receives in polling mode.
 *
 * \details Polling mode, RX buffer 2 bytes. Starts reception and transmission of 1
 *          byte, then calls Usart_Task() with ISR flags:
 * 1. TXE.
 * 2. TXE + TC + RXNE (RDR = 0x77).
 * 3. RXNE (RDR = 0x78).
 *
 * \par Expected results
 * 1. TDR = 0x11.
 * 2. TX complete callback 1x, RX buffer[ 0 ] = 0x77.
 * 3. RX complete callback 1x, RX buffer[ 1 ] = 0x78.
 */
void Ut_Usart_Task_PollMode_TransmitsAndReceives( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, 2u );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );

    /* TXE: first byte written */
    UT_USART_REG->ISR = USART_ISR_TXE;
    Usart_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_USART_REG->TDR );

    /* Byte received, TXE + TC: transmission complete */
    UT_USART_REG->RDR = 0x77u;
    UT_USART_REG->ISR = USART_ISR_TXE | USART_ISR_TC | USART_ISR_RXNE;
    Usart_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x77u, utUsart_RxBuf[ 0u ] );

    UT_USART_REG->RDR = 0x78u;
    UT_USART_REG->ISR = USART_ISR_RXNE;
    Usart_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x78u, utUsart_RxBuf[ 1u ] );
}


/**
 * \brief   Polling transmission is not complete before TC flag.
 *
 * \details Polling mode, transmission of 1 byte started. Usart_Task() is called 2x
 *          with TXE flag only.
 *
 * \par Expected results
 * - TX complete callback is not called (last frame not finished).
 */
void Ut_Usart_Task_PollModeTxInProgress_NoCompleteBeforeTc( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, 2u );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );

    UT_USART_REG->ISR = USART_ISR_TXE;
    Usart_Task();
    Usart_Task();

    /* All bytes written, last frame not finished (TC not set) */
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );
}

/* ====================== DATA HANDLING - DMA MODE ========================== */

/**
 * \brief   Usart_Set_DataConfig() in DMA mode initializes streams of USART1 requests.
 *
 * \details DMA mode, TX DMA2 stream 7 (low priority), RX DMA2 stream 2 (high priority),
 *          one shot RX buffer of 8 bytes with half callback.
 *
 * \par Expected results
 * - TX stream: channel 4, memory to peripheral, normal mode, TDR as peripheral address,
 *   8-bit, memory increment, TC and TE callbacks, no HT callback.
 * - RX stream: channel 4, peripheral to memory, normal mode, RDR as peripheral address,
 *   RX buffer and size, TC, HT and TE callbacks.
 * - TC / TE interrupts of both streams, HT of RX stream and NVIC lines enabled, USART
 *   interrupt enabled.
 */
void Ut_Usart_Dma_SetDataConfig_StreamsInitialized( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_DmaInitCnt );

    TEST_ASSERT_EQUAL( UT_USART_DMA,                 utUsart_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_USART_DMA_TX_CH,           utUsart_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( UT_USART_DMA_CHSEL,           utUsart_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,     utUsart_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,     utUsart_DmaConfig[ 0u ].TransferMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->TDR, utUsart_DmaConfig[ 0u ].PeriphAddress );
    TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC,       utUsart_DmaConfig[ 0u ].PeriphAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT,    utUsart_DmaConfig[ 0u ].MemoryAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,       utUsart_DmaConfig[ 0u ].PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,       utUsart_DmaConfig[ 0u ].MemoryTransferSize );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,             utUsart_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 0u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 0u ].TransferErrorCallback );
    TEST_ASSERT_NULL( utUsart_DmaConfig[ 0u ].HalfTransferCallback );

    TEST_ASSERT_EQUAL( UT_USART_DMA,                 utUsart_DmaConfig[ 1u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_USART_DMA_RX_CH,           utUsart_DmaConfig[ 1u ].DmaChannel );
    TEST_ASSERT_EQUAL( UT_USART_DMA_CHSEL,           utUsart_DmaConfig[ 1u ].PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY,     utUsart_DmaConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,     utUsart_DmaConfig[ 1u ].TransferMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->RDR, utUsart_DmaConfig[ 1u ].PeriphAddress );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utUsart_RxBuf, utUsart_DmaConfig[ 1u ].MemoryAddress );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RX_SIZE,      utUsart_DmaConfig[ 1u ].DataCount );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH,            utUsart_DmaConfig[ 1u ].Priority );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 1u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 1u ].HalfTransferCallback );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 1u ].TransferErrorCallback );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].TcIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].TeIrq );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].HtIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].NvicIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_RX_CH ].HtIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_RX_CH ].NvicIrq );
    TEST_ASSERT_NOT_NULL( utUsart_Isr );
}


/**
 * \brief   Usart_Set_DataConfig() accepts only DMA streams connected to the USART requests.
 *
 * \details DMA mode USART1: RX on the second stream of the request (DMA2 stream 5), then TX
 *          stream of other request (RX item of DMA2 stream 5), RX streams of other buses (UART4
 *          RX on DMA1 stream 2, USART6 RX on DMA2 stream 2), RX stream of other DMA (DMA1 stream 2
 *          with the USART1 bus), stream USART_DMA_CHANNEL_CNT.
 *
 * \par Expected results
 * - 8 streams per DMA controller.
 * - DMA2 stream 5 for RX: USART_REQUEST_OK, Dma_Init() with DMA2 stream 5 and channel 4.
 * - Other cases: USART_REQUEST_ERROR without DMA call (strict mocks).
 */
void Ut_Usart_Dma_StreamOfRequest_OnlyConnectedStreamsAccepted( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    TEST_ASSERT_EQUAL_UINT32( 8u, USART_DMA_CHANNEL_CNT );

    dataConfig.TxMode = USART_XFER_MODE_NONE;
    dataConfig.RxDma  = USART_RX_DMA_BUS1_DMA2_STREAM5;

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( DMA_PERIPH_2,       utUsart_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_5,       utUsart_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( UT_USART_DMA_CHSEL, utUsart_DmaConfig[ 0u ].PeripheralReqId );

    Ut_Usart_Release();

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)USART_RX_DMA_BUS1_DMA2_STREAM5;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = USART_RX_DMA_BUS4_DMA1_STREAM2;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = USART_RX_DMA_BUS6_DMA2_STREAM2;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = (usart_RxDma_t)USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_2, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_CNT, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() rejects DMA peripheral, bus and priority out of range.
 *
 * \details DMA mode with RX stream of DMA peripheral USART_DMA_PERIPH_CNT, with the unused RX
 *          stream, with TX stream of bus USART_BUS_CNT, then with TX priority out of range.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases, no DMA call (strict mocks).
 */
void Ut_Usart_Dma_SetDataConfig_InvalidPeriphOrPriority_ReturnsError( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    dataConfig.RxDma = (usart_RxDma_t)USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_CNT, USART_DMA_CHANNEL_2, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = USART_RX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)USART_DMA_ENCODE( USART_BUS_CNT, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig               = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDmaPriority = (usart_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Circular receive buffer uses circular DMA mode, other USART uses its streams.
 *
 * \details USART6 (DMA2: RX stream 2 channel 5), circular buffer without half callback, TX
 *          direction not used.
 *
 * \par Expected results
 * - One stream initialized: DMA2 stream 2, channel 5, USART6 RDR, circular mode, no HT
 *   callback / interrupt.
 * - Usart_Deinit() of USART6 disables the stream.
 */
void Ut_Usart_Dma_CircularBufferUsart6_CircularStream( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    dataConfig.TxMode         = USART_XFER_MODE_NONE;
    dataConfig.RxBufferMode   = USART_BUFFER_MODE_CIRCULAR;
    dataConfig.RxHalfCallback = NULL;
    dataConfig.RxDma          = USART_RX_DMA_BUS6_DMA2_STREAM2;

    utUsart_Nvic = NVIC_PERIPH_IRQ_USART6;
    Ut_Usart_Stub_DmaMocks();
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( NVIC_PERIPH_IRQ_USART6, UT_USART_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( NVIC_PERIPH_IRQ_USART6, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( NVIC_PERIPH_IRQ_USART6, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( USART_BUS_6, &dataConfig ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_PERIPH_2,               utUsart_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_2,               utUsart_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_5,          utUsart_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&USART6->RDR, utUsart_DmaConfig[ 0u ].PeriphAddress );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utUsart_DmaConfig[ 0u ].TransferMode );
    TEST_ASSERT_NULL( utUsart_DmaConfig[ 0u ].HalfTransferCallback );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_DmaChannel[ DMA_PERIPH_2 ][ DMA_STREAM_2 ].HtIrq );

    /* Release of USART6 data handling */
    Ut_Usart_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( USART_BUS_6 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaChannel[ DMA_PERIPH_2 ][ DMA_STREAM_2 ].InactiveCnt );
}


/**
 * \brief   Usart_Set_DataConfig() reports DMA initialization failure.
 *
 * \details Dma_Init() returns error.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, no DMA interrupt enabled.
 */
void Ut_Usart_Dma_InitFailure_ReturnsError( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    utUsart_DmaInitState = DMA_REQUEST_ERROR;

    Ut_Usart_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].TcIrq );
}


/**
 * \brief   DMA transmission arms TX channel, end of transmission is detected by USART TC.
 *
 * \details DMA mode. Transmission of 4 bytes started, TX channel transfer complete callback
 *          is called, then ISR with TC.
 *
 * \par Expected results
 * - ICR.TCCF written before the transmission, TX channel armed with buffer address and 4
 *   bytes, CR3.DMAT set.
 * - DMA transfer complete: USART TC interrupt enabled, no complete callback.
 * - TC interrupt: complete callback 1x, TCIE cleared, TX state inactive.
 */
void Ut_Usart_Dma_Transmission_CompletedByUsartTc( void )
{
    usart_DataConfig_t           dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t        txState    = USART_FUNCTION_ACTIVE;
    const ut_UsartDmaChannel_t * txChannel  = &utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ];

    Ut_Usart_Set_DataConfig( &dataConfig );

    UT_USART_REG->ICR = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );

    TEST_ASSERT_EQUAL_HEX32( USART_ICR_TCCF, UT_USART_REG->ICR & USART_ICR_TCCF );
    TEST_ASSERT_EQUAL_UINT32( 1u, txChannel->ActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utUsart_TxBuf, txChannel->MemoryAddr );
    TEST_ASSERT_EQUAL_UINT32( 4u, txChannel->DataCount );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAT, UT_USART_REG->CR3 & USART_CR3_DMAT );

    /* DMA transfer complete - last byte written to TDR */
    utUsart_DmaConfig[ 0u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );

    Ut_Usart_Call_Isr( USART_ISR_TC | USART_ISR_TXE );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );

    /* Stop without running transmission */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );
}


/**
 * \brief   Usart_Set_TxStop() stops running DMA transmission.
 *
 * \par Expected results
 * - TX channel disabled, CR3.DMAT and CR1.TCIE cleared, no complete callback.
 */
void Ut_Usart_Dma_TxStop_ChannelAndRequestDisabled( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    uint32_t           stopCnt    = 0u;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );

    stopCnt = utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].InactiveCnt;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( stopCnt + 1u, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ].InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_DMAT );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );
}


/**
 * \brief   DMA reception with idle line end reports count from the channel.
 *
 * \details DMA mode, RX end mode idle, one shot buffer 8 bytes. Reception started, DMA
 *          remaining count 5 (3 bytes received), ISR with IDLE.
 *
 * \par Expected results
 * - Start: RX channel armed with buffer and 8 bytes, CR3.DMAR / EIE, CR1.PEIE / IDLEIE set,
 *   RXNEIE not set (data are read by DMA).
 * - RX count 3, ICR.IDLECF written, end callback 1x with 3 bytes, reception stopped (channel
 *   disabled, DMAR cleared, interrupts disabled).
 */
void Ut_Usart_Dma_ReceptionIdleEnd_CountFromChannel( void )
{
    usart_DataConfig_t           dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_RxDataCnt_t            rxCnt      = 0u;
    usart_FunctionState_t        rxState    = USART_FUNCTION_ACTIVE;
    const ut_UsartDmaChannel_t * rxChannel  = &utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_RX_CH ];

    dataConfig.RxEndMode = USART_RX_END_IDLE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, rxChannel->ActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utUsart_RxBuf, rxChannel->MemoryAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RX_SIZE, rxChannel->DataCount );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAR | USART_CR3_EIE, UT_USART_REG->CR3 & ( USART_CR3_DMAR | USART_CR3_EIE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_PEIE | USART_CR1_IDLEIE, UT_USART_REG->CR1 & ( USART_CR1_PEIE | USART_CR1_IDLEIE | USART_CR1_RXNEIE ) );

    utUsart_DmaRemaining = 5u;
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL_UINT16( 3u, rxCnt );

    UT_USART_REG->ICR = 0u;
    Ut_Usart_Call_Isr( USART_ISR_IDLE );

    TEST_ASSERT_EQUAL_HEX32( USART_ICR_IDLECF, UT_USART_REG->ICR & USART_ICR_IDLECF );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 3u, utUsart_RxEndBytes );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
    TEST_ASSERT_EQUAL_UINT32( 2u, rxChannel->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_DMAR | USART_CR3_EIE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_PEIE | USART_CR1_IDLEIE ) );
}


/**
 * \brief   DMA reception reports half / full buffer from DMA callbacks.
 *
 * \details DMA mode, one shot buffer. DMA half transfer and transfer complete callbacks.
 *
 * \par Expected results
 * - Half callback 1x, complete callback 1x, reception stopped.
 */
void Ut_Usart_Dma_ReceptionHalfAndComplete_Callbacks( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    utUsart_DmaConfig[ 1u ].HalfTransferCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxHalfCnt );

    utUsart_DmaConfig[ 1u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
}


/**
 * \brief   Circular DMA reception keeps running after full buffer.
 *
 * \details DMA mode, circular buffer. DMA transfer complete callback.
 *
 * \par Expected results
 * - Complete callback 1x, reception active, channel not stopped.
 */
void Ut_Usart_Dma_CircularReception_KeepsRunning( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_INACTIVE;
    uint32_t              stopCnt    = 0u;

    dataConfig.RxBufferMode = USART_BUFFER_MODE_CIRCULAR;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utUsart_DmaConfig[ 1u ].TransferMode );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    stopCnt = utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_RX_CH ].InactiveCnt;

    utUsart_DmaConfig[ 1u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );
    TEST_ASSERT_EQUAL_UINT32( stopCnt, utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_RX_CH ].InactiveCnt );
}


/**
 * \brief   Reception errors in DMA mode are reported from USART interrupt, DMA keeps the data.
 *
 * \details DMA mode, reception running. ISR with RXNE + FE (byte pending for DMA), then
 *          with ORE.
 *
 * \par Expected results
 * - Framing error and overrun reported, no byte stored by CPU (buffer unchanged).
 */
void Ut_Usart_Dma_ReceptionErrors_ReportedFromUsartIsr( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->RDR = 0x7Eu;
    Ut_Usart_Call_Isr( USART_ISR_RXNE | USART_ISR_FE );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_FRAMING, utUsart_LastError );

    Ut_Usart_Call_Isr( USART_ISR_ORE );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_OVERRUN, utUsart_LastError );

    TEST_ASSERT_EQUAL_HEX8( 0u, utUsart_RxBuf[ 0u ] );
}


/**
 * \brief   DMA transfer errors stop the direction and are reported.
 *
 * \details DMA mode, transmission and reception running. TX channel error callback, RX
 *          channel error callback.
 *
 * \par Expected results
 * - Error callback with USART_XFER_ERROR_DMA_TRANSFER 2x, TX and RX state inactive.
 */
void Ut_Usart_Dma_TransferError_DirectionStoppedAndReported( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t state      = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );

    utUsart_DmaConfig[ 0u ].TransferErrorCallback();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_DMA_TRANSFER, utUsart_LastError );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, state );

    utUsart_DmaConfig[ 1u ].TransferErrorCallback();
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, state );
}


/**
 * \brief   DMA channel failure during transmission start is reported.
 *
 * \details DMA mode. Dma_Set_TransferActive() of TX channel fails.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, TX state inactive, DMAT not set.
 */
void Ut_Usart_Dma_TxStartChannelError_ReturnsError( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t txState    = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    Dma_Set_TransferActive_Stub( NULL );
    Dma_Set_TransferActive_ExpectAndReturn( UT_USART_DMA, UT_USART_DMA_TX_CH, DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_DMAT );
}


/**
 * \brief   DMA channel failure during reception start is reported.
 *
 * \details DMA mode. Dma_Set_TransferActive() of RX channel fails.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, RX state inactive, DMAR and reception interrupts not enabled.
 */
void Ut_Usart_Dma_RxStartChannelError_ReturnsError( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    Dma_Set_TransferActive_Stub( NULL );
    Dma_Set_TransferActive_ExpectAndReturn( UT_USART_DMA, UT_USART_DMA_RX_CH, DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_DMAR | USART_CR3_EIE ) );
}


/**
 * \brief   Usart_Deinit() releases DMA channels of the data handling.
 *
 * \par Expected results
 * - Both channels disabled, their interrupts and NVIC lines disabled, DMAT / DMAR cleared.
 */
void Ut_Usart_Dma_Deinit_ChannelsReleased( void )
{
    usart_DataConfig_t           dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    const ut_UsartDmaChannel_t * txChannel  = &utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_TX_CH ];
    const ut_UsartDmaChannel_t * rxChannel  = &utUsart_DmaChannel[ UT_USART_DMA ][ UT_USART_DMA_RX_CH ];

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    Ut_Usart_Ignore_PeriphMocks();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, txChannel->TcIrq + txChannel->TeIrq + txChannel->NvicIrq );
    TEST_ASSERT_EQUAL_UINT32( 0u, rxChannel->TcIrq + rxChannel->TeIrq + rxChannel->HtIrq + rxChannel->NvicIrq );
    TEST_ASSERT_TRUE( 0u < txChannel->InactiveCnt );
    TEST_ASSERT_TRUE( 0u < rxChannel->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_DMAT | USART_CR3_DMAR ) );
}


/**
 * \brief   Other USART/UART peripherals use their own interrupt, DMA streams and DMA
 *          callbacks.
 *
 * \details Every USART/UART of the MCU except USART1 (its items are used by other tests) in DMA
 *          mode (TX and RX, one shot buffer), every item of the DMA stream lists of the bus,
 *          items paired by the bus. Captured USART interrupt without flags, then DMA callbacks
 *          captured from Dma_Init() are called: RX half / complete / error, TX complete / error.
 *
 * \par Expected results
 * - TX / RX stream with the channel selection of the peripheral (STM32F7 DMA request mapping),
 *   TDR / RDR of the peripheral.
 * - Interrupt without flags: no callback.
 * - RX half: RxHalfCallback, RX complete: RxCompleteCallback, RX / TX error: ErrorCallback with
 *   USART_XFER_ERROR_DMA_TRANSFER, TX complete: TC interrupt enabled in CR1 of the peripheral.
 * - Usart_Deinit(): USART_REQUEST_OK.
 */
void Ut_Usart_Dma_OtherPeriphCallbacks_OwnPeripheralReported( void )
{
    const struct
    {
        usart_PeriphId_t      UsartId;
        USART_TypeDef *       PeriphReg;
        usart_TxDma_t         TxDma;
        usart_RxDma_t         RxDma;
        usart_DmaPeriphId_t   DmaId;
        usart_DmaChannelId_t  TxStream;
        usart_DmaChannelId_t  RxStream;
        dma_PeriphReqId_t     TxSel;
        dma_PeriphReqId_t     RxSel;
    }   periphLut[] =
    {
        { USART_BUS_2, USART2, USART_TX_DMA_BUS2_DMA1_STREAM6, USART_RX_DMA_BUS2_DMA1_STREAM5, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_6, USART_DMA_CHANNEL_5, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
        { USART_BUS_3, USART3, USART_TX_DMA_BUS3_DMA1_STREAM3, USART_RX_DMA_BUS3_DMA1_STREAM1, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_3, USART_DMA_CHANNEL_1, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
        { USART_BUS_3, USART3, USART_TX_DMA_BUS3_DMA1_STREAM4, USART_RX_DMA_BUS3_DMA1_STREAM1, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_4, USART_DMA_CHANNEL_1, DMA_REQ_CHANNEL_7, DMA_REQ_CHANNEL_4 },
        { USART_BUS_4, UART4,  USART_TX_DMA_BUS4_DMA1_STREAM4, USART_RX_DMA_BUS4_DMA1_STREAM2, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_4, USART_DMA_CHANNEL_2, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
        { USART_BUS_5, UART5,  USART_TX_DMA_BUS5_DMA1_STREAM7, USART_RX_DMA_BUS5_DMA1_STREAM0, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_7, USART_DMA_CHANNEL_0, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
        { USART_BUS_6, USART6, USART_TX_DMA_BUS6_DMA2_STREAM6, USART_RX_DMA_BUS6_DMA2_STREAM1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_6, USART_DMA_CHANNEL_1, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
        { USART_BUS_6, USART6, USART_TX_DMA_BUS6_DMA2_STREAM7, USART_RX_DMA_BUS6_DMA2_STREAM2, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, USART_DMA_CHANNEL_2, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
        { USART_BUS_7, UART7,  USART_TX_DMA_BUS7_DMA1_STREAM1, USART_RX_DMA_BUS7_DMA1_STREAM3, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_1, USART_DMA_CHANNEL_3, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
        { USART_BUS_8, UART8,  USART_TX_DMA_BUS8_DMA1_STREAM0, USART_RX_DMA_BUS8_DMA1_STREAM6, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_0, USART_DMA_CHANNEL_6, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
    };

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        usart_DataConfig_t         dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
        const dma_ConfigStruct_t * txConfig   = NULL;
        const dma_ConfigStruct_t * rxConfig   = NULL;

        dataConfig.TxDma = periphLut[ idx ].TxDma;
        dataConfig.RxDma = periphLut[ idx ].RxDma;

        Ut_Usart_Ignore_PeriphMocks();
        Nvic_Set_PeriphIrq_Handler_Stub( Ut_Usart_NvicAnyHandlerStub );
        utUsart_AnyIsr        = NULL;
        utUsart_DmaInitCnt    = 0u;
        utUsart_RxHalfCnt     = 0u;
        utUsart_RxCompleteCnt = 0u;
        utUsart_ErrorCnt      = 0u;

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( periphLut[ idx ].UsartId, &dataConfig ) );
        TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_DmaInitCnt );
        TEST_ASSERT_NOT_NULL( utUsart_AnyIsr );

        for( uint32_t cfgIdx = 0u; 2u > cfgIdx; cfgIdx++ )
        {
            if( DMA_DIR_MEMORY_TO_PERIPH == utUsart_DmaConfig[ cfgIdx ].Direction )
            {
                txConfig = &utUsart_DmaConfig[ cfgIdx ];
            }
            else
            {
                rxConfig = &utUsart_DmaConfig[ cfgIdx ];
            }
        }

        TEST_ASSERT_NOT_NULL( txConfig );
        TEST_ASSERT_NOT_NULL( rxConfig );
        TEST_ASSERT_EQUAL( (dma_PeriphId_t)periphLut[ idx ].DmaId,     txConfig->DmaPeriphId );
        TEST_ASSERT_EQUAL( (dma_PeriphId_t)periphLut[ idx ].DmaId,     rxConfig->DmaPeriphId );
        TEST_ASSERT_EQUAL( (dma_ChannelId_t)periphLut[ idx ].TxStream, txConfig->DmaChannel );
        TEST_ASSERT_EQUAL( (dma_ChannelId_t)periphLut[ idx ].RxStream, rxConfig->DmaChannel );
        TEST_ASSERT_EQUAL( periphLut[ idx ].TxSel, txConfig->PeripheralReqId );
        TEST_ASSERT_EQUAL( periphLut[ idx ].RxSel, rxConfig->PeripheralReqId );

        /* Channel selection of the list items equals the one used by the request map */
        TEST_ASSERT_EQUAL_UINT32( (uint32_t)periphLut[ idx ].TxSel >> DMA_SxCR_CHSEL_Pos, USART_DMA_BIT_MASK_DECODE_CHSEL( periphLut[ idx ].TxDma ) );
        TEST_ASSERT_EQUAL_UINT32( (uint32_t)periphLut[ idx ].RxSel >> DMA_SxCR_CHSEL_Pos, USART_DMA_BIT_MASK_DECODE_CHSEL( periphLut[ idx ].RxDma ) );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&periphLut[ idx ].PeriphReg->TDR, txConfig->PeriphAddress );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&periphLut[ idx ].PeriphReg->RDR, rxConfig->PeriphAddress );

        periphLut[ idx ].PeriphReg->ISR = 0u;
        utUsart_AnyIsr();
        TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_RxHalfCnt + utUsart_RxCompleteCnt + utUsart_ErrorCnt );

        rxConfig->HalfTransferCallback();
        TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxHalfCnt );
        rxConfig->TransferCompleteCallback();
        TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
        rxConfig->TransferErrorCallback();
        TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
        TEST_ASSERT_EQUAL( USART_XFER_ERROR_DMA_TRANSFER, utUsart_LastError );

        periphLut[ idx ].PeriphReg->CR1 &= ~USART_CR1_TCIE;
        txConfig->TransferCompleteCallback();
        TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, periphLut[ idx ].PeriphReg->CR1 & USART_CR1_TCIE );
        txConfig->TransferErrorCallback();
        TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_ErrorCnt );

        Ut_Usart_Ignore_PeriphMocks();
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( periphLut[ idx ].UsartId ) );
    }
}


/**
 * \brief   Items of the DMA stream lists carry bus, DMA peripheral, stream and channel selection of
 *          the stream.
 *
 * \details Expected values are written as (bus index, DMA peripheral index, stream number, channel
 *          selection number) taken from the DMA request mapping of the STM32F7 reference manuals,
 *          independently of the encoding macro.
 *
 * \par Expected results
 * - Every transmit and receive item of USART1 - UART8 carries the expected bit-fields.
 * - Unused items of both lists equal USART_DMA_CODE_UNUSED, the decoding macros return the fields.
 */
void Ut_Usart_DmaLists_Items_EncodeBusDmaStreamAndChannelSelection( void )
{
    /* USART1 (DMA2, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 0u, 1u, 7u, 4u ), USART_TX_DMA_BUS1_DMA2_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 0u, 1u, 2u, 4u ), USART_RX_DMA_BUS1_DMA2_STREAM2 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 0u, 1u, 5u, 4u ), USART_RX_DMA_BUS1_DMA2_STREAM5 );

    /* USART2 (DMA1, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 1u, 0u, 6u, 4u ), USART_TX_DMA_BUS2_DMA1_STREAM6 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 1u, 0u, 5u, 4u ), USART_RX_DMA_BUS2_DMA1_STREAM5 );

    /* USART3 (DMA1, channel selection 4, TX stream 4 with channel selection 7) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 2u, 0u, 3u, 4u ), USART_TX_DMA_BUS3_DMA1_STREAM3 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 2u, 0u, 4u, 7u ), USART_TX_DMA_BUS3_DMA1_STREAM4 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 2u, 0u, 1u, 4u ), USART_RX_DMA_BUS3_DMA1_STREAM1 );

    /* UART4 (DMA1, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 3u, 0u, 4u, 4u ), USART_TX_DMA_BUS4_DMA1_STREAM4 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 3u, 0u, 2u, 4u ), USART_RX_DMA_BUS4_DMA1_STREAM2 );

    /* UART5 (DMA1, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 4u, 0u, 7u, 4u ), USART_TX_DMA_BUS5_DMA1_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 4u, 0u, 0u, 4u ), USART_RX_DMA_BUS5_DMA1_STREAM0 );

    /* USART6 (DMA2, channel selection 5) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 5u, 1u, 6u, 5u ), USART_TX_DMA_BUS6_DMA2_STREAM6 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 5u, 1u, 7u, 5u ), USART_TX_DMA_BUS6_DMA2_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 5u, 1u, 1u, 5u ), USART_RX_DMA_BUS6_DMA2_STREAM1 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 5u, 1u, 2u, 5u ), USART_RX_DMA_BUS6_DMA2_STREAM2 );

    /* UART7 (DMA1, channel selection 5) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 6u, 0u, 1u, 5u ), USART_TX_DMA_BUS7_DMA1_STREAM1 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 6u, 0u, 3u, 5u ), USART_RX_DMA_BUS7_DMA1_STREAM3 );

    /* UART8 (DMA1, channel selection 5) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 7u, 0u, 0u, 5u ), USART_TX_DMA_BUS8_DMA1_STREAM0 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( 7u, 0u, 6u, 5u ), USART_RX_DMA_BUS8_DMA1_STREAM6 );

    /* Decoding of the fields */
    TEST_ASSERT_EQUAL_UINT32( USART_BUS_6,        USART_DMA_BIT_MASK_DECODE_PERIPH( USART_RX_DMA_BUS6_DMA2_STREAM2 ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_PERIPH_2, USART_DMA_BIT_MASK_DECODE_DMA( USART_RX_DMA_BUS6_DMA2_STREAM2 ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_CHANNEL_2, USART_DMA_BIT_MASK_DECODE_STREAM( USART_RX_DMA_BUS6_DMA2_STREAM2 ) );
    TEST_ASSERT_EQUAL_UINT32( 5u,                 USART_DMA_BIT_MASK_DECODE_CHSEL( USART_RX_DMA_BUS6_DMA2_STREAM2 ) );

    /* Unused stream */
    TEST_ASSERT_EQUAL_HEX32( USART_DMA_CODE_UNUSED, USART_TX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( USART_DMA_CODE_UNUSED, USART_RX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_UINT32( USART_BUS_CNT,         USART_DMA_BIT_MASK_DECODE_PERIPH( USART_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_PERIPH_CNT,  USART_DMA_BIT_MASK_DECODE_DMA( USART_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_CHANNEL_CNT, USART_DMA_BIT_MASK_DECODE_STREAM( USART_TX_DMA_UNUSED ) );
}

/* ======================== ADDITIONAL CONFIGURATIONS ======================= */

/**
 * \brief   Usart_Init() configures Driver Enable pin and data handling.
 *
 * \details Initializes USART1 with DE pin PA12, over-sampling 16 and data handling in
 *          polling mode.
 *
 * \par Expected results
 * - Gpio_Init() with PA12, alternate function 7, push-pull.
 * - USART_REQUEST_OK, data handling configuration reads back polling mode.
 */
void Ut_Usart_Init_DePinAndDataConfig_Configured( void )
{
    usart_BusConfig_t  config     = Ut_Usart_Get_BusConfig();
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );
    usart_DataConfig_t readConfig;
    gpio_Config_t      pinConfig  = { 0u };

    config.BusDePin     = USART_DE_PIN_BUS1_PA12;
    config.Oversampling = USART_OVERSAMPLING_16;
    config.DataConfig   = &dataConfig;

    pinConfig.PortId         = GPIO_PORT_A;
    pinConfig.PinId          = GPIO_PIN_ID_12;
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = GPIO_ALT_FUNC_7;

    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Gpio_Init_ExpectAndReturn( &pinConfig, GPIO_REQUEST_OK );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
    TEST_ASSERT_EQUAL( USART_XFER_MODE_POLL, readConfig.RxMode );
}


/**
 * \brief   Pins existing only on some STM32F7 lines are configured with their alternate
 *          function.
 *
 * \details STM32F76x / F77x: UART7 TX PB4 (AF12) and UART5 RX PB5 (AF1). STM32F72x / F73x:
 *          UART4 TX PH13 (AF8). Other lines (no line specific pin): test ignored.
 * \note    Bug AB#1145: pin enumerations taken over from STM32F4 missed the pins of the
 *          STM32F72x / F76x lines.
 *
 * \par Expected results
 * - Gpio_Init() with port, pin and alternate function of the pin, USART_REQUEST_OK.
 */
void Ut_Usart_InitGpio_LineSpecificPins_AlternateFunction( void )
{
    gpio_Config_t pinConfig = { 0u };

    pinConfig.PinMode    = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull    = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed   = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType = GPIO_PIN_OUTPUT_PUSHPULL;

#if defined(STM32F765xx) || \
    defined(STM32F767xx) || \
    defined(STM32F769xx) || \
    defined(STM32F777xx) || \
    defined(STM32F779xx)
    pinConfig.PortId         = GPIO_PORT_B;
    pinConfig.PinId          = GPIO_PIN_ID_4;
    pinConfig.PinAltFunction = GPIO_ALT_FUNC_12;
    Gpio_Init_ExpectAndReturn( &pinConfig, GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitTxGpio( USART_TX_PIN_BUS7_PB4 ) );

    pinConfig.PinId          = GPIO_PIN_ID_5;
    pinConfig.PinAltFunction = GPIO_ALT_FUNC_1;
    Gpio_Init_ExpectAndReturn( &pinConfig, GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitRxGpio( USART_RX_PIN_BUS5_PB5 ) );
#elif defined(STM32F722xx) || \
      defined(STM32F723xx) || \
      defined(STM32F730xx) || \
      defined(STM32F732xx) || \
      defined(STM32F733xx)
    pinConfig.PortId         = GPIO_PORT_H;
    pinConfig.PinId          = GPIO_PIN_ID_13;
    pinConfig.PinAltFunction = GPIO_ALT_FUNC_8;
    Gpio_Init_ExpectAndReturn( &pinConfig, GPIO_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitTxGpio( USART_TX_PIN_BUS4_PH13 ) );
#else
    TEST_IGNORE_MESSAGE( "No line specific USART pin on this MCU" );
#endif
}


/**
 * \brief   Usart_Init() reports error of clock state read.
 *
 * \details Rcc_Get_PeriphState() returns error.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, clock not enabled (strict mock), peripheral not enabled.
 */
void Ut_Usart_Init_ClockStateError_ReturnsError( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    Rcc_Get_PeriphState_ExpectAnyArgsAndReturn( RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Deinit() executes all steps and reports a failed step.
 *
 * \details Initialized USART1, Rcc_Set_PeriphInactive() returns error.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, peripheral disabled (CR1.UE = 0).
 */
void Ut_Usart_Deinit_ClockError_ReturnsError( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    Ut_Usart_Init( &config );

    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Deinit( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Driver Enable times with over-sampling 8 are calculated in 1/8 bit units.
 *
 * \details Kernel clock 64 MHz, over-sampling 8, 115200 Bd (read back 115315 Bd, sample clock
 *          922520 Hz). Sets assertion time 10 us and reads it back.
 *
 * \par Expected results
 * - DEAT = 9 (10 us x 922520 Hz / 999999), read back (9 + 1) / 922520 Hz = 10 us.
 */
void Ut_Usart_AssertDeassertTimes_Oversampling8_HalfBitUnits( void )
{
    usart_AssertTime_us_t   assertTime   = 0u;
    usart_DeassertTime_us_t deassertTime = 0u;

    Rcc_Get_PeriphClk_Stub( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_8 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_AssertDeassertTimes( UT_USART_BUS, 10u, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( 9u << USART_CR1_DEAT_Pos, UT_USART_REG->CR1 & USART_CR1_DEAT );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_AssertDeassertTimes( UT_USART_BUS, &assertTime, &deassertTime ) );
    TEST_ASSERT_EQUAL_UINT8( 10u, assertTime );
}


/**
 * \brief   Functions with peripheral identification reject peripheral out of range and NULL
 *          output pointers.
 *
 * \details Calls setters / getters of the public interface not covered by other tests with
 *          USART_BUS_CNT and getters of USART1 with NULL output pointer. Reads RX count
 *          without data handling.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases, no RCC / NVIC / GPIO / DMA call (strict mocks).
 */
void Ut_Usart_AllFunctions_InvalidArgs_ReturnsError( void )
{
    const usart_PeriphId_t  inv       = USART_BUS_CNT;
    usart_FlagState_t       flag      = USART_FLAG_INACTIVE;
    usart_FunctionState_t   func      = USART_FUNCTION_INACTIVE;
    usart_TransferMode_t    mode      = USART_TRANSFER_MODE_NONE;
    usart_FlowControl_t     flow      = USART_FLOW_CONTROL_NONE;
    usart_DeFeatureState_t  deState   = USART_DE_DISABLED;
    usart_DePolarity_t      dePol     = USART_DE_ACTIVE_HIGH;
    usart_AssertTime_us_t   assertT   = 0u;
    usart_DeassertTime_us_t deassertT = 0u;
    usart_HalfDuplex_t      half      = USART_HALF_DUPLEX_INACTIVE;
    usart_RxDataCnt_t       rxCnt     = 0u;
    usart_DataConfig_t      dataCfg;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TransferMode( inv, USART_TRANSFER_MODE_TX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TransferMode( inv, &mode ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TransferMode( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_FlowControl( inv, USART_FLOW_CONTROL_NONE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_FlowControl( inv, &flow ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DriverEnableState( inv, USART_DE_DISABLED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DriverEnableState( inv, &deState ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DriverEnablePolarity( inv, USART_DE_ACTIVE_HIGH ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DriverEnablePolarity( inv, &dePol ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_AssertDeassertTimes( inv, 1u, 1u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_AssertDeassertTimes( inv, &assertT, &deassertT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_AssertDeassertTimes( UT_USART_BUS, NULL, &deassertT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Oversampling( inv, USART_OVERSAMPLING_16 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_HalfDuplexState( inv, USART_HALF_DUPLEX_ACTIVE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_HalfDuplexState( inv, &half ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutActive( inv, 1u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxTimeoutState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PinLevels( inv, USART_RX_PIN_STANDARD, USART_TX_PIN_STANDARD ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( inv, &dataCfg ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStop( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxState( inv, &func ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStop( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxState( inv, &func ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxCount( inv, &rxCnt ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaTxRequestActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaTxRequestInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DmaTxReqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaRxRequestActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaRxRequestInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DmaRxReqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxNotEmptyIrqActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxNotEmptyIrqInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxNotEmptyIrqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxEmptyIrqActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxEmptyIrqInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxEmptyIrqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxCompleteIrqActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxCompleteIrqInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxCompleteIrqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IdleIrqActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IdleIrqInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_IdleIrqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutIrqActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutIrqInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxTimeoutIrqState( inv, &flag ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_ErrorIrqActive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_ErrorIrqInactive( inv ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_ErrorIrqState( inv, &flag ) );
}

/* ============================ FIXED DEFECTS =============================== */

/**
 * \brief   Getters reject NULL output pointer.
 *
 * \details Calls Usart_Get_Oversampling(), Usart_Get_RxTimeoutState() and
 *          Usart_Get_PinLevels() (each pointer) with NULL output pointer.
 * \note    Bug AB#968: STM32H5 implementation dereferences the output pointers without
 *          NULL check (write to address 0) - STM32F7 contains the fix of the STM32G4
 *          implementation.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases.
 */
void Ut_Usart_Getters_NullPtr_ReturnsError( void )
{
    usart_RxPinLevel_t rxLevel = USART_RX_PIN_STANDARD;
    usart_TxPinLevel_t txLevel = USART_TX_PIN_STANDARD;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Oversampling( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxTimeoutState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PinLevels( UT_USART_BUS, NULL, &txLevel ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PinLevels( UT_USART_BUS, &rxLevel, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Oversampling( USART_BUS_CNT, NULL ) );
}


/**
 * \brief   Idle interrupt is deactivated independently of the transmission complete
 *          interrupt.
 *
 * \details CR1: TCIE and IDLEIE set (idle interrupt enabled during running transmission).
 *          Deactivates the idle interrupt.
 * \note    Bug AB#464 (STM32H5 implementation AB#972): the deactivation read back TCIE
 *          instead of IDLEIE - error after the read-back timeout while TCIE is set.
 *
 * \par Expected results
 * - USART_REQUEST_OK, CR1.IDLEIE cleared, CR1.TCIE unchanged.
 */
void Ut_Usart_Set_IdleIrqInactive_TcIrqEnabled_IdleDisabled( void )
{
    UT_USART_REG->CR1 = USART_CR1_TCIE | USART_CR1_IDLEIE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IdleIrqInactive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, UT_USART_REG->CR1 & ( USART_CR1_TCIE | USART_CR1_IDLEIE ) );
}


/**
 * \brief   Interrupt source activation reports failed NVIC activation.
 *
 * \details Nvic_Set_PeriphIrq_Active() returns error. Activates RXNE, TXE, TC, IDLE and
 *          error interrupt.
 * \note    Bug AB#974 (STM32H5 implementation): the result of the NVIC activation was
 *          ignored - USART_REQUEST_OK returned although the interrupt can not be serviced.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR for every interrupt source (enable bits are set).
 */
void Ut_Usart_Set_IrqActive_NvicError_ReturnsError( void )
{
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxNotEmptyIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxEmptyIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxCompleteIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IdleIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_ErrorIrqActive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_IDLEIE | USART_CR1_PEIE,
                             UT_USART_REG->CR1 & ( USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_IDLEIE | USART_CR1_PEIE ) );
}


/**
 * \brief   Maximal receiver timeout is accepted.
 *
 * \details Activates receiver timeout USART_RX_TIMEOUT_MAX (0xFFFFFF).
 * \note    Bug AB#971: STM32H5 implementation refuses USART_RX_TIMEOUT_MAX - STM32F7 contains
 *          the fix of the STM32G4 implementation.
 *
 * \par Expected results
 * - USART_REQUEST_OK, CR2.RTOEN set, RTOR.RTO = 0xFFFFFF.
 */
void Ut_Usart_Set_RxTimeoutActive_MaximalValue_Accepted( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxTimeoutActive( UT_USART_BUS, USART_RX_TIMEOUT_MAX ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR2_RTOEN, UT_USART_REG->CR2 & USART_CR2_RTOEN );
    TEST_ASSERT_EQUAL_HEX32( 0xFFFFFFu, UT_USART_REG->RTOR & USART_RTOR_RTO );
}


/**
 * \brief   Driver Enable times of USART are calculated in sample time units without overflow.
 *
 * \details Kernel clock 170 MHz, over-sampling 16, 10.625 MBd (sample clock 170 MHz). Sets
 *          assertion / de-assertion time 177 us, then 1 us / 0 us and reads them back. Clears
 *          BRR and reads the times.
 * \note    Bug AB#970: STM32H5 implementation calculates time x baud rate x over-sampling in
 *          32 bits (177 x 170 MHz wraps to DEAT = 25) and divides by zero without configured
 *          baud rate - STM32F7 contains the fix of the STM32G4 implementation.
 *
 * \par Expected results
 * - 177 us: DEAT = DEDT = 31 (limit).
 * - 1 us / 0 us: DEAT = 31 (170 limited), DEDT = 0; read back 0 us / 0 us ((DEAT + 1) / 170 MHz
 *   and (DEDT + 1) / 170 MHz are below 1 us).
 * - BRR = 0: Usart_Get_AssertDeassertTimes() / Usart_Set_AssertDeassertTimes() return
 *   USART_REQUEST_ERROR.
 */
void Ut_Usart_AssertDeassertTimes_HighBaudrate_NoOverflow( void )
{
    usart_AssertTime_us_t   assertTime   = 0xFFu;
    usart_DeassertTime_us_t deassertTime = 0xFFu;

    utUsart_ClkHz = 170000000u;
    Rcc_Get_PeriphClk_Stub( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, 10625000u ) );
    TEST_ASSERT_EQUAL_HEX32( 16u, UT_USART_REG->BRR );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_AssertDeassertTimes( UT_USART_BUS, 177u, 177u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_DEAT | USART_CR1_DEDT, UT_USART_REG->CR1 & ( USART_CR1_DEAT | USART_CR1_DEDT ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_AssertDeassertTimes( UT_USART_BUS, 1u, 0u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_DEAT, UT_USART_REG->CR1 & ( USART_CR1_DEAT | USART_CR1_DEDT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_AssertDeassertTimes( UT_USART_BUS, &assertTime, &deassertTime ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, assertTime );
    TEST_ASSERT_EQUAL_UINT8( 0u, deassertTime );

    UT_USART_REG->BRR = 0u;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_AssertDeassertTimes( UT_USART_BUS, &assertTime, &deassertTime ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_AssertDeassertTimes( UT_USART_BUS, 1u, 1u ) );
}


/**
 * \brief   Driver Enable times of USART with usual baud rate are written and read back.
 *
 * \details Kernel clock 64 MHz, over-sampling 16, 115200 Bd (sample clock 1.8432 MHz). Sets
 *          assertion time 10 us and de-assertion time 5 us and reads them back.
 *
 * \par Expected results
 * - DEAT = 18, DEDT = 9 (time x 1.8432 MHz / 999999).
 * - Read back (DEAT + 1) / 1.8432 MHz = 10 us, (DEDT + 1) / 1.8432 MHz = 5 us.
 */
void Ut_Usart_AssertDeassertTimes_115200Bd_RegisterAndReadBack( void )
{
    usart_AssertTime_us_t   assertTime   = 0u;
    usart_DeassertTime_us_t deassertTime = 0u;

    Rcc_Get_PeriphClk_Stub( Ut_Usart_RccGetClkStub );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_AssertDeassertTimes( UT_USART_BUS, 10u, 5u ) );
    TEST_ASSERT_EQUAL_HEX32( ( 18u << USART_CR1_DEAT_Pos ) | ( 9u << USART_CR1_DEDT_Pos ), UT_USART_REG->CR1 & ( USART_CR1_DEAT | USART_CR1_DEDT ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_AssertDeassertTimes( UT_USART_BUS, &assertTime, &deassertTime ) );
    TEST_ASSERT_EQUAL_UINT8( 10u, assertTime );
    TEST_ASSERT_EQUAL_UINT8( 5u, deassertTime );
}


/**
 * \brief   Requested RTS pin is held at inactive level before RTSE is set.
 *
 * \details Initializes the RTS / DE pin PA12, enables transmitter and receiver (USART disabled)
 *          and requests RTS flow control.
 *
 * \par Expected results
 * - Pin initialization without RTS request: no change of the pin.
 * - USART_REQUEST_OK, pin set high and switched to GPIO output while RTSE = 0, RTSE set.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_FlowControl_RtsRequested_PinHeldBeforeRtse( void )
{
    Ut_Usart_Init_RtsPin();
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_RtsEventCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 1u ].Cr3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, UT_USART_REG->CR3 & USART_CR3_RTSE );
}


/**
 * \brief   Held RTS pin is switched to the alternate function after UE is set.
 *
 * \details RTS flow control requested with the USART disabled, then the USART is enabled.
 *
 * \par Expected results
 * - USART_REQUEST_OK, one pin change to the alternate function with UE and RE set.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_PeriphActive_RtsRequested_PinReleasedAfterUe( void )
{
    Ut_Usart_Init_RtsPin();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );
    utUsart_RtsEventCnt = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_AF, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE | USART_CR1_RE, utUsart_RtsEvent[ 0u ].Cr1 & ( USART_CR1_UE | USART_CR1_RE ) );
}


/**
 * \brief   RTS pin is held at inactive level before UE is cleared.
 *
 * \details RTS flow control requested, USART enabled (pin in alternate function), then the
 *          USART is disabled.
 *
 * \par Expected results
 * - USART_REQUEST_OK, pin set high and switched to GPIO output while UE = 1, UE cleared.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_PeriphInactive_RtsRequested_PinHeldBeforeUeCleared( void )
{
    Ut_Usart_Set_RtsActive();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, utUsart_RtsEvent[ 1u ].Cr1 & USART_CR1_UE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   RTS pin follows the receiver of the enabled USART.
 *
 * \details RTS flow control requested, USART enabled, transfer mode set to transmitter only and
 *          back to transmitter and receiver.
 *
 * \par Expected results
 * - Transmitter only: pin held (high, GPIO output) while RE = 1, RE cleared.
 * - Transmitter and receiver: pin switched to the alternate function with RE = 1.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_TransferMode_RtsRequested_PinFollowsReceiver( void )
{
    Ut_Usart_Set_RtsActive();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RE, utUsart_RtsEvent[ 1u ].Cr1 & USART_CR1_RE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_RE );

    utUsart_RtsEventCnt = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_AF, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RE, utUsart_RtsEvent[ 0u ].Cr1 & USART_CR1_RE );
}


/**
 * \brief   RTS pin is released after RTSE is cleared.
 *
 * \details RTS flow control requested, USART enabled, then the flow control is disabled.
 *
 * \par Expected results
 * - USART_REQUEST_OK, pin held before UE is cleared (configuration of CR3), switched to the
 *   alternate function with RTSE = 0, no further pin change after UE is set again.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_FlowControl_RtsDisabled_PinReleasedAfterRtse( void )
{
    Ut_Usart_Set_RtsActive();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_NONE ) );

    TEST_ASSERT_EQUAL_UINT32( 3u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, utUsart_RtsEvent[ 1u ].Cr1 & USART_CR1_UE );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_AF, utUsart_RtsEvent[ 2u ].Event );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 2u ].Cr3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   RTS flow control without RTS pin initialized by the module keeps the GPIO untouched.
 *
 * \details RTS flow control requested without Usart_InitDeGpio(), USART enabled and disabled.
 *          GPIO functions of the pin are not expected (strict mocks).
 *
 * \par Expected results
 * - USART_REQUEST_OK for all calls, RTSE set, no GPIO call.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_FlowControl_RtsWithoutPin_GpioUntouched( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, UT_USART_REG->CR3 & USART_CR3_RTSE );
}


/**
 * \brief   RTS flow control is not enabled if the RTS pin can not be held.
 *
 * \details GPIO stubs return error, RTS flow control requested with the USART disabled.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, output level request only (mode not changed), RTSE stays 0.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Set_FlowControl_RtsPinError_ReturnsErrorWithoutRtse( void )
{
    Ut_Usart_Init_RtsPin();
    utUsart_RtsGpioState = GPIO_REQUEST_ERROR;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_RTSE );
}


/**
 * \brief   Usart_Deinit() holds a requested RTS pin before the peripheral is disabled.
 *
 * \details RTS flow control requested, USART enabled, then de-initialized.
 *
 * \par Expected results
 * - USART_REQUEST_OK, pin set high and switched to GPIO output while UE = 1.
 * - Module does not control the pin afterwards (Usart_Set_PeriphActive() without pin change).
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Deinit_RtsRequested_PinHeldBeforeDisable( void )
{
    Ut_Usart_Set_RtsActive();

    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, utUsart_RtsEvent[ 1u ].Cr1 & USART_CR1_UE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_RtsEventCnt );
}


/**
 * \brief   Usart_Init() with RTS flow control keeps the RTS pin inactive until UE and RE are set.
 *
 * \details Initializes USART1 with RTS / DE pin PA12, RTS flow control, transmitter and
 *          receiver.
 *
 * \par Expected results
 * - USART_REQUEST_OK, pin held (high, GPIO output) with UE = 0 and RTSE = 0, then switched to
 *   the alternate function with UE, RE and RTSE set.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Init_RtsFlowControl_PinHeldUntilUeAndRe( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.BusDePin      = UT_USART_RTS_PIN;
    config.HwFlowControl = USART_FLOW_CONTROL_RTS;
    config.TransferMode  = USART_TRANSFER_MODE_TX_RX;

    Gpio_Set_PinLevel_StubWithCallback( Ut_Usart_GpioSetLevelStub );
    Gpio_Set_PinMode_StubWithCallback( Ut_Usart_GpioSetModeStub );

    Ut_Usart_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 3u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 1u ].Cr1 & USART_CR1_UE );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 1u ].Cr3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_AF, utUsart_RtsEvent[ 2u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE | USART_CR1_RE, utUsart_RtsEvent[ 2u ].Cr1 & ( USART_CR1_UE | USART_CR1_RE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, utUsart_RtsEvent[ 2u ].Cr3 & USART_CR3_RTSE );
}


/**
 * \brief   Usart_InitRtsGpio() registers the RTS pin like the DE pin - requested RTS is held before RTSE is set.
 *
 * \details Initializes the RTS pin PA12 with Usart_InitRtsGpio(), enables transmitter and receiver
 *          (USART disabled) and requests RTS flow control.
 *
 * \par Expected results
 * - Pin initialization without RTS request: no change of the pin.
 * - USART_REQUEST_OK, pin set high and switched to GPIO output while RTSE = 0, RTSE set.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_InitRtsGpio_RtsRequested_PinHeldBeforeRtse( void )
{
    Gpio_Set_PinLevel_StubWithCallback( Ut_Usart_GpioSetLevelStub );
    Gpio_Set_PinMode_StubWithCallback( Ut_Usart_GpioSetModeStub );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitRtsGpio( USART_RTS_PIN_BUS1_PA12 ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_RtsEventCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 1u ].Cr3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, UT_USART_REG->CR3 & USART_CR3_RTSE );
}


/**
 * \brief   Usart_Init() with the RTS pin (BusRtsPin) and RTS flow control keeps the pin inactive until UE and RE are set.
 *
 * \details Initializes USART1 with RTS pin PA12 passed as BusRtsPin, RTS flow control, transmitter
 *          and receiver.
 *
 * \par Expected results
 * - USART_REQUEST_OK, pin held (high, GPIO output) with UE = 0 and RTSE = 0, then switched to
 *   the alternate function with UE, RE and RTSE set.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void Ut_Usart_Init_RtsPinAndRtsFlowControl_PinHeldUntilUeAndRe( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_BusConfig();

    config.BusRtsPin     = USART_RTS_PIN_BUS1_PA12;
    config.HwFlowControl = USART_FLOW_CONTROL_RTS;
    config.TransferMode  = USART_TRANSFER_MODE_TX_RX;

    Gpio_Set_PinLevel_StubWithCallback( Ut_Usart_GpioSetLevelStub );
    Gpio_Set_PinMode_StubWithCallback( Ut_Usart_GpioSetModeStub );

    Ut_Usart_Init( &config );

    TEST_ASSERT_EQUAL_UINT32( 3u, utUsart_RtsEventCnt );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_LEVEL_HIGH, utUsart_RtsEvent[ 0u ].Event );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_OUTPUT, utUsart_RtsEvent[ 1u ].Event );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 1u ].Cr1 & USART_CR1_UE );
    TEST_ASSERT_EQUAL_HEX32( 0u, utUsart_RtsEvent[ 1u ].Cr3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_AF, utUsart_RtsEvent[ 2u ].Event );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE | USART_CR1_RE, utUsart_RtsEvent[ 2u ].Cr1 & ( USART_CR1_UE | USART_CR1_RE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, utUsart_RtsEvent[ 2u ].Cr3 & USART_CR3_RTSE );
}


/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief NVIC handler registration stub - stores ISR of the USART.
 */
static nvic_RequestState_t Ut_Usart_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( utUsart_Nvic, irqId );
    TEST_ASSERT_NOT_NULL( irqHandler );

    utUsart_Isr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief RCC kernel clock stub of USART1 - returns \ref utUsart_ClkHz.
 */
static rcc_RequestState_t Ut_Usart_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;

    if( UT_USART_RCC != periphId )
    {
        TEST_FAIL_MESSAGE( "Kernel clock of unexpected peripheral" );
    }
    else
    {
        /* Clock of tested peripheral */
    }

    *periphClk = utUsart_ClkHz;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC clock state stub - returns inactive clock.
 */
static rcc_RequestState_t Ut_Usart_RccGetStateStub( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt )
{
    (void)callCnt;
    (void)periphId;

    *funcState = RCC_FUNCTION_INACTIVE;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief Ignores all calls of RCC / NVIC / GPIO functions used by peripheral configuration
 *        and data handling, DMA functions are replaced by recording stubs.
 */
static void Ut_Usart_Ignore_PeriphMocks( void )
{
    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_OK );
    Gpio_Set_PinLevel_IgnoreAndReturn( GPIO_REQUEST_OK );
    Gpio_Set_PinMode_IgnoreAndReturn( GPIO_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_StubWithCallback( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
    Ut_Usart_Stub_DmaMocks();
}


/**
 * \brief Releases data handling of the previous test (static module context) and
 *        re-initializes the mocks.
 */
static void Ut_Usart_Release( void )
{
    Ut_Usart_Ignore_PeriphMocks();

    for( usart_PeriphId_t usartId = (usart_PeriphId_t)0u; USART_BUS_CNT > usartId; usartId++ )
    {
        (void)Usart_Deinit( usartId );
    }

    MockRcc_Port_Destroy();
    MockNvic_Port_Destroy();
    MockGpio_Port_Destroy();
    MockDma_Port_Destroy();
    MockRcc_Port_Init();
    MockNvic_Port_Init();
    MockGpio_Port_Init();
    MockDma_Port_Init();
}


/**
 * \brief Returns bus configuration of USART1 (defaults, oversampling 16).
 */
static usart_BusConfig_t Ut_Usart_Get_BusConfig( void )
{
    usart_BusConfig_t config;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &config ) );
    config.PeriphId = UT_USART_BUS;

    return ( config );
}


/**
 * \brief Returns data handling configuration of both directions in the same mode (DMA: TX
 *        DMA2 stream 7, RX DMA2 stream 2 - USART1 requests).
 *
 * \param xferMode [in]: Transmission and reception mode
 * \param rxSize   [in]: Reception buffer size
 */
static usart_DataConfig_t Ut_Usart_Get_DataConfig( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize )
{
    usart_DataConfig_t dataConfig;

    dataConfig.TxMode             = xferMode;
    dataConfig.RxMode             = xferMode;
    dataConfig.RxBuffer           = utUsart_RxBuf;
    dataConfig.RxBufferSize       = rxSize;
    dataConfig.RxBufferMode       = USART_BUFFER_MODE_ONE_SHOT;
    dataConfig.RxEndMode          = USART_RX_END_NONE;
    dataConfig.TxDma              = UT_USART_TX_DMA;
    dataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    dataConfig.RxDma              = UT_USART_RX_DMA;
    dataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    dataConfig.IrqPriority        = UT_USART_PRIO;
    dataConfig.TxCompleteCallback = Ut_Usart_TxCompleteCallback;
    dataConfig.RxHalfCallback     = Ut_Usart_RxHalfCallback;
    dataConfig.RxCompleteCallback = Ut_Usart_RxCompleteCallback;
    dataConfig.RxEndCallback      = Ut_Usart_RxEndCallback;
    dataConfig.ErrorCallback      = Ut_Usart_ErrorCallback;

    return ( dataConfig );
}


/**
 * \brief Initializes USART1 with ignored RCC / GPIO calls.
 *
 * \param busConfig [in]: Bus configuration
 */
static void Ut_Usart_Init( usart_BusConfig_t * const busConfig )
{
    Rcc_Get_PeriphState_StubWithCallback( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_StubWithCallback( Ut_Usart_RccGetClkStub );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( busConfig ) );
}


/**
 * \brief Calls the captured USART ISR with given ISR register flags.
 *
 * \param isrFlags [in]: Value of the ISR register
 */
static void Ut_Usart_Call_Isr( uint32_t isrFlags )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utUsart_Isr, "USART ISR not registered" );

    UT_USART_REG->ISR = isrFlags;
    utUsart_Isr();
}


/**
 * \brief NVIC handler registration stub of any USART/UART peripheral - stores the handler.
 */
static nvic_RequestState_t Ut_Usart_NvicAnyHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utUsart_AnyIsr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief Replaces DMA functions by recording stubs (\ref utUsart_DmaChannel).
 */
static void Ut_Usart_Stub_DmaMocks( void )
{
    Dma_Get_DefaultConfig_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Init_Stub( Ut_Usart_DmaInitStub );
    Dma_Set_TransferActive_Stub( Ut_Usart_DmaActiveStub );
    Dma_Set_TransferInactive_Stub( Ut_Usart_DmaInactiveStub );
    Dma_Set_MemoryAddr_Stub( Ut_Usart_DmaMemoryAddrStub );
    Dma_Set_DataCount_Stub( Ut_Usart_DmaDataCountStub );
    Dma_Get_DataCount_Stub( Ut_Usart_DmaGetDataCountStub );
    Dma_Set_HalfTransferIrqActive_Stub( Ut_Usart_DmaHtIrqOnStub );
    Dma_Set_HalfTransferIrqInactive_Stub( Ut_Usart_DmaHtIrqOffStub );
    Dma_Set_TransferCompleteIrqActive_Stub( Ut_Usart_DmaTcIrqOnStub );
    Dma_Set_TransferCompleteIrqInactive_Stub( Ut_Usart_DmaTcIrqOffStub );
    Dma_Set_TransferErrorIrqActive_Stub( Ut_Usart_DmaTeIrqOnStub );
    Dma_Set_TransferErrorIrqInactive_Stub( Ut_Usart_DmaTeIrqOffStub );
    Dma_Set_InterruptActive_Stub( Ut_Usart_DmaNvicOnStub );
    Dma_Set_InterruptInactive_Stub( Ut_Usart_DmaNvicOffStub );
    Dma_Set_TransferCompleteIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_HalfTransferIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );
    Dma_Set_TransferErrorIsrHandler_IgnoreAndReturn( DMA_REQUEST_OK );
}


/**
 * \brief Configures data handling of USART1 with ignored RCC / NVIC / GPIO and stubbed DMA
 *        calls.
 *
 * \param dataConfig [in]: Data handling configuration
 */
static void Ut_Usart_Set_DataConfig( const usart_DataConfig_t * const dataConfig )
{
    Ut_Usart_Ignore_PeriphMocks();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, dataConfig ) );
}


/** \brief Dma_Init() stub - captures configuration, returns \ref utUsart_DmaInitState */
static dma_RequestState_t Ut_Usart_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    if( UT_USART_DMA_CFG_CNT > utUsart_DmaInitCnt )
    {
        utUsart_DmaConfig[ utUsart_DmaInitCnt ] = *dmaConfig;
    }
    else
    {
        /* No action required */
    }

    utUsart_DmaInitCnt++;

    return ( utUsart_DmaInitState );
}


/** \brief Returns record of DMA channel calls */
static ut_UsartDmaChannel_t* Ut_Usart_Get_DmaChannel( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel )
{
    TEST_ASSERT_TRUE( DMA_PERIPH_CNT > dmaBus );
    TEST_ASSERT_TRUE( UT_USART_DMA_CHANNELS > dmaChannel );

    return ( &utUsart_DmaChannel[ dmaBus ][ dmaChannel ] );
}


/** \brief Dma_Set_TransferActive() stub */
static dma_RequestState_t Ut_Usart_DmaActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->ActiveCnt++;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferInactive() stub */
static dma_RequestState_t Ut_Usart_DmaInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->InactiveCnt++;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_MemoryAddr() stub */
static dma_RequestState_t Ut_Usart_DmaMemoryAddrStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->MemoryAddr = memoryAddr;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_DataCount() stub */
static dma_RequestState_t Ut_Usart_DmaDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->DataCount = dataCount;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Get_DataCount() stub - returns \ref utUsart_DmaRemaining */
static dma_RequestState_t Ut_Usart_DmaGetDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt )
{
    (void)callCnt;
    (void)Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel );
    *dataCount = utUsart_DmaRemaining;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_HalfTransferIrqActive() stub */
static dma_RequestState_t Ut_Usart_DmaHtIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->HtIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_HalfTransferIrqInactive() stub */
static dma_RequestState_t Ut_Usart_DmaHtIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->HtIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferCompleteIrqActive() stub */
static dma_RequestState_t Ut_Usart_DmaTcIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->TcIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferCompleteIrqInactive() stub */
static dma_RequestState_t Ut_Usart_DmaTcIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->TcIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferErrorIrqActive() stub */
static dma_RequestState_t Ut_Usart_DmaTeIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->TeIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferErrorIrqInactive() stub */
static dma_RequestState_t Ut_Usart_DmaTeIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->TeIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_InterruptActive() stub */
static dma_RequestState_t Ut_Usart_DmaNvicOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->NvicIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_InterruptInactive() stub */
static dma_RequestState_t Ut_Usart_DmaNvicOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaChannel( dmaBus, dmaChannel )->NvicIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Transmission complete callback */
static void Ut_Usart_TxCompleteCallback( void )
{
    utUsart_TxCompleteCnt++;
}


/** \brief Receive buffer half filled callback */
static void Ut_Usart_RxHalfCallback( void )
{
    utUsart_RxHalfCnt++;
}


/** \brief Receive buffer filled callback */
static void Ut_Usart_RxCompleteCallback( void )
{
    utUsart_RxCompleteCnt++;
}


/**
 * \brief End of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void Ut_Usart_RxEndCallback( usart_RxDataCnt_t rxCnt )
{
    utUsart_RxEndBytes = rxCnt;
    utUsart_RxEndCnt++;
}


/**
 * \brief Data transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void Ut_Usart_ErrorCallback( usart_XferErrorId_t errorId )
{
    utUsart_LastError = errorId;
    utUsart_ErrorCnt++;
}


/**
 * \brief GPIO output level stub of the RTS pin - records the call.
 */
static gpio_RequestState_t Ut_Usart_GpioSetLevelStub( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_PinLevel_t pinLevel, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_USART_RTS_PORT, portId );
    TEST_ASSERT_EQUAL( UT_USART_RTS_PIN_ID, pinId );

    if( GPIO_PIN_LEVEL_HIGH == pinLevel )
    {
        Ut_Usart_Add_RtsEvent( UT_USART_RTS_EV_LEVEL_HIGH );
    }
    else
    {
        Ut_Usart_Add_RtsEvent( UT_USART_RTS_EV_OTHER );
    }

    return ( utUsart_RtsGpioState );
}


/**
 * \brief GPIO pin mode stub of the RTS pin - records the call.
 */
static gpio_RequestState_t Ut_Usart_GpioSetModeStub( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_PinMode_t pinType, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_EQUAL( UT_USART_RTS_PORT, portId );
    TEST_ASSERT_EQUAL( UT_USART_RTS_PIN_ID, pinId );

    if( GPIO_PIN_MODE_OUTPUT == pinType )
    {
        Ut_Usart_Add_RtsEvent( UT_USART_RTS_EV_MODE_OUTPUT );
    }
    else if( GPIO_PIN_MODE_ALTERNATE == pinType )
    {
        Ut_Usart_Add_RtsEvent( UT_USART_RTS_EV_MODE_AF );
    }
    else
    {
        Ut_Usart_Add_RtsEvent( UT_USART_RTS_EV_OTHER );
    }

    return ( utUsart_RtsGpioState );
}


/**
 * \brief Records an RTS pin event with CR1 / CR3 of USART1 at the call.
 *
 * \param event [in]: UT_USART_RTS_EV_x
 */
static void Ut_Usart_Add_RtsEvent( uint32_t event )
{
    TEST_ASSERT_LESS_THAN_UINT32( UT_USART_RTS_EV_CNT, utUsart_RtsEventCnt );

    utUsart_RtsEvent[ utUsart_RtsEventCnt ].Event = event;
    utUsart_RtsEvent[ utUsart_RtsEventCnt ].Cr1   = UT_USART_REG->CR1;
    utUsart_RtsEvent[ utUsart_RtsEventCnt ].Cr3   = UT_USART_REG->CR3;
    utUsart_RtsEventCnt++;
}


/**
 * \brief Initializes RTS / DE pin PA12 of USART1 and replaces the GPIO pin functions by the
 *        recording stubs.
 */
static void Ut_Usart_Init_RtsPin( void )
{
    Gpio_Set_PinLevel_StubWithCallback( Ut_Usart_GpioSetLevelStub );
    Gpio_Set_PinMode_StubWithCallback( Ut_Usart_GpioSetModeStub );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitDeGpio( UT_USART_RTS_PIN ) );
}


/**
 * \brief Initializes the RTS pin, requests RTS flow control with transmitter and receiver and
 *        enables USART1 (pin in alternate function), the recorded events are cleared.
 */
static void Ut_Usart_Set_RtsActive( void )
{
    Ut_Usart_Init_RtsPin();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RTS_EV_MODE_AF, utUsart_RtsEvent[ utUsart_RtsEventCnt - 1u ].Event );

    utUsart_RtsEventCnt = 0u;
}
