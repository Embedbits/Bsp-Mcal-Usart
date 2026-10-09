/**
 * \author Mr.Nobody
 * \file Test_Usart.c
 * \ingroup Usart
 * \brief Unit tests of Universal Synchronous/Asynchronous Receiver/Transmitter (USART) module.
 *
 * Usart.c, Usart_Dma.c, Usart_Isr.c and Usart_Poll.c are compiled unchanged with real LL
 * drivers. USART registers are emulated by RegMem, RCC, NVIC, GPIO and DMA modules are mocked
 * by CMock. USART ISR registered in NVIC and DMA callbacks passed to Dma_Init() are captured
 * by stubs and called directly to test the data handling.
 *
 * \note Emulated registers are plain memory. Status flags (SR) are not cleared by the read of
 *       DR (clear sequence of STM32F4), tests preset the flags the module waits for. TC flag is
 *       cleared by LL with write of ~TC - other SR bits read as set afterwards, tests rewrite SR
 *       before the next ISR call.
 *
 * \note Data handling context of the module is static - setUp() releases the data handling of
 *       the previous test (Usart_Deinit with stubbed mocks) and re-initializes the mocks.
 */

/* ============================= INCLUDES =================================== */
#include <string.h>                         /* memset                         */
#include "unity.h"                          /* Unity testing framework        */
#include "RegMem.h"                         /* Register memory emulation      */
#include "Usart_Port.h"                     /* Module under test              */
#include "MockRcc_Port.h"                   /* RCC module mock                */
#include "MockNvic_Port.h"                  /* NVIC module mock               */
#include "MockGpio_Port.h"                  /* GPIO module mock               */
#include "MockDma_Port.h"                   /* DMA module mock                */
#include "Stm32_usart.h"                    /* USART registers definition     */
/* ============================= TYPEDEFS =================================== */

/** \brief Record of DMA stream calls of one stream */
typedef struct
{
    uint32_t         ActiveCnt;   /**< Count of Dma_Set_TransferActive() calls   */
    uint32_t         InactiveCnt; /**< Count of Dma_Set_TransferInactive() calls */
    dma_MemoryAddr_t MemoryAddr;  /**< Last memory address                       */
    dma_DataCount_t  DataCount;   /**< Last data count                           */
    uint32_t         HtIrq;       /**< Half transfer interrupt state (1 enabled) */
    uint32_t         TcIrq;       /**< Transfer complete interrupt state         */
    uint32_t         TeIrq;       /**< Transfer error interrupt state            */
    uint32_t         NvicIrq;     /**< Stream interrupt in NVIC state            */
}   ut_UsartDmaStream_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void                 Ut_Usart_Expect_ClockState      ( rcc_PeriphId_t periphId, rcc_FunctionState_t clockState );
static void                 Ut_Usart_Expect_PeriphClk       ( rcc_PeriphId_t periphId, rcc_FreqHz_t clockHz );
static void                 Ut_Usart_Expect_Reset           ( rcc_PeriphId_t periphId );
static void                 Ut_Usart_Expect_GpioInit        ( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_AltFunction_t altFunc, gpio_PinOutputType_t outType, gpio_PinPullCfg_t pull );
static usart_BusConfig_t    Ut_Usart_Get_Config             ( void );
static usart_DataConfig_t   Ut_Usart_Get_DataConfig         ( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize );
static void                 Ut_Usart_Stub_PeriphMocks       ( void );
static void                 Ut_Usart_Stub_DmaMocks          ( void );
static void                 Ut_Usart_Release                ( void );
static void                 Ut_Usart_Set_DataConfig         ( const usart_DataConfig_t * const dataConfig );
static void                 Ut_Usart_Call_Isr               ( uint32_t srFlags );

static nvic_RequestState_t  Ut_Usart_NvicSetHandlerStub     ( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt );
static rcc_RequestState_t   Ut_Usart_RccGetClkStub          ( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt );
static rcc_RequestState_t   Ut_Usart_RccGetStateStub        ( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaInitStub            ( dma_ConfigStruct_t * const dmaConfig, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaActiveStub          ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaInactiveStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaMemoryAddrStub      ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaDataCountStub       ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaGetDataCountStub    ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaHtIrqOnStub         ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaHtIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTcIrqOnStub         ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTcIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTeIrqOnStub         ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaTeIrqOffStub        ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaNvicOnStub          ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static dma_RequestState_t   Ut_Usart_DmaNvicOffStub         ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt );
static ut_UsartDmaStream_t* Ut_Usart_Get_DmaStream          ( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel );

static void                 Ut_Usart_TxCompleteCallback     ( void );
static void                 Ut_Usart_RxHalfCallback         ( void );
static void                 Ut_Usart_RxCompleteCallback     ( void );
static void                 Ut_Usart_RxEndCallback          ( usart_RxDataCnt_t rxCnt );
static void                 Ut_Usart_ErrorCallback          ( usart_XferErrorId_t errorId );
static void                 Ut_Usart_RxEndRestartCallback   ( usart_RxDataCnt_t rxCnt );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/** USART peripheral used by tests (APB1) */
#define UT_USART_BUS                        ( USART_BUS_2 )
#define UT_USART_REG                        ( USART2 )
#define UT_USART_RCC                        ( RCC_PERIPH_USART2 )
#define UT_USART_NVIC                       ( NVIC_PERIPH_IRQ_USART2 )

/** DMA streams connected to USART2 requests (RM0090) */
#define UT_USART_DMA                        ( DMA_PERIPH_1 )
#define UT_USART_DMA_TX_STREAM              ( DMA_STREAM_6 )
#define UT_USART_DMA_RX_STREAM              ( DMA_STREAM_5 )

/** DMA stream list items of USART2 used by data handling tests (DMA1 stream 6 TX, stream 5 RX) */
#define UT_USART_TX_DMA                     ( USART_TX_DMA_BUS2_DMA1_STREAM6 )
#define UT_USART_RX_DMA                     ( USART_RX_DMA_BUS2_DMA1_STREAM5 )

/** Encoded DMA stream from the bus, DMA peripheral index, stream number and channel selection number
 *  (bit-fields written independently of USART_DMA_ENCODE) */
#define UT_USART_DMA_CODE( BUS, DMA, STREAM, CHSEL )    ( ( (BUS) << 15u ) | ( (DMA) << 10u ) | ( (STREAM) << 5u ) | (CHSEL) )

/** Channel selection of the UART5 transmit stream (DMA1 stream 7): 8 on STM32F413 / STM32F423, 4 on the other devices */
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    #define UT_USART_UART5_TX_SEL           ( DMA_REQ_CHANNEL_8 )
    #define UT_USART_UART5_TX_CHSEL         ( 8u )
#else
    #define UT_USART_UART5_TX_SEL           ( DMA_REQ_CHANNEL_4 )
    #define UT_USART_UART5_TX_CHSEL         ( 4u )
#endif

/** UART peripheral (UART4 - not available on STM32F401 / F410 / F411 / F412) */
#if defined(UART4)
    #define UT_UART_BUS                     ( USART_BUS_4 )
    #define UT_UART_REG                     ( UART4 )
#endif /* UART4 */

/** Peripheral clock returned by RCC mock [Hz] */
#define UT_USART_CLK_HZ                     ( 42000000u )

/** Peripheral clock giving well-known BRR values [Hz] */
#define UT_USART_CLK_84MHZ                  ( 84000000u )

/** Baud rate of test configurations */
#define UT_USART_BAUDRATE                   ( 115200u )

/** USART2 TX pin of the half-duplex test (PA2 - exists on every device line), alternate function 7 */
#define UT_USART_HDX_TX_PIN                 ( USART_TX_PIN_BUS2_PA2 )
#define UT_USART_HDX_PORT                   ( GPIO_PORT_A )
#define UT_USART_HDX_PIN_ID                 ( GPIO_PIN_ID_2 )

/** BRR of 115200 Bd at 84 MHz, over-sampling by 16 (USARTDIV 45.5625) */
#define UT_USART_BRR_84MHZ_OVER16           ( 0x02D9u )

/** BRR of 115200 Bd at 84 MHz, over-sampling by 8 (USARTDIV 91.125, fraction bit 3 cleared) */
#define UT_USART_BRR_84MHZ_OVER8            ( 0x05B1u )

/** Interrupt priority of test configurations */
#define UT_USART_PRIO                       ( 6u )

/** Size of receive buffer */
#define UT_USART_RX_SIZE                    ( 8u )

/** Maximal count of captured DMA configurations */
#define UT_USART_DMA_CFG_CNT                ( 4u )

/** Count of DMA streams of one DMA peripheral */
#define UT_USART_DMA_STREAMS                ( 8u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** ISR registered in NVIC for the USART */
static nvic_IsrCallback_t       utUsart_Isr;

/** Receive buffer */
static usart_RxData_t           utUsart_RxBuf[ UT_USART_RX_SIZE ];

/** Transmit data */
static const usart_TxData_t     utUsart_TxBuf[ 4u ] = { 0x11u, 0x22u, 0x33u, 0x44u };

/** Clock state reported by RCC stub */
static rcc_FunctionState_t      utUsart_ClockState;

/** DMA configurations passed to Dma_Init() */
static dma_ConfigStruct_t       utUsart_DmaConfig[ UT_USART_DMA_CFG_CNT ];

/** Count of Dma_Init() calls */
static uint32_t                 utUsart_DmaInitCnt;

/** Return state of Dma_Init() stub */
static dma_RequestState_t       utUsart_DmaInitState;

/** Remaining count returned by Dma_Get_DataCount() stub */
static dma_DataCount_t          utUsart_DmaRemaining;

/** Records of DMA stream calls */
static ut_UsartDmaStream_t      utUsart_DmaStream[ DMA_PERIPH_CNT ][ UT_USART_DMA_STREAMS ];

/** Counts of callback calls */
static uint32_t                 utUsart_TxCompleteCnt;
static uint32_t                 utUsart_RxHalfCnt;
static uint32_t                 utUsart_RxCompleteCnt;
static uint32_t                 utUsart_RxEndCnt;
static uint32_t                 utUsart_ErrorCnt;

/** Parameters of the last callback calls */
static usart_RxDataCnt_t        utUsart_RxEndBytes;
static usart_XferErrorId_t      utUsart_LastError;
static uint32_t                 utUsart_ErrorMask;

/* ============================ TEST FIXTURE ================================ */

void setUp( void )
{
    Ut_Usart_Release();

    TEST_ASSERT_EQUAL( REGMEM_REQUEST_OK, RegMem_Reset() );

    utUsart_Isr           = NULL;
    utUsart_ClockState    = RCC_FUNCTION_INACTIVE;
    utUsart_DmaInitCnt    = 0u;
    utUsart_DmaInitState  = DMA_REQUEST_OK;
    utUsart_DmaRemaining  = 0u;
    utUsart_TxCompleteCnt = 0u;
    utUsart_RxHalfCnt     = 0u;
    utUsart_RxCompleteCnt = 0u;
    utUsart_RxEndCnt      = 0u;
    utUsart_ErrorCnt      = 0u;
    utUsart_RxEndBytes    = 0u;
    utUsart_LastError     = USART_XFER_ERROR_CNT;
    utUsart_ErrorMask     = 0u;

    (void)memset( utUsart_RxBuf, 0, sizeof( utUsart_RxBuf ) );
    (void)memset( utUsart_DmaConfig, 0, sizeof( utUsart_DmaConfig ) );
    (void)memset( utUsart_DmaStream, 0, sizeof( utUsart_DmaStream ) );
}


void tearDown( void )
{
    /* Mocks are verified by generated runner */
}

/* ========================== MODULE VERSION ================================ */

/**
 * \brief   Usart_Get_ModuleVersion() returns version of the module.
 *
 * \par Expected results
 * - Version is 2.0.0 (STM32H5 interface).
 */
void Ut_Usart_Get_ModuleVersion_ReturnsVersion( void )
{
    usart_ModuleVersion_t version = Usart_Get_ModuleVersion();

    TEST_ASSERT_EQUAL_UINT8( 2u, version.Major );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Minor );
    TEST_ASSERT_EQUAL_UINT8( 0u, version.Patch );
}

/* ========================= DEFAULT CONFIGURATION ========================== */

/**
 * \brief   Usart_Get_DefaultConfig() rejects null pointer.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Get_DefaultConfig_NullPtr_Error( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DefaultConfig( NULL ) );
}


/**
 * \brief   Usart_Get_DefaultConfig() fills every field of the configuration.
 *
 * \details Configuration structure is pre-filled with non default values.
 *
 * \par Expected results
 * - USART1, 115200 Bd, 8N1, TX and RX, no flow control, over-sampling by 8,
 *   features unsupported by STM32F4 inactive, no pins, no data handling.
 */
void Ut_Usart_Get_DefaultConfig_FillsDefaults( void )
{
    usart_BusConfig_t config;

    (void)memset( &config, 0xA5, sizeof( config ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &config ) );

    TEST_ASSERT_EQUAL( USART_BUS_1,                config.PeriphId );
    TEST_ASSERT_EQUAL_UINT32( 115200u,             config.BaudRate );
    TEST_ASSERT_EQUAL( USART_DATA_WIDTH_8,         config.DataWidth );
    TEST_ASSERT_EQUAL( USART_STOP_BITS_1,          config.StopBits );
    TEST_ASSERT_EQUAL( USART_PARITY_NONE,          config.Parity );
    TEST_ASSERT_EQUAL( USART_TRANSFER_MODE_TX_RX,  config.TransferMode );
    TEST_ASSERT_EQUAL( USART_FLOW_CONTROL_NONE,    config.HwFlowControl );
    TEST_ASSERT_EQUAL( USART_DE_DISABLED,          config.DriverEnableMode );
    TEST_ASSERT_EQUAL( USART_DE_ACTIVE_HIGH,       config.DriverEnablePolarity );
    TEST_ASSERT_EQUAL( USART_OVERSAMPLING_8,       config.Oversampling );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_INACTIVE, config.HalfDuplex );
    TEST_ASSERT_EQUAL_UINT32( 0u,                  config.RxTimeoutValue );
    TEST_ASSERT_EQUAL( USART_RX_PIN_STANDARD,      config.RxPinOperationLevels );
    TEST_ASSERT_EQUAL( USART_TX_PIN_STANDARD,      config.TxPinOperationLevels );
    TEST_ASSERT_NULL( config.DataConfig );
    TEST_ASSERT_EQUAL( USART_RX_PIN_UNUSED,        config.BusRxPin );
    TEST_ASSERT_EQUAL( USART_TX_PIN_UNUSED,        config.BusTxPin );
    TEST_ASSERT_EQUAL( USART_DE_PIN_UNUSED,        config.BusDePin );
    TEST_ASSERT_EQUAL( USART_CTS_PIN_UNUSED,       config.BusCtsPin );
    TEST_ASSERT_EQUAL( USART_RTS_PIN_UNUSED,       config.BusRtsPin );
}

/* ============================ INITIALIZATION ============================== */

/**
 * \brief   Usart_Init() rejects null pointer and invalid peripheral.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, no RCC access (strict mocks).
 */
void Ut_Usart_Init_InvalidConfig_ErrorWithoutAccess( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( NULL ) );

    config.PeriphId = USART_BUS_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
}


/**
 * \brief   Usart_Init() activates inactive clock, resets and configures the peripheral.
 *
 * \details USART2 with RX pin PA3 and TX pin PA2, 115200 Bd at 42 MHz,
 *          over-sampling by 16, 9 bit word, even parity, 2 stop bits.
 *
 * \par Expected results
 * - Clock activated, peripheral reset, RX pin with pull-up, TX pin push-pull.
 * - CR1: UE, M, PCE, TE, RE; CR2: 2 stop bits; BRR of 115200 Bd.
 */
void Ut_Usart_Init_ClockInactive_ClockActivatedAndConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    config.BusRxPin     = USART_RX_PIN_BUS2_PA3;
    config.BusTxPin     = USART_TX_PIN_BUS2_PA2;
    config.Oversampling = USART_OVERSAMPLING_16;
    config.DataWidth    = USART_DATA_WIDTH_9;
    config.Parity       = USART_PARITY_EVEN;
    config.StopBits     = USART_STOP_BITS_2;

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_3, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_UP );
    Ut_Usart_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_2, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_NONE );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE | USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_STOP_1, UT_USART_REG->CR2 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL_HEX32( __LL_USART_DIV_SAMPLING16( UT_USART_CLK_HZ, UT_USART_BAUDRATE ), UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Init() keeps active clock and skips not used pins.
 *
 * \par Expected results
 * - No clock activation, no GPIO initialization, over-sampling by 8 set.
 */
void Ut_Usart_Init_ClockActiveNoPins_NoClockActivationNoGpio( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE | USART_CR1_OVER8 | USART_CR1_TE | USART_CR1_RE, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( __LL_USART_DIV_SAMPLING8( UT_USART_CLK_HZ, UT_USART_BAUDRATE ), UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Init() in half-duplex mode configures TX pin as open-drain with pull-up.
 *
 * \par Expected results
 * - TX pin open-drain with pull-up, HDSEL set.
 */
void Ut_Usart_Init_HalfDuplex_TxPinOpenDrainAndHdselSet( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    config.BusTxPin   = UT_USART_HDX_TX_PIN;
    config.HalfDuplex = USART_HALF_DUPLEX_ACTIVE;

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_GpioInit( UT_USART_HDX_PORT, UT_USART_HDX_PIN_ID, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_OPENDRAIN, GPIO_PIN_PULL_UP );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR3_HDSEL, UT_USART_REG->CR3 );
}


/**
 * \brief   Usart_Init() does not configure pins of other peripherals.
 *
 * \details USART2 configured with RX pin PB7 and TX pin PB6 of USART1 (STM32H5 behavior - pin
 *          of other peripheral means "pin not used"; both pins exist on every device line).
 *
 * \par Expected results
 * - USART_REQUEST_OK, no GPIO initialization (strict mocks), peripheral enabled.
 */
void Ut_Usart_Init_PinOfOtherPeriph_PinNotConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    config.BusRxPin = USART_RX_PIN_BUS1_PB7;
    config.BusTxPin = USART_TX_PIN_BUS1_PB6;

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() configures the CTS and RTS pins of the hardware flow control.
 *
 * \details USART2 with CTS pin PA0 and RTS pin PA1 (AF7; the RTS code is built by the encoding macro -
 *          the item of the pin table does not exist on STM32F410Tx).
 *
 * \par Expected results
 * - CTS pin with pull-up, RTS pin without pull, both push-pull in the alternate function mode.
 * - USART_REQUEST_OK, peripheral enabled.
 */
void Ut_Usart_Init_FlowControlPins_GpioConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    config.BusCtsPin = USART_CTS_PIN_BUS2_PA0;
    config.BusRtsPin = (usart_RtsPin_t)USART_PIN_BIT_MASK_ENCODE( USART_BUS_2, GPIO_PORT_A, GPIO_PIN_ID_1, GPIO_ALT_FUNC_7 );

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_0, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_UP );
    Ut_Usart_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_1, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_NONE );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() does not configure flow control pins of other peripherals.
 *
 * \details USART2 configured with CTS pin PA11 and RTS pin PA12 of USART1 (the CTS code is built by
 *          the encoding macro - the item of the pin table does not exist on STM32F410Tx).
 *
 * \par Expected results
 * - USART_REQUEST_OK, no GPIO initialization (strict mocks), peripheral enabled.
 */
void Ut_Usart_Init_FlowControlPinsOfOtherPeriph_PinNotConfigured( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    config.BusCtsPin = (usart_CtsPin_t)USART_PIN_BIT_MASK_ENCODE( USART_BUS_1, GPIO_PORT_A, GPIO_PIN_ID_11, GPIO_ALT_FUNC_7 );
    config.BusRtsPin = USART_RTS_PIN_BUS1_PA12;

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() reports GPIO error of the flow control pins.
 *
 * \details Gpio_Init() mock returns error for the CTS pin, then for the RTS pin.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, the RTS pin is not processed after the CTS pin error, peripheral not enabled.
 */
void Ut_Usart_Init_FlowControlPinGpioError_ReturnsErrorPeripheralNotEnabled( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    config.BusCtsPin = USART_CTS_PIN_BUS2_PA0;
    config.BusRtsPin = (usart_RtsPin_t)USART_PIN_BIT_MASK_ENCODE( USART_BUS_2, GPIO_PORT_A, GPIO_PIN_ID_1, GPIO_ALT_FUNC_7 );

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_OK );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() reports failure of RCC and GPIO modules.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR for failed clock state read, clock activation, reset
 *   and GPIO initialization, peripheral not enabled.
 */
void Ut_Usart_Init_DependencyFailure_Error( void )
{
    usart_BusConfig_t config = Ut_Usart_Get_Config();

    Rcc_Get_PeriphState_ExpectAndReturn( UT_USART_RCC, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_INACTIVE );
    Rcc_Set_PeriphActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Rcc_Set_ResetActive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );

    config.BusRxPin = USART_RX_PIN_BUS2_PA3;
    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
}


/**
 * \brief   Usart_Init() rejects features not supported by STM32F4.
 *
 * \details Driver Enable, its polarity, inverted pins, receiver timeout and 7 bit word
 *          length, configured one by one.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, peripheral is not enabled.
 */
void Ut_Usart_Init_UnsupportedFeature_Error( void )
{
    usart_BusConfig_t configs[ 5u ];

    for( uint32_t idx = 0u; 5u > idx; idx++ )
    {
        configs[ idx ] = Ut_Usart_Get_Config();
    }

    configs[ 0u ].DriverEnableMode     = USART_DE_ENABLED;
    configs[ 1u ].DriverEnablePolarity = USART_DE_ACTIVE_LOW;
    configs[ 2u ].RxPinOperationLevels = USART_RX_PIN_INVERTED;
    configs[ 3u ].RxTimeoutValue       = 100u;
    configs[ 4u ].DataWidth            = USART_DATA_WIDTH_7;

    for( uint32_t idx = 0u; 5u > idx; idx++ )
    {
        Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
        Ut_Usart_Expect_Reset( UT_USART_RCC );
        Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

        TEST_ASSERT_EQUAL_MESSAGE( USART_REQUEST_ERROR, Usart_Init( &configs[ idx ] ), "Unsupported feature accepted" );
        TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );
    }
}


/**
 * \brief   Usart_Init() initializes data handling of the configuration.
 *
 * \details ISR mode data handling given by DataConfig of the bus configuration.
 *
 * \par Expected results
 * - USART_REQUEST_OK, NVIC priority 6, ISR registered, NVIC line enabled.
 * - Usart_Get_DataConfig() returns the configuration (copy).
 */
void Ut_Usart_Init_DataConfig_DataHandlingInitialized( void )
{
    usart_BusConfig_t  config     = Ut_Usart_Get_Config();
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_DataConfig_t readConfig;

    config.DataConfig = &dataConfig;

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );
    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, UT_USART_PRIO, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &config ) );
    TEST_ASSERT_NOT_NULL( utUsart_Isr );

    dataConfig.TxMode = USART_XFER_MODE_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
    TEST_ASSERT_EQUAL( USART_XFER_MODE_ISR, readConfig.TxMode );
    TEST_ASSERT_EQUAL_PTR( utUsart_RxBuf, readConfig.RxBuffer );
}


/**
 * \brief   Usart_Init() reports invalid data handling configuration.
 *
 * \details End of message by receiver timeout (not available on STM32F4).
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, data handling not initialized.
 */
void Ut_Usart_Init_InvalidDataConfig_Error( void )
{
    usart_BusConfig_t  config     = Ut_Usart_Get_Config();
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_DataConfig_t readConfig;

    dataConfig.RxEndMode = USART_RX_END_TIMEOUT;
    config.DataConfig    = &dataConfig;

    Ut_Usart_Expect_ClockState( UT_USART_RCC, RCC_FUNCTION_ACTIVE );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_HZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Init( &config ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
}

/* ========================== DE-INITIALIZATION ============================= */

/**
 * \brief   Usart_Deinit() deactivates interrupt, disables, resets the peripheral
 *          and deactivates clock.
 *
 * \par Expected results
 * - USART_REQUEST_OK, UE cleared, invalid peripheral rejected.
 */
void Ut_Usart_Deinit_PeriphReleased( void )
{
    UT_USART_REG->CR1 = USART_CR1_UE | USART_CR1_TE;

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Deinit( USART_BUS_CNT ) );
}


/**
 * \brief   Usart_Deinit() reports failure of NVIC and RCC modules.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, all steps executed.
 */
void Ut_Usart_Deinit_DependencyFailure_Error( void )
{
    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_ERROR );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Deinit( UT_USART_BUS ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    Ut_Usart_Expect_Reset( UT_USART_RCC );
    Rcc_Set_PeriphInactive_ExpectAndReturn( UT_USART_RCC, RCC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Deinit( UT_USART_BUS ) );
}


/**
 * \brief   Usart_Deinit() stops running data handling.
 *
 * \details ISR mode, reception and transmission running.
 *
 * \par Expected results
 * - Interrupt sources disabled, TX / RX state inactive, data handling released
 *   (Usart_Get_DataConfig() error).
 */
void Ut_Usart_Deinit_RunningDataHandling_Released( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t state      = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );

    Ut_Usart_Stub_PeriphMocks();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_IDLEIE | USART_CR1_PEIE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_EIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( UT_USART_BUS, &dataConfig ) );
}

/* ========================== PERIPHERAL STATE ============================== */

/**
 * \brief   Usart_Set_PeriphActive() / Usart_Set_PeriphInactive() control UE bit.
 *
 * \par Expected results
 * - Usart_Get_PeriphState() reports active / inactive state.
 * - Invalid peripheral and null pointer rejected.
 */
void Ut_Usart_PeriphState_ActiveInactive( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_UE, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PeriphActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PeriphInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PeriphState( USART_BUS_CNT, &state ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PeriphState( UT_USART_BUS, NULL ) );
}

/* ============================== BAUD-RATE ================================= */

/**
 * \brief   Usart_Set_Baudrate() writes BRR for over-sampling by 16.
 *
 * \details 115200 Bd at 84 MHz - USARTDIV 45.5625.
 *
 * \par Expected results
 * - BRR 0x2D9.
 */
void Ut_Usart_Set_Baudrate_Oversampling16_BrrWritten( void )
{
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_84MHZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_BRR_84MHZ_OVER16, UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Set_Baudrate() writes BRR for over-sampling by 8.
 *
 * \details 115200 Bd at 84 MHz - USARTDIV 91.125, fraction has 3 bits.
 *
 * \par Expected results
 * - BRR 0x5B1, bit 3 cleared.
 */
void Ut_Usart_Set_Baudrate_Oversampling8_BrrWritten( void )
{
    UT_USART_REG->CR1 = USART_CR1_OVER8;

    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_84MHZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_BRR_84MHZ_OVER8, UT_USART_REG->BRR );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->BRR & 0x8u );
}


/**
 * \brief   Usart_Set_Baudrate() rejects baud-rate out of divider range.
 *
 * \details Zero, too low (mantissa > 4095) and too high (mantissa 0) baud-rate.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, BRR unchanged.
 */
void Ut_Usart_Set_Baudrate_OutOfRange_Error( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, 0u ) );

    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_84MHZ );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, 1000u ) );

    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_84MHZ );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, 6000000u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Set_Baudrate() reports RCC failure and invalid peripheral.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, BRR unchanged.
 */
void Ut_Usart_Set_Baudrate_RccErrorOrInvalidPeriph_Error( void )
{
    Rcc_Get_PeriphClk_ExpectAndReturn( UT_USART_RCC, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( UT_USART_BUS, UT_USART_BAUDRATE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( USART_BUS_CNT, UT_USART_BAUDRATE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->BRR );
}


/**
 * \brief   Usart_Get_Baudrate() calculates baud-rate from BRR.
 *
 * \details BRR 0x2D9 at 84 MHz, over-sampling by 16.
 *
 * \par Expected results
 * - 115226 Bd (84 MHz / 729). Zero BRR, RCC error, null pointer and invalid ID rejected.
 */
void Ut_Usart_Get_Baudrate_CalculatedFromBrr( void )
{
    usart_Baudrate_t baudrate = 0u;

    UT_USART_REG->BRR = UT_USART_BRR_84MHZ_OVER16;
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_84MHZ );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( UT_USART_BUS, &baudrate ) );
    TEST_ASSERT_EQUAL_UINT32( 115226u, baudrate );

    UT_USART_REG->BRR = 0u;
    Ut_Usart_Expect_PeriphClk( UT_USART_RCC, UT_USART_CLK_84MHZ );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Baudrate( UT_USART_BUS, &baudrate ) );

    Rcc_Get_PeriphClk_ExpectAndReturn( UT_USART_RCC, NULL, RCC_REQUEST_ERROR );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Baudrate( UT_USART_BUS, &baudrate ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Baudrate( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Baudrate( USART_BUS_CNT, &baudrate ) );
}

/* ======================== FRAME CONFIGURATION ============================= */

/**
 * \brief   Usart_Set_DataWidth() configures 8 / 9 bit word, rejects 7 bit word.
 *
 * \par Expected results
 * - M bit set / cleared and read back by getter, 7 bit word rejected.
 */
void Ut_Usart_DataWidth_SetGet( void )
{
    usart_DataWidth_t dataWidth = USART_DATA_WIDTH_8;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataWidth( UT_USART_BUS, USART_DATA_WIDTH_9 ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_M, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataWidth( UT_USART_BUS, &dataWidth ) );
    TEST_ASSERT_EQUAL( USART_DATA_WIDTH_9, dataWidth );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataWidth( UT_USART_BUS, USART_DATA_WIDTH_8 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataWidth( UT_USART_BUS, USART_DATA_WIDTH_7 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataWidth( USART_BUS_CNT, USART_DATA_WIDTH_8 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataWidth( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Usart_Set_StopBits() configures stop bits of USART.
 *
 * \par Expected results
 * - STOP bits of CR2 for 0.5, 1, 1.5 and 2 stop bits, read back by getter.
 */
void Ut_Usart_StopBits_Usart_AllOptions( void )
{
    const usart_StopBits_t options[] = { USART_STOP_BITS_0_5, USART_STOP_BITS_1, USART_STOP_BITS_1_5, USART_STOP_BITS_2 };
    usart_StopBits_t       stopBits  = USART_STOP_BITS_1;

    for( uint32_t idx = 0u; ( sizeof( options ) / sizeof( options[ 0u ] ) ) > idx; idx++ )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_USART_BUS, options[ idx ] ) );
        TEST_ASSERT_EQUAL_HEX32( (uint32_t)options[ idx ], UT_USART_REG->CR2 & USART_CR2_STOP );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_StopBits( UT_USART_BUS, &stopBits ) );
        TEST_ASSERT_EQUAL( options[ idx ], stopBits );
    }

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_StopBits( USART_BUS_CNT, USART_STOP_BITS_1 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_StopBits( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Usart_Set_StopBits() rejects fractional stop bits on UART.
 *
 * \par Expected results
 * - 0.5 and 1.5 stop bits rejected on UART4, 2 stop bits accepted.
 * - MCUs without UART4: test ignored.
 */
void Ut_Usart_StopBits_UartFractional_Error( void )
{
#if defined(UART4)
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_StopBits( UT_UART_BUS, USART_STOP_BITS_0_5 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_StopBits( UT_UART_BUS, USART_STOP_BITS_1_5 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_UART_REG->CR2 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( UT_UART_BUS, USART_STOP_BITS_2 ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR2_STOP_1, UT_UART_REG->CR2 );
#else
    TEST_IGNORE_MESSAGE( "MCU without UART4" );
#endif /* UART4 */
}


/**
 * \brief   Usart_Set_Parity() configures parity, rejects invalid value.
 *
 * \par Expected results
 * - PCE / PS bits for even and odd parity, read back by getter.
 */
void Ut_Usart_Parity_SetGet( void )
{
    usart_Parity_t parity = USART_PARITY_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Parity( UT_USART_BUS, USART_PARITY_ODD ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_PCE | USART_CR1_PS, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( UT_USART_BUS, &parity ) );
    TEST_ASSERT_EQUAL( USART_PARITY_ODD, parity );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Parity( UT_USART_BUS, USART_PARITY_EVEN ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_PCE, UT_USART_REG->CR1 );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Parity( UT_USART_BUS, (usart_Parity_t)USART_CR1_PS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Parity( USART_BUS_CNT, USART_PARITY_NONE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Parity( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Usart_Set_TransferMode() configures transmitter and receiver.
 *
 * \par Expected results
 * - TE / RE bits, read back by getter, invalid value rejected.
 */
void Ut_Usart_TransferMode_SetGet( void )
{
    usart_TransferMode_t mode = USART_TRANSFER_MODE_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_RX ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RE, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TransferMode( UT_USART_BUS, &mode ) );
    TEST_ASSERT_EQUAL( USART_TRANSFER_MODE_RX, mode );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RE | USART_CR1_TE, UT_USART_REG->CR1 );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TransferMode( UT_USART_BUS, (usart_TransferMode_t)USART_CR1_UE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TransferMode( USART_BUS_CNT, USART_TRANSFER_MODE_TX ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TransferMode( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Usart_Set_FlowControl() configures RTS / CTS on USART, rejects it on UART without
 *          modem signals.
 *
 * \note    Device errata bug AB#664 (ES0182 2.11.9 "RTS is active while RE or UE = 0"): RTSE is
 *          not set while the USART is disabled, the requested RTS is reported by the getter.
 *
 * \par Expected results
 * - CTSE bit on USART2 (disabled USART: RTSE is not set), RTS / CTS read back by getter.
 * - UART4 accepts no flow control only if it has no modem signals (STM32F405 / 407 / 42x),
 *   CTS is accepted on MCUs with UART4 flow control (STM32F446). MCUs without UART4: UART
 *   part skipped.
 */
void Ut_Usart_FlowControl_UsartAndUart( void )
{
    usart_FlowControl_t flowControl = USART_FLOW_CONTROL_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS_CTS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_CTSE, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_FlowControl( UT_USART_BUS, &flowControl ) );
    TEST_ASSERT_EQUAL( USART_FLOW_CONTROL_RTS_CTS, flowControl );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_NONE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 );

#if defined(UART4)
    if( IS_UART_HWFLOW_INSTANCE( UT_UART_REG ) )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_UART_BUS, USART_FLOW_CONTROL_CTS ) );
        TEST_ASSERT_EQUAL_HEX32( USART_CR3_CTSE, UT_UART_REG->CR3 );
    }
    else
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_FlowControl( UT_UART_BUS, USART_FLOW_CONTROL_CTS ) );
    }

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_UART_BUS, USART_FLOW_CONTROL_NONE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_UART_REG->CR3 );
#endif /* UART4 */

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_FlowControl( UT_USART_BUS, (usart_FlowControl_t)USART_CR3_DMAT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_FlowControl( USART_BUS_CNT, USART_FLOW_CONTROL_NONE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_FlowControl( UT_USART_BUS, NULL ) );
}


/**
 * \brief   RTS output (RTSE) is enabled only while the USART and its receiver are enabled.
 *
 * \note    Device errata bug AB#664 (ES0182 2.11.9 "RTS is active while RE or UE = 0"): the RTS
 *          line is driven active as soon as RTSE is set, even with the USART or the receiver
 *          disabled - the peer would send data that are lost. RTSE has to follow UE and RE.
 *
 * \par Expected results
 * - RTS requested with disabled USART: RTSE not set, also after the receiver is enabled.
 * - RTSE set by Usart_Set_PeriphActive() with enabled receiver.
 * - Transmit-only mode clears RTSE, receiver enable sets it again.
 * - Usart_Set_PeriphInactive() clears RTSE and UE, RTS stays requested and is restored by the
 *   next Usart_Set_PeriphActive(). Flow control none clears RTSE immediately.
 */
void Ut_Usart_FlowControl_RtsOnlyWithUsartAndReceiverEnabled( void )
{
    usart_FlowControl_t flowControl = USART_FLOW_CONTROL_NONE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_RTS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_RTSE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX_RX ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_RTSE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, UT_USART_REG->CR3 & USART_CR3_RTSE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_TX ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_FlowControl( UT_USART_BUS, &flowControl ) );
    TEST_ASSERT_EQUAL( USART_FLOW_CONTROL_RTS, flowControl );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( UT_USART_BUS, USART_TRANSFER_MODE_RX ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, UT_USART_REG->CR3 & USART_CR3_RTSE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_RTSE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_UE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_RTSE, UT_USART_REG->CR3 & USART_CR3_RTSE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_FlowControl( UT_USART_BUS, USART_FLOW_CONTROL_NONE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_RTSE | USART_CR3_CTSE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_FlowControl( UT_USART_BUS, &flowControl ) );
    TEST_ASSERT_EQUAL( USART_FLOW_CONTROL_NONE, flowControl );
}


/**
 * \brief   Usart_Set_Oversampling() configures OVER8 bit.
 *
 * \par Expected results
 * - OVER8 set / cleared, read back by getter, invalid value rejected.
 */
void Ut_Usart_Oversampling_SetGet( void )
{
    usart_Oversampling_t oversampling = USART_OVERSAMPLING_16;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_8 ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_OVER8, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Oversampling( UT_USART_BUS, &oversampling ) );
    TEST_ASSERT_EQUAL( USART_OVERSAMPLING_8, oversampling );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Oversampling( UT_USART_BUS, USART_OVERSAMPLING_16 ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Oversampling( UT_USART_BUS, (usart_Oversampling_t)USART_CR1_M ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_Oversampling( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Usart_Set_HalfDuplexState() configures HDSEL bit.
 *
 * \par Expected results
 * - HDSEL set / cleared, getter reports the state (not over-sampling).
 */
void Ut_Usart_HalfDuplex_SetGet( void )
{
    usart_HalfDuplex_t halfDuplex = USART_HALF_DUPLEX_INACTIVE;

    UT_USART_REG->CR1 = USART_CR1_OVER8;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_HalfDuplexState( UT_USART_BUS, &halfDuplex ) );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_INACTIVE, halfDuplex );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_HalfDuplexState( UT_USART_BUS, USART_HALF_DUPLEX_ACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_HDSEL, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_HalfDuplexState( UT_USART_BUS, &halfDuplex ) );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_ACTIVE, halfDuplex );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_HalfDuplexState( UT_USART_BUS, USART_HALF_DUPLEX_INACTIVE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_HalfDuplexState( UT_USART_BUS, (usart_HalfDuplex_t)2u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_HalfDuplexState( USART_BUS_CNT, USART_HALF_DUPLEX_INACTIVE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_HalfDuplexState( UT_USART_BUS, NULL ) );
}

/* ===================== FEATURES NOT SUPPORTED BY F4 ======================= */

/**
 * \brief   Driver Enable (DE) functions accept inactive configuration only.
 *
 * \par Expected results
 * - DE disabled, active high polarity and zero times accepted, getters return
 *   them. DE enabled, active low polarity, non-zero times and DE pin rejected.
 */
void Ut_Usart_DriverEnable_InactiveOnly( void )
{
    usart_DeFeatureState_t  deState      = USART_DE_ENABLED;
    usart_DePolarity_t      dePolarity   = USART_DE_ACTIVE_LOW;
    usart_AssertTime_us_t   assertTime   = 1u;
    usart_DeassertTime_us_t deassertTime = 1u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_DriverEnableState( UT_USART_BUS, USART_DE_DISABLED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DriverEnableState( UT_USART_BUS, USART_DE_ENABLED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Get_DriverEnableState( UT_USART_BUS, &deState ) );
    TEST_ASSERT_EQUAL( USART_DE_DISABLED, deState );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_DriverEnablePolarity( UT_USART_BUS, USART_DE_ACTIVE_HIGH ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DriverEnablePolarity( UT_USART_BUS, USART_DE_ACTIVE_LOW ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Get_DriverEnablePolarity( UT_USART_BUS, &dePolarity ) );
    TEST_ASSERT_EQUAL( USART_DE_ACTIVE_HIGH, dePolarity );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_AssertDeassertTimes( UT_USART_BUS, 0u, 0u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_AssertDeassertTimes( UT_USART_BUS, 1u, 0u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Get_AssertDeassertTimes( UT_USART_BUS, &assertTime, &deassertTime ) );
    TEST_ASSERT_EQUAL_UINT8( 0u, assertTime );
    TEST_ASSERT_EQUAL_UINT8( 0u, deassertTime );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitDeGpio( USART_DE_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DriverEnableState( USART_BUS_CNT, &deState ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DriverEnablePolarity( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_AssertDeassertTimes( UT_USART_BUS, NULL, &deassertTime ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 | UT_USART_REG->CR2 | UT_USART_REG->CR3 );
}


/**
 * \brief   Receiver timeout functions accept inactive configuration only.
 *
 * \par Expected results
 * - Activation of timeout and its interrupt rejected, deactivation accepted, states inactive.
 */
void Ut_Usart_RxTimeout_InactiveOnly( void )
{
    usart_FlagState_t state = USART_FLAG_ACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutActive( UT_USART_BUS, 100u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_RxTimeoutInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Get_RxTimeoutState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );

    state = USART_FLAG_ACTIVE;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_RxTimeoutIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Get_RxTimeoutIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxTimeoutIrqInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxTimeoutState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxTimeoutIrqState( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Pin level functions accept standard levels only.
 *
 * \par Expected results
 * - Standard levels accepted and returned, inverted levels rejected.
 */
void Ut_Usart_PinLevels_StandardOnly( void )
{
    usart_RxPinLevel_t rxLevel = USART_RX_PIN_INVERTED;
    usart_TxPinLevel_t txLevel = USART_TX_PIN_INVERTED;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_PinLevels( UT_USART_BUS, USART_RX_PIN_STANDARD, USART_TX_PIN_STANDARD ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PinLevels( UT_USART_BUS, USART_RX_PIN_STANDARD, USART_TX_PIN_INVERTED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_PinLevels( UT_USART_BUS, USART_RX_PIN_INVERTED, USART_TX_PIN_STANDARD ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Get_PinLevels( UT_USART_BUS, &rxLevel, &txLevel ) );
    TEST_ASSERT_EQUAL( USART_RX_PIN_STANDARD, rxLevel );
    TEST_ASSERT_EQUAL( USART_TX_PIN_STANDARD, txLevel );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_PinLevels( UT_USART_BUS, &rxLevel, NULL ) );
}

/* ================================ GPIO ==================================== */

/**
 * \brief   Usart_InitRxGpio() / Usart_InitTxGpio() configure pin in alternate function.
 *
 * \details USART6 RX PC7 and TX PC6 (AF8, the codes are built by the encoding macro - the items
 *          of the pin tables do not exist on STM32F410Cx / STM32F412Cx), unused pins rejected.
 *
 * \par Expected results
 * - GPIO initialized with alternate function 8, GPIO error reported.
 * - MCUs without USART6 (STM32F410Tx): test ignored.
 */
void Ut_Usart_InitGpio_AlternateFunction( void )
{
#if defined(USART6)
    const usart_RxPin_t rxPin = (usart_RxPin_t)USART_PIN_BIT_MASK_ENCODE( USART_BUS_6, GPIO_PORT_C, GPIO_PIN_ID_7, GPIO_ALT_FUNC_8 );
    const usart_TxPin_t txPin = (usart_TxPin_t)USART_PIN_BIT_MASK_ENCODE( USART_BUS_6, GPIO_PORT_C, GPIO_PIN_ID_6, GPIO_ALT_FUNC_8 );

    Ut_Usart_Expect_GpioInit( GPIO_PORT_C, GPIO_PIN_ID_7, GPIO_ALT_FUNC_8, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_UP );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitRxGpio( rxPin ) );

    Ut_Usart_Expect_GpioInit( GPIO_PORT_C, GPIO_PIN_ID_6, GPIO_ALT_FUNC_8, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_NONE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitTxGpio( txPin ) );

    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitTxGpio( txPin ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitRxGpio( USART_RX_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitTxGpio( USART_TX_PIN_UNUSED ) );
#else
    TEST_IGNORE_MESSAGE( "MCU without USART6" );
#endif /* USART6 */
}


/**
 * \brief   Usart_InitCtsGpio() / Usart_InitRtsGpio() configure pin in alternate function.
 *
 * \details USART2 CTS PA0 and USART1 RTS PA12 (AF7, both items exist on every device line), unused
 *          pins rejected.
 *
 * \par Expected results
 * - CTS pin initialized with pull-up, RTS pin without pull, alternate function 7, GPIO error reported.
 */
void Ut_Usart_InitFlowControlGpio_AlternateFunction( void )
{
    Ut_Usart_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_0, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_UP );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitCtsGpio( USART_CTS_PIN_BUS2_PA0 ) );

    Ut_Usart_Expect_GpioInit( GPIO_PORT_A, GPIO_PIN_ID_12, GPIO_ALT_FUNC_7, GPIO_PIN_OUTPUT_PUSHPULL, GPIO_PIN_PULL_NONE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_InitRtsGpio( USART_RTS_PIN_BUS1_PA12 ) );

    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitCtsGpio( USART_CTS_PIN_BUS2_PA0 ) );

    Gpio_Init_ExpectAnyArgsAndReturn( GPIO_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitRtsGpio( USART_RTS_PIN_BUS1_PA12 ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitCtsGpio( USART_CTS_PIN_UNUSED ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_InitRtsGpio( USART_RTS_PIN_UNUSED ) );
}

/* ============================= DATA ACCESS ================================ */

/**
 * \brief   Register address getters return address of data register.
 *
 * \par Expected results
 * - Transmit and receive register address is &USART2->DR.
 */
void Ut_Usart_Get_RegisterAddr_DataRegister( void )
{
    usart_TxRegAddr_t txAddr = 0u;
    usart_RxRegAddr_t rxAddr = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxRegisterAddr( UT_USART_BUS, &txAddr ) );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->DR, txAddr );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxRegisterAddr( UT_USART_BUS, &rxAddr ) );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->DR, rxAddr );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxRegisterAddr( USART_BUS_CNT, &txAddr ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxRegisterAddr( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Usart_SendData() / Usart_ReadData() access 8 bit data of data register.
 *
 * \par Expected results
 * - Written / read data are 8 bit, invalid peripheral ignored / 0.
 */
void Ut_Usart_SendReadData_EightBitData( void )
{
    Usart_SendData( UT_USART_BUS, 0xA5u );
    TEST_ASSERT_EQUAL_HEX32( 0xA5u, UT_USART_REG->DR );

    UT_USART_REG->DR = 0x1C3u;
    TEST_ASSERT_EQUAL_HEX8( 0xC3u, Usart_ReadData( UT_USART_BUS ) );

    Usart_SendData( USART_BUS_CNT, 0x55u );
    TEST_ASSERT_EQUAL_HEX8( 0u, Usart_ReadData( USART_BUS_CNT ) );
}

/* ============================== INTERRUPTS ================================ */

/**
 * \brief   NVIC interrupt activation, deactivation and priority.
 *
 * \par Expected results
 * - Handler registered and IRQ activated / deactivated, priority set and read,
 *   NVIC errors reported, invalid peripheral rejected.
 */
void Ut_Usart_Interrupts_NvicHandling( void )
{
    usart_IrqPrio_t prio = 0u;

    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( UT_USART_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsActive( UT_USART_BUS ) );

    Nvic_Set_PeriphIrq_Handler_ExpectAndReturn( UT_USART_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_IgnoreArg_irqHandler();
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_InterruptsActive( UT_USART_BUS ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_InterruptsInactive( UT_USART_BUS ) );

    Nvic_Set_PeriphIrq_Inactive_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsInactive( UT_USART_BUS ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, 3u, NVIC_REQUEST_OK );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IrqPriority( UT_USART_BUS, 3u ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, NULL, NVIC_REQUEST_OK );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_IrqPriority( UT_USART_BUS, &prio ) );

    Nvic_Get_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, NULL, NVIC_REQUEST_ERROR );
    Nvic_Get_PeriphIrq_Prio_IgnoreArg_irqPrio();
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_IrqPriority( UT_USART_BUS, &prio ) );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, 3u, NVIC_REQUEST_ERROR );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IrqPriority( UT_USART_BUS, 3u ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_InterruptsInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IrqPriority( USART_BUS_CNT, 3u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_IrqPriority( UT_USART_BUS, NULL ) );
}


/**
 * \brief   DMA request activation of transmission and reception.
 *
 * \par Expected results
 * - DMAT / DMAR bits of CR3 set and cleared, states reported.
 */
void Ut_Usart_DmaRequests_ActiveInactive( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaTxRequestActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaRxRequestActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAT | USART_CR3_DMAR, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DmaTxReqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DmaRxReqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaTxRequestInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAR, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DmaTxReqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DmaRxRequestInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DmaRxReqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaTxRequestActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaTxRequestInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaRxRequestActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DmaRxRequestInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DmaTxReqState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DmaRxReqState( UT_USART_BUS, NULL ) );
}


/**
 * \brief   Peripheral interrupt requests are activated and deactivated.
 *
 * \details RXNE, TXE, TC, IDLE and error (EIE + PEIE) interrupts, pending IDLE and error flags.
 *
 * \par Expected results
 * - Enable bits of CR1 / CR3 set and cleared, states reported, TC flag cleared
 *   before activation of TC interrupt.
 */
void Ut_Usart_IrqRequests_ActiveInactive( void )
{
    usart_FlagState_t state = USART_FLAG_INACTIVE;

    UT_USART_REG->SR = USART_SR_IDLE | USART_SR_FE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxNotEmptyIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxEmptyIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IdleIrqActive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_ErrorIrqActive( UT_USART_BUS ) );

    UT_USART_REG->SR = USART_SR_TC;
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxCompleteIrqActive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE | USART_CR1_IDLEIE | USART_CR1_PEIE, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_EIE, UT_USART_REG->CR3 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->SR & USART_SR_TC );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxNotEmptyIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxEmptyIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxCompleteIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_IdleIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_ErrorIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, state );

    /* Deactivation of IDLE interrupt keeps TC interrupt */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_IdleIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, UT_USART_REG->CR1 & USART_CR1_TCIE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxNotEmptyIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxEmptyIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxCompleteIrqInactive( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_ErrorIrqInactive( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxNotEmptyIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxEmptyIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxCompleteIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_ErrorIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_IdleIrqState( UT_USART_BUS, &state ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, state );
}


/**
 * \brief   Interrupt request functions reject invalid peripheral and null pointer.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR for every function.
 */
void Ut_Usart_IrqRequests_InvalidArgs_Error( void )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxNotEmptyIrqActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxNotEmptyIrqInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxNotEmptyIrqState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxEmptyIrqActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxEmptyIrqInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxEmptyIrqState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxCompleteIrqActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxCompleteIrqInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxCompleteIrqState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IdleIrqActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_IdleIrqInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_IdleIrqState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_ErrorIrqActive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_ErrorIrqInactive( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_ErrorIrqState( UT_USART_BUS, NULL ) );
}

/* ================== DATA HANDLING - CONFIGURATION ========================= */

/**
 * \brief   Usart_Set_DataConfig() rejects invalid configuration without access.
 *
 * \details NULL configuration, peripheral out of range, NULL RX buffer, RX buffer size 0,
 *          TX / RX mode out of range, RX buffer mode and RX end mode out of range.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases, no NVIC / DMA call (strict mocks).
 */
void Ut_Usart_Set_DataConfig_InvalidConfig_ErrorWithoutAccess( void )
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
    dataConfig.RxMode = USART_XFER_MODE_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    dataConfig.RxBufferMode = USART_BUFFER_MODE_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    dataConfig.RxEndMode = USART_RX_END_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() rejects end of message by receiver timeout.
 *
 * \details STM32F4 USART has no receiver timeout - RX end mode timeout in ISR and POLL mode.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Set_DataConfig_TimeoutEnd_Error( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    dataConfig.RxEndMode = USART_RX_END_TIMEOUT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );
    dataConfig.RxEndMode = USART_RX_END_TIMEOUT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() rejects DMA streams out of the DMA stream lists.
 *
 * \details DMA mode USART2: unused streams, the transmit stream item in the receive member and
 *          the other way, the stream items of USART1 (DMA2 S7, DMA2 S5), stream and DMA
 *          peripheral out of range, invalid priority.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases, no DMA call (strict mocks).
 */
void Ut_Usart_Set_DataConfig_DmaInvalidStream_Error( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    dataConfig.TxDma = USART_TX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = USART_RX_DMA_UNUSED;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = (usart_RxDma_t)UT_USART_TX_DMA;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)UT_USART_RX_DMA;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = USART_TX_DMA_BUS1_DMA2_STREAM7;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = USART_RX_DMA_BUS1_DMA2_STREAM5;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDma = (usart_TxDma_t)USART_DMA_ENCODE( USART_BUS_2, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_CNT, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig       = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.RxDma = (usart_RxDma_t)USART_DMA_ENCODE( USART_BUS_2, USART_DMA_PERIPH_CNT, USART_DMA_CHANNEL_5, 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    dataConfig               = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    dataConfig.TxDmaPriority = (usart_DmaPriority_t)DMA_PRIORITY_CNT;
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


/**
 * \brief   Usart_Set_DataConfig() rejects a new configuration during running transfer.
 *
 * \details ISR mode, reception running, then transmission running.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR while a transfer runs, configuration accepted after stop.
 */
void Ut_Usart_Set_DataConfig_RunningTransfer_Error( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );

    Ut_Usart_Set_DataConfig( &dataConfig );
}


/**
 * \brief   Usart_Get_DataConfig() reports not configured data handling and invalid arguments.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Get_DataConfig_NotInitialized_Error( void )
{
    usart_DataConfig_t readConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_DataConfig( USART_BUS_CNT, &readConfig ) );
}


/**
 * \brief   Transmission / reception functions reject invalid state and arguments.
 *
 * \details Without data handling configuration: transmission start (valid data, NULL data,
 *          size 0, peripheral out of range), reception start, state / count getters with
 *          invalid arguments. With data handling of transmission only: reception start.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR in all cases, stop functions without transfer return OK.
 */
void Ut_Usart_Set_TxStart_InvalidArgsOrNotInitialized_Error( void )
{
    usart_FunctionState_t state      = USART_FUNCTION_INACTIVE;
    usart_RxDataCnt_t     rxCnt      = 0u;
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, NULL, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 0u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( USART_BUS_CNT, utUsart_TxBuf, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_TxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK,    Usart_Set_RxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStop( USART_BUS_CNT ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStop( USART_BUS_CNT ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_TxState( USART_BUS_CNT, &state ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxState( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxState( USART_BUS_CNT, &state ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxCount( UT_USART_BUS, NULL ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxCount( USART_BUS_CNT, &rxCnt ) );

    dataConfig.RxMode = USART_XFER_MODE_NONE;
    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
}

/* ====================== DATA HANDLING - ISR MODE ========================== */

/**
 * \brief   Usart_Set_DataConfig() in interrupt mode configures NVIC.
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
    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Active_ExpectAndReturn( UT_USART_NVIC, NVIC_REQUEST_OK );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_NOT_NULL( utUsart_Isr );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataConfig( UT_USART_BUS, &readConfig ) );
    TEST_ASSERT_EQUAL( USART_XFER_MODE_ISR, readConfig.TxMode );
    TEST_ASSERT_EQUAL_PTR( utUsart_RxBuf, readConfig.RxBuffer );
}


/**
 * \brief   Usart_Set_DataConfig() reports NVIC failure.
 *
 * \details Interrupt priority configuration fails.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Set_DataConfig_NvicError_Error( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    Nvic_Set_PeriphIrq_Prio_ExpectAndReturn( UT_USART_NVIC, UT_USART_PRIO, NVIC_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
}


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
 * 2. DR = 0x11, then 0x22.
 * 3. TXE interrupt disabled, TC interrupt enabled, no complete callback yet.
 * 4. TC flag cleared, TC interrupt disabled, complete callback 1x, TX state inactive.
 */
void Ut_Usart_Isr_Transmission_BytesWrittenAndCompleteCallback( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t txState    = USART_FUNCTION_INACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TXEIE, UT_USART_REG->CR1 & USART_CR1_TXEIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, txState );

    /* Second transmission while the first one is running */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );

    Ut_Usart_Call_Isr( USART_SR_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_USART_REG->DR );
    Ut_Usart_Call_Isr( USART_SR_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0x22u, UT_USART_REG->DR );

    /* All bytes written - TXE interrupt replaced by TC interrupt */
    Ut_Usart_Call_Isr( USART_SR_TXE );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TXEIE );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );

    Ut_Usart_Call_Isr( USART_SR_TXE | USART_SR_TC );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->SR & USART_SR_TC );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_TxCompleteCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
}


/**
 * \brief   Usart_Set_TxStop() stops running ISR transmission without callback.
 *
 * \details ISR mode, transmission of 4 bytes started, 1 byte written, stopped.
 *
 * \par Expected results
 * - TXE / TC interrupts disabled, TX state inactive, no complete callback, TC event ignored.
 */
void Ut_Usart_Isr_TxStop_InterruptsDisabledNoCallback( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t txState    = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );
    Ut_Usart_Call_Isr( USART_SR_TXE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_TXEIE | USART_CR1_TCIE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );

    Ut_Usart_Call_Isr( USART_SR_TXE | USART_SR_TC );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );
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

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_RXNEIE | USART_CR1_PEIE, UT_USART_REG->CR1 & ( USART_CR1_RXNEIE | USART_CR1_PEIE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_EIE, UT_USART_REG->CR3 & USART_CR3_EIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );

    /* Second start of running reception */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( UT_USART_BUS ) );

    for( uint32_t byteIdx = 0u; 4u > byteIdx; byteIdx++ )
    {
        UT_USART_REG->DR = 0xA0u + byteIdx;
        Ut_Usart_Call_Isr( USART_SR_RXNE );

        if( 1u == byteIdx )
        {
            TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxHalfCnt );
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
            TEST_ASSERT_EQUAL_UINT16( 2u, rxCnt );
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
 * - Reception keeps running (RX state active), RX count 1.
 */
void Ut_Usart_Isr_CircularReception_WrapsToBufferStart( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, 2u );
    usart_FunctionState_t rxState    = USART_FUNCTION_INACTIVE;
    usart_RxDataCnt_t     rxCnt      = 0u;

    dataConfig.RxBufferMode = USART_BUFFER_MODE_CIRCULAR;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    for( uint32_t byteIdx = 0u; 3u > byteIdx; byteIdx++ )
    {
        UT_USART_REG->DR = 0x10u + byteIdx;
        Ut_Usart_Call_Isr( USART_SR_RXNE );
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x12u, utUsart_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX8( 0x11u, utUsart_RxBuf[ 1u ] );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL_UINT16( 1u, rxCnt );
}


/**
 * \brief   Idle line ends the message in interrupt mode.
 *
 * \details Interrupt mode, RX end mode idle, buffer 8 bytes. Starts reception,
 *          receives 2 bytes and calls ISR with IDLE.
 *
 * \par Expected results
 * - CR1.IDLEIE set after start.
 * - RX end callback 1x with 2 bytes, no complete callback, reception stopped (one shot).
 */
void Ut_Usart_Isr_IdleLine_RxEndCallbackWithCount( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_ACTIVE;

    dataConfig.RxEndMode = USART_RX_END_IDLE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_IDLEIE, UT_USART_REG->CR1 & USART_CR1_IDLEIE );

    UT_USART_REG->DR = 0x55u;
    Ut_Usart_Call_Isr( USART_SR_RXNE );
    UT_USART_REG->DR = 0x66u;
    Ut_Usart_Call_Isr( USART_SR_RXNE );

    Ut_Usart_Call_Isr( USART_SR_IDLE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 2u, utUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x66u, utUsart_RxBuf[ 1u ] );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
}


/**
 * \brief   Reception can be restarted from end of message callback (one shot buffer).
 *
 * \details Interrupt mode, RX end mode idle, end callback restarts the reception. 1 byte
 *          received, IDLE, next byte received.
 *
 * \par Expected results
 * - End callback 1x with 1 byte, reception active, next byte stored to buffer start.
 */
void Ut_Usart_Isr_RxEndCallback_ReceptionRestarted( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_INACTIVE;

    dataConfig.RxEndMode     = USART_RX_END_IDLE;
    dataConfig.RxEndCallback = Ut_Usart_RxEndRestartCallback;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->DR = 0x31u;
    Ut_Usart_Call_Isr( USART_SR_RXNE );
    Ut_Usart_Call_Isr( USART_SR_IDLE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 1u, utUsart_RxEndBytes );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );

    UT_USART_REG->DR = 0x32u;
    Ut_Usart_Call_Isr( USART_SR_RXNE );
    TEST_ASSERT_EQUAL_HEX8( 0x32u, utUsart_RxBuf[ 0u ] );
}


/**
 * \brief   Reception errors in interrupt mode are reported, received data are stored.
 *
 * \details Interrupt mode, reception started.
 * 1. ISR with RXNE + PE (data 0x41).
 * 2. ISR with RXNE + ORE + NE + FE (data 0x42).
 *
 * \par Expected results
 * 1. Error callback 1x (parity), byte 0x41 stored.
 * 2. Error callback 3x more (framing, noise, overrun - in this order), byte 0x42 stored.
 */
void Ut_Usart_Isr_ReceptionErrors_ReportedDataStored( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->DR = 0x41u;
    Ut_Usart_Call_Isr( USART_SR_RXNE | USART_SR_PE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_PARITY, utUsart_LastError );
    TEST_ASSERT_EQUAL_HEX8( 0x41u, utUsart_RxBuf[ 0u ] );

    UT_USART_REG->DR = 0x42u;
    Ut_Usart_Call_Isr( USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE );

    TEST_ASSERT_EQUAL_UINT32( 4u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX32( ( 1u << USART_XFER_ERROR_PARITY  ) | ( 1u << USART_XFER_ERROR_FRAMING ) |
                             ( 1u << USART_XFER_ERROR_NOISE   ) | ( 1u << USART_XFER_ERROR_OVERRUN ), utUsart_ErrorMask );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_OVERRUN, utUsart_LastError );
    TEST_ASSERT_EQUAL_HEX8( 0x42u, utUsart_RxBuf[ 1u ] );
}


/**
 * \brief   ISR ignores flags of not enabled interrupt sources.
 *
 * \details ISR mode configured, nothing started. ISR called with RXNE, IDLE, TXE, TC and FE.
 *
 * \par Expected results
 * - No callback, no data stored, DR not written.
 */
void Ut_Usart_Isr_FlagsWithoutEnabledInterrupt_NoProcessing( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );

    UT_USART_REG->DR = 0x99u;
    Ut_Usart_Call_Isr( USART_SR_RXNE | USART_SR_IDLE | USART_SR_TXE | USART_SR_TC | USART_SR_FE );

    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt + utUsart_RxCompleteCnt + utUsart_RxEndCnt + utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8( 0u, utUsart_RxBuf[ 0u ] );
    TEST_ASSERT_EQUAL_HEX32( 0x99u, UT_USART_REG->DR );
}


/**
 * \brief   Usart_Set_RxStop() stops running reception.
 *
 * \details Interrupt mode with idle end. Starts reception, stops it and stops it again
 *          without running reception.
 *
 * \par Expected results
 * - CR1.RXNEIE, PEIE, IDLEIE and CR3.EIE cleared, RX state inactive.
 * - Second stop: USART_REQUEST_OK.
 */
void Ut_Usart_Set_RxStop_RunningReception_InterruptsDisabled( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_ACTIVE;

    dataConfig.RxEndMode = USART_RX_END_IDLE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & ( USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_IDLEIE ) );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_EIE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );

    /* Stop without running reception */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( UT_USART_BUS ) );
}

/* ====================== DATA HANDLING - POLL MODE ========================= */

/**
 * \brief   Usart_Task() transmits and receives in polling mode.
 *
 * \details Polling mode, RX buffer 2 bytes. Starts reception and transmission of 1
 *          byte, then calls Usart_Task() with SR flags:
 * 1. TXE.
 * 2. TXE + TC + RXNE (DR = 0x77).
 * 3. RXNE (DR = 0x78).
 *
 * \par Expected results
 * 1. DR = 0x11.
 * 2. TX complete callback 1x, RX buffer[ 0 ] = 0x77.
 * 3. RX complete callback 1x, RX buffer[ 1 ] = 0x78.
 */
void Ut_Usart_Task_PollMode_TransmitsAndReceives( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, 2u );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );

    /* TXE: first byte written */
    UT_USART_REG->SR = USART_SR_TXE;
    Usart_Task();
    TEST_ASSERT_EQUAL_HEX32( 0x11u, UT_USART_REG->DR );

    /* Byte received, TXE + TC: transmission complete */
    UT_USART_REG->DR = 0x77u;
    UT_USART_REG->SR = USART_SR_TXE | USART_SR_TC | USART_SR_RXNE;
    Usart_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x77u, utUsart_RxBuf[ 0u ] );

    UT_USART_REG->DR = 0x78u;
    UT_USART_REG->SR = USART_SR_RXNE;
    Usart_Task();
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8( 0x78u, utUsart_RxBuf[ 1u ] );
}


/**
 * \brief   Polling transmission is not complete before TC flag.
 *
 * \details Polling mode, transmission of 1 byte started. Usart_Task() is called 2x
 *          with TXE flag only, then with full DR (no flag).
 *
 * \par Expected results
 * - TX complete callback is not called (last frame not finished).
 */
void Ut_Usart_Task_PollModeTxInProgress_NoCompleteBeforeTc( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, 2u );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 1u ) );

    UT_USART_REG->SR = USART_SR_TXE;
    Usart_Task();
    Usart_Task();

    UT_USART_REG->SR = 0u;
    Usart_Task();

    /* All bytes written, last frame not finished (TC not set) */
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );
}


/**
 * \brief   Polling reception reports errors and end of message.
 *
 * \details Polling mode, RX end mode idle. Usart_Task() with RXNE + FE (DR = 0x21), then
 *          with IDLE.
 *
 * \par Expected results
 * - Framing error callback 1x, byte stored, end callback 1x with 1 byte.
 */
void Ut_Usart_Task_PollMode_ErrorAndIdleEnd( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );

    dataConfig.TxMode    = USART_XFER_MODE_NONE;
    dataConfig.RxEndMode = USART_RX_END_IDLE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->DR = 0x21u;
    UT_USART_REG->SR = USART_SR_RXNE | USART_SR_FE;
    Usart_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_FRAMING, utUsart_LastError );
    TEST_ASSERT_EQUAL_HEX8( 0x21u, utUsart_RxBuf[ 0u ] );

    UT_USART_REG->SR = USART_SR_IDLE;
    Usart_Task();

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 1u, utUsart_RxEndBytes );
}


/**
 * \brief   Usart_Task() ignores peripherals without polling data handling.
 *
 * \details ISR mode configured and running, Usart_Task() with RXNE and TXE flags.
 *
 * \par Expected results
 * - No data moved by the task.
 */
void Ut_Usart_Task_IsrMode_NoPolling( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_ISR, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->DR = 0x5Au;
    UT_USART_REG->SR = USART_SR_RXNE | USART_SR_TXE;
    Usart_Task();

    TEST_ASSERT_EQUAL_HEX8( 0u, utUsart_RxBuf[ 0u ] );
}

/* ====================== DATA HANDLING - DMA MODE ========================== */

/**
 * \brief   Usart_Set_DataConfig() in DMA mode initializes streams of USART2 requests.
 *
 * \details DMA mode, TX DMA1 stream 6 (low priority), RX DMA1 stream 5 (high priority),
 *          one shot RX buffer of 8 bytes with half callback.
 *
 * \par Expected results
 * - TX stream: channel 4, memory to peripheral, normal mode, DR as peripheral address,
 *   8-bit, memory increment, TC and TE callbacks, no HT callback.
 * - RX stream: channel 4, peripheral to memory, normal mode, RX buffer and size, TC, HT and
 *   TE callbacks.
 * - TC / TE interrupts of both streams, HT of RX stream and NVIC lines enabled, USART
 *   interrupt enabled.
 */
void Ut_Usart_Dma_SetDataConfig_StreamsInitialized( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_DmaInitCnt );

    TEST_ASSERT_EQUAL( DMA_PERIPH_1,                 utUsart_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_USART_DMA_TX_STREAM,       utUsart_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_4,            utUsart_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_MEMORY_TO_PERIPH,     utUsart_DmaConfig[ 0u ].Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,     utUsart_DmaConfig[ 0u ].TransferMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)&UT_USART_REG->DR, utUsart_DmaConfig[ 0u ].PeriphAddress );
    TEST_ASSERT_EQUAL( DMA_PERIPH_ADDR_STATIC,       utUsart_DmaConfig[ 0u ].PeriphAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_MEMORY_ADDR_INCREMENT,    utUsart_DmaConfig[ 0u ].MemoryAddrIncrement );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,       utUsart_DmaConfig[ 0u ].PeriphTransferSize );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_SIZE_8BIT,       utUsart_DmaConfig[ 0u ].MemoryTransferSize );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_LOW,             utUsart_DmaConfig[ 0u ].Priority );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 0u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 0u ].TransferErrorCallback );
    TEST_ASSERT_NULL( utUsart_DmaConfig[ 0u ].HalfTransferCallback );

    TEST_ASSERT_EQUAL( DMA_PERIPH_1,                 utUsart_DmaConfig[ 1u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( UT_USART_DMA_RX_STREAM,       utUsart_DmaConfig[ 1u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_4,            utUsart_DmaConfig[ 1u ].PeripheralReqId );
    TEST_ASSERT_EQUAL( DMA_DIR_PERIPH_TO_MEMORY,     utUsart_DmaConfig[ 1u ].Direction );
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_NORMAL,     utUsart_DmaConfig[ 1u ].TransferMode );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utUsart_RxBuf, utUsart_DmaConfig[ 1u ].MemoryAddress );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RX_SIZE,      utUsart_DmaConfig[ 1u ].DataCount );
    TEST_ASSERT_EQUAL( DMA_PRIORITY_HIGH,            utUsart_DmaConfig[ 1u ].Priority );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 1u ].TransferCompleteCallback );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 1u ].HalfTransferCallback );
    TEST_ASSERT_NOT_NULL( utUsart_DmaConfig[ 1u ].TransferErrorCallback );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].TcIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].TeIrq );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].HtIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].NvicIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_RX_STREAM ].HtIrq );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_RX_STREAM ].NvicIrq );
    TEST_ASSERT_NOT_NULL( utUsart_Isr );
}


/**
 * \brief   Circular receive buffer uses circular DMA mode, other USART uses its streams.
 *
 * \details USART6 (DMA2: TX stream 7 channel 5, RX stream 2 channel 5), circular buffer
 *          without half callback, TX direction not used.
 *
 * \par Expected results
 * - One stream initialized: DMA2 stream 2, channel 5, circular mode, no HT callback /
 *   interrupt.
 * - MCUs without USART6 (STM32F410Tx): test ignored.
 */
void Ut_Usart_Dma_CircularBufferUsart6_CircularStream( void )
{
#if defined(USART6)
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    dataConfig.TxMode         = USART_XFER_MODE_NONE;
    dataConfig.RxBufferMode   = USART_BUFFER_MODE_CIRCULAR;
    dataConfig.RxHalfCallback = NULL;
    dataConfig.RxDma          = USART_RX_DMA_BUS6_DMA2_STREAM2;

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
    TEST_ASSERT_EQUAL( DMA_TRANSFER_MODE_CIRCULAR, utUsart_DmaConfig[ 0u ].TransferMode );
    TEST_ASSERT_NULL( utUsart_DmaConfig[ 0u ].HalfTransferCallback );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_DmaStream[ DMA_PERIPH_2 ][ DMA_STREAM_2 ].HtIrq );

    /* Release of USART6 data handling */
    Ut_Usart_Stub_PeriphMocks();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( USART_BUS_6 ) );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaStream[ DMA_PERIPH_2 ][ DMA_STREAM_2 ].InactiveCnt );
#else
    TEST_IGNORE_MESSAGE( "MCU without USART6" );
#endif /* USART6 */
}


/**
 * \brief   USART2 reception is served also by DMA1 stream 7 on the newer STM32F4 devices.
 *
 * \details DMA reception of USART2 (transmission not used) on the second receive stream
 *          USART_RX_DMA_BUS2_DMA1_STREAM7. Devices where USART2_RX is served by stream 5 only:
 *          test ignored.
 *
 * \par Expected results
 * - One stream initialized: DMA1 stream 7, channel selection 6.
 */
void Ut_Usart_Dma_Usart2SecondRxStream_Initialized( void )
{
#if defined(STM32F410Cx) || \
    defined(STM32F410Rx) || \
    defined(STM32F410Tx) || \
    defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    dataConfig.TxMode = USART_XFER_MODE_NONE;
    dataConfig.RxDma  = USART_RX_DMA_BUS2_DMA1_STREAM7;

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_DmaInitCnt );
    TEST_ASSERT_EQUAL( DMA_PERIPH_1,      utUsart_DmaConfig[ 0u ].DmaPeriphId );
    TEST_ASSERT_EQUAL( DMA_STREAM_7,      utUsart_DmaConfig[ 0u ].DmaChannel );
    TEST_ASSERT_EQUAL( DMA_REQ_CHANNEL_6, utUsart_DmaConfig[ 0u ].PeripheralReqId );
    TEST_ASSERT_EQUAL_UINT32( 6u, USART_DMA_BIT_MASK_DECODE_CHSEL( USART_RX_DMA_BUS2_DMA1_STREAM7 ) );
#else
    TEST_IGNORE_MESSAGE( "USART2_RX is served by DMA1 stream 5 only" );
#endif /* STM32F410 / F411 / F412 / F413 / F423 */
}


/**
 * \brief   Usart_Set_DataConfig() reports DMA initialization failure.
 *
 * \details Dma_Init() returns error.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR.
 */
void Ut_Usart_Dma_InitFailure_Error( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    utUsart_DmaInitState = DMA_REQUEST_ERROR;

    Ut_Usart_Stub_DmaMocks();
    Ut_Usart_Stub_PeriphMocks();

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_DataConfig( UT_USART_BUS, &dataConfig ) );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].TcIrq );
}


/**
 * \brief   DMA transmission arms TX stream, end of transmission is detected by USART TC.
 *
 * \details DMA mode. SR.TC set before the transmission of 4 bytes. TX stream transfer
 *          complete callback is called, then ISR with TC.
 *
 * \par Expected results
 * - TC flag cleared before the transmission, TX stream armed with buffer address and 4
 *   bytes, DMAT set.
 * - DMA transfer complete: USART TC interrupt enabled, no complete callback.
 * - TC interrupt: complete callback 1x, TCIE cleared, TX state inactive.
 */
void Ut_Usart_Dma_Transmission_CompletedByUsartTc( void )
{
    usart_DataConfig_t          dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t       txState    = USART_FUNCTION_ACTIVE;
    const ut_UsartDmaStream_t * txStream   = &utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ];

    Ut_Usart_Set_DataConfig( &dataConfig );

    UT_USART_REG->SR = USART_SR_TC | USART_SR_TXE;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );

    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->SR & USART_SR_TC );
    TEST_ASSERT_EQUAL_UINT32( 1u, txStream->ActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utUsart_TxBuf, txStream->MemoryAddr );
    TEST_ASSERT_EQUAL_UINT32( 4u, txStream->DataCount );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAT, UT_USART_REG->CR3 & USART_CR3_DMAT );

    /* DMA transfer complete - last byte written to DR */
    utUsart_DmaConfig[ 0u ].TransferCompleteCallback();
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_TCIE, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );

    Ut_Usart_Call_Isr( USART_SR_TC | USART_SR_TXE );
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
 * - TX stream disabled, DMAT and TCIE cleared, no complete callback.
 */
void Ut_Usart_Dma_TxStop_StreamAndRequestDisabled( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    uint32_t           stopCnt    = 0u;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );

    stopCnt = utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].InactiveCnt;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( stopCnt + 1u, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ].InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_DMAT );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR1 & USART_CR1_TCIE );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt );
}


/**
 * \brief   DMA reception with idle line end reports count from the stream.
 *
 * \details DMA mode, RX end mode idle, one shot buffer 8 bytes. Reception started, DMA
 *          remaining count 5 (3 bytes received), ISR with IDLE (no pending byte).
 *
 * \par Expected results
 * - Start: RX stream armed with buffer and 8 bytes, DMAR, EIE, PEIE and IDLEIE set.
 * - RX count 3, end callback 1x with 3 bytes, reception stopped (stream disabled, DMAR
 *   cleared, interrupts disabled).
 */
void Ut_Usart_Dma_ReceptionIdleEnd_CountFromStream( void )
{
    usart_DataConfig_t          dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_RxDataCnt_t           rxCnt      = 0u;
    usart_FunctionState_t       rxState    = USART_FUNCTION_ACTIVE;
    const ut_UsartDmaStream_t * rxStream   = &utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_RX_STREAM ];

    dataConfig.RxEndMode = USART_RX_END_IDLE;

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, rxStream->ActiveCnt );
    TEST_ASSERT_EQUAL_HEX32( (uint32_t)(uintptr_t)utUsart_RxBuf, rxStream->MemoryAddr );
    TEST_ASSERT_EQUAL_UINT32( UT_USART_RX_SIZE, rxStream->DataCount );
    TEST_ASSERT_EQUAL_HEX32( USART_CR3_DMAR | USART_CR3_EIE, UT_USART_REG->CR3 & ( USART_CR3_DMAR | USART_CR3_EIE ) );
    TEST_ASSERT_EQUAL_HEX32( USART_CR1_PEIE | USART_CR1_IDLEIE, UT_USART_REG->CR1 & ( USART_CR1_PEIE | USART_CR1_IDLEIE | USART_CR1_RXNEIE ) );

    utUsart_DmaRemaining = 5u;
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( UT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL_UINT16( 3u, rxCnt );

    Ut_Usart_Call_Isr( USART_SR_IDLE );

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 3u, utUsart_RxEndBytes );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
    TEST_ASSERT_EQUAL_UINT32( 2u, rxStream->InactiveCnt );
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
 * - Complete callback 1x, reception active, stream not stopped.
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

    stopCnt = utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_RX_STREAM ].InactiveCnt;

    utUsart_DmaConfig[ 1u ].TransferCompleteCallback();

    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );
    TEST_ASSERT_EQUAL_UINT32( stopCnt, utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_RX_STREAM ].InactiveCnt );
}


/**
 * \brief   Reception errors in DMA mode are reported from USART interrupt, DMA keeps the data.
 *
 * \details DMA mode, reception running. ISR with RXNE + FE (byte pending for DMA), then
 *          with ORE only (byte already taken by DMA).
 *
 * \par Expected results
 * - Framing error and overrun reported, no byte stored by CPU (buffer unchanged).
 */
void Ut_Usart_Dma_ReceptionErrors_ReportedFromUsartIsr( void )
{
    usart_DataConfig_t dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    UT_USART_REG->DR = 0x7Eu;
    Ut_Usart_Call_Isr( USART_SR_RXNE | USART_SR_FE );
    TEST_ASSERT_EQUAL_UINT32( 1u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_FRAMING, utUsart_LastError );

    Ut_Usart_Call_Isr( USART_SR_ORE );
    TEST_ASSERT_EQUAL_UINT32( 2u, utUsart_ErrorCnt );
    TEST_ASSERT_EQUAL( USART_XFER_ERROR_OVERRUN, utUsart_LastError );

    TEST_ASSERT_EQUAL_HEX8( 0u, utUsart_RxBuf[ 0u ] );
}


/**
 * \brief   DMA transfer errors stop the direction and are reported.
 *
 * \details DMA mode, transmission and reception running. TX stream error callback, RX
 *          stream error callback.
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
 * \brief   DMA stream failure during transmission start is reported.
 *
 * \details DMA mode. Dma_Set_TransferActive() of TX stream fails.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, TX state inactive, DMAT not set.
 */
void Ut_Usart_Dma_TxStartStreamError_Error( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t txState    = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    Dma_Set_TransferActive_Stub( NULL );
    Dma_Set_TransferActive_ExpectAndReturn( UT_USART_DMA, UT_USART_DMA_TX_STREAM, DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 2u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & USART_CR3_DMAT );
}


/**
 * \brief   DMA stream failure during reception start is reported.
 *
 * \details DMA mode. Dma_Set_TransferActive() of RX stream fails.
 *
 * \par Expected results
 * - USART_REQUEST_ERROR, RX state inactive, DMAR and reception interrupts not enabled.
 */
void Ut_Usart_Dma_RxStartStreamError_Error( void )
{
    usart_DataConfig_t    dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    usart_FunctionState_t rxState    = USART_FUNCTION_ACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    Dma_Set_TransferActive_Stub( NULL );
    Dma_Set_TransferActive_ExpectAndReturn( UT_USART_DMA, UT_USART_DMA_RX_STREAM, DMA_REQUEST_ERROR );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_RxStart( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( UT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_DMAR | USART_CR3_EIE ) );
}


/**
 * \brief   Usart_Deinit() releases DMA streams of the data handling.
 *
 * \par Expected results
 * - Both streams disabled, their interrupts and NVIC lines disabled, DMAT / DMAR cleared.
 */
void Ut_Usart_Dma_Deinit_StreamsReleased( void )
{
    usart_DataConfig_t          dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
    const ut_UsartDmaStream_t * txStream   = &utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_TX_STREAM ];
    const ut_UsartDmaStream_t * rxStream   = &utUsart_DmaStream[ UT_USART_DMA ][ UT_USART_DMA_RX_STREAM ];

    Ut_Usart_Set_DataConfig( &dataConfig );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );

    Ut_Usart_Stub_PeriphMocks();
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    TEST_ASSERT_EQUAL_UINT32( 0u, txStream->TcIrq + txStream->TeIrq + txStream->NvicIrq );
    TEST_ASSERT_EQUAL_UINT32( 0u, rxStream->TcIrq + rxStream->TeIrq + rxStream->HtIrq + rxStream->NvicIrq );
    TEST_ASSERT_TRUE( 0u < txStream->InactiveCnt );
    TEST_ASSERT_TRUE( 0u < rxStream->InactiveCnt );
    TEST_ASSERT_EQUAL_HEX32( 0u, UT_USART_REG->CR3 & ( USART_CR3_DMAT | USART_CR3_DMAR ) );
}

/* ======================= OTHER MODES / OTHER PERIPHERALS ================== */

/** ISR registered in NVIC for any USART/UART peripheral */
static nvic_IsrCallback_t utUsart_AnyIsr;

/** NVIC handler registration stub of any USART/UART peripheral - stores the handler */
static nvic_RequestState_t Ut_Usart_NvicAnyHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)irqId;
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    utUsart_AnyIsr = irqHandler;

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief   Polling transmission is stopped by Usart_Set_TxStop(), transmission without
 *          transmit data handling is refused, polling data handling is released.
 *
 * \details USART2 TX and RX in POLL mode: Usart_Set_TxStart(), Usart_Set_TxStop(),
 *          Usart_Deinit(). USART2 with TX mode NONE: Usart_Set_TxStart().
 *
 * \par Expected results
 * - POLL: transmission ACTIVE after the start, INACTIVE after the stop, no callback,
 *   Usart_Deinit() returns USART_REQUEST_OK.
 * - TX mode NONE: Usart_Set_TxStart() returns USART_REQUEST_ERROR, transmission INACTIVE.
 */
void Ut_Usart_Poll_TxStopAndDeinit_NoneTxRefused( void )
{
    usart_DataConfig_t     dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_POLL, UT_USART_RX_SIZE );
    usart_FunctionState_t  txState    = USART_FUNCTION_INACTIVE;

    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, txState );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( UT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
    TEST_ASSERT_EQUAL_UINT32( 0u, utUsart_TxCompleteCnt + utUsart_ErrorCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( UT_USART_BUS ) );

    dataConfig.TxMode = USART_XFER_MODE_NONE;
    Ut_Usart_Set_DataConfig( &dataConfig );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( UT_USART_BUS, utUsart_TxBuf, 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( UT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
}


/**
 * \brief   Other USART/UART peripherals use their own interrupt, DMA streams and DMA callbacks.
 *
 * \details Every USART/UART of the MCU except USART2 (its items are used by other tests) in DMA
 *          mode (TX and RX, one shot buffer), every item of the DMA stream lists of the bus,
 *          items paired by the bus. Captured USART interrupt without flags, then DMA callbacks
 *          captured from Dma_Init() are called: RX half / complete / error, TX complete /
 *          error. MCUs with USART2 only: test ignored.
 *
 * \par Expected results
 * - TX / RX stream with the request channel of the peripheral (RM0090 DMA request mapping),
 *   the channel selection stored in the list items equals the request map.
 * - Interrupt without flags: no callback.
 * - RX half: RxHalfCallback, RX complete: RxCompleteCallback, RX / TX error: ErrorCallback with
 *   USART_XFER_ERROR_DMA_TRANSFER, TX complete: TC interrupt enabled in CR1 of the peripheral.
 * - Usart_Deinit(): USART_REQUEST_OK.
 */
void Ut_Usart_Dma_OtherPeriphCallbacks_OwnPeripheralReported( void )
{
#if defined(USART1) || defined(USART3) || defined(UART4) || defined(UART5) || defined(USART6) || defined(UART7) || defined(UART8)
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
#ifdef USART1
        { USART_BUS_1, USART1, USART_TX_DMA_BUS1_DMA2_STREAM7, USART_RX_DMA_BUS1_DMA2_STREAM2, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, USART_DMA_CHANNEL_2, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
        { USART_BUS_1, USART1, USART_TX_DMA_BUS1_DMA2_STREAM7, USART_RX_DMA_BUS1_DMA2_STREAM5, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, USART_DMA_CHANNEL_5, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
#endif /* USART1 */
#ifdef USART3
        { USART_BUS_3, USART3, USART_TX_DMA_BUS3_DMA1_STREAM3, USART_RX_DMA_BUS3_DMA1_STREAM1, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_3, USART_DMA_CHANNEL_1, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
        { USART_BUS_3, USART3, USART_TX_DMA_BUS3_DMA1_STREAM4, USART_RX_DMA_BUS3_DMA1_STREAM1, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_4, USART_DMA_CHANNEL_1, DMA_REQ_CHANNEL_7, DMA_REQ_CHANNEL_4 },
#endif /* USART3 */
#ifdef UART4
        { USART_BUS_4, UART4,  USART_TX_DMA_BUS4_DMA1_STREAM4, USART_RX_DMA_BUS4_DMA1_STREAM2, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_4, USART_DMA_CHANNEL_2, DMA_REQ_CHANNEL_4, DMA_REQ_CHANNEL_4 },
#endif /* UART4 */
#ifdef UART5
        { USART_BUS_5, UART5,  USART_TX_DMA_BUS5_DMA1_STREAM7, USART_RX_DMA_BUS5_DMA1_STREAM0, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_7, USART_DMA_CHANNEL_0, UT_USART_UART5_TX_SEL, DMA_REQ_CHANNEL_4 },
#endif /* UART5 */
#ifdef USART6
        { USART_BUS_6, USART6, USART_TX_DMA_BUS6_DMA2_STREAM6, USART_RX_DMA_BUS6_DMA2_STREAM1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_6, USART_DMA_CHANNEL_1, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
        { USART_BUS_6, USART6, USART_TX_DMA_BUS6_DMA2_STREAM7, USART_RX_DMA_BUS6_DMA2_STREAM2, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, USART_DMA_CHANNEL_2, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
#endif /* USART6 */
#ifdef UART7
        { USART_BUS_7, UART7,  USART_TX_DMA_BUS7_DMA1_STREAM1, USART_RX_DMA_BUS7_DMA1_STREAM3, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_1, USART_DMA_CHANNEL_3, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
#endif /* UART7 */
#ifdef UART8
        { USART_BUS_8, UART8,  USART_TX_DMA_BUS8_DMA1_STREAM0, USART_RX_DMA_BUS8_DMA1_STREAM6, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_0, USART_DMA_CHANNEL_6, DMA_REQ_CHANNEL_5, DMA_REQ_CHANNEL_5 },
#endif /* UART8 */
    };

    for( uint32_t idx = 0u; ( sizeof( periphLut ) / sizeof( periphLut[ 0u ] ) ) > idx; idx++ )
    {
        usart_DataConfig_t         dataConfig = Ut_Usart_Get_DataConfig( USART_XFER_MODE_DMA, UT_USART_RX_SIZE );
        const dma_ConfigStruct_t * txConfig   = NULL;
        const dma_ConfigStruct_t * rxConfig   = NULL;

        dataConfig.TxDma = periphLut[ idx ].TxDma;
        dataConfig.RxDma = periphLut[ idx ].RxDma;

        Ut_Usart_Stub_PeriphMocks();
        Ut_Usart_Stub_DmaMocks();
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

        periphLut[ idx ].PeriphReg->SR = 0u;
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

        Ut_Usart_Stub_PeriphMocks();
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( periphLut[ idx ].UsartId ) );
    }
#else
    TEST_IGNORE_MESSAGE( "MCU with USART2 only" );
#endif /* USART1 OR USART3 OR UART4 OR UART5 OR USART6 OR UART7 OR UART8 */
}


/**
 * \brief   Items of the DMA stream lists carry bus, DMA peripheral, stream and channel selection of
 *          the stream.
 *
 * \details Expected values are written as (bus, DMA peripheral index, stream number, channel
 *          selection number) taken from the DMA request mapping of the STM32F4 reference manuals,
 *          independently of the encoding macro.
 *
 * \par Expected results
 * - Every transmit and receive item of the USART / UART buses of the MCU carries the expected
 *   bit-fields.
 * - Unused items of both lists equal USART_DMA_CODE_UNUSED, the decoding macros return the fields.
 */
void Ut_Usart_DmaLists_Items_EncodeBusDmaStreamAndChannelSelection( void )
{
    /* USART1 (DMA2, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_1, 1u, 7u, 4u ), USART_TX_DMA_BUS1_DMA2_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_1, 1u, 2u, 4u ), USART_RX_DMA_BUS1_DMA2_STREAM2 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_1, 1u, 5u, 4u ), USART_RX_DMA_BUS1_DMA2_STREAM5 );

    /* USART2 (DMA1, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_2, 0u, 6u, 4u ), USART_TX_DMA_BUS2_DMA1_STREAM6 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_2, 0u, 5u, 4u ), USART_RX_DMA_BUS2_DMA1_STREAM5 );
#if defined(STM32F410Cx) || \
    defined(STM32F410Rx) || \
    defined(STM32F410Tx) || \
    defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_2, 0u, 7u, 6u ), USART_RX_DMA_BUS2_DMA1_STREAM7 );
#endif /* STM32F410 / F411 / F412 / F413 / F423 */

#if defined(USART3)
    /* USART3 (DMA1, channel selection 4, TX stream 4 with channel selection 7) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_3, 0u, 3u, 4u ), USART_TX_DMA_BUS3_DMA1_STREAM3 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_3, 0u, 4u, 7u ), USART_TX_DMA_BUS3_DMA1_STREAM4 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_3, 0u, 1u, 4u ), USART_RX_DMA_BUS3_DMA1_STREAM1 );
#endif /* USART3 */

#if defined(UART4)
    /* UART4 (DMA1, channel selection 4) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_4, 0u, 4u, 4u ), USART_TX_DMA_BUS4_DMA1_STREAM4 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_4, 0u, 2u, 4u ), USART_RX_DMA_BUS4_DMA1_STREAM2 );
#endif /* UART4 */

#if defined(UART5)
    /* UART5 (DMA1, channel selection 4, TX channel selection 8 on STM32F413 / STM32F423) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_5, 0u, 7u, UT_USART_UART5_TX_CHSEL ), USART_TX_DMA_BUS5_DMA1_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_5, 0u, 0u, 4u ), USART_RX_DMA_BUS5_DMA1_STREAM0 );
#endif /* UART5 */

#if defined(USART6)
    /* USART6 (DMA2, channel selection 5) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_6, 1u, 6u, 5u ), USART_TX_DMA_BUS6_DMA2_STREAM6 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_6, 1u, 7u, 5u ), USART_TX_DMA_BUS6_DMA2_STREAM7 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_6, 1u, 1u, 5u ), USART_RX_DMA_BUS6_DMA2_STREAM1 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_6, 1u, 2u, 5u ), USART_RX_DMA_BUS6_DMA2_STREAM2 );
#endif /* USART6 */

#if defined(UART7)
    /* UART7 (DMA1, channel selection 5) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_7, 0u, 1u, 5u ), USART_TX_DMA_BUS7_DMA1_STREAM1 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_7, 0u, 3u, 5u ), USART_RX_DMA_BUS7_DMA1_STREAM3 );
#endif /* UART7 */

#if defined(UART8)
    /* UART8 (DMA1, channel selection 5) */
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_8, 0u, 0u, 5u ), USART_TX_DMA_BUS8_DMA1_STREAM0 );
    TEST_ASSERT_EQUAL_HEX32( UT_USART_DMA_CODE( USART_BUS_8, 0u, 6u, 5u ), USART_RX_DMA_BUS8_DMA1_STREAM6 );
#endif /* UART8 */

    /* Decoding of the fields */
    TEST_ASSERT_EQUAL_UINT32( USART_BUS_1,        USART_DMA_BIT_MASK_DECODE_PERIPH( USART_RX_DMA_BUS1_DMA2_STREAM5 ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_PERIPH_2, USART_DMA_BIT_MASK_DECODE_DMA( USART_RX_DMA_BUS1_DMA2_STREAM5 ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_CHANNEL_5, USART_DMA_BIT_MASK_DECODE_STREAM( USART_RX_DMA_BUS1_DMA2_STREAM5 ) );
    TEST_ASSERT_EQUAL_UINT32( 4u,                 USART_DMA_BIT_MASK_DECODE_CHSEL( USART_RX_DMA_BUS1_DMA2_STREAM5 ) );

    /* Unused stream */
    TEST_ASSERT_EQUAL_HEX32( USART_DMA_CODE_UNUSED, USART_TX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_HEX32( USART_DMA_CODE_UNUSED, USART_RX_DMA_UNUSED );
    TEST_ASSERT_EQUAL_UINT32( USART_BUS_CNT,         USART_DMA_BIT_MASK_DECODE_PERIPH( USART_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_PERIPH_CNT,  USART_DMA_BIT_MASK_DECODE_DMA( USART_TX_DMA_UNUSED ) );
    TEST_ASSERT_EQUAL_UINT32( USART_DMA_CHANNEL_CNT, USART_DMA_BIT_MASK_DECODE_STREAM( USART_TX_DMA_UNUSED ) );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Expects read of peripheral clock state.
 *
 * \param periphId   [in]: RCC peripheral
 * \param clockState [in]: Reported clock state
 */
static void Ut_Usart_Expect_ClockState( rcc_PeriphId_t periphId, rcc_FunctionState_t clockState )
{
    static rcc_FunctionState_t reportedState[ 8u ];
    static uint32_t            reportedIdx = 0u;

    reportedIdx                  = ( reportedIdx + 1u ) % 8u;
    reportedState[ reportedIdx ] = clockState;

    Rcc_Get_PeriphState_ExpectAndReturn( periphId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphState_IgnoreArg_funcState();
    Rcc_Get_PeriphState_ReturnThruPtr_funcState( &reportedState[ reportedIdx ] );
}


/**
 * \brief Expects read of peripheral clock frequency.
 *
 * \param periphId [in]: RCC peripheral
 * \param clockHz  [in]: Reported clock frequency
 */
static void Ut_Usart_Expect_PeriphClk( rcc_PeriphId_t periphId, rcc_FreqHz_t clockHz )
{
    static rcc_FreqHz_t reportedClk[ 8u ];
    static uint32_t     reportedIdx = 0u;

    /* Every expectation keeps own storage - more expectations can be queued */
    reportedIdx                = ( reportedIdx + 1u ) % 8u;
    reportedClk[ reportedIdx ] = clockHz;

    Rcc_Get_PeriphClk_ExpectAndReturn( periphId, NULL, RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_IgnoreArg_periphClk();
    Rcc_Get_PeriphClk_ReturnThruPtr_periphClk( &reportedClk[ reportedIdx ] );
}


/**
 * \brief Expects reset of the peripheral through RCC.
 *
 * \param periphId [in]: RCC peripheral
 */
static void Ut_Usart_Expect_Reset( rcc_PeriphId_t periphId )
{
    Rcc_Set_ResetActive_ExpectAndReturn( periphId, RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_ExpectAndReturn( periphId, RCC_REQUEST_OK );
}


/**
 * \brief Expects initialization of GPIO pin in alternate function mode.
 *
 * \param portId  [in]: GPIO port
 * \param pinId   [in]: GPIO pin
 * \param altFunc [in]: Alternate function
 * \param outType [in]: Output type
 * \param pull    [in]: Pull resistor configuration
 */
static void Ut_Usart_Expect_GpioInit( gpio_PortId_t portId, gpio_PinId_t pinId, gpio_AltFunction_t altFunc, gpio_PinOutputType_t outType, gpio_PinPullCfg_t pull )
{
    static gpio_Config_t expectedConfig[ 4u ];
    static uint32_t      expectedIdx = 0u;

    expectedIdx = ( expectedIdx + 1u ) % 4u;

    (void)memset( &expectedConfig[ expectedIdx ], 0, sizeof( gpio_Config_t ) );

    expectedConfig[ expectedIdx ].PortId         = portId;
    expectedConfig[ expectedIdx ].PinId          = pinId;
    expectedConfig[ expectedIdx ].PinMode        = GPIO_PIN_MODE_ALTERNATE;
    expectedConfig[ expectedIdx ].PinPull        = pull;
    expectedConfig[ expectedIdx ].PinSpeed       = GPIO_PIN_SPEED_HIGH;
    expectedConfig[ expectedIdx ].PinOutType     = outType;
    expectedConfig[ expectedIdx ].PinAltFunction = altFunc;
    expectedConfig[ expectedIdx ].PinActiveLevel = GPIO_PIN_LEVEL_HIGH;

    Gpio_Init_ExpectAndReturn( &expectedConfig[ expectedIdx ], GPIO_REQUEST_OK );
}


/**
 * \brief Returns configuration of USART2 used by tests (default configuration, USART2).
 *
 * \return Bus configuration
 */
static usart_BusConfig_t Ut_Usart_Get_Config( void )
{
    usart_BusConfig_t config;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &config ) );

    config.PeriphId = UT_USART_BUS;

    return ( config );
}


/**
 * \brief Returns data handling configuration of both directions in the same mode (DMA:
 *        USART2 streams DMA1 S6 / S5).
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
 * \brief Replaces RCC / NVIC / GPIO functions by stubs (any call accepted).
 */
static void Ut_Usart_Stub_PeriphMocks( void )
{
    Rcc_Get_PeriphState_Stub( Ut_Usart_RccGetStateStub );
    Rcc_Set_PeriphActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_PeriphInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetActive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Set_ResetInactive_IgnoreAndReturn( RCC_REQUEST_OK );
    Rcc_Get_PeriphClk_Stub( Ut_Usart_RccGetClkStub );
    Gpio_Init_IgnoreAndReturn( GPIO_REQUEST_OK );
    Nvic_Set_PeriphIrq_Handler_Stub( Ut_Usart_NvicSetHandlerStub );
    Nvic_Set_PeriphIrq_Prio_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Active_IgnoreAndReturn( NVIC_REQUEST_OK );
    Nvic_Set_PeriphIrq_Inactive_IgnoreAndReturn( NVIC_REQUEST_OK );
}


/**
 * \brief Replaces DMA functions by recording stubs (\ref utUsart_DmaStream).
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
 * \brief Releases data handling of the previous test (static module context) and
 *        re-initializes the mocks.
 */
static void Ut_Usart_Release( void )
{
    Ut_Usart_Stub_PeriphMocks();
    Ut_Usart_Stub_DmaMocks();

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
 * \brief Configures data handling of USART2 with stubbed NVIC / DMA calls.
 *
 * \param dataConfig [in]: Data handling configuration
 */
static void Ut_Usart_Set_DataConfig( const usart_DataConfig_t * const dataConfig )
{
    Ut_Usart_Stub_PeriphMocks();
    Ut_Usart_Stub_DmaMocks();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( UT_USART_BUS, dataConfig ) );
}


/**
 * \brief Calls the captured USART ISR with given status register flags.
 *
 * \param srFlags [in]: Value of the status register
 */
static void Ut_Usart_Call_Isr( uint32_t srFlags )
{
    TEST_ASSERT_NOT_NULL_MESSAGE( utUsart_Isr, "USART ISR not registered" );

    UT_USART_REG->SR = srFlags;
    utUsart_Isr();
}


/**
 * \brief NVIC handler registration stub - stores ISR of the USART.
 */
static nvic_RequestState_t Ut_Usart_NvicSetHandlerStub( nvic_PeriphIrqList_t irqId, const nvic_IsrCallback_t irqHandler, int callCnt )
{
    (void)callCnt;

    TEST_ASSERT_NOT_NULL( irqHandler );

    if( UT_USART_NVIC == irqId )
    {
        utUsart_Isr = irqHandler;
    }
    else
    {
        /* Handler of other peripheral */
    }

    return ( NVIC_REQUEST_OK );
}


/**
 * \brief RCC clock stub - returns \ref UT_USART_CLK_HZ.
 */
static rcc_RequestState_t Ut_Usart_RccGetClkStub( rcc_PeriphId_t periphId, rcc_FreqHz_t * const periphClk, int callCnt )
{
    (void)callCnt;
    (void)periphId;

    *periphClk = UT_USART_CLK_HZ;

    return ( RCC_REQUEST_OK );
}


/**
 * \brief RCC clock state stub - returns \ref utUsart_ClockState.
 */
static rcc_RequestState_t Ut_Usart_RccGetStateStub( rcc_PeriphId_t periphId, rcc_FunctionState_t * const funcState, int callCnt )
{
    (void)callCnt;
    (void)periphId;

    *funcState = utUsart_ClockState;

    return ( RCC_REQUEST_OK );
}


/** \brief Dma_Init() stub - captures configuration, returns \ref utUsart_DmaInitState */
static dma_RequestState_t Ut_Usart_DmaInitStub( dma_ConfigStruct_t * const dmaConfig, int callCnt )
{
    (void)callCnt;

    if( UT_USART_DMA_CFG_CNT > utUsart_DmaInitCnt )
    {
        utUsart_DmaConfig[ utUsart_DmaInitCnt ] = *dmaConfig;
    }

    utUsart_DmaInitCnt++;

    return ( utUsart_DmaInitState );
}


/** \brief Returns record of DMA stream calls */
static ut_UsartDmaStream_t* Ut_Usart_Get_DmaStream( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel )
{
    TEST_ASSERT_TRUE( DMA_PERIPH_CNT > dmaBus );
    TEST_ASSERT_TRUE( UT_USART_DMA_STREAMS > dmaChannel );

    return ( &utUsart_DmaStream[ dmaBus ][ dmaChannel ] );
}


/** \brief Dma_Set_TransferActive() stub */
static dma_RequestState_t Ut_Usart_DmaActiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->ActiveCnt++;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferInactive() stub */
static dma_RequestState_t Ut_Usart_DmaInactiveStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->InactiveCnt++;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_MemoryAddr() stub */
static dma_RequestState_t Ut_Usart_DmaMemoryAddrStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_MemoryAddr_t memoryAddr, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->MemoryAddr = memoryAddr;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_DataCount() stub */
static dma_RequestState_t Ut_Usart_DmaDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t dataCount, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->DataCount = dataCount;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Get_DataCount() stub - returns \ref utUsart_DmaRemaining */
static dma_RequestState_t Ut_Usart_DmaGetDataCountStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, dma_DataCount_t * const dataCount, int callCnt )
{
    (void)callCnt;
    (void)Ut_Usart_Get_DmaStream( dmaBus, dmaChannel );
    *dataCount = utUsart_DmaRemaining;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_HalfTransferIrqActive() stub */
static dma_RequestState_t Ut_Usart_DmaHtIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->HtIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_HalfTransferIrqInactive() stub */
static dma_RequestState_t Ut_Usart_DmaHtIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->HtIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferCompleteIrqActive() stub */
static dma_RequestState_t Ut_Usart_DmaTcIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->TcIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferCompleteIrqInactive() stub */
static dma_RequestState_t Ut_Usart_DmaTcIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->TcIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferErrorIrqActive() stub */
static dma_RequestState_t Ut_Usart_DmaTeIrqOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->TeIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_TransferErrorIrqInactive() stub */
static dma_RequestState_t Ut_Usart_DmaTeIrqOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->TeIrq = 0u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_InterruptActive() stub */
static dma_RequestState_t Ut_Usart_DmaNvicOnStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->NvicIrq = 1u;
    return ( DMA_REQUEST_OK );
}


/** \brief Dma_Set_InterruptInactive() stub */
static dma_RequestState_t Ut_Usart_DmaNvicOffStub( dma_PeriphId_t dmaBus, dma_ChannelId_t dmaChannel, int callCnt )
{
    (void)callCnt;
    Ut_Usart_Get_DmaStream( dmaBus, dmaChannel )->NvicIrq = 0u;
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
 * \brief End of received message callback restarting the reception.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void Ut_Usart_RxEndRestartCallback( usart_RxDataCnt_t rxCnt )
{
    Ut_Usart_RxEndCallback( rxCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( UT_USART_BUS ) );
}


/**
 * \brief Data transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void Ut_Usart_ErrorCallback( usart_XferErrorId_t errorId )
{
    utUsart_LastError  = errorId;
    utUsart_ErrorMask |= ( 1u << (uint32_t)errorId );
    utUsart_ErrorCnt++;
}
