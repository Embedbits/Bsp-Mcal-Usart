/**
 * \author Mr.Nobody
 * \file ItTest_Usart.c
 * \ingroup Usart
 * \brief Integration tests of USART module on target.
 *
 * Usart module runs on the MCU together with real RCC, NVIC, GPIO and DMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): baud rate generation, data transfer in polling,
 * interrupt and DMA mode, end of message (idle line, receiver timeout) detection,
 * circular reception buffer, parity and frame timing (LPUART1 is not covered).
 *
 * Single bus tests: no external wiring - USART works in single wire half-duplex mode, the
 * receiver receives every frame sent by the transmitter on the TX pin.
 *
 * Pair tests: two U(S)ARTs of the board are crossed (TX of A -> RX of B, TX of B -> RX of A).
 * Loop tests: TX of a U(S)ART is wired to its own RX. Pins of the wires in the board blocks below.
 * Fixture of the board (EmBi_Platform IntegrationTesting, "Board fixtures"): the wiring is the
 * feature UART_PAIR / UART_LOOP - the pair / loop tests are ignored on a board with a fixture
 * without it (a board without a fixture is expected to be wired).
 *
 * Boards (named by the MCU as the detection of the connected boards does):
 * - STM32H723xG - NUCLEO-H723ZG (the detection names every board with ID 0x483
 *   and 1 MB flash STM32H723xG)
 * - STM32H7A3xI / STM32H7A3xIQ - NUCLEO-H7A3ZI-Q (the detection names every
 *   board with ID 0x480 and 2 MB flash STM32H7A3xI, STM32H7A3xIQ by a board file
 *   override)
 * - STM32H755xI - NUCLEO-H755ZI-Q (dual-core, the tests run on the Cortex-M7)
 * All are Nucleo-144 boards with the common Zio / morpho pinout.
 *
 * Used resources (see board configuration below):
 * - IT_USART_BUS (USART6), TX pin IT_USART_TX_PIN (PC6) - not connected on the board
 * - DMA2 stream 6 (TX), streams 1 / 2 (RX), requests routed by DMAMUX1 (DMA mode, single bus)
 * - IT_USART_PAIR_A_BUS / IT_USART_PAIR_B_BUS on their TX / RX pins (pair tests), IT_USART_LOOP_BUS
 *   on its TX / RX pins (loop tests): DMA1 streams 0 / 1 (bus A), 2 / 3 (bus B), 4 / 5 (loop)
 * - DWT cycle counter - reference time base for frame timing
 * - PLL1 - system clock 200 MHz set by \ref setUp on a board with its supply configuration
 *   (RCC_BOARD_PWR_SUPPLY, AB#2208): at the reset clock (HSI 64 MHz) one call of the interrupt handler of the
 *   module takes up to 4594 of the 5556 core clock cycles of one frame at 115200 Bd (DWT, NUCLEO-H723ZG). A board
 *   without the define keeps the reset clock - the voltage scale cannot be raised before the supply configuration
 *   is written.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usart_Port.h"                     /* Module under test              */
#include "Dwt_Port.h"                       /* Reference time base            */
#include "Gpio_Port.h"                      /* Pull-up of single wire line    */
#include "Stm32.h"                          /* SystemCoreClock                */
#include "Rcc_Port.h"                       /* Clock configuration of the set */
/* ============================= TYPEDEFS =================================== */

/** \brief Bus of a test case (USART with its single wire pin) */
typedef struct
{
    usart_PeriphId_t     Bus;          /**< USART/UART bus                        */
    usart_TxPin_t        TxPin;        /**< TX pin (single wire line)             */
    gpio_PortId_t        TxPort;       /**< Port of TX pin                        */
    gpio_PinId_t         TxPinId;      /**< Pin of TX pin                         */
    usart_Oversampling_t Oversampling; /**< Over-sampling                         */
}   it_UsartBus_t;

/** Bus of the UART pair */
typedef enum
{
    IT_USART_PAIR_A = 0u, /**< Bus A of the pair */
    IT_USART_PAIR_B,      /**< Bus B of the pair */
    IT_USART_PAIR_CNT     /**< Count of buses of the pair */
}   it_UsartPairBus_t;


/** Callback record and buffers of one bus of the pair */
typedef struct
{
    volatile uint32_t           TxCompleteCnt; /**< TX complete callbacks              */
    volatile uint32_t           RxCompleteCnt; /**< RX complete callbacks              */
    volatile uint32_t           RxEndCnt;      /**< RX end (idle line) callbacks       */
    volatile usart_RxDataCnt_t  RxEndBytes;    /**< Byte count of the last RX end      */
    volatile uint32_t           ErrorCnt;      /**< Error callbacks                    */
    usart_DataConfig_t          DataConfig;    /**< Data handling configuration        */
}   it_UsartPair_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void ItUsart_Init             ( usart_XferMode_t xferMode,
                                       usart_RxDataCnt_t rxSize,
                                       usart_BufferMode_t bufferMode,
                                       usart_RxEndMode_t rxEndMode,
                                       usart_Parity_t parity,
                                       usart_DataWidth_t dataWidth );
static void ItUsart_Wait            ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void ItUsart_Transfer        ( usart_TxDataCnt_t txSize );
static void ItUsart_Init_Clock      ( void );
static void ItUsart_Init_RefTime    ( void );
static void ItUsart_Delay_Frames    ( uint32_t frameCnt );
static void ItUsart_Check_FrameTime( void );

static void ItUsart_TxCompleteCallback  ( void );
static void ItUsart_RxHalfCallback      ( void );
static void ItUsart_RxCompleteCallback  ( void );
static void ItUsart_RxEndCallback       ( usart_RxDataCnt_t rxCnt );
static void ItUsart_ErrorCallback       ( usart_XferErrorId_t errorId );

static void             ItUsart_RequirePair     ( void );
static void             ItUsart_Init_Pair       ( usart_XferMode_t xferMode,
                                                   usart_RxDataCnt_t rxSize,
                                                   usart_RxEndMode_t rxEndMode,
                                                   usart_Baudrate_t baudrate );
static void             ItUsart_Init_PairBus    ( it_UsartPairBus_t pairBus,
                                                   usart_XferMode_t xferMode,
                                                   usart_RxDataCnt_t rxSize,
                                                   usart_RxEndMode_t rxEndMode,
                                                   usart_Baudrate_t baudrate );
static void             ItUsart_Transfer_Pair   ( it_UsartPairBus_t txBus,
                                                   it_UsartPairBus_t rxBus,
                                                   usart_TxDataCnt_t txSize );
static void             ItUsart_Exchange_Pair   ( usart_TxDataCnt_t dataCnt );
static usart_PeriphId_t ItUsart_Get_PairBusId   ( it_UsartPairBus_t pairBus );
static void             ItUsart_PairA_TxComplete( void );
static void             ItUsart_PairA_RxComplete( void );
static void             ItUsart_PairA_RxEnd     ( usart_RxDataCnt_t rxCnt );
static void             ItUsart_PairA_Error     ( usart_XferErrorId_t errorId );
static void             ItUsart_PairB_TxComplete( void );
static void             ItUsart_PairB_RxComplete( void );
static void             ItUsart_PairB_RxEnd     ( usart_RxDataCnt_t rxCnt );
static void             ItUsart_PairB_Error     ( usart_XferErrorId_t errorId );

static void             ItUsart_RequireLoop     ( void );
static void             ItUsart_Set_DataConfig  ( usart_XferMode_t xferMode,
                                                   usart_RxDataCnt_t rxSize,
                                                   usart_BufferMode_t bufferMode,
                                                   usart_RxEndMode_t rxEndMode );
static void             ItUsart_Init_Loop       ( usart_XferMode_t xferMode,
                                                   usart_RxDataCnt_t rxSize );
static void             ItUsart_Transfer_Loop   ( usart_TxDataCnt_t txSize );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32H723xG)  || \
    defined(IT_BOARD_STM32H755xI)  || \
    defined(IT_BOARD_STM32H7A3xI)  || \
    defined(IT_BOARD_STM32H7A3xIQ)

    /* NUCLEO-H723ZG / NUCLEO-H755ZI-Q / NUCLEO-H7A3ZI-Q (Nucleo-144) */

    /** USART6, PC6 (USART6_TX, Zio / morpho connector) - not connected on the board */
    #define IT_USART_BUS                    ( USART_BUS_6 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS6_PC6 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_C )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_6 )

    /** Pair (inventory wires of the fixture feature UART_PAIR): UART4 TX PC10 -> UART5 RX PD2,
     *  UART5 TX PC12 -> UART4 RX PC11 */
    #define IT_USART_PAIR_AVAILABLE         ( 1u )
    #define IT_USART_PAIR_A_BUS             ( USART_BUS_4 )
    #define IT_USART_PAIR_A_TX_PIN          ( USART_TX_PIN_BUS4_PC10 )
    #define IT_USART_PAIR_A_RX_PIN          ( USART_RX_PIN_BUS4_PC11 )
    #define IT_USART_PAIR_B_BUS             ( USART_BUS_5 )
    #define IT_USART_PAIR_B_TX_PIN          ( USART_TX_PIN_BUS5_PC12 )
    #define IT_USART_PAIR_B_RX_PIN          ( USART_RX_PIN_BUS5_PD2 )

    /** Loop (fixture feature UART_LOOP) */
    #define IT_USART_LOOP_AVAILABLE         ( 1u )
    #if defined(IT_BOARD_STM32H7A3xI)  || \
        defined(IT_BOARD_STM32H7A3xIQ)

        /* NUCLEO-H7A3ZI-Q: USART2 TX PA2 -> USART2 RX PA3 */
        #define IT_USART_LOOP_BUS           ( USART_BUS_2 )
        #define IT_USART_LOOP_TX_PIN        ( USART_TX_PIN_BUS2_PA2 )
        #define IT_USART_LOOP_RX_PIN        ( USART_RX_PIN_BUS2_PA3 )
    #else

        /* NUCLEO-H723ZG / NUCLEO-H755ZI-Q: UART7 TX PF7 -> UART7 RX PF6 */
        #define IT_USART_LOOP_BUS           ( USART_BUS_7 )
        #define IT_USART_LOOP_TX_PIN        ( USART_TX_PIN_BUS7_PF7 )
        #define IT_USART_LOOP_RX_PIN        ( USART_RX_PIN_BUS7_PF6 )
    #endif /* NUCLEO-H7A3ZI-Q */

#else
    #error "Board of Usart integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** Baud rate [Bd] */
#define IT_USART_BAUDRATE                   ( 115200u )

/** Size of data buffers */
#define IT_USART_BUF_SIZE                   ( 64u )

/** Maximal count of wait loop iterations (~ hundreds of frames, Usart_Task called) */
#define IT_USART_WAIT_LOOPS                 ( 2000000u )

/** Minimal count of core clock cycles of one pass of the frame delay loop - the bound of the passes is the number
 *  of core clock cycles of the delay, so the bound never ends the delay before the time is reached */
#define IT_USART_MIN_CYCLES_PER_PASS        ( 1u )

/** Count of bytes of frame timing measurement */
#define IT_USART_TIMING_BYTES               ( 50u )

/** Count of wait loop iterations of line settling after pull-up activation (several frames) */
#define IT_USART_LINE_SETTLE_LOOPS          ( 50000u )

/** Bits of one 8N1 frame (start + 8 data + stop) */
#define IT_USART_FRAME_BITS                 ( 10u )

/** Receiver timeout of timeout end mode [bits] (two frames) */
#define IT_USART_RTO_BITS                   ( 22u )

/** Baud rate of the fast pair test [Bd] */
#define IT_USART_FAST_BAUDRATE              ( 921600u )

/** Byte counts of the pair tests (short / long transfer, message of the idle line test) */
#define IT_USART_PAIR_SHORT                 ( 16u )
#define IT_USART_PAIR_LONG                  ( 48u )
#define IT_USART_PAIR_MSG                   ( 10u )

/** Byte counts of the loop tests (polling / interrupt transfer; the DMA transfer fills the whole buffer) */
#define IT_USART_LOOP_SHORT                 ( 16u )
#define IT_USART_LOOP_LONG                  ( 48u )

/** First bytes of the data patterns of the pair buses */
#define IT_USART_PAIR_SEED_A                ( 0x41u )
#define IT_USART_PAIR_SEED_B                ( 0x81u )

/** DMA1 streams of the pair buses and of the loop bus (requests routed by DMAMUX1) */
#define IT_USART_PAIR_DMA_PERIPH            ( USART_DMA_PERIPH_1 )
#define IT_USART_PAIR_A_DMA_TX              ( USART_DMA_CHANNEL_0 )
#define IT_USART_PAIR_A_DMA_RX              ( USART_DMA_CHANNEL_1 )
#define IT_USART_PAIR_B_DMA_TX              ( USART_DMA_CHANNEL_2 )
#define IT_USART_PAIR_B_DMA_RX              ( USART_DMA_CHANNEL_3 )
#define IT_USART_LOOP_DMA_TX                ( USART_DMA_CHANNEL_4 )
#define IT_USART_LOOP_DMA_RX                ( USART_DMA_CHANNEL_5 )

/** Interrupt priority of the data handling */
#define IT_USART_IRQ_PRIO                   ( 5u )

/** PLL1 of the system clock: HSI 64 MHz / 4 = 16 MHz reference, x 25 = 400 MHz VCO, P / 2 = 200 MHz (VOS1 limit
 *  of the AHB clock of every classic STM32H7 line, APB clocks / 2 = 100 MHz) */
#define IT_USART_PLL_M                      ( 4u )
#define IT_USART_PLL_N                      ( 25u )
#define IT_USART_PLL_P                      ( 2u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Transmitted data */
static usart_TxData_t           itUsart_TxBuf[ IT_USART_BUF_SIZE ];

/** Received data */
static usart_RxData_t           itUsart_RxBuf[ IT_USART_BUF_SIZE ];

/** Data handling configuration (copied by the module) */
static usart_DataConfig_t       itUsart_DataConfig;

/** Bus of the test case (USART6) */
static it_UsartBus_t            itUsart_Bus;

/** Receiver timeout of the bus configuration [bits], 0 - inactive */
static usart_RxTimeout_t        itUsart_RxTimeout;

/** DMA stream of the reception */
static usart_DmaPeriphId_t      itUsart_RxDmaPeriph;
static usart_DmaChannelId_t     itUsart_RxDmaChannel;

/** Counts of callback calls */
static volatile uint32_t        itUsart_TxCompleteCnt;
static volatile uint32_t        itUsart_RxHalfCnt;
static volatile uint32_t        itUsart_RxCompleteCnt;
static volatile uint32_t        itUsart_RxEndCnt;
static volatile uint32_t        itUsart_ErrorCnt;

/** Parameter of the last RxEnd callback */
static volatile usart_RxDataCnt_t   itUsart_RxEndBytes;

/** Parameter of the last error callback */
static volatile usart_XferErrorId_t itUsart_ErrorId;

/** Callback records, transmitted and received data of the buses of the pair */
static it_UsartPair_t               itUsart_Pair[ IT_USART_PAIR_CNT ];
static usart_TxData_t               itUsart_PairTx[ IT_USART_PAIR_CNT ][ IT_USART_BUF_SIZE ];
static usart_RxData_t               itUsart_PairRx[ IT_USART_PAIR_CNT ][ IT_USART_BUF_SIZE ];

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    ItUsart_Init_Clock();

    itUsart_TxCompleteCnt = 0u;
    itUsart_RxHalfCnt     = 0u;
    itUsart_RxCompleteCnt = 0u;
    itUsart_RxEndCnt      = 0u;
    itUsart_ErrorCnt      = 0u;
    itUsart_RxEndBytes    = 0u;
    itUsart_ErrorId       = USART_XFER_ERROR_CNT;
    itUsart_RxTimeout     = 0u;
    itUsart_RxDmaPeriph   = USART_DMA_PERIPH_2;
    itUsart_RxDmaChannel  = USART_DMA_CHANNEL_1;

    itUsart_Bus.Bus          = IT_USART_BUS;
    itUsart_Bus.TxPin        = IT_USART_TX_PIN;
    itUsart_Bus.TxPort       = IT_USART_TX_PORT;
    itUsart_Bus.TxPinId      = IT_USART_TX_PIN_ID;
    itUsart_Bus.Oversampling = USART_OVERSAMPLING_8;

    for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
    {
        itUsart_TxBuf[ byteIdx ] = (usart_TxData_t)( 0x30u + ( byteIdx * 7u ) );
        itUsart_RxBuf[ byteIdx ] = 0u;
    }
}


void tearDown( void )
{
    /* Every test case runs after system reset */
}

/* =============================== TESTS ==================================== */

/*----------------------------- Configuration --------------------------------*/

/**
 * \brief   Usart_Init() on target configures half-duplex bus.
 *
 * \details Initializes the bus in half-duplex mode (115200 Bd, 8N1, polling mode) and
 *          reads the configuration back.
 *
 * \par Expected results
 * - 8 data bits, 1 stop bit, no parity, half-duplex active, peripheral enabled.
 * - Baud rate reads back 115200 Bd +- 1 %.
 */
void It_Usart_Init_HalfDuplex_ConfigurationReadBack( void )
{
    usart_Baudrate_t     baudrate    = 0u;
    usart_DataWidth_t    dataWidth   = USART_DATA_WIDTH_7;
    usart_StopBits_t     stopBits    = USART_STOP_BITS_2;
    usart_Parity_t       parity      = USART_PARITY_ODD;
    usart_HalfDuplex_t   halfDuplex  = USART_HALF_DUPLEX_INACTIVE;
    usart_FlagState_t    periphState = USART_FLAG_INACTIVE;

    ItUsart_Init( USART_XFER_MODE_POLL,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataWidth( IT_USART_BUS, &dataWidth ) );
    TEST_ASSERT_EQUAL( USART_DATA_WIDTH_8, dataWidth );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_StopBits( IT_USART_BUS, &stopBits ) );
    TEST_ASSERT_EQUAL( USART_STOP_BITS_1, stopBits );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( IT_USART_BUS, &parity ) );
    TEST_ASSERT_EQUAL( USART_PARITY_NONE, parity );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_HalfDuplexState( IT_USART_BUS, &halfDuplex ) );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_ACTIVE, halfDuplex );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( IT_USART_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, periphState );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( IT_USART_BUS, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( IT_USART_BAUDRATE / 100u, IT_USART_BAUDRATE, baudrate );
}


/**
 * \brief   Usart_Deinit() on target disables the peripheral and data handling.
 *
 * \details Initializes the bus in interrupt mode, deinitializes it, reads the
 *          peripheral state and starts transmission.
 *
 * \par Expected results
 * - USART_REQUEST_OK, peripheral state inactive.
 * - Transmission start: USART_REQUEST_ERROR.
 */
void It_Usart_Deinit_InitializedBus_PeripheralInactive( void )
{
    usart_FlagState_t periphState = USART_FLAG_ACTIVE;

    ItUsart_Init( USART_XFER_MODE_ISR,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( IT_USART_BUS ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( IT_USART_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( USART_FLAG_INACTIVE, periphState );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 1u ) );
}

/*----------------------------- Data transfer --------------------------------*/

/**
 * \brief   Data transfer in polling mode.
 *
 * \details Polling mode, one-shot RX buffer of 16 bytes. Starts reception and
 *          transmission of 16 bytes (single wire loopback), Usart_Task() is called
 *          during waiting.
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_TxStart_PollMode_LoopbackDataReceived( void )
{
    ItUsart_Init( USART_XFER_MODE_POLL,
                  16u,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 16u );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 16u );
}


/**
 * \brief   Data transfer in interrupt mode.
 *
 * \details Interrupt mode, one-shot RX buffer of 32 bytes. Starts reception and
 *          transmission of 32 bytes (single wire loopback).
 *
 * \par Expected results
 * - TX complete callback 1x, RX half callback 1x, RX complete callback 1x, no error
 *   callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_TxStart_IsrMode_LoopbackDataReceived( void )
{
    ItUsart_Init( USART_XFER_MODE_ISR,
                  32u,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 32u );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 32u );
}


/**
 * \brief   Data transfer in DMA mode.
 *
 * \details DMA mode (DMA2 stream 6 TX, stream 1 RX - requests of USART6 by DMAMUX1),
 *          one-shot RX buffer of 64 bytes. Starts reception and transmission of 64 bytes
 *          (single wire loopback).
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_TxStart_DmaMode_LoopbackDataReceived( void )
{
    ItUsart_Init( USART_XFER_MODE_DMA,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( IT_USART_BUF_SIZE );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, IT_USART_BUF_SIZE );
}


/**
 * \brief   Second DMA transmission after the first one completed.
 *
 * \details DMA mode, one-shot RX buffer of 16 bytes. Transfers 8 bytes, waits for TX
 *          complete, restarts reception and transfers the next 8 bytes.
 *
 * \par Expected results
 * - TX complete callback 2x, no error callback.
 * - Both blocks received (second block at RX buffer start after reception restart).
 */
void It_Usart_Set_TxStart_DmaModeTwice_BothBlocksReceived( void )
{
    ItUsart_Init( USART_XFER_MODE_DMA,
                  16u,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 8u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 8u );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( IT_USART_BUS ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, &itUsart_TxBuf[ 8u ], 8u ) );
    ItUsart_Wait( &itUsart_TxCompleteCnt, 2u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 8u ], itUsart_RxBuf, 8u );
}


/**
 * \brief   Transmission state is active until transmission complete.
 *
 * \details Interrupt mode. Starts reception and transmission of 8 bytes, reads TX
 *          state, starts second transmission, waits for TX complete and reads TX state.
 *
 * \par Expected results
 * - TX state active during transmission, second start: USART_REQUEST_ERROR.
 * - TX state inactive after TX complete callback.
 */
void It_Usart_Get_TxState_RunningTransmission_ActiveUntilComplete( void )
{
    usart_FunctionState_t txState = USART_FUNCTION_INACTIVE;

    ItUsart_Init( USART_XFER_MODE_ISR,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 8u ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( IT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, txState );

    /* Second transmission while the first one is running */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 8u ) );

    ItUsart_Wait( &itUsart_TxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( IT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
}

/*------------------------- Reception modes ----------------------------------*/

/**
 * \brief   Idle line on target ends the received message.
 *
 * \details Interrupt mode, RX end mode idle, RX buffer of 64 bytes. Transfers 5 bytes
 *          and waits for RX end callback.
 *
 * \par Expected results
 * - RX end callback 1x with 5 bytes, no RX complete callback.
 * - RX count 5, received data equal transmitted data.
 */
void It_Usart_Set_RxStart_IdleEndMode_RxEndCallbackWithByteCount( void )
{
    usart_RxDataCnt_t rxCnt = 0u;

    ItUsart_Init( USART_XFER_MODE_ISR,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_IDLE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 5u );
    ItUsart_Wait( &itUsart_RxEndCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 5u, itUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_RxCompleteCnt );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( IT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL_UINT16( 5u, rxCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 5u );
}


/**
 * \brief   Receiver timeout on target ends the received message.
 *
 * \details Interrupt mode, receiver timeout 22 bits, RX end mode timeout, RX buffer of 64
 *          bytes. Transfers 7 bytes and waits for RX end callback.
 *
 * \par Expected results
 * - RX end callback 1x with 7 bytes, no RX complete / error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_RxStart_TimeoutEndMode_RxEndCallbackWithByteCount( void )
{
    itUsart_RxTimeout = IT_USART_RTO_BITS;

    ItUsart_Init( USART_XFER_MODE_ISR,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_TIMEOUT,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 7u );
    ItUsart_Wait( &itUsart_RxEndCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 7u, itUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 7u );
}


/**
 * \brief   Idle line ends the message received by DMA - count from the DMA channel.
 *
 * \details DMA mode, RX end mode idle, RX buffer of 64 bytes. Transfers 9 bytes and waits
 *          for RX end callback.
 *
 * \par Expected results
 * - RX end callback 1x with 9 bytes (buffer size - remaining count of the channel), no RX
 *   complete callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_RxStart_DmaIdleEndMode_RxEndCallbackWithByteCount( void )
{
    ItUsart_Init( USART_XFER_MODE_DMA,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_IDLE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 9u );
    ItUsart_Wait( &itUsart_RxEndCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 9u, itUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 9u );
}


/**
 * \brief   Circular reception on target wraps around the buffer.
 *
 * \details Interrupt mode, circular RX buffer of 8 bytes. Transfers 20 bytes.
 *
 * \par Expected results
 * - RX half callback 3x (byte 4, 12, 20), RX complete callback 2x (byte 8, 16).
 * - Buffer contains bytes 16 - 19 at index 0 - 3 and bytes 12 - 15 at index 4 - 7.
 */
void It_Usart_Set_RxStart_CircularBuffer_WrapsAroundWithCallbacks( void )
{
    ItUsart_Init( USART_XFER_MODE_ISR,
                  8u,
                  USART_BUFFER_MODE_CIRCULAR,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    /* 20 bytes into 8 byte buffer: half at 4 / 12 / 20, complete at 8 / 16 */
    ItUsart_Transfer( 20u );
    ItUsart_Wait( &itUsart_RxHalfCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 16u ], &itUsart_RxBuf[ 0u ], 4u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 12u ], &itUsart_RxBuf[ 4u ], 4u );
}


/**
 * \brief   Circular DMA reception on the other receive stream wraps around the buffer.
 *
 * \details DMA mode, TX DMA2 stream 6, RX DMA2 stream 2 (the USART6_RX request is
 *          routed to any stream), circular RX buffer of 8 bytes. Transfers 20 bytes.
 *
 * \par Expected results
 * - RX half callback 3x (byte 4, 12, 20), RX complete callback 2x (byte 8, 16).
 * - Buffer contains bytes 16 - 19 at index 0 - 3 and bytes 12 - 15 at index 4 - 7.
 */
void It_Usart_Set_RxStart_DmaCircularBufferOtherStream_WrapsAroundWithCallbacks( void )
{
    itUsart_RxDmaPeriph  = USART_DMA_PERIPH_2;
    itUsart_RxDmaChannel = USART_DMA_CHANNEL_2;

    ItUsart_Init( USART_XFER_MODE_DMA,
                  8u,
                  USART_BUFFER_MODE_CIRCULAR,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 20u );
    ItUsart_Wait( &itUsart_RxHalfCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 16u ], &itUsart_RxBuf[ 0u ], 4u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 12u ], &itUsart_RxBuf[ 4u ], 4u );
}


/**
 * \brief   Data transfer with even parity.
 *
 * \details Interrupt mode, 9-bit frame (8 data bits + even parity bit). Reads parity
 *          and transfers 16 bytes.
 *
 * \par Expected results
 * - Parity reads back even.
 * - No error callback (no parity error), received data equal transmitted data.
 */
void It_Usart_Init_EvenParity9Bit_LoopbackDataReceived( void )
{
    usart_Parity_t parity = USART_PARITY_NONE;

    /* 8 data bits + parity bit */
    ItUsart_Init( USART_XFER_MODE_ISR,
                  16u,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_EVEN,
                  USART_DATA_WIDTH_9 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( IT_USART_BUS, &parity ) );
    TEST_ASSERT_EQUAL( USART_PARITY_EVEN, parity );

    ItUsart_Transfer( 16u );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 16u );
}


/**
 * \brief   Frame format of enabled peripheral on target is applied.
 *
 * \details Initializes the bus (8N1, interrupt mode), then sets 9 data bits, 2 stop
 *          bits and odd parity while the peripheral is enabled (fields writable only
 *          with UE = 0). Reads the configuration back and transfers 16 bytes.
 *
 * \par Expected results
 * - 9 data bits, 2 stop bits, odd parity read back, peripheral stays enabled.
 * - No error callback, received data equal transmitted data.
 */
void It_Usart_Set_FrameFormat_EnabledPeripheral_AppliedAndStaysEnabled( void )
{
    usart_DataWidth_t dataWidth   = USART_DATA_WIDTH_8;
    usart_StopBits_t  stopBits    = USART_STOP_BITS_1;
    usart_Parity_t    parity      = USART_PARITY_NONE;
    usart_FlagState_t periphState = USART_FLAG_INACTIVE;

    ItUsart_Init( USART_XFER_MODE_ISR,
                  16u,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    /* M, STOP and PCE / PS fields are writable only with UE = 0 - peripheral is enabled */
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataWidth( IT_USART_BUS, USART_DATA_WIDTH_9 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( IT_USART_BUS, USART_STOP_BITS_2 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Parity( IT_USART_BUS, USART_PARITY_ODD ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataWidth( IT_USART_BUS, &dataWidth ) );
    TEST_ASSERT_EQUAL( USART_DATA_WIDTH_9, dataWidth );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_StopBits( IT_USART_BUS, &stopBits ) );
    TEST_ASSERT_EQUAL( USART_STOP_BITS_2, stopBits );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( IT_USART_BUS, &parity ) );
    TEST_ASSERT_EQUAL( USART_PARITY_ODD, parity );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( IT_USART_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, periphState );

    /* Communication works with the new frame format */
    ItUsart_Transfer( 16u );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 16u );
}

/**
 * \brief   Deactivation of RXNE interrupt keeps a pending received byte.
 *
 * \details Polling mode, reception started. One byte is written to the data register
 *          (Usart_SendData, single wire loopback), after 3 frame times (byte received,
 *          Usart_Task not called) the RXNE interrupt is deactivated, then Usart_Task() runs.
 * \note    Bug AB#465 (STM32H5 implementation AB#973): Usart_Set_RxNotEmptyIrqInactive() read
 *          the receive data register - RXNE was cleared and the pending byte was lost. The
 *          test is an integration test because reads of emulated registers have no side
 *          effect in unit tests.
 *
 * \par Expected results
 * - Usart_Set_RxNotEmptyIrqInactive(): USART_REQUEST_OK.
 * - RX count 1, received byte equals the sent byte.
 */
void It_Usart_Set_RxNotEmptyIrqInactive_PendingByte_NotLost( void )
{
    usart_RxDataCnt_t rxCnt = 0u;

    ItUsart_Init( USART_XFER_MODE_POLL,
                  4u,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );
    ItUsart_Init_RefTime();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );

    Usart_SendData( IT_USART_BUS, itUsart_TxBuf[ 0u ] );
    ItUsart_Delay_Frames( 3u );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxNotEmptyIrqInactive( IT_USART_BUS ) );

    for( uint32_t loopIdx = 0u; ( IT_USART_WAIT_LOOPS / 100u ) > loopIdx; loopIdx++ )
    {
        Usart_Task();
    }

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( IT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL_UINT16( 1u, rxCnt );
    TEST_ASSERT_EQUAL_HEX8( itUsart_TxBuf[ 0u ], itUsart_RxBuf[ 0u ] );
}

/*------------------------------ Frame timing --------------------------------*/

/**
 * \brief   Real baud rate of USART matches configured baud rate.
 *
 * \details Interrupt mode, 115200 Bd 8N1. Starts reception and transmission of 50
 *          bytes and measures time until TX complete callback by DWT cycle counter.
 *
 * \par Expected results
 * - TX complete callback 1x.
 * - Transmission time = 50 x 10 bits / 115200 Bd (4340 us) +- 5 %.
 */
void It_Usart_Set_Baudrate_115200_FrameTimeMatchesBaudrate( void )
{
    ItUsart_Init( USART_XFER_MODE_ISR,
                  IT_USART_BUF_SIZE,
                  USART_BUFFER_MODE_ONE_SHOT,
                  USART_RX_END_NONE,
                  USART_PARITY_NONE,
                  USART_DATA_WIDTH_8 );

    ItUsart_Check_FrameTime();
}

/*------------------------------ UART pair (UART_PAIR) -----------------------*/

/**
 * \brief   Bus A transmits to bus B and bus B to bus A in polling mode.
 *
 * \details Both buses of the pair in polling mode (115200 Bd, 8N1, one-shot RX buffers of
 *          16 bytes). A -> B 16 bytes, then B -> A 16 bytes, Usart_Task() called while waiting.
 *
 * \par Expected results
 * - TX complete of the sender and RX complete of the receiver once per direction, no error.
 * - Received data equal the transmitted data in both directions.
 */
void It_Usart_Pair_PollBothDirections_DataReceived( void )
{
    ItUsart_RequirePair();

    ItUsart_Init_Pair( USART_XFER_MODE_POLL, IT_USART_PAIR_SHORT, USART_RX_END_NONE, IT_USART_BAUDRATE );

    ItUsart_Transfer_Pair( IT_USART_PAIR_A, IT_USART_PAIR_B, IT_USART_PAIR_SHORT );
    ItUsart_Transfer_Pair( IT_USART_PAIR_B, IT_USART_PAIR_A, IT_USART_PAIR_SHORT );
}


/**
 * \brief   Both buses transmit at the same time in interrupt mode.
 *
 * \details Both buses in interrupt mode, one-shot RX buffers of 48 bytes; reception of both
 *          started, then transmission A -> B and B -> A (full duplex on both buses).
 *
 * \par Expected results
 * - TX complete and RX complete on both buses once, no error.
 * - Data of A received by B and data of B received by A.
 */
void It_Usart_Pair_IsrFullDuplex_DataExchanged( void )
{
    ItUsart_RequirePair();

    ItUsart_Init_Pair( USART_XFER_MODE_ISR, IT_USART_PAIR_LONG, USART_RX_END_NONE, IT_USART_BAUDRATE );

    ItUsart_Exchange_Pair( IT_USART_PAIR_LONG );
}


/**
 * \brief   Both buses transmit at the same time in DMA mode at 921600 Bd.
 *
 * \details Both buses in DMA mode (bus A DMA1 streams 0 / 1, bus B streams 2 / 3),
 *          921600 Bd, one-shot RX buffers of 64 bytes; full duplex exchange of 64 bytes.
 *
 * \par Expected results
 * - TX complete and RX complete on both buses once, no error.
 * - Data of A received by B and data of B received by A.
 */
void It_Usart_Pair_DmaFullDuplexHighBaudrate_DataExchanged( void )
{
    ItUsart_RequirePair();

    ItUsart_Init_Pair( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_RX_END_NONE, IT_USART_FAST_BAUDRATE );

    ItUsart_Exchange_Pair( IT_USART_BUF_SIZE );
}


/**
 * \brief   End of message on the receiving bus of the pair (idle line).
 *
 * \details Bus B receives in interrupt mode with end of message detection (idle line), RX
 *          buffer of 64 bytes; bus A transmits a message of 10 bytes.
 *
 * \par Expected results
 * - RX end callback of bus B once with 10 bytes, no RX complete, no error.
 * - Received data equal the transmitted message.
 */
void It_Usart_Pair_IdleEndMode_RxEndWithByteCount( void )
{
    ItUsart_RequirePair();

    ItUsart_Init_Pair( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_RX_END_IDLE, IT_USART_BAUDRATE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_PAIR_B_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_PAIR_A_BUS,
                                                            itUsart_PairTx[ IT_USART_PAIR_A ],
                                                            IT_USART_PAIR_MSG ) );
    ItUsart_Wait( &itUsart_Pair[ IT_USART_PAIR_B ].RxEndCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_Pair[ IT_USART_PAIR_A ].TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_Pair[ IT_USART_PAIR_B ].RxEndCnt );
    TEST_ASSERT_EQUAL_UINT32( IT_USART_PAIR_MSG, itUsart_Pair[ IT_USART_PAIR_B ].RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Pair[ IT_USART_PAIR_B ].RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Pair[ IT_USART_PAIR_A ].ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Pair[ IT_USART_PAIR_B ].ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_PairTx[ IT_USART_PAIR_A ],
                                  itUsart_PairRx[ IT_USART_PAIR_B ],
                                  IT_USART_PAIR_MSG );
}

/*------------------------------ UART loop (UART_LOOP) -----------------------*/

/**
 * \brief   Data transfer from TX to RX of one U(S)ART in polling mode.
 *
 * \details Loop bus in polling mode (115200 Bd, 8N1, full duplex, one-shot RX buffer of 16 bytes).
 *          Starts reception and transmission of 16 bytes, Usart_Task() is called while waiting.
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Loop_PollMode_DataReceived( void )
{
    ItUsart_RequireLoop();

    ItUsart_Init_Loop( USART_XFER_MODE_POLL, IT_USART_LOOP_SHORT );
    ItUsart_Transfer_Loop( IT_USART_LOOP_SHORT );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, IT_USART_LOOP_SHORT );
}


/**
 * \brief   Data transfer from TX to RX of one U(S)ART in interrupt mode.
 *
 * \details Loop bus in interrupt mode (115200 Bd, 8N1, full duplex, one-shot RX buffer of 48 bytes).
 *          Starts reception and transmission of 48 bytes.
 *
 * \par Expected results
 * - TX complete callback 1x, RX half callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Loop_IsrMode_DataReceived( void )
{
    ItUsart_RequireLoop();

    ItUsart_Init_Loop( USART_XFER_MODE_ISR, IT_USART_LOOP_LONG );
    ItUsart_Transfer_Loop( IT_USART_LOOP_LONG );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, IT_USART_LOOP_LONG );
}


/**
 * \brief   Data transfer from TX to RX of one U(S)ART in DMA mode.
 *
 * \details Loop bus in DMA mode (DMA1 stream 4 TX, stream 5 RX, 115200 Bd, 8N1, full duplex, one-shot
 *          RX buffer of 64 bytes). Starts reception and transmission of 64 bytes.
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Loop_DmaMode_DataReceived( void )
{
    ItUsart_RequireLoop();

    ItUsart_Init_Loop( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE );
    ItUsart_Transfer_Loop( IT_USART_BUF_SIZE );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, IT_USART_BUF_SIZE );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes the bus of the test case (\ref itUsart_Bus) in half-duplex mode with
 *        data handling of both directions.
 *
 * \param xferMode   [in]: Transmission and reception mode
 * \param rxSize     [in]: Reception buffer size
 * \param bufferMode [in]: Reception buffer mode
 * \param rxEndMode  [in]: End of message detection
 * \param parity     [in]: Parity
 * \param dataWidth  [in]: Data width (incl. parity bit)
 */
static void ItUsart_Init( usart_XferMode_t xferMode,
                          usart_RxDataCnt_t rxSize,
                          usart_BufferMode_t bufferMode,
                          usart_RxEndMode_t rxEndMode,
                          usart_Parity_t parity,
                          usart_DataWidth_t dataWidth )
{
    usart_BusConfig_t busConfig;

    ItUsart_Set_DataConfig( xferMode, rxSize, bufferMode, rxEndMode );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.PeriphId       = itUsart_Bus.Bus;
    busConfig.BaudRate       = IT_USART_BAUDRATE;
    busConfig.DataWidth      = dataWidth;
    busConfig.Parity         = parity;
    busConfig.Oversampling   = itUsart_Bus.Oversampling;
    busConfig.HalfDuplex     = USART_HALF_DUPLEX_ACTIVE;
    busConfig.RxTimeoutValue = itUsart_RxTimeout;
    busConfig.BusTxPin       = itUsart_Bus.TxPin;
    busConfig.DataConfig     = &itUsart_DataConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );

    /* Single wire line is released by USART when not transmitting - pull-up of the line
     * (external on the board in the application, RM0385 / RM0410 / RM0431 single-wire half-duplex mode) */
    TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Set_PinPull( itUsart_Bus.TxPort, itUsart_Bus.TxPinId, GPIO_PIN_PULL_UP ) );

    /* Frame possibly started on the floating line before pull-up is finished before
     * the reception start (Usart_Set_RxStart discards stale data) */
    for( volatile uint32_t loopIdx = 0u; IT_USART_LINE_SETTLE_LOOPS > loopIdx; loopIdx++ )
    {
        /* Busy wait */
    }
}


/**
 * \brief Starts reception and transmission of the first bytes of the TX buffer
 *        and waits for the end of the transmission.
 *
 * \param txSize [in]: Count of transmitted bytes
 */
static void ItUsart_Transfer( usart_TxDataCnt_t txSize )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( itUsart_Bus.Bus ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( itUsart_Bus.Bus, itUsart_TxBuf, txSize ) );

    ItUsart_Wait( &itUsart_TxCompleteCnt, 1u );
}


/**
 * \brief Waits until counter reaches required value or timeout (Usart_Task is
 *        called - data handling in polling mode).
 *
 * \param counter     [in]: Callback call counter
 * \param expectedCnt [in]: Required count
 */
static void ItUsart_Wait( volatile const uint32_t * const counter, uint32_t expectedCnt )
{
    for( uint32_t loopIdx = 0u; IT_USART_WAIT_LOOPS > loopIdx; loopIdx++ )
    {
        if( expectedCnt <= *counter )
        {
            break;
        }
        else
        {
            Usart_Task();
        }
    }

    /* Few more frames for pending events (idle line, last byte) */
    for( uint32_t loopIdx = 0u; ( IT_USART_WAIT_LOOPS / 100u ) > loopIdx; loopIdx++ )
    {
        Usart_Task();
    }
}


/**
 * \brief Initializes Rcc with the clock configuration of the test set: system clock PLL1 P 200 MHz from HSI,
 *        VOS1, AHB 200 MHz, APB 100 MHz (the reset clock HSI 64 MHz leaves no margin for the interrupt data
 *        handling at 115200 Bd, AB#2208) - on a board with its supply configuration (RCC_BOARD_PWR_SUPPLY);
 *        without it the voltage scale cannot be raised and the reset clock is kept.
 */
static void ItUsart_Init_Clock( void )
{
#if defined(RCC_BOARD_PWR_SUPPLY)
    rcc_ConfigStruct_t clockConfig;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_DefaultConfig( &clockConfig ) );

    clockConfig.SystemClockSource                    = RCC_SYSTEM_CLOCK_SOURCE_PLL;
    clockConfig.VoltageScaling                       = RCC_PWR_VOLTAGE_SCALE_1;
    clockConfig.APB1_Divider                         = RCC_APB1_DIVIDER_2;
    clockConfig.APB2_Divider                         = RCC_APB2_DIVIDER_2;
    clockConfig.APB3_Divider                         = RCC_APB3_DIVIDER_2;
    clockConfig.APB4_Divider                         = RCC_APB4_DIVIDER_2;
    clockConfig.Pll_Config[ RCC_PLL_1 ].Pll_Source   = RCC_PLL_SRC_HSI;
    clockConfig.Pll_Config[ RCC_PLL_1 ].M_Divider    = IT_USART_PLL_M;
    clockConfig.Pll_Config[ RCC_PLL_1 ].N_Multiplier = IT_USART_PLL_N;
    clockConfig.Pll_Config[ RCC_PLL_1 ].P_Divider    = IT_USART_PLL_P;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Init( &clockConfig ) );
#else
    /* Supply configuration of the board not known: the voltage scale of the reset cannot be raised, the test set
       runs at the default clock configuration of StartUp (HSI 64 MHz) */
    return;
#endif /* RCC_BOARD_PWR_SUPPLY */
}


/**
 * \brief Initializes and starts reference time base (DWT cycle counter).
 */
static void ItUsart_Init_RefTime( void )
{
    dwt_Config_t dwtConfig;

    TEST_ASSERT_EQUAL( DWT_REQUEST_OK, Dwt_Get_DefaultConfig( &dwtConfig ) );
    dwtConfig.CycleCounterState = DWT_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( DWT_REQUEST_OK, Dwt_Init( &dwtConfig ) );
}


/**
 * \brief Busy waits for given count of 8N1 frame times (DWT cycle counter, \ref
 *        ItUsart_Init_RefTime() is required before).
 *
 * \param frameCnt [in]: Count of frame times
 */
static void ItUsart_Delay_Frames( uint32_t frameCnt )
{
    const uint32_t           waitCycles = (uint32_t)( ( (uint64_t)frameCnt * IT_USART_FRAME_BITS * SystemCoreClock ) / IT_USART_BAUDRATE );
    const dwt_CounterValue_t startCnt   = Dwt_Get_CycleCounter();
    dwt_CounterValue_t       elapsed    = 0u;

    for( uint32_t passIdx = 0u;
         ( ( waitCycles / IT_USART_MIN_CYCLES_PER_PASS ) > passIdx ) &&
         ( waitCycles > elapsed );
         passIdx++ )
    {
        elapsed = Dwt_Get_CycleCounter() - startCnt;
    }
}


/**
 * \brief Measures transmission time of \ref IT_USART_TIMING_BYTES bytes (8N1, back to back)
 *        by DWT cycle counter and compares it with the configured baud rate (+- 5 %).
 */
static void ItUsart_Check_FrameTime( void )
{
    /* Expected duration of the transmission [core clock cycles] */
    const uint32_t expectedCycles = (uint32_t)( ( (uint64_t)IT_USART_TIMING_BYTES * IT_USART_FRAME_BITS * SystemCoreClock ) / IT_USART_BAUDRATE );

    ItUsart_Init_RefTime();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( itUsart_Bus.Bus ) );

    const dwt_CounterValue_t startCnt = Dwt_Get_CycleCounter();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( itUsart_Bus.Bus, itUsart_TxBuf, IT_USART_TIMING_BYTES ) );

    for( uint32_t loopIdx = 0u; IT_USART_WAIT_LOOPS > loopIdx; loopIdx++ )
    {
        if( 0u != itUsart_TxCompleteCnt )
        {
            break;
        }
        else
        {
            /* Wait for transmission complete callback (interrupt mode) */
        }
    }

    const dwt_CounterValue_t endCnt = Dwt_Get_CycleCounter();

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_UINT32_WITHIN( expectedCycles / 20u, expectedCycles, endCnt - startCnt );
}


/** \brief Transmission complete callback */
static void ItUsart_TxCompleteCallback( void )
{
    itUsart_TxCompleteCnt++;
}


/** \brief Reception buffer half filled callback */
static void ItUsart_RxHalfCallback( void )
{
    itUsart_RxHalfCnt++;
}


/** \brief Reception buffer filled callback */
static void ItUsart_RxCompleteCallback( void )
{
    itUsart_RxCompleteCnt++;
}


/**
 * \brief End of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void ItUsart_RxEndCallback( usart_RxDataCnt_t rxCnt )
{
    itUsart_RxEndBytes = rxCnt;
    itUsart_RxEndCnt++;
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItUsart_ErrorCallback( usart_XferErrorId_t errorId )
{
    itUsart_ErrorId = errorId;
    itUsart_ErrorCnt++;
}


/**
 * \brief   Ignores the test on a board whose UART pair is not wired.
 *
 * \details Fixture of the board (EmBi_Platform, IntegrationTesting README "Board fixtures"):
 *          feature UART_PAIR. A board without a fixture (IT_FIXTURE not defined) is expected to
 *          have the wiring of its board block.
 */
static void ItUsart_RequirePair( void )
{
#if ( 0u == IT_USART_PAIR_AVAILABLE )
    TEST_IGNORE_MESSAGE( "No wiring of a UART pair known for the board" );
#elif defined(IT_FEATURE_UART_PAIR) || \
      !defined(IT_FIXTURE)
    /* TX / RX of the buses A and B are crossed */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture UART_PAIR (two UARTs crossed)" );
#endif
}


/**
 * \brief   Ignores the test on a board whose UART loop (TX connected to RX) is not wired.
 *
 * \details Fixture of the board (EmBi_Platform, IntegrationTesting README "Board fixtures"):
 *          feature UART_LOOP. A board without a fixture (IT_FIXTURE not defined) is expected to
 *          have the wiring of its board block.
 */
static void ItUsart_RequireLoop( void )
{
#if ( 0u == IT_USART_LOOP_AVAILABLE )
    TEST_IGNORE_MESSAGE( "No wiring of a UART loop known for the board" );
#elif defined(IT_FEATURE_UART_LOOP) || \
      !defined(IT_FIXTURE)
    /* TX and RX of the loop bus are connected */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture UART_LOOP (TX connected to RX)" );
#endif
}


/**
 * \brief Fills the data handling configuration of the single bus (callbacks of the single bus tests,
 *        DMA2 stream 6 TX, receive stream of the test case).
 *
 * \param xferMode   [in]: Transmission and reception mode
 * \param rxSize     [in]: Reception buffer size
 * \param bufferMode [in]: Reception buffer mode
 * \param rxEndMode  [in]: End of message detection
 */
static void ItUsart_Set_DataConfig( usart_XferMode_t xferMode,
                                    usart_RxDataCnt_t rxSize,
                                    usart_BufferMode_t bufferMode,
                                    usart_RxEndMode_t rxEndMode )
{
    itUsart_DataConfig.TxMode             = xferMode;
    itUsart_DataConfig.RxMode             = xferMode;
    itUsart_DataConfig.RxBuffer           = itUsart_RxBuf;
    itUsart_DataConfig.RxBufferSize       = rxSize;
    itUsart_DataConfig.RxBufferMode       = bufferMode;
    itUsart_DataConfig.RxEndMode          = rxEndMode;
    itUsart_DataConfig.TxDmaPeriphId      = USART_DMA_PERIPH_2;
    itUsart_DataConfig.TxDmaChannelId     = USART_DMA_CHANNEL_6;
    itUsart_DataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    itUsart_DataConfig.RxDmaPeriphId      = itUsart_RxDmaPeriph;
    itUsart_DataConfig.RxDmaChannelId     = itUsart_RxDmaChannel;
    itUsart_DataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    itUsart_DataConfig.IrqPriority        = IT_USART_IRQ_PRIO;
    itUsart_DataConfig.TxCompleteCallback = ItUsart_TxCompleteCallback;
    itUsart_DataConfig.RxHalfCallback     = ItUsart_RxHalfCallback;
    itUsart_DataConfig.RxCompleteCallback = ItUsart_RxCompleteCallback;
    itUsart_DataConfig.RxEndCallback      = ItUsart_RxEndCallback;
    itUsart_DataConfig.ErrorCallback      = ItUsart_ErrorCallback;
}


/**
 * \brief Initializes the loop bus in full duplex mode (8N1) with data handling of both directions.
 *
 * \param xferMode [in]: Transmission and reception mode
 * \param rxSize   [in]: Reception buffer size (one-shot)
 */
static void ItUsart_Init_Loop( usart_XferMode_t xferMode,
                               usart_RxDataCnt_t rxSize )
{
    usart_BusConfig_t busConfig;

    ItUsart_Set_DataConfig( xferMode, rxSize, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );
    itUsart_DataConfig.TxDmaPeriphId  = IT_USART_PAIR_DMA_PERIPH;
    itUsart_DataConfig.TxDmaChannelId = IT_USART_LOOP_DMA_TX;
    itUsart_DataConfig.RxDmaPeriphId  = IT_USART_PAIR_DMA_PERIPH;
    itUsart_DataConfig.RxDmaChannelId = IT_USART_LOOP_DMA_RX;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.PeriphId   = IT_USART_LOOP_BUS;
    busConfig.BaudRate   = IT_USART_BAUDRATE;
    busConfig.DataWidth  = USART_DATA_WIDTH_8;
    busConfig.Parity     = USART_PARITY_NONE;
    busConfig.HalfDuplex = USART_HALF_DUPLEX_INACTIVE;
    busConfig.BusTxPin   = IT_USART_LOOP_TX_PIN;
    busConfig.BusRxPin   = IT_USART_LOOP_RX_PIN;
    busConfig.DataConfig = &itUsart_DataConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );
}


/**
 * \brief Starts reception and transmission of the first bytes of the TX buffer on the loop bus
 *        and waits for the end of the transmission.
 *
 * \param txSize [in]: Count of transmitted bytes
 */
static void ItUsart_Transfer_Loop( usart_TxDataCnt_t txSize )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_LOOP_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_LOOP_BUS, itUsart_TxBuf, txSize ) );

    ItUsart_Wait( &itUsart_TxCompleteCnt, 1u );
}


/**
 * \brief Initializes both buses of the pair (full duplex, 8N1) with data handling and fills
 *        the transmit buffers with different patterns.
 *
 * \param xferMode  [in]: Transmission and reception mode of both buses
 * \param rxSize    [in]: Reception buffer size (one-shot)
 * \param rxEndMode [in]: End of message detection
 * \param baudrate  [in]: Baud rate [Bd]
 */
static void ItUsart_Init_Pair( usart_XferMode_t xferMode,
                               usart_RxDataCnt_t rxSize,
                               usart_RxEndMode_t rxEndMode,
                               usart_Baudrate_t baudrate )
{
    for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
    {
        itUsart_PairTx[ IT_USART_PAIR_A ][ byteIdx ] = (usart_TxData_t)( IT_USART_PAIR_SEED_A + byteIdx );
        itUsart_PairTx[ IT_USART_PAIR_B ][ byteIdx ] = (usart_TxData_t)( IT_USART_PAIR_SEED_B + byteIdx );
        itUsart_PairRx[ IT_USART_PAIR_A ][ byteIdx ] = 0u;
        itUsart_PairRx[ IT_USART_PAIR_B ][ byteIdx ] = 0u;
    }

    ItUsart_Init_PairBus( IT_USART_PAIR_A, xferMode, rxSize, rxEndMode, baudrate );
    ItUsart_Init_PairBus( IT_USART_PAIR_B, xferMode, rxSize, rxEndMode, baudrate );
}


/**
 * \brief Initializes one bus of the pair.
 *
 * \param pairBus   [in]: Bus of the pair
 * \param xferMode  [in]: Transmission and reception mode
 * \param rxSize    [in]: Reception buffer size (one-shot)
 * \param rxEndMode [in]: End of message detection
 * \param baudrate  [in]: Baud rate [Bd]
 */
static void ItUsart_Init_PairBus( it_UsartPairBus_t pairBus,
                                  usart_XferMode_t xferMode,
                                  usart_RxDataCnt_t rxSize,
                                  usart_RxEndMode_t rxEndMode,
                                  usart_Baudrate_t baudrate )
{
    it_UsartPair_t * const pair = &itUsart_Pair[ pairBus ];
    usart_BusConfig_t      busConfig;

    pair->TxCompleteCnt = 0u;
    pair->RxCompleteCnt = 0u;
    pair->RxEndCnt      = 0u;
    pair->RxEndBytes    = 0u;
    pair->ErrorCnt      = 0u;

    pair->DataConfig.TxMode         = xferMode;
    pair->DataConfig.RxMode         = xferMode;
    pair->DataConfig.RxBuffer       = itUsart_PairRx[ pairBus ];
    pair->DataConfig.RxBufferSize   = rxSize;
    pair->DataConfig.RxBufferMode   = USART_BUFFER_MODE_ONE_SHOT;
    pair->DataConfig.RxEndMode      = rxEndMode;
    pair->DataConfig.TxDmaPeriphId  = IT_USART_PAIR_DMA_PERIPH;
    pair->DataConfig.TxDmaPriority  = USART_DMA_PRIORITY_LOW;
    pair->DataConfig.RxDmaPeriphId  = IT_USART_PAIR_DMA_PERIPH;
    pair->DataConfig.RxDmaPriority  = USART_DMA_PRIORITY_HIGH;
    pair->DataConfig.IrqPriority    = IT_USART_IRQ_PRIO;
    pair->DataConfig.RxHalfCallback = USART_NULL_PTR;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.BaudRate   = baudrate;
    busConfig.DataWidth  = USART_DATA_WIDTH_8;
    busConfig.Parity     = USART_PARITY_NONE;
    busConfig.HalfDuplex = USART_HALF_DUPLEX_INACTIVE;

    if( IT_USART_PAIR_A == pairBus )
    {
        pair->DataConfig.TxDmaChannelId     = IT_USART_PAIR_A_DMA_TX;
        pair->DataConfig.RxDmaChannelId     = IT_USART_PAIR_A_DMA_RX;
        pair->DataConfig.TxCompleteCallback = ItUsart_PairA_TxComplete;
        pair->DataConfig.RxCompleteCallback = ItUsart_PairA_RxComplete;
        pair->DataConfig.RxEndCallback      = ItUsart_PairA_RxEnd;
        pair->DataConfig.ErrorCallback      = ItUsart_PairA_Error;
        busConfig.PeriphId                  = IT_USART_PAIR_A_BUS;
        busConfig.BusTxPin                  = IT_USART_PAIR_A_TX_PIN;
        busConfig.BusRxPin                  = IT_USART_PAIR_A_RX_PIN;
    }
    else
    {
        pair->DataConfig.TxDmaChannelId     = IT_USART_PAIR_B_DMA_TX;
        pair->DataConfig.RxDmaChannelId     = IT_USART_PAIR_B_DMA_RX;
        pair->DataConfig.TxCompleteCallback = ItUsart_PairB_TxComplete;
        pair->DataConfig.RxCompleteCallback = ItUsart_PairB_RxComplete;
        pair->DataConfig.RxEndCallback      = ItUsart_PairB_RxEnd;
        pair->DataConfig.ErrorCallback      = ItUsart_PairB_Error;
        busConfig.PeriphId                  = IT_USART_PAIR_B_BUS;
        busConfig.BusTxPin                  = IT_USART_PAIR_B_TX_PIN;
        busConfig.BusRxPin                  = IT_USART_PAIR_B_RX_PIN;
    }

    busConfig.DataConfig = &pair->DataConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );
}


/**
 * \brief Transfers the first bytes of the transmit buffer of one bus to the other bus and checks
 *        the callbacks and the data.
 *
 * \param txBus  [in]: Transmitting bus of the pair
 * \param rxBus  [in]: Receiving bus of the pair
 * \param txSize [in]: Count of bytes (equal to the RX buffer size)
 */
static void ItUsart_Transfer_Pair( it_UsartPairBus_t txBus,
                                   it_UsartPairBus_t rxBus,
                                   usart_TxDataCnt_t txSize )
{
    const usart_PeriphId_t txId = ItUsart_Get_PairBusId( txBus );
    const usart_PeriphId_t rxId = ItUsart_Get_PairBusId( rxBus );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( rxId ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( txId, itUsart_PairTx[ txBus ], txSize ) );
    ItUsart_Wait( &itUsart_Pair[ rxBus ].RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Pair[ txBus ].TxCompleteCnt, "TX not complete" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Pair[ rxBus ].RxCompleteCnt, "RX not complete" );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Pair[ txBus ].ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Pair[ rxBus ].ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_PairTx[ txBus ], itUsart_PairRx[ rxBus ], txSize );
}


/**
 * \brief Full duplex exchange: reception of both buses started, both transmit, checks the
 *        callbacks and the data of both directions.
 *
 * \param dataCnt [in]: Count of bytes per direction (equal to the RX buffer size)
 */
static void ItUsart_Exchange_Pair( usart_TxDataCnt_t dataCnt )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_PAIR_A_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_PAIR_B_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_PAIR_A_BUS,
                                                            itUsart_PairTx[ IT_USART_PAIR_A ],
                                                            dataCnt ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_PAIR_B_BUS,
                                                            itUsart_PairTx[ IT_USART_PAIR_B ],
                                                            dataCnt ) );
    ItUsart_Wait( &itUsart_Pair[ IT_USART_PAIR_A ].RxCompleteCnt, 1u );
    ItUsart_Wait( &itUsart_Pair[ IT_USART_PAIR_B ].RxCompleteCnt, 1u );

    for( uint32_t busIdx = 0u; (uint32_t)IT_USART_PAIR_CNT > busIdx; busIdx++ )
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Pair[ busIdx ].TxCompleteCnt, "TX not complete" );
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Pair[ busIdx ].RxCompleteCnt, "RX not complete" );
        TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Pair[ busIdx ].ErrorCnt );
    }

    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_PairTx[ IT_USART_PAIR_A ],
                                  itUsart_PairRx[ IT_USART_PAIR_B ],
                                  dataCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_PairTx[ IT_USART_PAIR_B ],
                                  itUsart_PairRx[ IT_USART_PAIR_A ],
                                  dataCnt );
}


/**
 * \brief Returns the USART peripheral of a bus of the pair.
 *
 * \param pairBus [in]: Bus of the pair
 */
static usart_PeriphId_t ItUsart_Get_PairBusId( it_UsartPairBus_t pairBus )
{
    usart_PeriphId_t periphId = IT_USART_PAIR_A_BUS;

    if( IT_USART_PAIR_B == pairBus )
    {
        periphId = IT_USART_PAIR_B_BUS;
    }
    else
    {
        /* Bus A */
    }

    return ( periphId );
}


/** \brief Bus A: transmission complete callback */
static void ItUsart_PairA_TxComplete( void )
{
    itUsart_Pair[ IT_USART_PAIR_A ].TxCompleteCnt++;
}


/** \brief Bus A: reception buffer filled callback */
static void ItUsart_PairA_RxComplete( void )
{
    itUsart_Pair[ IT_USART_PAIR_A ].RxCompleteCnt++;
}


/**
 * \brief Bus A: end of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void ItUsart_PairA_RxEnd( usart_RxDataCnt_t rxCnt )
{
    itUsart_Pair[ IT_USART_PAIR_A ].RxEndBytes = rxCnt;
    itUsart_Pair[ IT_USART_PAIR_A ].RxEndCnt++;
}


/**
 * \brief Bus A: transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItUsart_PairA_Error( usart_XferErrorId_t errorId )
{
    (void)errorId;
    itUsart_Pair[ IT_USART_PAIR_A ].ErrorCnt++;
}


/** \brief Bus B: transmission complete callback */
static void ItUsart_PairB_TxComplete( void )
{
    itUsart_Pair[ IT_USART_PAIR_B ].TxCompleteCnt++;
}


/** \brief Bus B: reception buffer filled callback */
static void ItUsart_PairB_RxComplete( void )
{
    itUsart_Pair[ IT_USART_PAIR_B ].RxCompleteCnt++;
}


/**
 * \brief Bus B: end of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void ItUsart_PairB_RxEnd( usart_RxDataCnt_t rxCnt )
{
    itUsart_Pair[ IT_USART_PAIR_B ].RxEndBytes = rxCnt;
    itUsart_Pair[ IT_USART_PAIR_B ].RxEndCnt++;
}


/**
 * \brief Bus B: transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItUsart_PairB_Error( usart_XferErrorId_t errorId )
{
    (void)errorId;
    itUsart_Pair[ IT_USART_PAIR_B ].ErrorCnt++;
}
