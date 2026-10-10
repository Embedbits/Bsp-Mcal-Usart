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
 * circular reception buffer, parity and frame timing (STM32F7 has no LPUART).
 *
 * No external wiring is used - USART works in single wire half-duplex mode, the
 * receiver receives every frame sent by the transmitter on the TX pin.
 *
 * Boards (named by the MCU as the detection of the connected boards does):
 * - STM32F745xG / STM32F746xG - 32F746GDISCOVERY (STM32F746NG); the detection
 *   names every board with ID 0x449 and 1 MB flash STM32F745xG
 * - STM32F722xE, STM32F756xG, STM32F765xI / STM32F767xI - NUCLEO-F722ZE,
 *   NUCLEO-F756ZG (board file override with the name STM32F756xG), NUCLEO-F767ZI
 *   (Nucleo-144 boards, common pinout)
 *
 * Used resources (see board configuration below):
 * - IT_USART_BUS (USART6), TX pin IT_USART_TX_PIN (PC6) - not connected on the board
 * - DMA2 stream 6 (TX), streams 1 / 2 (RX), channel 5 of USART6 (DMA mode)
 * - DWT cycle counter - reference time base for frame timing
 * - IT_USART_RTS_BUS (UART4) with RTS pin IT_USART_RTS_PIN (PA15, UART4_RTS) - not
 *   connected on the board, level read back by GPIO (device errata ES0334 "RTS is
 *   active while RE = 0 or UE = 0", bug AB#1296)
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usart_Port.h"                     /* Module under test              */
#include "Dwt_Port.h"                       /* Reference time base            */
#include "Gpio_Port.h"                      /* Pull-up of single wire line    */
#include "Stm32.h"                          /* SystemCoreClock                */
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

/* ======================= FORWARD DECLARATIONS ============================= */

static void It_Usart_Init            ( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize, usart_BufferMode_t bufferMode,
                                      usart_RxEndMode_t rxEndMode, usart_Parity_t parity, usart_DataWidth_t dataWidth );
static void It_Usart_Wait           ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void It_Usart_Transfer       ( usart_TxDataCnt_t txSize );
static void It_Usart_Init_RefTime   ( void );
static void It_Usart_Delay_Frames   ( uint32_t frameCnt );
static void It_Usart_Check_FrameTime( void );
static gpio_PinLevel_t It_Usart_Get_RtsLevel( void );

static void It_Usart_TxCompleteCallback ( void );
static void It_Usart_RxHalfCallback     ( void );
static void It_Usart_RxCompleteCallback ( void );
static void It_Usart_RxEndCallback      ( usart_RxDataCnt_t rxCnt );
static void It_Usart_ErrorCallback      ( usart_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32F745xG) || \
    defined(IT_BOARD_STM32F746xG)

    /* 32F746GDISCOVERY */

    /** USART6, Arduino D1 (PC6, USART6_TX) - not connected on the board */
    #define IT_USART_BUS                    ( USART_BUS_6 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS6_PC6 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_C )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_6 )

    /** USART6 DMA2 streams (TX stream 6, RX stream 1 and its second stream 2, channel selection 5) */
    #define IT_USART_DMA_TX                 ( USART_TX_DMA_BUS6_DMA2_STREAM6 )
    #define IT_USART_DMA_RX                 ( USART_RX_DMA_BUS6_DMA2_STREAM1 )
    #define IT_USART_DMA_RX_OTHER           ( USART_RX_DMA_BUS6_DMA2_STREAM2 )

    /** UART4 RTS, Arduino D9 (PA15, UART4_RTS) - not connected on the board */
    #define IT_USART_RTS_BUS                ( USART_BUS_4 )
    #define IT_USART_RTS_PIN                ( USART_DE_PIN_BUS4_PA15 )
    #define IT_USART_RTS_PORT               ( GPIO_PORT_A )
    #define IT_USART_RTS_PIN_ID             ( GPIO_PIN_ID_15 )

#elif defined(IT_BOARD_STM32F722xE) || \
      defined(IT_BOARD_STM32F756xG) || \
      defined(IT_BOARD_STM32F765xI) || \
      defined(IT_BOARD_STM32F767xI)

    /* NUCLEO-F722ZE / NUCLEO-F756ZG / NUCLEO-F767ZI (Nucleo-144) */

    /** USART6, PC6 (USART6_TX, Zio / morpho connector) - not connected on the board */
    #define IT_USART_BUS                    ( USART_BUS_6 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS6_PC6 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_C )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_6 )

    /** USART6 DMA2 streams (TX stream 6, RX stream 1 and its second stream 2, channel selection 5) */
    #define IT_USART_DMA_TX                 ( USART_TX_DMA_BUS6_DMA2_STREAM6 )
    #define IT_USART_DMA_RX                 ( USART_RX_DMA_BUS6_DMA2_STREAM1 )
    #define IT_USART_DMA_RX_OTHER           ( USART_RX_DMA_BUS6_DMA2_STREAM2 )

    /** UART4 RTS, PA15 (UART4_RTS, Zio / morpho connector) - not connected on the board */
    #define IT_USART_RTS_BUS                ( USART_BUS_4 )
    #define IT_USART_RTS_PIN                ( USART_DE_PIN_BUS4_PA15 )
    #define IT_USART_RTS_PORT               ( GPIO_PORT_A )
    #define IT_USART_RTS_PIN_ID             ( GPIO_PIN_ID_15 )

#else
    #error "Board of Usart integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** Baud rate [Bd] */
#define IT_USART_BAUDRATE                   ( 115200u )

/** Size of data buffers */
#define IT_USART_BUF_SIZE                   ( 64u )

/** Maximal count of wait loop iterations (~ hundreds of frames, Usart_Task called) */
#define IT_USART_WAIT_LOOPS                 ( 2000000u )

/** Count of bytes of frame timing measurement */
#define IT_USART_TIMING_BYTES               ( 50u )

/** Count of wait loop iterations of line settling after pull-up activation (several frames) */
#define IT_USART_LINE_SETTLE_LOOPS          ( 50000u )

/** Bits of one 8N1 frame (start + 8 data + stop) */
#define IT_USART_FRAME_BITS                 ( 10u )

/** Receiver timeout of timeout end mode [bits] (two frames) */
#define IT_USART_RTO_BITS                   ( 22u )

/** Interrupt priority of the data handling */
#define IT_USART_IRQ_PRIO                   ( 5u )

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
static usart_RxDma_t            itUsart_RxDma;

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

/* ============================= TEST SETUP ================================= */

void setUp( void )
{
    itUsart_TxCompleteCnt = 0u;
    itUsart_RxHalfCnt     = 0u;
    itUsart_RxCompleteCnt = 0u;
    itUsart_RxEndCnt      = 0u;
    itUsart_ErrorCnt      = 0u;
    itUsart_RxEndBytes    = 0u;
    itUsart_ErrorId       = USART_XFER_ERROR_CNT;
    itUsart_RxTimeout     = 0u;
    itUsart_RxDma         = IT_USART_DMA_RX;

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

    It_Usart_Init( USART_XFER_MODE_POLL, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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

    It_Usart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    It_Usart_Init( USART_XFER_MODE_POLL, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 16u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

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
    It_Usart_Init( USART_XFER_MODE_ISR, 32u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 32u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 32u );
}


/**
 * \brief   Data transfer in DMA mode.
 *
 * \details DMA mode (DMA2 stream 6 TX, stream 1 RX - channel 5 of USART6),
 *          one-shot RX buffer of 64 bytes. Starts reception and transmission of 64 bytes
 *          (single wire loopback).
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_TxStart_DmaMode_LoopbackDataReceived( void )
{
    It_Usart_Init( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( IT_USART_BUF_SIZE );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

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
    It_Usart_Init( USART_XFER_MODE_DMA, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 8u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 8u );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( IT_USART_BUS ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, &itUsart_TxBuf[ 8u ], 8u ) );
    It_Usart_Wait( &itUsart_TxCompleteCnt, 2u );

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

    It_Usart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 8u ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( IT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, txState );

    /* Second transmission while the first one is running */
    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 8u ) );

    It_Usart_Wait( &itUsart_TxCompleteCnt, 1u );

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

    It_Usart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 5u );
    It_Usart_Wait( &itUsart_RxEndCnt, 1u );

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

    It_Usart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_TIMEOUT,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 7u );
    It_Usart_Wait( &itUsart_RxEndCnt, 1u );

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
    It_Usart_Init( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 9u );
    It_Usart_Wait( &itUsart_RxEndCnt, 1u );

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
    It_Usart_Init( USART_XFER_MODE_ISR, 8u, USART_BUFFER_MODE_CIRCULAR, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    /* 20 bytes into 8 byte buffer: half at 4 / 12 / 20, complete at 8 / 16 */
    It_Usart_Transfer( 20u );
    It_Usart_Wait( &itUsart_RxHalfCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 16u ], &itUsart_RxBuf[ 0u ], 4u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 12u ], &itUsart_RxBuf[ 4u ], 4u );
}


/**
 * \brief   Circular DMA reception on the other receive stream wraps around the buffer.
 *
 * \details DMA mode, TX DMA2 stream 6, RX DMA2 stream 2 (second stream of the USART6_RX
 *          request), circular RX buffer of 8 bytes. Transfers 20 bytes.
 *
 * \par Expected results
 * - RX half callback 3x (byte 4, 12, 20), RX complete callback 2x (byte 8, 16).
 * - Buffer contains bytes 16 - 19 at index 0 - 3 and bytes 12 - 15 at index 4 - 7.
 */
void It_Usart_Set_RxStart_DmaCircularBufferOtherStream_WrapsAroundWithCallbacks( void )
{
    itUsart_RxDma = IT_USART_DMA_RX_OTHER;

    It_Usart_Init( USART_XFER_MODE_DMA, 8u, USART_BUFFER_MODE_CIRCULAR, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Transfer( 20u );
    It_Usart_Wait( &itUsart_RxHalfCnt, 3u );

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
    It_Usart_Init( USART_XFER_MODE_ISR, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_EVEN, USART_DATA_WIDTH_9 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( IT_USART_BUS, &parity ) );
    TEST_ASSERT_EQUAL( USART_PARITY_EVEN, parity );

    It_Usart_Transfer( 16u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

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

    It_Usart_Init( USART_XFER_MODE_ISR, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    It_Usart_Transfer( 16u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

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

    It_Usart_Init( USART_XFER_MODE_POLL, 4u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );
    It_Usart_Init_RefTime();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );

    Usart_SendData( IT_USART_BUS, itUsart_TxBuf[ 0u ] );
    It_Usart_Delay_Frames( 3u );

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
    It_Usart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    It_Usart_Check_FrameTime();
}

/*------------------------------ Flow control --------------------------------*/

/**
 * \brief   RTS output is inactive while the USART or its receiver is disabled.
 *
 * \details UART4 with RTS flow control on PA15 (not connected on the board), transmitter and
 *          receiver, no data handling. The RTS pin level (input data register) is read after
 *          the initialization, after disabling and enabling the USART, after disabling and
 *          enabling the receiver, after a baud rate change and after the de-initialization.
 *
 * \par Expected results
 * - USART and receiver enabled (receiver ready): RTS low.
 * - USART disabled, receiver disabled, de-initialized: RTS high.
 *
 * \note  Device errata bug AB#1296 (ES0334 "RTS is active while RE = 0 or UE = 0")
 */
void It_Usart_Set_PeriphInactive_RtsFlowControl_RtsInactiveWhileDisabled( void )
{
    usart_BusConfig_t busConfig;
    gpio_PinLevel_t   rtsLevel = GPIO_PIN_LEVEL_LOW;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.PeriphId      = IT_USART_RTS_BUS;
    busConfig.BaudRate      = IT_USART_BAUDRATE;
    busConfig.TransferMode  = USART_TRANSFER_MODE_TX_RX;
    busConfig.HwFlowControl = USART_FLOW_CONTROL_RTS;
    busConfig.BusDePin      = IT_USART_RTS_PIN;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_LOW, rtsLevel, "RTS after initialization" );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( IT_USART_RTS_BUS ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_HIGH, rtsLevel, "RTS with UE = 0" );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( IT_USART_RTS_BUS ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_LOW, rtsLevel, "RTS with UE = 1" );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( IT_USART_RTS_BUS, USART_TRANSFER_MODE_TX ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_HIGH, rtsLevel, "RTS with RE = 0" );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TransferMode( IT_USART_RTS_BUS, USART_TRANSFER_MODE_TX_RX ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_LOW, rtsLevel, "RTS with RE = 1" );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( IT_USART_RTS_BUS, IT_USART_BAUDRATE / 2u ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_LOW, rtsLevel, "RTS after baud rate change" );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( IT_USART_RTS_BUS ) );
    rtsLevel = It_Usart_Get_RtsLevel();
    TEST_ASSERT_EQUAL_MESSAGE( GPIO_PIN_LEVEL_HIGH, rtsLevel, "RTS after de-initialization" );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Returns level of the RTS pin (input data register).
 *
 * \return Level of IT_USART_RTS_PIN
 */
static gpio_PinLevel_t It_Usart_Get_RtsLevel( void )
{
    gpio_PinLevel_t pinLevel = GPIO_PIN_LEVEL_LOW;

    TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Get_PinLevel( IT_USART_RTS_PORT, IT_USART_RTS_PIN_ID, &pinLevel ) );

    return ( pinLevel );
}


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
static void It_Usart_Init( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize, usart_BufferMode_t bufferMode,
                          usart_RxEndMode_t rxEndMode, usart_Parity_t parity, usart_DataWidth_t dataWidth )
{
    usart_BusConfig_t busConfig;

    itUsart_DataConfig.TxMode             = xferMode;
    itUsart_DataConfig.RxMode             = xferMode;
    itUsart_DataConfig.RxBuffer           = itUsart_RxBuf;
    itUsart_DataConfig.RxBufferSize       = rxSize;
    itUsart_DataConfig.RxBufferMode       = bufferMode;
    itUsart_DataConfig.RxEndMode          = rxEndMode;
    itUsart_DataConfig.TxDma              = IT_USART_DMA_TX;
    itUsart_DataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    itUsart_DataConfig.RxDma              = itUsart_RxDma;
    itUsart_DataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    itUsart_DataConfig.IrqPriority        = IT_USART_IRQ_PRIO;
    itUsart_DataConfig.TxCompleteCallback = It_Usart_TxCompleteCallback;
    itUsart_DataConfig.RxHalfCallback     = It_Usart_RxHalfCallback;
    itUsart_DataConfig.RxCompleteCallback = It_Usart_RxCompleteCallback;
    itUsart_DataConfig.RxEndCallback      = It_Usart_RxEndCallback;
    itUsart_DataConfig.ErrorCallback      = It_Usart_ErrorCallback;

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
static void It_Usart_Transfer( usart_TxDataCnt_t txSize )
{
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( itUsart_Bus.Bus ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( itUsart_Bus.Bus, itUsart_TxBuf, txSize ) );

    It_Usart_Wait( &itUsart_TxCompleteCnt, 1u );
}


/**
 * \brief Waits until counter reaches required value or timeout (Usart_Task is
 *        called - data handling in polling mode).
 *
 * \param counter     [in]: Callback call counter
 * \param expectedCnt [in]: Required count
 */
static void It_Usart_Wait( volatile const uint32_t * const counter, uint32_t expectedCnt )
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
 * \brief Initializes and starts reference time base (DWT cycle counter).
 */
static void It_Usart_Init_RefTime( void )
{
    dwt_Config_t dwtConfig;

    TEST_ASSERT_EQUAL( DWT_REQUEST_OK, Dwt_Get_DefaultConfig( &dwtConfig ) );
    dwtConfig.CycleCounterState = DWT_FUNCTION_ACTIVE;

    TEST_ASSERT_EQUAL( DWT_REQUEST_OK, Dwt_Init( &dwtConfig ) );
}


/**
 * \brief Busy waits for given count of 8N1 frame times (DWT cycle counter, \ref
 *        It_Usart_Init_RefTime() is required before).
 *
 * \param frameCnt [in]: Count of frame times
 */
static void It_Usart_Delay_Frames( uint32_t frameCnt )
{
    const uint32_t           waitCycles = (uint32_t)( ( (uint64_t)frameCnt * IT_USART_FRAME_BITS * SystemCoreClock ) / IT_USART_BAUDRATE );
    const dwt_CounterValue_t startCnt   = Dwt_Get_CycleCounter();
    dwt_CounterValue_t       elapsed    = 0u;

    while( waitCycles > elapsed )
    {
        elapsed = Dwt_Get_CycleCounter() - startCnt;
    }
}


/**
 * \brief Measures transmission time of \ref IT_USART_TIMING_BYTES bytes (8N1, back to back)
 *        by DWT cycle counter and compares it with the configured baud rate (+- 5 %).
 */
static void It_Usart_Check_FrameTime( void )
{
    /* Expected duration of the transmission [core clock cycles] */
    const uint32_t expectedCycles = (uint32_t)( ( (uint64_t)IT_USART_TIMING_BYTES * IT_USART_FRAME_BITS * SystemCoreClock ) / IT_USART_BAUDRATE );

    It_Usart_Init_RefTime();

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
static void It_Usart_TxCompleteCallback( void )
{
    itUsart_TxCompleteCnt++;
}


/** \brief Reception buffer half filled callback */
static void It_Usart_RxHalfCallback( void )
{
    itUsart_RxHalfCnt++;
}


/** \brief Reception buffer filled callback */
static void It_Usart_RxCompleteCallback( void )
{
    itUsart_RxCompleteCnt++;
}


/**
 * \brief End of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void It_Usart_RxEndCallback( usart_RxDataCnt_t rxCnt )
{
    itUsart_RxEndBytes = rxCnt;
    itUsart_RxEndCnt++;
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_Usart_ErrorCallback( usart_XferErrorId_t errorId )
{
    itUsart_ErrorId = errorId;
    itUsart_ErrorCnt++;
}
