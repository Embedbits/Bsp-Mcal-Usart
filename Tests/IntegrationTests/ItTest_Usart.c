/**
 * \author Mr.Nobody
 * \file ItTest_Usart.c
 * \ingroup Usart
 * \brief Integration tests of USART module on target.
 *
 * Usart module runs on the MCU together with real RCC, NVIC, GPIO and GPDMA
 * modules and hardware. Tests verify behavior which cannot be verified by unit
 * tests (emulated registers): baud rate generation, data transfer in polling,
 * interrupt and DMA mode, end of message (idle line) detection, circular
 * reception buffer, parity and frame timing.
 *
 * No external wiring is used - USART works in single wire half-duplex mode,
 * the receiver receives every frame sent by the transmitter on the TX pin.
 *
 * Used resources (see board configuration below):
 * - IT_USART_BUS, TX pin IT_USART_TX_PIN - not connected on the board
 * - GPDMA1 channels 5 / 6 (DMA mode)
 * - TIM2 - reference time base for frame timing
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usart_Port.h"                     /* Module under test              */
#include "Tim_Port.h"                       /* Reference time base            */
#include "Gpio_Port.h"                      /* Pull-up of single wire line    */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void It_Usart_Init            ( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize, usart_BufferMode_t bufferMode,
                                      usart_RxEndMode_t rxEndMode, usart_Parity_t parity, usart_DataWidth_t dataWidth );
static void It_Usart_Wait           ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void It_Usart_Transfer       ( usart_TxDataCnt_t txSize );
static void It_Usart_Init_RefTime   ( void );

static void It_Usart_TxCompleteCallback ( void );
static void It_Usart_RxHalfCallback     ( void );
static void It_Usart_RxCompleteCallback ( void );
static void It_Usart_RxEndCallback      ( usart_RxDataCnt_t rxCnt );
static void It_Usart_ErrorCallback      ( usart_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32H503xB)

    /** USART3, Arduino D7 (PA8, USART3_TX) - not connected on the board */
    #define IT_USART_BUS                    ( USART_BUS_3 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS3_PA8 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_A )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_8 )

#elif defined(IT_BOARD_STM32H523xE) || \
      defined(IT_BOARD_STM32H533xE)

    /** USART1, Arduino D10 (PB6, USART1_TX) - not connected on the board (UART2 is ST-LINK VCP).
     *  USART3 TX on PA8 is defined for STM32H503 only. */
    #define IT_USART_BUS                    ( USART_BUS_1 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS1_PB6 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_B )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_6 )

#elif defined(IT_BOARD_STM32H562xI) || \
      defined(IT_BOARD_STM32H563xI) || \
      defined(IT_BOARD_STM32H573xI) || \
      defined(IT_BOARD_STM32H5E5xJ)

    /** USART1, PB6 (USART1_TX) - not connected on the board (USART3 is ST-LINK VCP) */
    #define IT_USART_BUS                    ( USART_BUS_1 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS1_PB6 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_B )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_6 )

#else
    #error "Board of Usart integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** Baud rate [Bd] */
#define IT_USART_BAUDRATE                   ( 115200u )

/** Size of data buffers */
#define IT_USART_BUF_SIZE                   ( 64u )

/** Maximal count of wait loop iterations (~ hundreds of frames, Usart_Task called) */
#define IT_USART_WAIT_LOOPS                 ( 2000000u )

/** Reference timer (1 count = 1 us) */
#define IT_USART_REF_TIM                    ( TIM_PERIPH_2 )

/** Count of bytes of frame timing measurement */
#define IT_USART_TIMING_BYTES               ( 50u )

/** Count of wait loop iterations of line settling after pull-up activation (several frames) */
#define IT_USART_LINE_SETTLE_LOOPS          ( 50000u )

/** Bits of one 8N1 frame (start + 8 data + stop) */
#define IT_USART_FRAME_BITS                 ( 10u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Transmitted data */
static usart_TxData_t           itUsart_TxBuf[ IT_USART_BUF_SIZE ];

/** Received data */
static usart_RxData_t           itUsart_RxBuf[ IT_USART_BUF_SIZE ];

/** Data handling configuration (copied by the module) */
static usart_DataConfig_t       itUsart_DataConfig;

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
    usart_Baudrate_t     baudrate   = 0u;
    usart_DataWidth_t    dataWidth  = USART_DATA_WIDTH_7;
    usart_StopBits_t     stopBits   = USART_STOP_BITS_2;
    usart_Parity_t       parity     = USART_PARITY_ODD;
    usart_HalfDuplex_t   halfDuplex = USART_HALF_DUPLEX_INACTIVE;
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
 * \details DMA mode (GPDMA1 channel 5 TX, channel 6 RX), one-shot RX buffer of 64
 *          bytes. Starts reception and transmission of 64 bytes (single wire loopback).
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

/*------------------------------ Frame timing --------------------------------*/

/**
 * \brief   Real baud rate matches configured baud rate.
 *
 * \details Interrupt mode, 115200 Bd 8N1. Starts reception and transmission of 50
 *          bytes and measures time until TX complete callback by TIM2 (1 MHz).
 *
 * \par Expected results
 * - TX complete callback 1x.
 * - Transmission time = 50 x 10 bits / 115200 Bd (4340 us) +- 5 %.
 */
void It_Usart_Set_Baudrate_115200_FrameTimeMatchesBaudrate( void )
{
    tim_Counter_t startCnt = 0u;
    tim_Counter_t endCnt   = 0u;

    /* Expected duration of the transmission (8N1 frames, back to back) [us] */
    const tim_Counter_t expectedTime = ( IT_USART_TIMING_BYTES * IT_USART_FRAME_BITS * 1000000u ) / IT_USART_BAUDRATE;

    It_Usart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );
    It_Usart_Init_RefTime();

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_USART_REF_TIM, &startCnt ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, IT_USART_TIMING_BYTES ) );

    for( uint32_t loopIdx = 0u; ( IT_USART_WAIT_LOOPS > loopIdx ) && ( 0u == itUsart_TxCompleteCnt ); loopIdx++ )
    {
        /* Wait for transmission complete callback (interrupt mode) */
    }

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_Counter( IT_USART_REF_TIM, &endCnt ) );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_UINT32_WITHIN( expectedTime / 20u, expectedTime, endCnt - startCnt );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes USART1 in half-duplex mode with data handling of both directions.
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
    itUsart_DataConfig.TxDmaPeriphId      = USART_DMA_PERIPH_1;
    itUsart_DataConfig.TxDmaChannelId     = USART_DMA_CHANNEL_5;
    itUsart_DataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    itUsart_DataConfig.RxDmaPeriphId      = USART_DMA_PERIPH_1;
    itUsart_DataConfig.RxDmaChannelId     = USART_DMA_CHANNEL_6;
    itUsart_DataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    itUsart_DataConfig.IrqPriority        = 5u;
    itUsart_DataConfig.TxCompleteCallback = It_Usart_TxCompleteCallback;
    itUsart_DataConfig.RxHalfCallback     = It_Usart_RxHalfCallback;
    itUsart_DataConfig.RxCompleteCallback = It_Usart_RxCompleteCallback;
    itUsart_DataConfig.RxEndCallback      = It_Usart_RxEndCallback;
    itUsart_DataConfig.ErrorCallback      = It_Usart_ErrorCallback;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.PeriphId     = IT_USART_BUS;
    busConfig.BaudRate     = IT_USART_BAUDRATE;
    busConfig.DataWidth    = dataWidth;
    busConfig.Parity       = parity;
    busConfig.HalfDuplex   = USART_HALF_DUPLEX_ACTIVE;
    busConfig.BusTxPin     = IT_USART_TX_PIN;
    busConfig.DataConfig   = &itUsart_DataConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );

    /* Single wire line is released by USART when not transmitting - pull-up of the line
     * (external on the board in the application, RM0481 half-duplex mode) */
    TEST_ASSERT_EQUAL( GPIO_REQUEST_OK, Gpio_Set_PinPull( IT_USART_TX_PORT, IT_USART_TX_PIN_ID, GPIO_PIN_PULL_UP ) );

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
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, txSize ) );

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
    for( uint32_t loopIdx = 0u; ( IT_USART_WAIT_LOOPS > loopIdx ) && ( expectedCnt > *counter ); loopIdx++ )
    {
        Usart_Task();
    }

    /* Few more frames for pending events (idle line, last byte) */
    for( uint32_t loopIdx = 0u; ( IT_USART_WAIT_LOOPS / 100u ) > loopIdx; loopIdx++ )
    {
        Usart_Task();
    }
}


/**
 * \brief Initializes and starts reference time base (TIM2, 1 MHz, free running).
 */
static void It_Usart_Init_RefTime( void )
{
    tim_PeriphConfig_t timConfig;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Get_DefaultConfig( &timConfig ) );
    timConfig.PeriphId         = IT_USART_REF_TIM;
    timConfig.TimerFrequency   = 1000000u;
    timConfig.RefreshFrequency = 1u;

    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Init( &timConfig ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Set_Period( IT_USART_REF_TIM, 0xFFFFFFFFu ) );
    TEST_ASSERT_EQUAL( TIM_REQUEST_OK, Tim_Start( IT_USART_REF_TIM ) );
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
