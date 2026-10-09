/**
 * \author Mr.Nobody
 * \file ItTest_Usart.c
 * \ingroup Usart
 * \brief Integration tests of Universal Synchronous/Asynchronous Receiver/Transmitter (USART) module on target.
 *
 * Usart module runs on the MCU together with real RCC, NVIC, GPIO and DMA modules and
 * hardware. Tests verify behavior which cannot be verified by unit tests (emulated
 * registers): clock activation, real baud-rate, data handling in polling, interrupt and DMA
 * mode (also mixed), end of message (idle line) detection, circular reception buffer, parity,
 * framing error (break) and overrun detection with the flag clear sequence of STM32F4 (SR read
 * followed by DR read by CPU or DMA).
 *
 * No external wiring is used - USART works in single wire half-duplex mode (TX pin open-drain
 * with pull-up), the receiver receives every frame sent by the transmitter on the TX pin.
 *
 * Used resources (see board configuration below):
 * - IT_USART_BUS, TX pin IT_USART_TX_PIN - not connected on the board
 * - DMA streams IT_USART_DMA_TX / IT_USART_DMA_RX (items of the DMA stream lists of the bus, DMA mode)
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usart_Port.h"                     /* Module under test              */
#include "Rcc_Port.h"                       /* USART clock state verification */
#include "Dma_Port.h"                       /* DMA stream state verification  */
#include "Stm32_usart.h"                    /* USART status flags, break      */
/* ============================= TYPEDEFS =================================== */

/* ======================= FORWARD DECLARATIONS ============================= */

static void     It_Usart_Init_Bus           ( usart_DataWidth_t dataWidth, usart_Parity_t parity, const usart_DataConfig_t * const dataConfig );
static void     It_Usart_Init               ( usart_XferMode_t txMode, usart_XferMode_t rxMode, usart_RxDataCnt_t rxSize,
                                              usart_BufferMode_t bufferMode, usart_RxEndMode_t rxEndMode );
static void     It_Usart_Transfer           ( usart_TxDataCnt_t txSize );
static void     It_Usart_Wait               ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static uint32_t It_Usart_WaitRxne           ( void );
static void     It_Usart_WaitLoops          ( uint32_t loopCnt );

static void     It_Usart_TxCompleteCallback ( void );
static void     It_Usart_RxHalfCallback     ( void );
static void     It_Usart_RxCompleteCallback ( void );
static void     It_Usart_RxEndCallback      ( usart_RxDataCnt_t rxCnt );
static void     It_Usart_RxEndRestartCallback( usart_RxDataCnt_t rxCnt );
static void     It_Usart_ErrorCallback      ( usart_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
/* Boards are named by their MCU (IT_BOARD_<MCU>, name of the board from the detection) */
#if defined(IT_BOARD_STM32F405xG) || \
    defined(IT_BOARD_STM32F407xG) || \
    defined(IT_BOARD_STM32F415xG) || \
    defined(IT_BOARD_STM32F417xG)

    /** USART2 TX PA2 (header P1) - not connected on the board */
    #define IT_USART_BUS                    ( USART_BUS_2 )
    #define IT_USART_REG                    ( USART2 )
    #define IT_USART_RCC                    ( RCC_PERIPH_USART2 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS2_PA2 )

    /** DMA streams of USART2 (RM0090: DMA1 stream 6 / 5, channel selection 4) */
    #define IT_USART_DMA_TX                 ( USART_TX_DMA_BUS2_DMA1_STREAM6 )
    #define IT_USART_DMA_RX                 ( USART_RX_DMA_BUS2_DMA1_STREAM5 )

#elif defined(IT_BOARD_STM32F401xE) || \
      defined(IT_BOARD_STM32F411xE) || \
      defined(IT_BOARD_STM32F446xE)

    /** USART1 TX PA9 (Arduino D8) - not connected on the board (USART2 PA2 / PA3 is ST-LINK VCP) */
    #define IT_USART_BUS                    ( USART_BUS_1 )
    #define IT_USART_REG                    ( USART1 )
    #define IT_USART_RCC                    ( RCC_PERIPH_USART1 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS1_PA9 )

    /** DMA streams of USART1 (RM0368 / RM0383 / RM0390: DMA2 stream 7 / 2, channel selection 4) */
    #define IT_USART_DMA_TX                 ( USART_TX_DMA_BUS1_DMA2_STREAM7 )
    #define IT_USART_DMA_RX                 ( USART_RX_DMA_BUS1_DMA2_STREAM2 )

#else
    #error "Board of Usart integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/** DMA peripheral and stream of the receive stream (decoded from the item of the stream list) */
#define IT_USART_DMA_RX_PERIPH              ( (dma_PeriphId_t)USART_DMA_BIT_MASK_DECODE_DMA( IT_USART_DMA_RX ) )
#define IT_USART_DMA_RX_STREAM              ( (dma_ChannelId_t)USART_DMA_BIT_MASK_DECODE_STREAM( IT_USART_DMA_RX ) )

/** Baud rate [Bd] */
#define IT_USART_BAUDRATE                   ( 115200u )

/** Size of data buffers */
#define IT_USART_BUF_SIZE                   ( 64u )

/** Maximal count of wait loop iterations (~ hundreds of frames, Usart_Task called) */
#define IT_USART_WAIT_LOOPS                 ( 2000000u )

/** Count of wait loop iterations of several frames (line settling, frames in progress) */
#define IT_USART_FRAMES_LOOPS               ( 50000u )

/** Interrupt priority of the tests */
#define IT_USART_IRQ_PRIO                   ( 5u )

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

/** Bit mask of reported errors (1 << usart_XferErrorId_t) */
static volatile uint32_t        itUsart_ErrorMask;

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
    itUsart_ErrorMask     = 0u;

    for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
    {
        itUsart_TxBuf[ byteIdx ] = (usart_TxData_t)( 0x30u + ( byteIdx * 7u ) );
        itUsart_RxBuf[ byteIdx ] = 0u;
    }
}


void tearDown( void )
{
    (void)Usart_Deinit( IT_USART_BUS );
}

/* =============================== TESTS ==================================== */

/*----------------------------- Initialization -------------------------------*/

/**
 * \brief   Usart_Init() activates clock and configures half-duplex bus.
 *
 * \details USART clock of the board is inactive after reset. 115200 Bd, 8N1, half-duplex,
 *          no data handling.
 *
 * \par Expected results
 * - Clock active, peripheral enabled, half-duplex active, 8 data bits, 1 stop bit, no
 *   parity.
 * - Baud-rate read back within 1 % of required value.
 */
void It_Usart_Init_ClockInactive_ClockActivatedAndBaudrateSet( void )
{
    rcc_FunctionState_t clockState  = RCC_FUNCTION_INACTIVE;
    usart_FlagState_t   periphState = USART_FLAG_INACTIVE;
    usart_HalfDuplex_t  halfDuplex  = USART_HALF_DUPLEX_INACTIVE;
    usart_DataWidth_t   dataWidth   = USART_DATA_WIDTH_9;
    usart_StopBits_t    stopBits    = USART_STOP_BITS_2;
    usart_Parity_t      parity      = USART_PARITY_ODD;
    usart_Baudrate_t    baudrate    = 0u;

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_USART_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clockState );

    It_Usart_Init_Bus( USART_DATA_WIDTH_8, USART_PARITY_NONE, NULL );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_USART_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_ACTIVE, clockState );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_PeriphState( IT_USART_BUS, &periphState ) );
    TEST_ASSERT_EQUAL( USART_FLAG_ACTIVE, periphState );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_HalfDuplexState( IT_USART_BUS, &halfDuplex ) );
    TEST_ASSERT_EQUAL( USART_HALF_DUPLEX_ACTIVE, halfDuplex );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DataWidth( IT_USART_BUS, &dataWidth ) );
    TEST_ASSERT_EQUAL( USART_DATA_WIDTH_8, dataWidth );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_StopBits( IT_USART_BUS, &stopBits ) );
    TEST_ASSERT_EQUAL( USART_STOP_BITS_1, stopBits );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Parity( IT_USART_BUS, &parity ) );
    TEST_ASSERT_EQUAL( USART_PARITY_NONE, parity );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( IT_USART_BUS, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( IT_USART_BAUDRATE / 100u, IT_USART_BAUDRATE, baudrate );
}


/**
 * \brief   Usart_Deinit() disables the peripheral and deactivates its clock.
 *
 * \par Expected results
 * - USART_REQUEST_OK, clock inactive.
 */
void It_Usart_Deinit_ClockDeactivated( void )
{
    rcc_FunctionState_t clockState = RCC_FUNCTION_ACTIVE;

    It_Usart_Init_Bus( USART_DATA_WIDTH_8, USART_PARITY_NONE, NULL );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_USART_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clockState );
}

/*------------------------- Direct register access ---------------------------*/

/**
 * \brief   Bytes written by Usart_SendData() are read by Usart_ReadData().
 *
 * \details No data handling, 32 bytes, every byte is sent after reception of the previous one.
 *
 * \par Expected results
 * - Every received byte equals the sent byte, no reception error flag.
 */
void It_Usart_SendData_HalfDuplexEcho_BytesReceived( void )
{
    It_Usart_Init_Bus( USART_DATA_WIDTH_8, USART_PARITY_NONE, NULL );

    (void)Usart_ReadData( IT_USART_BUS );

    for( uint32_t idx = 0u; 32u > idx; idx++ )
    {
        Usart_SendData( IT_USART_BUS, itUsart_TxBuf[ idx ] );

        const uint32_t statusReg = It_Usart_WaitRxne();

        TEST_ASSERT_EQUAL_HEX32_MESSAGE( 0u, statusReg & ( USART_SR_PE | USART_SR_FE | USART_SR_NE | USART_SR_ORE ), "Reception error" );
        TEST_ASSERT_EQUAL_HEX8( itUsart_TxBuf[ idx ], Usart_ReadData( IT_USART_BUS ) );
    }
}


/**
 * \brief   9 bit words with odd parity carry 8 data bits.
 *
 * \details No data handling, 9 bit word, odd parity, 2 stop bits.
 *
 * \par Expected results
 * - Data bits are received, no parity error flag.
 */
void It_Usart_SendData_NineBitOddParity_DataReceived( void )
{
    It_Usart_Init_Bus( USART_DATA_WIDTH_9, USART_PARITY_ODD, NULL );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_StopBits( IT_USART_BUS, USART_STOP_BITS_2 ) );

    (void)Usart_ReadData( IT_USART_BUS );

    for( uint32_t idx = 0u; 8u > idx; idx++ )
    {
        Usart_SendData( IT_USART_BUS, itUsart_TxBuf[ idx ] );

        const uint32_t statusReg = It_Usart_WaitRxne();

        TEST_ASSERT_EQUAL_HEX32_MESSAGE( 0u, statusReg & USART_SR_PE, "Parity error" );
        TEST_ASSERT_EQUAL_HEX8( itUsart_TxBuf[ idx ], Usart_ReadData( IT_USART_BUS ) );
    }
}

/*---------------------------- Data handling --------------------------------*/

/**
 * \brief   Data transfer in polling mode.
 *
 * \details Polling mode, one-shot RX buffer of 16 bytes. Starts reception and transmission
 *          of 16 bytes (single wire loopback), Usart_Task() moves the data.
 *
 * \par Expected results
 * - TX complete callback 1x, RX half callback 1x, RX complete callback 1x, no error.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_TxStart_PollMode_LoopbackDataReceived( void )
{
    It_Usart_Init( USART_XFER_MODE_POLL, USART_XFER_MODE_POLL, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    It_Usart_Transfer( 16u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxHalfCnt );
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
 * - TX complete callback 1x, RX half callback 1x, RX complete callback 1x, no error.
 * - Received data equal transmitted data, reception stopped (one shot buffer).
 */
void It_Usart_Set_TxStart_IsrMode_LoopbackDataReceived( void )
{
    usart_FunctionState_t rxState = USART_FUNCTION_ACTIVE;

    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, 32u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    It_Usart_Transfer( 32u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 32u );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( IT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, rxState );
}


/**
 * \brief   Data transfer in DMA mode, two consecutive transfers.
 *
 * \details DMA mode (streams of the board), one-shot RX buffer of 64 bytes. Starts
 *          reception and transmission of 64 bytes, then the second transfer with inverted
 *          data.
 *
 * \par Expected results
 * - TX complete callback and RX complete callback after every transfer, no error.
 * - Received data equal transmitted data in both transfers, RX count 64.
 */
void It_Usart_Set_TxStart_DmaMode_LoopbackDataReceivedTwice( void )
{
    usart_RxDataCnt_t rxCnt = 0u;

    It_Usart_Init( USART_XFER_MODE_DMA, USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    for( uint32_t xferIdx = 0u; 2u > xferIdx; xferIdx++ )
    {
        if( 0u != xferIdx )
        {
            for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
            {
                itUsart_TxBuf[ byteIdx ] = (usart_TxData_t)~itUsart_TxBuf[ byteIdx ];
                itUsart_RxBuf[ byteIdx ] = 0u;
            }
        }

        It_Usart_Transfer( IT_USART_BUF_SIZE );
        It_Usart_Wait( &itUsart_RxCompleteCnt, xferIdx + 1u );

        TEST_ASSERT_EQUAL_UINT32( xferIdx + 1u, itUsart_TxCompleteCnt );
        TEST_ASSERT_EQUAL_UINT32( xferIdx + 1u, itUsart_RxCompleteCnt );
        TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
        TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, IT_USART_BUF_SIZE );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( IT_USART_BUS, &rxCnt ) );
        TEST_ASSERT_EQUAL_UINT16( IT_USART_BUF_SIZE, rxCnt );
    }
}


/**
 * \brief   Data transfer with transmission by DMA and reception by interrupt.
 *
 * \details TX DMA mode, RX interrupt mode, one-shot RX buffer of 24 bytes.
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error, data equal.
 */
void It_Usart_Set_TxStart_DmaTxIsrRx_LoopbackDataReceived( void )
{
    It_Usart_Init( USART_XFER_MODE_DMA, USART_XFER_MODE_ISR, 24u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    It_Usart_Transfer( 24u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 24u );
}


/**
 * \brief   Data transfer with transmission by interrupt and reception by DMA.
 *
 * \details TX interrupt mode, RX DMA mode, one-shot RX buffer of 24 bytes.
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error, data equal.
 */
void It_Usart_Set_TxStart_IsrTxDmaRx_LoopbackDataReceived( void )
{
    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_DMA, 24u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    It_Usart_Transfer( 24u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 24u );
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

    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

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


/**
 * \brief   Usart_Set_TxStop() stops running transmission.
 *
 * \details Interrupt mode, RX buffer 64 bytes. Transmission of 64 bytes started and
 *          stopped immediately, waiting for several frames.
 *
 * \par Expected results
 * - TX state inactive, no TX complete callback, less than 64 bytes received.
 */
void It_Usart_Set_TxStop_IsrMode_TransmissionStopped( void )
{
    usart_FunctionState_t txState = USART_FUNCTION_ACTIVE;
    usart_RxDataCnt_t     rxCnt   = IT_USART_BUF_SIZE;

    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, IT_USART_BUF_SIZE ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStop( IT_USART_BUS ) );

    It_Usart_WaitLoops( IT_USART_FRAMES_LOOPS );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_TxState( IT_USART_BUS, &txState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_INACTIVE, txState );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( IT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_LESS_THAN_UINT16( IT_USART_BUF_SIZE, rxCnt );
}

/*------------------------- Reception modes ----------------------------------*/

/**
 * \brief   Idle line ends the received message (interrupt reception).
 *
 * \details Interrupt mode, RX end mode idle, RX buffer of 64 bytes. Transfers 5 bytes
 *          and waits for RX end callback.
 *
 * \par Expected results
 * - RX end callback 1x with 5 bytes, no RX complete callback.
 * - RX count 5, received data equal transmitted data.
 */
void It_Usart_Set_RxStart_IsrIdleEnd_RxEndCallbackWithByteCount( void )
{
    usart_RxDataCnt_t rxCnt = 0u;

    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE );

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
 * \brief   Idle line ends the received message (DMA reception).
 *
 * \details DMA mode, RX end mode idle, RX buffer of 64 bytes. Transfers 11 bytes and waits
 *          for RX end callback (count from DMA stream).
 *
 * \par Expected results
 * - RX end callback 1x with 11 bytes, no RX complete callback, no error.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_RxStart_DmaIdleEnd_RxEndCallbackWithByteCount( void )
{
    It_Usart_Init( USART_XFER_MODE_DMA, USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE );

    It_Usart_Transfer( 11u );
    It_Usart_Wait( &itUsart_RxEndCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 11u, itUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 11u );
}


/**
 * \brief   Reception restarted from end of message callback receives the next message.
 *
 * \details Interrupt mode, RX end mode idle, end callback restarts the reception. Two
 *          messages of 3 bytes.
 *
 * \par Expected results
 * - RX end callback 2x with 3 bytes, second message at buffer start.
 */
void It_Usart_Set_RxStart_RestartFromRxEndCallback_SecondMessageReceived( void )
{
    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE );

    itUsart_DataConfig.RxEndCallback = It_Usart_RxEndRestartCallback;
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataConfig( IT_USART_BUS, &itUsart_DataConfig ) );

    It_Usart_Transfer( 3u );
    It_Usart_Wait( &itUsart_RxEndCnt, 1u );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, &itUsart_TxBuf[ 3u ], 3u ) );
    It_Usart_Wait( &itUsart_RxEndCnt, 2u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 3u, itUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 3u ], itUsart_RxBuf, 3u );
}


/**
 * \brief   Circular reception wraps around the buffer (interrupt reception).
 *
 * \details Interrupt mode, circular RX buffer of 8 bytes. Transfers 20 bytes.
 *
 * \par Expected results
 * - RX half callback 3x (byte 4, 12, 20), RX complete callback 2x (byte 8, 16).
 * - Buffer contains bytes 16 - 19 at index 0 - 3 and bytes 12 - 15 at index 4 - 7.
 */
void It_Usart_Set_RxStart_IsrCircularBuffer_WrapsAroundWithCallbacks( void )
{
    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, 8u, USART_BUFFER_MODE_CIRCULAR, USART_RX_END_NONE );

    It_Usart_Transfer( 20u );
    It_Usart_Wait( &itUsart_RxHalfCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 16u ], &itUsart_RxBuf[ 0u ], 4u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 12u ], &itUsart_RxBuf[ 4u ], 4u );
}


/**
 * \brief   Circular reception wraps around the buffer (DMA circular mode).
 *
 * \details DMA mode, circular RX buffer of 8 bytes. Transfers 20 bytes.
 *
 * \par Expected results
 * - RX half callback 3x, RX complete callback 2x, RX count 4, reception keeps running.
 * - Buffer contains bytes 16 - 19 at index 0 - 3 and bytes 12 - 15 at index 4 - 7.
 */
void It_Usart_Set_RxStart_DmaCircularBuffer_WrapsAroundWithCallbacks( void )
{
    usart_RxDataCnt_t     rxCnt   = 0u;
    usart_FunctionState_t rxState = USART_FUNCTION_INACTIVE;

    It_Usart_Init( USART_XFER_MODE_DMA, USART_XFER_MODE_DMA, 8u, USART_BUFFER_MODE_CIRCULAR, USART_RX_END_NONE );

    It_Usart_Transfer( 20u );
    It_Usart_Wait( &itUsart_RxHalfCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 16u ], &itUsart_RxBuf[ 0u ], 4u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 12u ], &itUsart_RxBuf[ 4u ], 4u );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxCount( IT_USART_BUS, &rxCnt ) );
    TEST_ASSERT_EQUAL_UINT16( 4u, rxCnt );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_RxState( IT_USART_BUS, &rxState ) );
    TEST_ASSERT_EQUAL( USART_FUNCTION_ACTIVE, rxState );
}

/*------------------------- Frame format and errors --------------------------*/

/**
 * \brief   Data transfer with even parity (interrupt mode).
 *
 * \details Interrupt mode, 9-bit frame (8 data bits + even parity bit), 16 bytes.
 *
 * \par Expected results
 * - No error callback (no parity error), received data equal transmitted data.
 */
void It_Usart_Init_EvenParity9Bit_LoopbackDataReceived( void )
{
    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphInactive( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_DataWidth( IT_USART_BUS, USART_DATA_WIDTH_9 ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Parity( IT_USART_BUS, USART_PARITY_EVEN ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_PeriphActive( IT_USART_BUS ) );

    It_Usart_Transfer( 16u );
    It_Usart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 16u );
}


/**
 * \brief   Break character is reported as framing error, reception continues.
 *
 * \details Interrupt mode, RX buffer 8 bytes. Reception started, break character sent
 *          (SBK), then 2 bytes transferred.
 *
 * \par Expected results
 * - Framing error reported, flags cleared by the clear sequence - the following bytes are
 *   received without further error.
 */
void It_Usart_Isr_BreakCharacter_FramingErrorReported( void )
{
    It_Usart_Init( USART_XFER_MODE_ISR, USART_XFER_MODE_ISR, 8u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );

    LL_USART_RequestBreakSending( IT_USART_REG );
    It_Usart_Wait( &itUsart_ErrorCnt, 1u );

    TEST_ASSERT_EQUAL_HEX32( 1u << USART_XFER_ERROR_FRAMING, itUsart_ErrorMask & ( 1u << USART_XFER_ERROR_FRAMING ) );

    itUsart_ErrorMask = 0u;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 2u ) );
    It_Usart_Wait( &itUsart_TxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_HEX32( 0u, itUsart_ErrorMask );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, &itUsart_RxBuf[ 1u ], 2u );
}


/**
 * \brief   Overrun is reported in polling reception, data of the first frame stay valid.
 *
 * \details TX DMA mode, RX polling mode. 4 bytes transmitted by DMA without calling
 *          Usart_Task() (frames following the first one are lost), then Usart_Task() is called.
 *
 * \par Expected results
 * - Overrun error reported, first byte received.
 */
void It_Usart_Poll_Overrun_ErrorReportedFirstByteValid( void )
{
    It_Usart_Init( USART_XFER_MODE_DMA, USART_XFER_MODE_POLL, 8u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, 4u ) );

    /* No Usart_Task() call during the transmission (TX complete reported by interrupt) */
    for( uint32_t loopIdx = 0u; ( IT_USART_WAIT_LOOPS > loopIdx ) && ( 0u == itUsart_TxCompleteCnt ); loopIdx++ )
    {
        /* Busy wait */
    }

    It_Usart_Wait( &itUsart_ErrorCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_HEX32( 1u << USART_XFER_ERROR_OVERRUN, itUsart_ErrorMask & ( 1u << USART_XFER_ERROR_OVERRUN ) );
    TEST_ASSERT_EQUAL_HEX8( itUsart_TxBuf[ 0u ], itUsart_RxBuf[ 0u ] );
}

/*-------------------------- Resources release -------------------------------*/

/**
 * \brief   Usart_Deinit() stops running DMA reception and releases the DMA stream.
 *
 * \details DMA mode, reception started (no data), Usart_Deinit().
 *
 * \par Expected results
 * - RX DMA stream disabled, DMA receive request inactive, clock of the USART inactive.
 */
void It_Usart_Deinit_DmaReception_StreamReleased( void )
{
    dma_FunctionState_t xferState  = DMA_FUNCTION_ACTIVE;
    rcc_FunctionState_t clockState = RCC_FUNCTION_ACTIVE;

    It_Usart_Init( USART_XFER_MODE_DMA, USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( DMA_REQUEST_OK, Dma_Get_TransferState( IT_USART_DMA_RX_PERIPH, IT_USART_DMA_RX_STREAM, &xferState ) );
    TEST_ASSERT_EQUAL( DMA_FUNCTION_ACTIVE, xferState );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( IT_USART_BUS ) );

    TEST_ASSERT_EQUAL( DMA_REQUEST_OK, Dma_Get_TransferState( IT_USART_DMA_RX_PERIPH, IT_USART_DMA_RX_STREAM, &xferState ) );
    TEST_ASSERT_EQUAL( DMA_FUNCTION_INACTIVE, xferState );
    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphState( IT_USART_RCC, &clockState ) );
    TEST_ASSERT_EQUAL( RCC_FUNCTION_INACTIVE, clockState );
}

/* ========================== LOCAL FUNCTIONS =============================== */

/**
 * \brief Initializes USART of the board in half-duplex mode (TX pin open-drain with
 *        pull-up), 115200 Bd, over-sampling by 16.
 *
 * \param dataWidth  [in]: Data width (incl. parity bit)
 * \param parity     [in]: Parity
 * \param dataConfig [in]: Data handling configuration (NULL - no data handling)
 */
static void It_Usart_Init_Bus( usart_DataWidth_t dataWidth, usart_Parity_t parity, const usart_DataConfig_t * const dataConfig )
{
    usart_BusConfig_t busConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.PeriphId     = IT_USART_BUS;
    busConfig.BaudRate     = IT_USART_BAUDRATE;
    busConfig.DataWidth    = dataWidth;
    busConfig.Parity       = parity;
    busConfig.Oversampling = USART_OVERSAMPLING_16;
    busConfig.HalfDuplex   = USART_HALF_DUPLEX_ACTIVE;
    busConfig.BusTxPin     = IT_USART_TX_PIN;
    busConfig.DataConfig   = dataConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );

    /* Frame possibly started on the line during pin configuration is finished before the
     * reception start (Usart_Set_RxStart discards stale data) */
    It_Usart_WaitLoops( IT_USART_FRAMES_LOOPS );
}


/**
 * \brief Initializes the bus (8N1) with data handling of the given modes.
 *
 * \param txMode     [in]: Transmission mode
 * \param rxMode     [in]: Reception mode
 * \param rxSize     [in]: Reception buffer size
 * \param bufferMode [in]: Reception buffer mode
 * \param rxEndMode  [in]: End of received message detection
 */
static void It_Usart_Init( usart_XferMode_t txMode, usart_XferMode_t rxMode, usart_RxDataCnt_t rxSize,
                           usart_BufferMode_t bufferMode, usart_RxEndMode_t rxEndMode )
{
    itUsart_DataConfig.TxMode             = txMode;
    itUsart_DataConfig.RxMode             = rxMode;
    itUsart_DataConfig.RxBuffer           = itUsart_RxBuf;
    itUsart_DataConfig.RxBufferSize       = rxSize;
    itUsart_DataConfig.RxBufferMode       = bufferMode;
    itUsart_DataConfig.RxEndMode          = rxEndMode;
    itUsart_DataConfig.TxDma              = IT_USART_DMA_TX;
    itUsart_DataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    itUsart_DataConfig.RxDma              = IT_USART_DMA_RX;
    itUsart_DataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    itUsart_DataConfig.IrqPriority        = IT_USART_IRQ_PRIO;
    itUsart_DataConfig.TxCompleteCallback = It_Usart_TxCompleteCallback;
    itUsart_DataConfig.RxHalfCallback     = It_Usart_RxHalfCallback;
    itUsart_DataConfig.RxCompleteCallback = It_Usart_RxCompleteCallback;
    itUsart_DataConfig.RxEndCallback      = It_Usart_RxEndCallback;
    itUsart_DataConfig.ErrorCallback      = It_Usart_ErrorCallback;

    It_Usart_Init_Bus( USART_DATA_WIDTH_8, USART_PARITY_NONE, &itUsart_DataConfig );
}


/**
 * \brief Starts reception and transmission of the first bytes of the TX buffer
 *        and waits for the end of the transmission.
 *
 * \param txSize [in]: Count of transmitted bytes
 */
static void It_Usart_Transfer( usart_TxDataCnt_t txSize )
{
    const uint32_t txCompleteCnt = itUsart_TxCompleteCnt;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( IT_USART_BUS ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( IT_USART_BUS, itUsart_TxBuf, txSize ) );

    It_Usart_Wait( &itUsart_TxCompleteCnt, txCompleteCnt + 1u );
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
 * \brief Waits for received data (RXNE flag) - reception without data handling.
 *
 * \return Status register with RXNE flag (error flags of the frame)
 */
static uint32_t It_Usart_WaitRxne( void )
{
    uint32_t statusReg = 0u;

    for( uint32_t loopCnt = 0u; IT_USART_WAIT_LOOPS > loopCnt; loopCnt++ )
    {
        statusReg = IT_USART_REG->SR;

        if( 0u != ( statusReg & USART_SR_RXNE ) )
        {
            break;
        }
    }

    TEST_ASSERT_BITS_HIGH_MESSAGE( USART_SR_RXNE, statusReg, "Data not received" );

    return ( statusReg );
}


/**
 * \brief Busy wait.
 *
 * \param loopCnt [in]: Count of loop iterations
 */
static void It_Usart_WaitLoops( uint32_t loopCnt )
{
    for( volatile uint32_t loopIdx = 0u; loopCnt > loopIdx; loopIdx++ )
    {
        /* Busy wait */
    }
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
 * \brief End of received message callback restarting the reception (one shot buffer).
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void It_Usart_RxEndRestartCallback( usart_RxDataCnt_t rxCnt )
{
    It_Usart_RxEndCallback( rxCnt );

    (void)Usart_Set_RxStart( IT_USART_BUS );
}


/**
 * \brief Transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void It_Usart_ErrorCallback( usart_XferErrorId_t errorId )
{
    itUsart_ErrorId    = errorId;
    itUsart_ErrorMask |= ( 1u << (uint32_t)errorId );
    itUsart_ErrorCnt++;
}
