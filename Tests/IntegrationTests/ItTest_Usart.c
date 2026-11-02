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
 * circular reception buffer, parity, frame timing and LPUART1.
 *
 * Single bus tests: no external wiring - USART / LPUART works in single wire half-duplex
 * mode, the receiver receives every frame sent by the transmitter on the TX pin.
 *
 * Pair tests: two U(S)ARTs of the board are crossed (TX of A -> RX of B, TX of B -> RX of A).
 * Loop tests: TX of a U(S)ART is wired to its own RX. Pins of the wires in the board block below.
 * Fixture of the board (EmBi_Platform IntegrationTesting, "Board fixtures"): the wiring is the
 * feature UART_PAIR / UART_LOOP - the pair / loop tests are ignored on a board with a fixture without
 * it (a board without a fixture is expected to be wired) and on a board without a wiring known to the
 * test set. The only STM32L4 board of the inventory is NUCLEO-L4R5ZI (STM32L4R5xI): USART3 <-> UART5
 * PC10 - PD2, PC12 - PC11 and the USART2 loop PA2 - PA3.
 *
 * Used resources (see board configuration below):
 * - IT_USART_BUS, TX pin IT_USART_TX_PIN - not connected on the board
 * - LPUART1, TX pin IT_LPUART_TX_PIN - not connected on the board or ST-LINK virtual COM port
 *   receive line (only listens)
 * - IT_USART_PAIR_A_BUS / IT_USART_PAIR_B_BUS on their TX / RX pins (pair tests), IT_USART_LOOP_BUS
 *   on its TX / RX pins (loop tests)
 * - DMA channels of the STM32L4 fixed request mapping (valid on STM32L4+ too): USART1 TX DMA1
 *   channel 4, RX DMA1 channel 5 / DMA2 channel 7, LPUART1 TX DMA2 channel 6, RX DMA2 channel 7
 *   (single bus tests), USART3 DMA1 channel 2 / 3, UART5 DMA2 channel 1 / 2 (pair), USART2 DMA1
 *   channel 7 / 6 (loop)
 * - DWT cycle counter - reference time base for frame timing
 *
 * \note Boards are named by the MCU detected by integration testing (DEV_ID and flash size - the
 *       first MCU of the same identification in the Ral presets, other MCUs of the
 *       identification are listed too). Boards sharing the name use pins valid on every board of
 *       the group (STM32CubeMX board files).
 * \note STM32L4 errata sheets not reviewed yet.
 */

/* ============================= INCLUDES =================================== */
#include "unity.h"                          /* Unity testing framework        */
#include "IntegrationTesting.h"             /* Integration testing on target  */
#include "Usart_Port.h"                     /* Module under test              */
#include "Dwt_Port.h"                       /* Reference time base            */
#include "Gpio_Port.h"                      /* Pull-up of single wire line    */
#include "Rcc_Port.h"                       /* Kernel clock of LPUART1        */
#include "Stm32.h"                          /* SystemCoreClock                */
/* ============================= TYPEDEFS =================================== */

/** \brief Bus of a test case (USART or LPUART with its single wire pin) */
typedef struct
{
    usart_PeriphId_t     Bus;          /**< USART/UART bus                        */
    usart_TxPin_t        TxPin;        /**< TX pin (single wire line)             */
    gpio_PortId_t        TxPort;       /**< Port of TX pin                        */
    gpio_PinId_t         TxPinId;      /**< Pin of TX pin                         */
    usart_Oversampling_t Oversampling; /**< Over-sampling (LPUART: 16 only)       */
    usart_TxDma_t        TxDma;        /**< DMA channel of the transmission       */
}   it_UsartBus_t;

/** \brief Link of the UART wiring: the pair (buses A and B crossed) and the loop (TX wired to RX of one bus) */
typedef enum
{
    IT_USART_LINK_PAIR_A = 0u, /**< Bus A of the pair                  */
    IT_USART_LINK_PAIR_B,      /**< Bus B of the pair                  */
    IT_USART_LINK_LOOP,        /**< Bus of the loop (TX wired to RX)   */
    IT_USART_LINK_CNT          /**< Count of links                     */
}   it_UsartLink_t;


/** \brief Bus, pins, DMA channels and callbacks of a link (board dependent, \ref itUsart_LinkDesc) */
typedef struct
{
    usart_PeriphId_t         Bus;        /**< USART/UART bus                    */
    usart_TxPin_t            TxPin;      /**< TX pin                            */
    usart_RxPin_t            RxPin;      /**< RX pin                            */
    usart_TxDma_t            TxDma;      /**< DMA channel of the transmission   */
    usart_RxDma_t            RxDma;      /**< DMA channel of the reception      */
    usart_XferCallback_t    *TxComplete; /**< Transmission complete callback    */
    usart_XferCallback_t    *RxComplete; /**< Reception complete callback       */
    usart_RxEndCallback_t   *RxEnd;      /**< End of received message callback  */
    usart_XferErrCallback_t *Error;      /**< Transfer error callback           */
}   it_UsartLinkDesc_t;


/** \brief Callback counters and data handling configuration of a link */
typedef struct
{
    volatile uint32_t           TxCompleteCnt; /**< TX complete callbacks              */
    volatile uint32_t           RxCompleteCnt; /**< RX complete callbacks              */
    volatile uint32_t           RxEndCnt;      /**< RX end (idle line) callbacks       */
    volatile usart_RxDataCnt_t  RxEndBytes;    /**< Byte count of the last RX end      */
    volatile uint32_t           ErrorCnt;      /**< Error callbacks                    */
    volatile usart_XferErrorId_t ErrorId;      /**< Error of the last error callback   */
    usart_DataConfig_t          DataConfig;    /**< Data handling configuration        */
}   it_UsartLinkData_t;

/* ======================= FORWARD DECLARATIONS ============================= */

static void ItUsart_Init            ( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize, usart_BufferMode_t bufferMode,
                                      usart_RxEndMode_t rxEndMode, usart_Parity_t parity, usart_DataWidth_t dataWidth );
static void ItUsart_Wait            ( volatile const uint32_t * const counter, uint32_t expectedCnt );
static void ItUsart_Transfer        ( usart_TxDataCnt_t txSize );
static void ItUsart_Clear_Counters  ( void );
static usart_FlagState_t ItUsart_Get_BusOverrun( void );
static void ItUsart_Init_RefTime    ( void );
static void ItUsart_Delay_Frames    ( uint32_t frameCnt );
static void ItUsart_Check_FrameTime( void );

static void ItUsart_TxCompleteCallback  ( void );
static void ItUsart_RxHalfCallback      ( void );
static void ItUsart_RxCompleteCallback  ( void );
static void ItUsart_RxEndCallback       ( usart_RxDataCnt_t rxCnt );
static void ItUsart_ErrorCallback       ( usart_XferErrorId_t errorId );

static void             ItUsart_RequirePair     ( void );
static void             ItUsart_RequireLoop     ( void );
static void             ItUsart_Init_Pair       ( usart_XferMode_t xferMode,
                                                  usart_RxDataCnt_t rxSize,
                                                  usart_RxEndMode_t rxEndMode,
                                                  usart_Baudrate_t baudrate );
static void             ItUsart_Init_Loop       ( usart_XferMode_t xferMode,
                                                  usart_RxDataCnt_t rxSize,
                                                  usart_Baudrate_t baudrate );
static void             ItUsart_Init_Link       ( it_UsartLink_t link,
                                                  usart_XferMode_t xferMode,
                                                  usart_RxDataCnt_t rxSize,
                                                  usart_RxEndMode_t rxEndMode,
                                                  usart_Baudrate_t baudrate );
static void             ItUsart_Fill_Patterns   ( void );
static void             ItUsart_Clear_Link      ( it_UsartLink_t link );
static usart_FlagState_t ItUsart_Get_Overrun    ( it_UsartLink_t link );
static void             ItUsart_Transfer_Link   ( it_UsartLink_t txLink,
                                                  it_UsartLink_t rxLink,
                                                  usart_TxDataCnt_t txSize );
static void             ItUsart_Exchange_Pair   ( usart_TxDataCnt_t dataCnt );
static void             ItUsart_PairA_TxComplete( void );
static void             ItUsart_PairA_RxComplete( void );
static void             ItUsart_PairA_RxEnd     ( usart_RxDataCnt_t rxCnt );
static void             ItUsart_PairA_Error     ( usart_XferErrorId_t errorId );
static void             ItUsart_PairB_TxComplete( void );
static void             ItUsart_PairB_RxComplete( void );
static void             ItUsart_PairB_RxEnd     ( usart_RxDataCnt_t rxCnt );
static void             ItUsart_PairB_Error     ( usart_XferErrorId_t errorId );
static void             ItUsart_Loop_TxComplete ( void );
static void             ItUsart_Loop_RxComplete ( void );
static void             ItUsart_Loop_RxEnd      ( usart_RxDataCnt_t rxCnt );
static void             ItUsart_Loop_Error      ( usart_XferErrorId_t errorId );

/* ========================= SYMBOLIC CONSTANTS ============================= */

/*----------------------------- Board configuration --------------------------*/
#if defined(IT_BOARD_STM32L412xB) || \
    defined(IT_BOARD_STM32L422xB) || \
    defined(IT_BOARD_STM32L431xC) || \
    defined(IT_BOARD_STM32L432xC) || \
    defined(IT_BOARD_STM32L433xC) || \
    defined(IT_BOARD_STM32L442xC) || \
    defined(IT_BOARD_STM32L443xC)

    /** NUCLEO-L412KB / NUCLEO-L432KC (Nucleo-32 MB1180) and NUCLEO-L412RB-P / NUCLEO-L433RC-P
     *  (Nucleo-64-P MB1319): USART1, PA9 (USART1_TX, Nucleo-32 Arduino D1) - not connected on the boards */
    #define IT_USART_BUS                    ( USART_BUS_1 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS1_PA9 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_A )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_9 )

    /** LPUART1, PA2 (LPUART1_TX) - ST-LINK virtual COM port, receive line of ST-LINK */
    #define IT_LPUART_TX_PIN                ( USART_TX_PIN_LPBUS1_PA2 )
    #define IT_LPUART_TX_PORT               ( GPIO_PORT_A )
    #define IT_LPUART_TX_PIN_ID             ( GPIO_PIN_ID_2 )

#elif defined(IT_BOARD_STM32L451xE) || \
      defined(IT_BOARD_STM32L452xE) || \
      defined(IT_BOARD_STM32L462xE) || \
      defined(IT_BOARD_STM32L471xG) || \
      defined(IT_BOARD_STM32L475xG) || \
      defined(IT_BOARD_STM32L476xG) || \
      defined(IT_BOARD_STM32L486xG)

    /** NUCLEO-L452RE / NUCLEO-L476RG (Nucleo-64 MB1136) and NUCLEO-L452RE-P (Nucleo-64-P MB1319):
     *  USART1, PA9 (USART1_TX, Arduino D8) - not connected on the boards */
    #define IT_USART_BUS                    ( USART_BUS_1 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS1_PA9 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_A )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_9 )

    /** LPUART1, PB11 (LPUART1_TX, morpho) - not connected on the boards (STM32L47x / L48x have no
     *  LPUART1 on PA2) */
    #define IT_LPUART_TX_PIN                ( USART_TX_PIN_LPBUS1_PB11 )
    #define IT_LPUART_TX_PORT               ( GPIO_PORT_B )
    #define IT_LPUART_TX_PIN_ID             ( GPIO_PIN_ID_11 )

#elif defined(IT_BOARD_STM32L496xG) || \
      defined(IT_BOARD_STM32L4A6xG) || \
      defined(IT_BOARD_STM32L4P5xG) || \
      defined(IT_BOARD_STM32L4Q5xG) || \
      defined(IT_BOARD_STM32L4R5xI) || \
      defined(IT_BOARD_STM32L4R7xI) || \
      defined(IT_BOARD_STM32L4R9xI) || \
      defined(IT_BOARD_STM32L4S5xI) || \
      defined(IT_BOARD_STM32L4S7xI) || \
      defined(IT_BOARD_STM32L4S9xI)

    /** NUCLEO-L496ZG / L496ZG-P / L4A6ZG / L4P5ZG / L4R5ZI / L4R5ZI-P (Nucleo-144 MB1312):
     *  USART1, PB6 (USART1_TX) - not connected on the boards (PA9 is the USB VBUS sense input) */
    #define IT_USART_BUS                    ( USART_BUS_1 )
    #define IT_USART_TX_PIN                 ( USART_TX_PIN_BUS1_PB6 )
    #define IT_USART_TX_PORT                ( GPIO_PORT_B )
    #define IT_USART_TX_PIN_ID              ( GPIO_PIN_ID_6 )

    /** LPUART1 TX (the virtual COM port pin PG7 is powered by VDDIO2):
     *  - NUCLEO-L4R5ZI: PC1 (LPUART1_TX, AF8) - not connected on the board and not a wire of the common
     *    board wiring (checked on the IT agent: floats with both internal pulls, no other pin of the wires
     *    follows it driven high or low). PB11 is the I2C2 SDA wire (PB9 - PB11, pull-up 4.7 kOhm) there.
     *  - other boards of the group: PB11 (LPUART1_TX) - not connected per the STM32CubeMX board files
     *    (not verified on hardware, boards not on the IT agent) */
    #if defined(IT_BOARD_STM32L4R5xI)
        #define IT_LPUART_TX_PIN            ( USART_TX_PIN_LPBUS1_PC1 )
        #define IT_LPUART_TX_PORT           ( GPIO_PORT_C )
        #define IT_LPUART_TX_PIN_ID         ( GPIO_PIN_ID_1 )
    #else
        #define IT_LPUART_TX_PIN            ( USART_TX_PIN_LPBUS1_PB11 )
        #define IT_LPUART_TX_PORT           ( GPIO_PORT_B )
        #define IT_LPUART_TX_PIN_ID         ( GPIO_PIN_ID_11 )
    #endif /* IT_BOARD_STM32L4R5xI */


    /** NUCLEO-L4R5ZI (inventory wires of the fixture features UART_PAIR and UART_LOOP):
     *  - pair: USART3 TX PC10 -> UART5 RX PD2, UART5 TX PC12 -> USART3 RX PC11
     *  - loop: USART2 TX PA2 -> USART2 RX PA3
     *  DMA channels of the fixed request mapping of the classic STM32L4 (valid through DMAMUX1 of STM32L4+ too):
     *  USART3 DMA1 channel 2 / 3, UART5 DMA2 channel 1 / 2, USART2 DMA1 channel 7 / 6 */
    #if defined(IT_BOARD_STM32L4R5xI)
        #define IT_USART_PAIR_AVAILABLE     ( 1u )
        #define IT_USART_PAIR_A_BUS         ( USART_BUS_3 )
        #define IT_USART_PAIR_A_TX_PIN      ( USART_TX_PIN_BUS3_PC10 )
        #define IT_USART_PAIR_A_RX_PIN      ( USART_RX_PIN_BUS3_PC11 )
        #define IT_USART_PAIR_A_TX_DMA      ( USART_TX_DMA_BUS3_DMA1_CHANNEL2 )
        #define IT_USART_PAIR_A_RX_DMA      ( USART_RX_DMA_BUS3_DMA1_CHANNEL3 )
        #define IT_USART_PAIR_B_BUS         ( USART_BUS_5 )
        #define IT_USART_PAIR_B_TX_PIN      ( USART_TX_PIN_BUS5_PC12 )
        #define IT_USART_PAIR_B_RX_PIN      ( USART_RX_PIN_BUS5_PD2 )
        #define IT_USART_PAIR_B_TX_DMA      ( USART_TX_DMA_BUS5_DMA2_CHANNEL1 )
        #define IT_USART_PAIR_B_RX_DMA      ( USART_RX_DMA_BUS5_DMA2_CHANNEL2 )
        #define IT_USART_LOOP_AVAILABLE     ( 1u )
        #define IT_USART_LOOP_BUS           ( USART_BUS_2 )
        #define IT_USART_LOOP_TX_PIN        ( USART_TX_PIN_BUS2_PA2 )
        #define IT_USART_LOOP_RX_PIN        ( USART_RX_PIN_BUS2_PA3 )
        #define IT_USART_LOOP_TX_DMA        ( USART_TX_DMA_BUS2_DMA1_CHANNEL7 )
        #define IT_USART_LOOP_RX_DMA        ( USART_RX_DMA_BUS2_DMA1_CHANNEL6 )
    #endif /* IT_BOARD_STM32L4R5xI */

#else
    #error "Board of Usart integration tests is not defined (INTEGRATION_TEST_BOARD)."
#endif

/* Boards without a known wiring of a UART pair / loop - the pair / loop tests are ignored (the pin
 * and DMA items are placeholders). Wiring of the inventory (EmBi_Tester): NUCLEO-L4R5ZI only. */
#if !defined(IT_USART_PAIR_AVAILABLE)
    #define IT_USART_PAIR_AVAILABLE         ( 0u )
    #define IT_USART_PAIR_A_BUS             ( USART_BUS_1 )
    #define IT_USART_PAIR_A_TX_PIN          ( USART_TX_PIN_UNUSED )
    #define IT_USART_PAIR_A_RX_PIN          ( USART_RX_PIN_UNUSED )
    #define IT_USART_PAIR_A_TX_DMA          ( USART_TX_DMA_UNUSED )
    #define IT_USART_PAIR_A_RX_DMA          ( USART_RX_DMA_UNUSED )
    #define IT_USART_PAIR_B_BUS             ( USART_BUS_2 )
    #define IT_USART_PAIR_B_TX_PIN          ( USART_TX_PIN_UNUSED )
    #define IT_USART_PAIR_B_RX_PIN          ( USART_RX_PIN_UNUSED )
    #define IT_USART_PAIR_B_TX_DMA          ( USART_TX_DMA_UNUSED )
    #define IT_USART_PAIR_B_RX_DMA          ( USART_RX_DMA_UNUSED )
#endif /* IT_USART_PAIR_AVAILABLE */

#if !defined(IT_USART_LOOP_AVAILABLE)
    #define IT_USART_LOOP_AVAILABLE         ( 0u )
    #define IT_USART_LOOP_BUS               ( USART_BUS_2 )
    #define IT_USART_LOOP_TX_PIN            ( USART_TX_PIN_UNUSED )
    #define IT_USART_LOOP_RX_PIN            ( USART_RX_PIN_UNUSED )
    #define IT_USART_LOOP_TX_DMA            ( USART_TX_DMA_UNUSED )
    #define IT_USART_LOOP_RX_DMA            ( USART_RX_DMA_UNUSED )
#endif /* IT_USART_LOOP_AVAILABLE */

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

/** Interrupt priority of the data handling */
#define IT_USART_IRQ_PRIO                   ( 5u )


/** Baud rate of the fast pair / loop test [Bd] */
#define IT_USART_FAST_BAUDRATE              ( 921600u )

/** Byte counts of the pair / loop tests (short / long transfer, message of the idle line test) */
#define IT_USART_PAIR_SHORT                 ( 16u )
#define IT_USART_PAIR_LONG                  ( 48u )
#define IT_USART_PAIR_MSG                   ( 10u )

/** Count of attempts of a transfer disturbed by an overrun (the debug probe halts the core for a few
 *  milliseconds when it reads the result mailbox - the UART without FIFO loses bytes in polling /
 *  interrupt mode, the transfer is repeated; other errors and differences of data are not repeated).
 *  Used by the single bus tests (Bug AB#1995) and by the pair / loop tests. */
#define IT_USART_ATTEMPTS                   ( 3u )

/** First byte of the data pattern of the first link and the step between the links */
#define IT_USART_LINK_SEED_FIRST            ( 0x41u )
#define IT_USART_LINK_SEED_STEP             ( 0x40u )

/* ============================== MACROS ==================================== */

/* ========================== LOCAL VARIABLES =============================== */

/** Transmitted data */
static usart_TxData_t           itUsart_TxBuf[ IT_USART_BUF_SIZE ];

/** Received data */
static usart_RxData_t           itUsart_RxBuf[ IT_USART_BUF_SIZE ];

/** Data handling configuration (copied by the module) */
static usart_DataConfig_t       itUsart_DataConfig;

/** Bus of the test case (USART1 by default) */
static it_UsartBus_t            itUsart_Bus;

/** Receiver timeout of the bus configuration [bits], 0 - inactive */
static usart_RxTimeout_t        itUsart_RxTimeout;

/** DMA channel of the reception */
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


/** Bus, pins, DMA channels and callbacks of the links of the wiring (pair, loop) */
static const it_UsartLinkDesc_t itUsart_LinkDesc[ IT_USART_LINK_CNT ] =
{
    [ IT_USART_LINK_PAIR_A ] =
    {
        .Bus        = IT_USART_PAIR_A_BUS,
        .TxPin      = IT_USART_PAIR_A_TX_PIN,
        .RxPin      = IT_USART_PAIR_A_RX_PIN,
        .TxDma      = IT_USART_PAIR_A_TX_DMA,
        .RxDma      = IT_USART_PAIR_A_RX_DMA,
        .TxComplete = ItUsart_PairA_TxComplete,
        .RxComplete = ItUsart_PairA_RxComplete,
        .RxEnd      = ItUsart_PairA_RxEnd,
        .Error      = ItUsart_PairA_Error
    },
    [ IT_USART_LINK_PAIR_B ] =
    {
        .Bus        = IT_USART_PAIR_B_BUS,
        .TxPin      = IT_USART_PAIR_B_TX_PIN,
        .RxPin      = IT_USART_PAIR_B_RX_PIN,
        .TxDma      = IT_USART_PAIR_B_TX_DMA,
        .RxDma      = IT_USART_PAIR_B_RX_DMA,
        .TxComplete = ItUsart_PairB_TxComplete,
        .RxComplete = ItUsart_PairB_RxComplete,
        .RxEnd      = ItUsart_PairB_RxEnd,
        .Error      = ItUsart_PairB_Error
    },
    [ IT_USART_LINK_LOOP ] =
    {
        .Bus        = IT_USART_LOOP_BUS,
        .TxPin      = IT_USART_LOOP_TX_PIN,
        .RxPin      = IT_USART_LOOP_RX_PIN,
        .TxDma      = IT_USART_LOOP_TX_DMA,
        .RxDma      = IT_USART_LOOP_RX_DMA,
        .TxComplete = ItUsart_Loop_TxComplete,
        .RxComplete = ItUsart_Loop_RxComplete,
        .RxEnd      = ItUsart_Loop_RxEnd,
        .Error      = ItUsart_Loop_Error
    }
};

/** Callback records, transmitted and received data of the links */
static it_UsartLinkData_t       itUsart_Link[ IT_USART_LINK_CNT ];
static usart_TxData_t           itUsart_LinkTx[ IT_USART_LINK_CNT ][ IT_USART_BUF_SIZE ];
static usart_RxData_t           itUsart_LinkRx[ IT_USART_LINK_CNT ][ IT_USART_BUF_SIZE ];

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
    itUsart_RxDma         = USART_RX_DMA_BUS1_DMA1_CHANNEL5;

    itUsart_Bus.Bus          = IT_USART_BUS;
    itUsart_Bus.TxPin        = IT_USART_TX_PIN;
    itUsart_Bus.TxPort       = IT_USART_TX_PORT;
    itUsart_Bus.TxPinId      = IT_USART_TX_PIN_ID;
    itUsart_Bus.Oversampling = USART_OVERSAMPLING_8;
    itUsart_Bus.TxDma        = USART_TX_DMA_BUS1_DMA1_CHANNEL4;

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

    ItUsart_Init( USART_XFER_MODE_POLL, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
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

    ItUsart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
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
    ItUsart_Init( USART_XFER_MODE_POLL, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    ItUsart_Init( USART_XFER_MODE_ISR, 32u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
 * \details DMA mode (DMA1 channel 4 TX, channel 5 RX - USART1 requests of the STM32L4 mapping),
 *          one-shot RX buffer of 64 bytes. Starts reception and transmission of 64 bytes
 *          (single wire loopback).
 *
 * \par Expected results
 * - TX complete callback 1x, RX complete callback 1x, no error callback.
 * - Received data equal transmitted data.
 */
void It_Usart_Set_TxStart_DmaMode_LoopbackDataReceived( void )
{
    ItUsart_Init( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    ItUsart_Init( USART_XFER_MODE_DMA, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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

    ItUsart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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

    ItUsart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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

    ItUsart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_TIMEOUT,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    ItUsart_Init( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    ItUsart_Init( USART_XFER_MODE_ISR, 8u, USART_BUFFER_MODE_CIRCULAR, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    /* 20 bytes into 8 byte buffer: half at 4 / 12 / 20, complete at 8 / 16 */
    ItUsart_Transfer( 20u );
    ItUsart_Wait( &itUsart_RxHalfCnt, 3u );

    TEST_ASSERT_EQUAL_UINT32( 2u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 3u, itUsart_RxHalfCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 16u ], &itUsart_RxBuf[ 0u ], 4u );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( &itUsart_TxBuf[ 12u ], &itUsart_RxBuf[ 4u ], 4u );
}


/**
 * \brief   Circular DMA reception on DMA2 wraps around the buffer.
 *
 * \details DMA mode, TX DMA1 channel 4, RX DMA2 channel 7 (USART1 RX on DMA2), circular RX
 *          buffer of 8 bytes. Transfers 20 bytes.
 *
 * \par Expected results
 * - RX half callback 3x (byte 4, 12, 20), RX complete callback 2x (byte 8, 16).
 * - Buffer contains bytes 16 - 19 at index 0 - 3 and bytes 12 - 15 at index 4 - 7.
 */
void It_Usart_Set_RxStart_DmaCircularBufferDma2_WrapsAroundWithCallbacks( void )
{
    itUsart_RxDma = USART_RX_DMA_BUS1_DMA2_CHANNEL7;

    ItUsart_Init( USART_XFER_MODE_DMA, 8u, USART_BUFFER_MODE_CIRCULAR, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

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
    ItUsart_Init( USART_XFER_MODE_ISR, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_EVEN, USART_DATA_WIDTH_9 );

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

    ItUsart_Init( USART_XFER_MODE_ISR, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
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

    ItUsart_Init( USART_XFER_MODE_POLL, 4u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );
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
    ItUsart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    ItUsart_Check_FrameTime();
}

/*-------------------------------- LPUART1 -----------------------------------*/

/**
 * \brief   LPUART1 transfers data in polling and interrupt mode.
 *
 * \details LPUART1 half-duplex on IT_LPUART_TX_PIN (over-sampling 16, 115200 Bd), polling mode,
 *          16 bytes transferred, then the bus is deinitialized and initialized in
 *          interrupt mode, 16 bytes transferred.
 *
 * \par Expected results
 * - Baud rate reads back 115200 Bd +- 1 %, over-sampling 16.
 * - Both modes: TX / RX complete callback, no error callback, received data equal
 *   transmitted data.
 */
void It_Usart_Lpuart_PollAndIsrMode_LoopbackDataReceived( void )
{
    usart_Baudrate_t     baudrate     = 0u;
    usart_Oversampling_t oversampling = USART_OVERSAMPLING_8;

    itUsart_Bus.Bus          = USART_BUS_LPUART1;
    itUsart_Bus.TxPin        = IT_LPUART_TX_PIN;
    itUsart_Bus.TxPort       = IT_LPUART_TX_PORT;
    itUsart_Bus.TxPinId      = IT_LPUART_TX_PIN_ID;
    itUsart_Bus.Oversampling = USART_OVERSAMPLING_16;

    ItUsart_Init( USART_XFER_MODE_POLL, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( USART_BUS_LPUART1, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( IT_USART_BAUDRATE / 100u, IT_USART_BAUDRATE, baudrate );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Oversampling( USART_BUS_LPUART1, &oversampling ) );
    TEST_ASSERT_EQUAL( USART_OVERSAMPLING_16, oversampling );

    ItUsart_Transfer( 16u );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 16u );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Deinit( USART_BUS_LPUART1 ) );

    itUsart_TxCompleteCnt = 0u;
    itUsart_RxCompleteCnt = 0u;

    for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
    {
        itUsart_RxBuf[ byteIdx ] = 0u;
    }

    ItUsart_Init( USART_XFER_MODE_ISR, 16u, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 16u );
    ItUsart_Wait( &itUsart_RxCompleteCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 16u );
}


/**
 * \brief   LPUART1 transfers data in DMA mode with idle line end of message.
 *
 * \details LPUART1 half-duplex on IT_LPUART_TX_PIN, DMA mode (DMA2 channel 6 TX, channel 7
 *          RX), RX end mode idle, 12 bytes transferred.
 *
 * \par Expected results
 * - RX end callback 1x with 12 bytes, no error callback, received data equal transmitted
 *   data.
 */
void It_Usart_Lpuart_DmaIdleEndMode_LoopbackDataReceived( void )
{
    itUsart_Bus.Bus          = USART_BUS_LPUART1;
    itUsart_Bus.TxPin        = IT_LPUART_TX_PIN;
    itUsart_Bus.TxPort       = IT_LPUART_TX_PORT;
    itUsart_Bus.TxPinId      = IT_LPUART_TX_PIN_ID;
    itUsart_Bus.Oversampling = USART_OVERSAMPLING_16;
    itUsart_Bus.TxDma        = USART_TX_DMA_LPBUS1_DMA2_CHANNEL6;
    itUsart_RxDma            = USART_RX_DMA_LPBUS1_DMA2_CHANNEL7;

    ItUsart_Init( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_IDLE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    ItUsart_Transfer( 12u );
    ItUsart_Wait( &itUsart_RxEndCnt, 1u );

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_RxEndCnt );
    TEST_ASSERT_EQUAL_UINT16( 12u, itUsart_RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_TxBuf, itUsart_RxBuf, 12u );
}


/**
 * \brief   Real baud rate of LPUART1 matches configured baud rate.
 *
 * \details LPUART1 half-duplex on IT_LPUART_TX_PIN, interrupt mode, 115200 Bd 8N1. Measures time of
 *          50 bytes by DWT cycle counter.
 *
 * \par Expected results
 * - Transmission time = 50 x 10 bits / 115200 Bd (4340 us) +- 5 %.
 */
void It_Usart_Lpuart_Set_Baudrate_115200_FrameTimeMatchesBaudrate( void )
{
    itUsart_Bus.Bus          = USART_BUS_LPUART1;
    itUsart_Bus.TxPin        = IT_LPUART_TX_PIN;
    itUsart_Bus.TxPort       = IT_LPUART_TX_PORT;
    itUsart_Bus.TxPinId      = IT_LPUART_TX_PIN_ID;
    itUsart_Bus.Oversampling = USART_OVERSAMPLING_16;

    ItUsart_Init( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    ItUsart_Check_FrameTime();
}


/**
 * \brief   LPUART1 refuses baud rate with kernel clock / baud rate ratio 3.5 and accepts
 *          ratio 4.
 *
 * \details LPUART1 initialized (115200 Bd), kernel clock read from RCC. Sets baud rate
 *          kernel clock x 2 / 7 (ratio 3.5), then kernel clock / 4 (ratio 4).
 * \note    Device errata (ES0430 2.17.1 / ES0431 2.13.1 / ES0523 2.13.1 "Possible LPUART
 *          transmitter issue when using low BRR[15:0] value"): non-integer ratio of kernel
 *          clock and baud rate in the range 3 - 4 is refused (LPUARTDIV 0x301 - 0x3FF).
 *
 * \par Expected results
 * - Ratio 3.5: USART_REQUEST_ERROR, baud rate stays 115200 Bd +- 1 %.
 * - Ratio 4: USART_REQUEST_OK, baud rate reads back kernel clock / 4 +- 1 %.
 */
void It_Usart_Lpuart_Set_Baudrate_ErrataRatio_Refused( void )
{
    rcc_FreqHz_t     kernelClk = 0u;
    usart_Baudrate_t baudrate  = 0u;

    itUsart_Bus.Bus          = USART_BUS_LPUART1;
    itUsart_Bus.TxPin        = IT_LPUART_TX_PIN;
    itUsart_Bus.TxPort       = IT_LPUART_TX_PORT;
    itUsart_Bus.TxPinId      = IT_LPUART_TX_PIN_ID;
    itUsart_Bus.Oversampling = USART_OVERSAMPLING_16;

    ItUsart_Init( USART_XFER_MODE_NONE, IT_USART_BUF_SIZE, USART_BUFFER_MODE_ONE_SHOT, USART_RX_END_NONE,
                  USART_PARITY_NONE, USART_DATA_WIDTH_8 );

    TEST_ASSERT_EQUAL( RCC_REQUEST_OK, Rcc_Get_PeriphClk( RCC_PERIPH_LPUART1_PCLK1, &kernelClk ) );

    TEST_ASSERT_EQUAL( USART_REQUEST_ERROR, Usart_Set_Baudrate( USART_BUS_LPUART1, ( kernelClk * 2u ) / 7u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( USART_BUS_LPUART1, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( IT_USART_BAUDRATE / 100u, IT_USART_BAUDRATE, baudrate );

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_Baudrate( USART_BUS_LPUART1, kernelClk / 4u ) );
    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_Baudrate( USART_BUS_LPUART1, &baudrate ) );
    TEST_ASSERT_UINT32_WITHIN( ( kernelClk / 4u ) / 100u, kernelClk / 4u, baudrate );
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

    ItUsart_Transfer_Link( IT_USART_LINK_PAIR_A, IT_USART_LINK_PAIR_B, IT_USART_PAIR_SHORT );
    ItUsart_Transfer_Link( IT_USART_LINK_PAIR_B, IT_USART_LINK_PAIR_A, IT_USART_PAIR_SHORT );
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
 * \details Both buses in DMA mode (bus A DMA1 channels 2 / 3, bus B DMA2 channels 1 / 2),
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
    const usart_PeriphId_t busA = itUsart_LinkDesc[ IT_USART_LINK_PAIR_A ].Bus;
    const usart_PeriphId_t busB = itUsart_LinkDesc[ IT_USART_LINK_PAIR_B ].Bus;

    ItUsart_RequirePair();

    ItUsart_Init_Pair( USART_XFER_MODE_ISR, IT_USART_BUF_SIZE, USART_RX_END_IDLE, IT_USART_BAUDRATE );

    for( uint32_t attempt = 0u; IT_USART_ATTEMPTS > attempt; attempt++ )
    {
        ItUsart_Clear_Link( IT_USART_LINK_PAIR_A );
        ItUsart_Clear_Link( IT_USART_LINK_PAIR_B );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( busB ) );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( busA,
                                                                itUsart_LinkTx[ IT_USART_LINK_PAIR_A ],
                                                                IT_USART_PAIR_MSG ) );
        ItUsart_Wait( &itUsart_Link[ IT_USART_LINK_PAIR_B ].RxEndCnt, 1u );

        const usart_FlagState_t overrunA = ItUsart_Get_Overrun( IT_USART_LINK_PAIR_A );
        const usart_FlagState_t overrunB = ItUsart_Get_Overrun( IT_USART_LINK_PAIR_B );

        if( ( USART_FLAG_ACTIVE == overrunA ) ||
            ( USART_FLAG_ACTIVE == overrunB )    )
        {
            /* Bytes lost while the core was halted - the reception is stopped, the message repeated */
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( busB ) );
        }
        else
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_Link[ IT_USART_LINK_PAIR_A ].TxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 1u, itUsart_Link[ IT_USART_LINK_PAIR_B ].RxEndCnt );
    TEST_ASSERT_EQUAL_UINT32( IT_USART_PAIR_MSG, itUsart_Link[ IT_USART_LINK_PAIR_B ].RxEndBytes );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Link[ IT_USART_LINK_PAIR_B ].RxCompleteCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Link[ IT_USART_LINK_PAIR_A ].ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Link[ IT_USART_LINK_PAIR_B ].ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_LinkTx[ IT_USART_LINK_PAIR_A ],
                                  itUsart_LinkRx[ IT_USART_LINK_PAIR_B ],
                                  IT_USART_PAIR_MSG );
}

/*------------------------------ UART loop (UART_LOOP) -----------------------*/

/**
 * \brief   TX of one bus wired to its RX, polling mode.
 *
 * \details Bus of the loop in full duplex polling mode (115200 Bd, 8N1, one-shot RX buffer of
 *          16 bytes). Reception and transmission of 16 bytes started, Usart_Task() called while waiting.
 *
 * \par Expected results
 * - TX complete and RX complete callback once, no error.
 * - Received data equal the transmitted data.
 */
void It_Usart_Loop_Poll_DataReceived( void )
{
    ItUsart_RequireLoop();

    ItUsart_Init_Loop( USART_XFER_MODE_POLL, IT_USART_PAIR_SHORT, IT_USART_BAUDRATE );

    ItUsart_Transfer_Link( IT_USART_LINK_LOOP, IT_USART_LINK_LOOP, IT_USART_PAIR_SHORT );
}


/**
 * \brief   TX of one bus wired to its RX, interrupt mode.
 *
 * \details Bus of the loop in full duplex interrupt mode (115200 Bd, 8N1, one-shot RX buffer of
 *          48 bytes). Reception and transmission of 48 bytes started.
 *
 * \par Expected results
 * - TX complete and RX complete callback once, no error.
 * - Received data equal the transmitted data.
 */
void It_Usart_Loop_Isr_DataReceived( void )
{
    ItUsart_RequireLoop();

    ItUsart_Init_Loop( USART_XFER_MODE_ISR, IT_USART_PAIR_LONG, IT_USART_BAUDRATE );

    ItUsart_Transfer_Link( IT_USART_LINK_LOOP, IT_USART_LINK_LOOP, IT_USART_PAIR_LONG );
}


/**
 * \brief   TX of one bus wired to its RX, DMA mode at 921600 Bd.
 *
 * \details Bus of the loop in full duplex DMA mode (DMA1 channels 7 / 6), 921600 Bd, one-shot RX
 *          buffer of 64 bytes. Reception and transmission of 64 bytes started.
 *
 * \par Expected results
 * - TX complete and RX complete callback once, no error.
 * - Received data equal the transmitted data.
 */
void It_Usart_Loop_DmaHighBaudrate_DataReceived( void )
{
    ItUsart_RequireLoop();

    ItUsart_Init_Loop( USART_XFER_MODE_DMA, IT_USART_BUF_SIZE, IT_USART_FAST_BAUDRATE );

    ItUsart_Transfer_Link( IT_USART_LINK_LOOP, IT_USART_LINK_LOOP, IT_USART_BUF_SIZE );
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
static void ItUsart_Init( usart_XferMode_t xferMode, usart_RxDataCnt_t rxSize, usart_BufferMode_t bufferMode,
                          usart_RxEndMode_t rxEndMode, usart_Parity_t parity, usart_DataWidth_t dataWidth )
{
    usart_BusConfig_t busConfig;

    itUsart_DataConfig.TxMode             = xferMode;
    itUsart_DataConfig.RxMode             = xferMode;
    itUsart_DataConfig.RxBuffer           = itUsart_RxBuf;
    itUsart_DataConfig.RxBufferSize       = rxSize;
    itUsart_DataConfig.RxBufferMode       = bufferMode;
    itUsart_DataConfig.RxEndMode          = rxEndMode;
    itUsart_DataConfig.TxDma              = itUsart_Bus.TxDma;
    itUsart_DataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    itUsart_DataConfig.RxDma              = itUsart_RxDma;
    itUsart_DataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    itUsart_DataConfig.IrqPriority        = IT_USART_IRQ_PRIO;
    itUsart_DataConfig.TxCompleteCallback = ItUsart_TxCompleteCallback;
    itUsart_DataConfig.RxHalfCallback     = ItUsart_RxHalfCallback;
    itUsart_DataConfig.RxCompleteCallback = ItUsart_RxCompleteCallback;
    itUsart_DataConfig.RxEndCallback      = ItUsart_RxEndCallback;
    itUsart_DataConfig.ErrorCallback      = ItUsart_ErrorCallback;

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
     * (external on the board in the application, RM0440 single-wire half-duplex mode) */
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
 *        and waits for the end of the transmission. A transfer disturbed by an overrun (the debug
 *        probe halts the core when it reads the result mailbox, the UART has no receive FIFO) is
 *        repeated at most \ref IT_USART_ATTEMPTS times (reception stopped, callback counters and
 *        receive buffer cleared); the last attempt is not cleared, so its error fails the test
 *        and other errors are not repeated.
 *
 * \param txSize [in]: Count of transmitted bytes
 */
static void ItUsart_Transfer( usart_TxDataCnt_t txSize )
{
    const uint32_t lastAttempt = IT_USART_ATTEMPTS - 1u;

    for( uint32_t attempt = 0u; IT_USART_ATTEMPTS > attempt; attempt++ )
    {
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( itUsart_Bus.Bus ) );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( itUsart_Bus.Bus, itUsart_TxBuf, txSize ) );

        ItUsart_Wait( &itUsart_TxCompleteCnt, 1u );

        const usart_FlagState_t overrun = ItUsart_Get_BusOverrun();

        if( ( USART_FLAG_ACTIVE == overrun     ) &&
            ( lastAttempt        > attempt     )    )
        {
            TEST_MESSAGE( "Overrun - bytes lost while the core was halted, transfer repeated" );
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( itUsart_Bus.Bus ) );
            ItUsart_Clear_Counters();
        }
        else
        {
            break;
        }
    }
}


/**
 * \brief Clears the callback counters and the receive buffer of the bus of the test case.
 */
static void ItUsart_Clear_Counters( void )
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
        itUsart_RxBuf[ byteIdx ] = 0u;
    }
}


/**
 * \brief Returns whether the last error callback of the bus of the test case reported an overrun.
 *
 * \return USART_FLAG_ACTIVE if an overrun was reported (data lost), otherwise USART_FLAG_INACTIVE
 */
static usart_FlagState_t ItUsart_Get_BusOverrun( void )
{
    usart_FlagState_t overrun = USART_FLAG_INACTIVE;

    if( ( 0u                       != itUsart_ErrorCnt ) &&
        ( USART_XFER_ERROR_OVERRUN == itUsart_ErrorId  )    )
    {
        overrun = USART_FLAG_ACTIVE;
    }
    else
    {
        /* No error or an error other than the overrun */
    }

    return ( overrun );
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
    dwt_CounterValue_t       elapsedCnt = Dwt_Get_CycleCounter() - startCnt;

    for( uint32_t loopIdx = 0u;
         ( ( waitCycles / IT_USART_MIN_CYCLES_PER_PASS ) > loopIdx    ) &&
         ( waitCycles                                    > elapsedCnt );
         loopIdx++ )
    {
        /* Busy wait */
        elapsedCnt = Dwt_Get_CycleCounter() - startCnt;
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
 * \brief   Ignores the test on a board whose UART loop is not wired.
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
    /* TX of the bus is wired to its RX */
#else
    TEST_IGNORE_MESSAGE( "Board without the fixture UART_LOOP (TX wired to RX of one UART)" );
#endif
}


/**
 * \brief Fills the transmit buffers of the links with different patterns and clears the receive
 *        buffers.
 */
static void ItUsart_Fill_Patterns( void )
{
    for( uint32_t linkIdx = 0u; (uint32_t)IT_USART_LINK_CNT > linkIdx; linkIdx++ )
    {
        const uint32_t seed = IT_USART_LINK_SEED_FIRST + ( linkIdx * IT_USART_LINK_SEED_STEP );

        for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
        {
            itUsart_LinkTx[ linkIdx ][ byteIdx ] = (usart_TxData_t)( seed + byteIdx );
            itUsart_LinkRx[ linkIdx ][ byteIdx ] = 0u;
        }
    }
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
    ItUsart_Fill_Patterns();

    ItUsart_Init_Link( IT_USART_LINK_PAIR_A, xferMode, rxSize, rxEndMode, baudrate );
    ItUsart_Init_Link( IT_USART_LINK_PAIR_B, xferMode, rxSize, rxEndMode, baudrate );

    /* The RX line of the bus initialized first floats until the TX of the other bus is initialized - a frame
     * possibly started on it is finished before the reception start (Usart_Set_RxStart discards stale data) */
    for( volatile uint32_t loopIdx = 0u; IT_USART_LINE_SETTLE_LOOPS > loopIdx; loopIdx++ )
    {
        /* Busy wait */
    }
}


/**
 * \brief Initializes the bus of the loop (full duplex, 8N1) with data handling and fills the
 *        transmit buffers with different patterns.
 *
 * \param xferMode [in]: Transmission and reception mode
 * \param rxSize   [in]: Reception buffer size (one-shot)
 * \param baudrate [in]: Baud rate [Bd]
 */
static void ItUsart_Init_Loop( usart_XferMode_t xferMode,
                               usart_RxDataCnt_t rxSize,
                               usart_Baudrate_t baudrate )
{
    ItUsart_Fill_Patterns();

    ItUsart_Init_Link( IT_USART_LINK_LOOP, xferMode, rxSize, USART_RX_END_NONE, baudrate );
}


/**
 * \brief Initializes the bus of one link (full duplex, 8N1) with the pins and the DMA channels of
 *        the board and the data handling of both directions.
 *
 * \param link      [in]: Link of the wiring
 * \param xferMode  [in]: Transmission and reception mode
 * \param rxSize    [in]: Reception buffer size (one-shot)
 * \param rxEndMode [in]: End of message detection
 * \param baudrate  [in]: Baud rate [Bd]
 */
static void ItUsart_Init_Link( it_UsartLink_t link,
                               usart_XferMode_t xferMode,
                               usart_RxDataCnt_t rxSize,
                               usart_RxEndMode_t rxEndMode,
                               usart_Baudrate_t baudrate )
{
    const it_UsartLinkDesc_t * const desc = &itUsart_LinkDesc[ link ];
    it_UsartLinkData_t * const       data = &itUsart_Link[ link ];
    usart_BusConfig_t                busConfig;

    data->TxCompleteCnt = 0u;
    data->RxCompleteCnt = 0u;
    data->RxEndCnt      = 0u;
    data->RxEndBytes    = 0u;
    data->ErrorCnt      = 0u;
    data->ErrorId       = USART_XFER_ERROR_CNT;

    data->DataConfig.TxMode             = xferMode;
    data->DataConfig.RxMode             = xferMode;
    data->DataConfig.RxBuffer           = itUsart_LinkRx[ link ];
    data->DataConfig.RxBufferSize       = rxSize;
    data->DataConfig.RxBufferMode       = USART_BUFFER_MODE_ONE_SHOT;
    data->DataConfig.RxEndMode          = rxEndMode;
    data->DataConfig.TxDma              = desc->TxDma;
    data->DataConfig.TxDmaPriority      = USART_DMA_PRIORITY_LOW;
    data->DataConfig.RxDma              = desc->RxDma;
    data->DataConfig.RxDmaPriority      = USART_DMA_PRIORITY_HIGH;
    data->DataConfig.IrqPriority        = IT_USART_IRQ_PRIO;
    data->DataConfig.TxCompleteCallback = desc->TxComplete;
    data->DataConfig.RxHalfCallback     = USART_NULL_PTR;
    data->DataConfig.RxCompleteCallback = desc->RxComplete;
    data->DataConfig.RxEndCallback      = desc->RxEnd;
    data->DataConfig.ErrorCallback      = desc->Error;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Get_DefaultConfig( &busConfig ) );
    busConfig.PeriphId   = desc->Bus;
    busConfig.BaudRate   = baudrate;
    busConfig.DataWidth  = USART_DATA_WIDTH_8;
    busConfig.Parity     = USART_PARITY_NONE;
    busConfig.HalfDuplex = USART_HALF_DUPLEX_INACTIVE;
    busConfig.BusTxPin   = desc->TxPin;
    busConfig.BusRxPin   = desc->RxPin;
    busConfig.DataConfig = &data->DataConfig;

    TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Init( &busConfig ) );
}


/**
 * \brief Clears the callback counters and the receive buffer of a link.
 *
 * \param link [in]: Link of the wiring
 */
static void ItUsart_Clear_Link( it_UsartLink_t link )
{
    it_UsartLinkData_t * const data = &itUsart_Link[ link ];

    data->TxCompleteCnt = 0u;
    data->RxCompleteCnt = 0u;
    data->RxEndCnt      = 0u;
    data->RxEndBytes    = 0u;
    data->ErrorCnt      = 0u;
    data->ErrorId       = USART_XFER_ERROR_CNT;

    for( uint32_t byteIdx = 0u; IT_USART_BUF_SIZE > byteIdx; byteIdx++ )
    {
        itUsart_LinkRx[ link ][ byteIdx ] = 0u;
    }
}


/**
 * \brief Returns whether the last error callback of a link reported an overrun.
 *
 * \param link [in]: Link of the wiring
 *
 * \return USART_FLAG_ACTIVE if an overrun was reported (data lost), otherwise USART_FLAG_INACTIVE
 */
static usart_FlagState_t ItUsart_Get_Overrun( it_UsartLink_t link )
{
    usart_FlagState_t overrun = USART_FLAG_INACTIVE;

    if( ( 0u                       != itUsart_Link[ link ].ErrorCnt ) &&
        ( USART_XFER_ERROR_OVERRUN == itUsart_Link[ link ].ErrorId  )    )
    {
        overrun = USART_FLAG_ACTIVE;
    }
    else
    {
        /* No error or an error other than the overrun */
    }

    return ( overrun );
}


/**
 * \brief Transfers the first bytes of the transmit buffer of one link to the receive buffer of
 *        a link (the same link for the loop) and checks the callbacks and the data. A transfer
 *        disturbed by an overrun is repeated (\ref IT_USART_ATTEMPTS).
 *
 * \param txLink [in]: Transmitting link
 * \param rxLink [in]: Receiving link
 * \param txSize [in]: Count of bytes (equal to the RX buffer size)
 */
static void ItUsart_Transfer_Link( it_UsartLink_t txLink,
                                   it_UsartLink_t rxLink,
                                   usart_TxDataCnt_t txSize )
{
    const usart_PeriphId_t txId = itUsart_LinkDesc[ txLink ].Bus;
    const usart_PeriphId_t rxId = itUsart_LinkDesc[ rxLink ].Bus;

    for( uint32_t attempt = 0u; IT_USART_ATTEMPTS > attempt; attempt++ )
    {
        ItUsart_Clear_Link( txLink );
        ItUsart_Clear_Link( rxLink );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( rxId ) );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( txId, itUsart_LinkTx[ txLink ], txSize ) );
        ItUsart_Wait( &itUsart_Link[ rxLink ].RxCompleteCnt, 1u );

        const usart_FlagState_t txOverrun = ItUsart_Get_Overrun( txLink );
        const usart_FlagState_t rxOverrun = ItUsart_Get_Overrun( rxLink );

        if( ( USART_FLAG_ACTIVE == txOverrun ) ||
            ( USART_FLAG_ACTIVE == rxOverrun )    )
        {
            /* Bytes lost while the core was halted - the reception is stopped, the transfer repeated */
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( rxId ) );
        }
        else
        {
            break;
        }
    }

    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Link[ txLink ].TxCompleteCnt, "TX not complete" );
    TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Link[ rxLink ].RxCompleteCnt, "RX not complete" );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Link[ txLink ].ErrorCnt );
    TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Link[ rxLink ].ErrorCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_LinkTx[ txLink ], itUsart_LinkRx[ rxLink ], txSize );
}


/**
 * \brief Full duplex exchange of the pair: reception of both buses started, both transmit, checks
 *        the callbacks and the data of both directions. An exchange disturbed by an overrun is
 *        repeated (\ref IT_USART_ATTEMPTS).
 *
 * \param dataCnt [in]: Count of bytes per direction (equal to the RX buffer size)
 */
static void ItUsart_Exchange_Pair( usart_TxDataCnt_t dataCnt )
{
    const usart_PeriphId_t busA = itUsart_LinkDesc[ IT_USART_LINK_PAIR_A ].Bus;
    const usart_PeriphId_t busB = itUsart_LinkDesc[ IT_USART_LINK_PAIR_B ].Bus;

    for( uint32_t attempt = 0u; IT_USART_ATTEMPTS > attempt; attempt++ )
    {
        ItUsart_Clear_Link( IT_USART_LINK_PAIR_A );
        ItUsart_Clear_Link( IT_USART_LINK_PAIR_B );

        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( busA ) );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStart( busB ) );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( busA,
                                                                itUsart_LinkTx[ IT_USART_LINK_PAIR_A ],
                                                                dataCnt ) );
        TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_TxStart( busB,
                                                                itUsart_LinkTx[ IT_USART_LINK_PAIR_B ],
                                                                dataCnt ) );
        ItUsart_Wait( &itUsart_Link[ IT_USART_LINK_PAIR_A ].RxCompleteCnt, 1u );
        ItUsart_Wait( &itUsart_Link[ IT_USART_LINK_PAIR_B ].RxCompleteCnt, 1u );

        const usart_FlagState_t overrunA = ItUsart_Get_Overrun( IT_USART_LINK_PAIR_A );
        const usart_FlagState_t overrunB = ItUsart_Get_Overrun( IT_USART_LINK_PAIR_B );

        if( ( USART_FLAG_ACTIVE == overrunA ) ||
            ( USART_FLAG_ACTIVE == overrunB )    )
        {
            /* Bytes lost while the core was halted - the receptions are stopped, the exchange repeated */
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( busA ) );
            TEST_ASSERT_EQUAL( USART_REQUEST_OK, Usart_Set_RxStop( busB ) );
        }
        else
        {
            break;
        }
    }

    for( uint32_t linkIdx = (uint32_t)IT_USART_LINK_PAIR_A; (uint32_t)IT_USART_LINK_LOOP > linkIdx; linkIdx++ )
    {
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Link[ linkIdx ].TxCompleteCnt, "TX not complete" );
        TEST_ASSERT_EQUAL_UINT32_MESSAGE( 1u, itUsart_Link[ linkIdx ].RxCompleteCnt, "RX not complete" );
        TEST_ASSERT_EQUAL_UINT32( 0u, itUsart_Link[ linkIdx ].ErrorCnt );
    }

    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_LinkTx[ IT_USART_LINK_PAIR_A ],
                                  itUsart_LinkRx[ IT_USART_LINK_PAIR_B ],
                                  dataCnt );
    TEST_ASSERT_EQUAL_HEX8_ARRAY( itUsart_LinkTx[ IT_USART_LINK_PAIR_B ],
                                  itUsart_LinkRx[ IT_USART_LINK_PAIR_A ],
                                  dataCnt );
}


/** \brief Bus A of the pair: transmission complete callback */
static void ItUsart_PairA_TxComplete( void )
{
    itUsart_Link[ IT_USART_LINK_PAIR_A ].TxCompleteCnt++;
}


/** \brief Bus A of the pair: reception buffer filled callback */
static void ItUsart_PairA_RxComplete( void )
{
    itUsart_Link[ IT_USART_LINK_PAIR_A ].RxCompleteCnt++;
}


/**
 * \brief Bus A of the pair: end of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void ItUsart_PairA_RxEnd( usart_RxDataCnt_t rxCnt )
{
    itUsart_Link[ IT_USART_LINK_PAIR_A ].RxEndBytes = rxCnt;
    itUsart_Link[ IT_USART_LINK_PAIR_A ].RxEndCnt++;
}


/**
 * \brief Bus A of the pair: transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItUsart_PairA_Error( usart_XferErrorId_t errorId )
{
    itUsart_Link[ IT_USART_LINK_PAIR_A ].ErrorId = errorId;
    itUsart_Link[ IT_USART_LINK_PAIR_A ].ErrorCnt++;
}


/** \brief Bus B of the pair: transmission complete callback */
static void ItUsart_PairB_TxComplete( void )
{
    itUsart_Link[ IT_USART_LINK_PAIR_B ].TxCompleteCnt++;
}


/** \brief Bus B of the pair: reception buffer filled callback */
static void ItUsart_PairB_RxComplete( void )
{
    itUsart_Link[ IT_USART_LINK_PAIR_B ].RxCompleteCnt++;
}


/**
 * \brief Bus B of the pair: end of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void ItUsart_PairB_RxEnd( usart_RxDataCnt_t rxCnt )
{
    itUsart_Link[ IT_USART_LINK_PAIR_B ].RxEndBytes = rxCnt;
    itUsart_Link[ IT_USART_LINK_PAIR_B ].RxEndCnt++;
}


/**
 * \brief Bus B of the pair: transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItUsart_PairB_Error( usart_XferErrorId_t errorId )
{
    itUsart_Link[ IT_USART_LINK_PAIR_B ].ErrorId = errorId;
    itUsart_Link[ IT_USART_LINK_PAIR_B ].ErrorCnt++;
}


/** \brief Bus of the loop: transmission complete callback */
static void ItUsart_Loop_TxComplete( void )
{
    itUsart_Link[ IT_USART_LINK_LOOP ].TxCompleteCnt++;
}


/** \brief Bus of the loop: reception buffer filled callback */
static void ItUsart_Loop_RxComplete( void )
{
    itUsart_Link[ IT_USART_LINK_LOOP ].RxCompleteCnt++;
}


/**
 * \brief Bus of the loop: end of received message callback.
 *
 * \param rxCnt [in]: Count of received bytes
 */
static void ItUsart_Loop_RxEnd( usart_RxDataCnt_t rxCnt )
{
    itUsart_Link[ IT_USART_LINK_LOOP ].RxEndBytes = rxCnt;
    itUsart_Link[ IT_USART_LINK_LOOP ].RxEndCnt++;
}


/**
 * \brief Bus of the loop: transfer error callback.
 *
 * \param errorId [in]: Error identification
 */
static void ItUsart_Loop_Error( usart_XferErrorId_t errorId )
{
    itUsart_Link[ IT_USART_LINK_LOOP ].ErrorId = errorId;
    itUsart_Link[ IT_USART_LINK_LOOP ].ErrorCnt++;
}
