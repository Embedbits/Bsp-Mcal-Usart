/**
 * \defgroup Usart Usart
 * \brief Usart module
 */

/**
 * \author Mr.Nobody
 * \file Usart_Types.h
 * \ingroup Usart
 * \brief Universal Synchronous-Asynchronous Receiver Transmitter (USART)
 *        module types definitions
 *
 * This file contains the types definitions used across the module and are
 * available for other modules through Port file.
 *
 * \note STM32F4 USART does not support receiver timeout, Driver Enable (DE),
 *       pin level inversion and 7 bit word length. Corresponding types are kept
 *       for compatibility of the public interface (STM32H5), only inactive /
 *       standard configuration is accepted by the module.
 *
 */

#ifndef USART_USART_TYPES_H
#define USART_USART_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Dma_Types.h"                      /* DMA types definitions          */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define USART_NULL_PTR                        ( ( void* ) 0u )

/** Maximum count of bits used for RX timeout feature (feature not available on STM32F4) */
#define USART_RX_TIMEOUT_MAX                  ( 0x00FFFFFF )

/** Minimum count of bits used for RX timeout feature (feature inactive) */
#define USART_RX_TIMEOUT_MIN                  ( 0x00 )

/** Peripheral identification bit offset in encoded value */
#define USART_BIT_MASK_PERIPH_BIT_OFFSET      ( 15u )

/** Port identification bit offset in encoded value */
#define USART_BIT_MASK_PORT_BIT_OFFSET        ( 10u )

/** Pin identification bit offset in encoded value */
#define USART_BIT_MASK_PIN_BIT_OFFSET         ( 5u )

/** Alternative function identification bit offset in encoded value */
#define USART_BIT_MASK_AF_BIT_OFFSET          ( 0u )

/** USART / UART bus identification bit offset in encoded DMA stream value */
#define USART_DMA_BIT_MASK_PERIPH_BIT_OFFSET  ( 15u )

/** DMA peripheral identification bit offset in encoded DMA stream value */
#define USART_DMA_BIT_MASK_DMA_BIT_OFFSET     ( 10u )

/** Stream identification bit offset in encoded DMA stream value */
#define USART_DMA_BIT_MASK_STREAM_BIT_OFFSET  ( 5u )

/** Channel selection (CHSEL) bit offset in encoded DMA stream value */
#define USART_DMA_BIT_MASK_CHSEL_BIT_OFFSET   ( 0u )

/* ========================== EXPORTED MACROS =============================== */

/** Encode channel pin configuration into single 16bit bit-mask */
#define USART_PIN_BIT_MASK_ENCODE(PERIPH_ID,PORT_ID,PIN_ID,AF_ID)   ( ( PERIPH_ID  << USART_BIT_MASK_PERIPH_BIT_OFFSET  ) | \
                                                                      ( PORT_ID    << USART_BIT_MASK_PORT_BIT_OFFSET    ) | \
                                                                      ( PIN_ID     << USART_BIT_MASK_PIN_BIT_OFFSET     ) | \
                                                                      ( AF_ID      << USART_BIT_MASK_AF_BIT_OFFSET      )   )

/** Extract Peripheral ID from encoded value */
#define USART_BIT_MASK_DECODE_PERIPH(CODED_VAL)               ( ( CODED_VAL >> USART_BIT_MASK_PERIPH_BIT_OFFSET ) & 0x1F )

/** Extract port ID from encoded value */
#define USART_BIT_MASK_DECODE_PORT(CODED_VAL)                 ( ( CODED_VAL >> USART_BIT_MASK_PORT_BIT_OFFSET ) & 0x1F )

/** Extract pin ID from encoded value */
#define USART_BIT_MASK_DECODE_PIN(CODED_VAL)                  ( ( CODED_VAL >> USART_BIT_MASK_PIN_BIT_OFFSET ) & 0x1F )

/** Extract alternative function ID from encoded value */
#define USART_BIT_MASK_DECODE_AF(CODED_VAL)                   ( ( CODED_VAL >> USART_BIT_MASK_AF_BIT_OFFSET ) & 0x1F )

/**
 * \brief Encodes DMA stream (USART / UART bus, DMA peripheral, stream, channel selection) into single
 *        value of \ref usart_DmaCode_t
 *
 * The macro defines the values of the DMA stream lists \ref usart_TxDma_t and \ref usart_RxDma_t, e.g. the
 * USART1 transmit request on DMA2 stream 7 (channel selection 4) is \ref USART_TX_DMA_BUS1_DMA2_STREAM7:
 * USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, 4u )
 */
#define USART_DMA_ENCODE( BUS_ID, DMA_ID, STREAM_ID, CHSEL )   ( (usart_DmaCode_t)( ( (uint32_t)(BUS_ID) << USART_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) | \
                                                                                    ( (uint32_t)(DMA_ID)    << USART_DMA_BIT_MASK_DMA_BIT_OFFSET    ) | \
                                                                                    ( (uint32_t)(STREAM_ID) << USART_DMA_BIT_MASK_STREAM_BIT_OFFSET ) | \
                                                                                    ( (uint32_t)(CHSEL)     << USART_DMA_BIT_MASK_CHSEL_BIT_OFFSET  )   ) )

/** Stream is not configured by the module (value of the *_DMA_UNUSED items of the DMA stream lists) */
#define USART_DMA_CODE_UNUSED                 USART_DMA_ENCODE( USART_BUS_CNT, USART_DMA_PERIPH_CNT, USART_DMA_CHANNEL_CNT, 0u )

/** Extract USART / UART bus ID from encoded DMA stream value */
#define USART_DMA_BIT_MASK_DECODE_PERIPH( CODED_VAL ) ( ( (CODED_VAL) >> USART_DMA_BIT_MASK_PERIPH_BIT_OFFSET ) & 0x1Fu )

/** Extract DMA peripheral ID from encoded DMA stream value */
#define USART_DMA_BIT_MASK_DECODE_DMA( CODED_VAL )    ( ( (CODED_VAL) >> USART_DMA_BIT_MASK_DMA_BIT_OFFSET ) & 0x1Fu )

/** Extract stream ID from encoded DMA stream value */
#define USART_DMA_BIT_MASK_DECODE_STREAM( CODED_VAL ) ( ( (CODED_VAL) >> USART_DMA_BIT_MASK_STREAM_BIT_OFFSET ) & 0x1Fu )

/** Extract channel selection (CHSEL) from encoded DMA stream value */
#define USART_DMA_BIT_MASK_DECODE_CHSEL( CODED_VAL )  ( ( (CODED_VAL) >> USART_DMA_BIT_MASK_CHSEL_BIT_OFFSET ) & 0x1Fu )

/* ============================== TYPEDEFS ================================== */

/** \brief Type signaling major version of SW module */
typedef uint8_t usart_MajorVersion_t;


/** \brief Type signaling minor version of SW module */
typedef uint8_t usart_MinorVersion_t;


/** \brief Type signaling patch version of SW module */
typedef uint8_t usart_PatchVersion_t;


/** \brief Type signaling actual version of SW module */
typedef struct
{
    usart_MajorVersion_t Major; /**< Major version */
    usart_MinorVersion_t Minor; /**< Minor version */
    usart_PatchVersion_t Patch; /**< Patch version */
}   usart_ModuleVersion_t;


/** Function status enumeration */
typedef enum
{
    USART_FUNCTION_INACTIVE = 0u, /**< Function status is inactive */
    USART_FUNCTION_ACTIVE         /**< Function status is active   */
}   usart_FunctionState_t;


/** Flag states enumeration */
typedef enum
{
    USART_FLAG_INACTIVE = 0u, /**< Inactive flag state */
    USART_FLAG_ACTIVE         /**< Active flag state   */
}   usart_FlagState_t;


/** Enumeration used to signal request processing state */
typedef enum
{
    USART_REQUEST_ERROR = 0u, /**< Processing request failed  */
    USART_REQUEST_OK          /**< Processing request succeed */
}   usart_RequestState_t;


/** USART/UART baud-rate value representation */
typedef uint32_t usart_Baudrate_t;

/** Frequency values type represented in Hz */
typedef uint32_t usart_FreqHz_t;

/** Receiver timeout value in terms of number of bits during which there is
 * no activity on the RX line (feature not available on STM32F4). */
typedef uint32_t usart_RxTimeout_t;

/** Transmit register address */
typedef uint32_t usart_TxRegAddr_t;

/** Receive register address */
typedef uint32_t usart_RxRegAddr_t;

/** Type representing transmit data */
typedef uint8_t usart_TxData_t;

/** Type representing receive data */
typedef uint8_t usart_RxData_t;

/** Driver Enable (DE) pin assertion time representation */
typedef uint8_t usart_AssertTime_us_t;

/** Driver Enable (DE) pin de-assertion time representation */
typedef uint8_t usart_DeassertTime_us_t;

/** Type representing count of transmitted data */
typedef uint16_t usart_TxDataCnt_t;

/** Type representing count of received data */
typedef uint16_t usart_RxDataCnt_t;

/** Type representing address of memory where received data shall be stored */
typedef uint32_t usart_RxDataAddr_t;

/** Type representing address of memory from which data shall be transmitted */
typedef uint32_t usart_TxDataAddr_t;

/** \brief Interrupt priority type definition */
typedef uint32_t usart_IrqPrio_t;

/** \brief Encoded DMA stream (value of \ref usart_TxDma_t or \ref usart_RxDma_t) */
typedef uint32_t usart_DmaCode_t;


/** \brief USART/UART bus identification */
typedef enum
{
#ifdef USART1
    USART_BUS_1 = 0u, /**< USART bus 1 ID */
#endif
#ifdef USART2
    USART_BUS_2,      /**< USART bus 2 ID */
#endif
#ifdef USART3
    USART_BUS_3,      /**< USART bus 3 ID */
#endif
#ifdef UART4
    USART_BUS_4,      /**< UART bus 4 ID  */
#endif
#ifdef UART5
    USART_BUS_5,      /**< UART bus 5 ID  */
#endif
#ifdef USART6
    USART_BUS_6,      /**< USART bus 6 ID */
#endif
#ifdef UART7
    USART_BUS_7,      /**< UART bus 7 ID  */
#endif
#ifdef UART8
    USART_BUS_8,      /**< UART bus 8 ID  */
#endif
    USART_BUS_CNT
}   usart_PeriphId_t;


/** \brief List of RX pins available for USART/UART peripherals */
typedef enum
{
#if !defined(STM32F410Tx)
    USART_RX_PIN_BUS1_PA10  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART1 RX pin connected to PA10  */
#endif
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
    USART_RX_PIN_BUS1_PB3          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1  , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_7  ), /**< USART1 RX pin connected to PB3 */
#endif
    USART_RX_PIN_BUS1_PB7   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_B   , GPIO_PIN_ID_7   , GPIO_ALT_FUNC_7   ), /**< USART1 RX pin connected to PB7   */

    USART_RX_PIN_BUS2_PA3   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PA3   */
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F410Tx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_RX_PIN_BUS2_PD6   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_6   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PD6   */
#endif

#if defined(USART3)
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_RX_PIN_BUS3_PB11  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PB11  */
#endif
#if defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx) || \
    defined(STM32F446xx)
    USART_RX_PIN_BUS3_PC5          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3  , GPIO_PORT_C   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_7  ), /**< USART3 RX pin connected to PC5 */
#endif
#if !defined(STM32F412Cx)
    USART_RX_PIN_BUS3_PC11  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_C   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PC11  */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_RX_PIN_BUS3_PD9   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PD9   */
#endif
#endif /* USART3 */

#if defined(UART4)
    USART_RX_PIN_BUS4_PA1   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_A   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_8   ), /**< UART4 RX pin connected to PA1    */
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_RX_PIN_BUS4_PA11         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4  , GPIO_PORT_A   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_11 ), /**< UART4 RX pin connected to PA11 */
#endif
    USART_RX_PIN_BUS4_PC11  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_C   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_8   ), /**< UART4 RX pin connected to PC11   */
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_RX_PIN_BUS4_PD0          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4  , GPIO_PORT_D   , GPIO_PIN_ID_0  , GPIO_ALT_FUNC_11 ), /**< UART4 RX pin connected to PD0 */
#endif
#endif /* UART4 */

#if defined(UART5)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_RX_PIN_BUS5_PB5          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_B   , GPIO_PIN_ID_5  , GPIO_ALT_FUNC_11 ), /**< UART5 RX pin connected to PB5 */
    USART_RX_PIN_BUS5_PB8          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_B   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_11 ), /**< UART5 RX pin connected to PB8 */
    USART_RX_PIN_BUS5_PB12         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_B   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_11 ), /**< UART5 RX pin connected to PB12 */
#endif
    USART_RX_PIN_BUS5_PD2   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_D   , GPIO_PIN_ID_2   , GPIO_ALT_FUNC_8   ), /**< UART5 RX pin connected to PD2    */
#if defined(STM32F446xx)
    USART_RX_PIN_BUS5_PE7          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_E   , GPIO_PIN_ID_7  , GPIO_ALT_FUNC_8  ), /**< UART5 RX pin connected to PE7 */
#endif
#endif /* UART5 */

#if defined(USART6)
#if defined(STM32F410Cx) || \
    defined(STM32F410Rx) || \
    defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F401xC) || \
    defined(STM32F401xE) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_RX_PIN_BUS6_PA12         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6  , GPIO_PORT_A   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_8  ), /**< USART6 RX pin connected to PA12 */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F412Cx)
    USART_RX_PIN_BUS6_PC7   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_C   , GPIO_PIN_ID_7   , GPIO_ALT_FUNC_8   ), /**< USART6 RX pin connected to PC7   */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F412Vx) && \
    !defined(STM32F401xC) && \
    !defined(STM32F401xE) && \
    !defined(STM32F411xE)
    USART_RX_PIN_BUS6_PG9   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_G   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_8   ), /**< USART6 RX pin connected to PG9   */
#endif
#endif /* USART6 */

#if defined(UART7)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_RX_PIN_BUS7_PA8          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7  , GPIO_PORT_A   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_8  ), /**< UART7 RX pin connected to PA8 */
    USART_RX_PIN_BUS7_PB3          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7  , GPIO_PORT_B   , GPIO_PIN_ID_3  , GPIO_ALT_FUNC_8  ), /**< UART7 RX pin connected to PB3 */
#endif
    USART_RX_PIN_BUS7_PE7   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7   , GPIO_PORT_E   , GPIO_PIN_ID_7   , GPIO_ALT_FUNC_8   ), /**< UART7 RX pin connected to PE7    */
    USART_RX_PIN_BUS7_PF6   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7   , GPIO_PORT_F   , GPIO_PIN_ID_6   , GPIO_ALT_FUNC_8   ), /**< UART7 RX pin connected to PF6    */
#endif /* UART7 */

#if defined(UART8)
    USART_RX_PIN_BUS8_PE0   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_8   , GPIO_PORT_E   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_8   ), /**< UART8 RX pin connected to PE0    */
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_RX_PIN_BUS8_PF8          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_8  , GPIO_PORT_F   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_8  ), /**< UART8 RX pin connected to PF8 */
#endif
#endif /* UART8 */

    USART_RX_PIN_UNUSED     = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin    */
}   usart_RxPin_t;


/** \brief List of TX pins available for USART/UART peripherals */
typedef enum
{
#if !defined(STM32F410Tx)
    USART_TX_PIN_BUS1_PA9   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< USART1 TX pin connected to PA9   */
#endif
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
    USART_TX_PIN_BUS1_PA15         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1  , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_7  ), /**< USART1 TX pin connected to PA15 */
#endif
    USART_TX_PIN_BUS1_PB6   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_B   , GPIO_PIN_ID_6   , GPIO_ALT_FUNC_7   ), /**< USART1 TX pin connected to PB6   */

    USART_TX_PIN_BUS2_PA2   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_2   , GPIO_ALT_FUNC_7   ), /**< USART2 TX pin connected to PA2   */
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F410Tx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_TX_PIN_BUS2_PD5   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_5   , GPIO_ALT_FUNC_7   ), /**< USART2 TX pin connected to PD5   */
#endif

#if defined(USART3)
    USART_TX_PIN_BUS3_PB10  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PB10  */
#if !defined(STM32F412Cx)
    USART_TX_PIN_BUS3_PC10  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_C   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PC10  */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_TX_PIN_BUS3_PD8   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_8   , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PD8   */
#endif
#endif /* USART3 */

#if defined(UART4)
    USART_TX_PIN_BUS4_PA0   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_A   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_8   ), /**< UART4 TX pin connected to PA0    */
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_PIN_BUS4_PA12         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4  , GPIO_PORT_A   , GPIO_PIN_ID_12 , GPIO_ALT_FUNC_11 ), /**< UART4 TX pin connected to PA12 */
#endif
#if !defined(STM32F413xx) && \
    !defined(STM32F423xx)
    USART_TX_PIN_BUS4_PC10  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_C   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_8   ), /**< UART4 TX pin connected to PC10   */
#endif
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_PIN_BUS4_PD1          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4  , GPIO_PORT_D   , GPIO_PIN_ID_1  , GPIO_ALT_FUNC_11 ), /**< UART4 TX pin connected to PD1 */
    USART_TX_PIN_BUS4_PD10         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4  , GPIO_PORT_D   , GPIO_PIN_ID_10 , GPIO_ALT_FUNC_8  ), /**< UART4 TX pin connected to PD10 */
#endif
#endif /* UART4 */

#if defined(UART5)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_PIN_BUS5_PB6          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_B   , GPIO_PIN_ID_6  , GPIO_ALT_FUNC_11 ), /**< UART5 TX pin connected to PB6 */
    USART_TX_PIN_BUS5_PB9          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_B   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_11 ), /**< UART5 TX pin connected to PB9 */
    USART_TX_PIN_BUS5_PB13         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_B   , GPIO_PIN_ID_13 , GPIO_ALT_FUNC_11 ), /**< UART5 TX pin connected to PB13 */
#endif
    USART_TX_PIN_BUS5_PC12  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_C   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_8   ), /**< UART5 TX pin connected to PC12   */
#if defined(STM32F446xx)
    USART_TX_PIN_BUS5_PE8          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5  , GPIO_PORT_E   , GPIO_PIN_ID_8  , GPIO_ALT_FUNC_8  ), /**< UART5 TX pin connected to PE8 */
#endif
#endif /* UART5 */

#if defined(USART6)
#if defined(STM32F410Cx) || \
    defined(STM32F410Rx) || \
    defined(STM32F412Cx) || \
    defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F401xC) || \
    defined(STM32F401xE) || \
    defined(STM32F411xE) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_PIN_BUS6_PA11         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6  , GPIO_PORT_A   , GPIO_PIN_ID_11 , GPIO_ALT_FUNC_8  ), /**< USART6 TX pin connected to PA11 */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F412Cx)
    USART_TX_PIN_BUS6_PC6   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_C   , GPIO_PIN_ID_6   , GPIO_ALT_FUNC_8   ), /**< USART6 TX pin connected to PC6   */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F412Vx) && \
    !defined(STM32F401xC) && \
    !defined(STM32F401xE) && \
    !defined(STM32F411xE)
    USART_TX_PIN_BUS6_PG14  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_G   , GPIO_PIN_ID_14  , GPIO_ALT_FUNC_8   ), /**< USART6 TX pin connected to PG14  */
#endif
#endif /* USART6 */

#if defined(UART7)
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_PIN_BUS7_PA15         = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7  , GPIO_PORT_A   , GPIO_PIN_ID_15 , GPIO_ALT_FUNC_8  ), /**< UART7 TX pin connected to PA15 */
    USART_TX_PIN_BUS7_PB4          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7  , GPIO_PORT_B   , GPIO_PIN_ID_4  , GPIO_ALT_FUNC_8  ), /**< UART7 TX pin connected to PB4 */
#endif
    USART_TX_PIN_BUS7_PE8   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7   , GPIO_PORT_E   , GPIO_PIN_ID_8   , GPIO_ALT_FUNC_8   ), /**< UART7 TX pin connected to PE8    */
    USART_TX_PIN_BUS7_PF7   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_7   , GPIO_PORT_F   , GPIO_PIN_ID_7   , GPIO_ALT_FUNC_8   ), /**< UART7 TX pin connected to PF7    */
#endif /* UART7 */

#if defined(UART8)
    USART_TX_PIN_BUS8_PE1   = USART_PIN_BIT_MASK_ENCODE( USART_BUS_8   , GPIO_PORT_E   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_8   ), /**< UART8 TX pin connected to PE1    */
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_PIN_BUS8_PF9          = USART_PIN_BIT_MASK_ENCODE( USART_BUS_8  , GPIO_PORT_F   , GPIO_PIN_ID_9  , GPIO_ALT_FUNC_8  ), /**< UART8 TX pin connected to PF9 */
#endif
#endif /* UART8 */

    USART_TX_PIN_UNUSED     = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin    */
}   usart_TxPin_t;


/**
 * \brief List of Driver Enable (DE) pins available for USART/UART peripherals
 *
 * \note STM32F4 USART has no Driver Enable (DE) output, no pin is available.
 */
typedef enum
{

    USART_DE_PIN_UNUSED     = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin    */
}   usart_DePin_t;


/** \brief List of Clear To Send (CTS) pins available for USART/UART peripherals (input of the hardware flow control) */
typedef enum
{
#if !defined(STM32F410Tx)
    USART_CTS_PIN_BUS1_PA11 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_7   ), /**< USART1 CTS pin connected to PA11 */
#endif

    USART_CTS_PIN_BUS2_PA0  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_7   ), /**< USART2 CTS pin connected to PA0 */
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F410Tx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_CTS_PIN_BUS2_PD3  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_7   ), /**< USART2 CTS pin connected to PD3 */
#endif

#if defined(USART3)
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F412Vx) && \
    !defined(STM32F412Zx) && \
    !defined(STM32F413xx) && \
    !defined(STM32F423xx)
    USART_CTS_PIN_BUS3_PB13 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_13  , GPIO_ALT_FUNC_7   ), /**< USART3 CTS pin connected to PB13 */
#endif
#if defined(STM32F412Rx) || \
    defined(STM32F412Vx) || \
    defined(STM32F412Zx) || \
    defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_CTS_PIN_BUS3_PB13 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_13  , GPIO_ALT_FUNC_8   ), /**< USART3 CTS pin connected to PB13 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_CTS_PIN_BUS3_PD11 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_7   ), /**< USART3 CTS pin connected to PD11 */
#endif
#endif /* USART3 */

#if defined(UART4)
#if defined(STM32F446xx)
    USART_CTS_PIN_BUS4_PB0  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_B   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_8   ), /**< UART4 CTS pin connected to PB0 */
#endif
#endif /* UART4 */

#if defined(UART5)
#if defined(STM32F446xx)
    USART_CTS_PIN_BUS5_PC9  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_C   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< UART5 CTS pin connected to PC9 */
#endif
#endif /* UART5 */

#if defined(USART6)
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F412Vx) && \
    !defined(STM32F401xC) && \
    !defined(STM32F401xE) && \
    !defined(STM32F411xE)
    USART_CTS_PIN_BUS6_PG13 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_G   , GPIO_PIN_ID_13  , GPIO_ALT_FUNC_8   ), /**< USART6 CTS pin connected to PG13 */
    USART_CTS_PIN_BUS6_PG15 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_G   , GPIO_PIN_ID_15  , GPIO_ALT_FUNC_8   ), /**< USART6 CTS pin connected to PG15 */
#endif
#endif /* USART6 */

    USART_CTS_PIN_UNUSED    = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin */
}   usart_CtsPin_t;


/**
 * \brief List of Request To Send (RTS) pins available for USART/UART peripherals (output of the hardware flow control)
 *
 * STM32F4 has no Driver Enable output, the pads are used by the RTS output of the hardware flow control only.
 */
typedef enum
{
    USART_RTS_PIN_BUS1_PA12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_7   ), /**< USART1 RTS pin connected to PA12 */

#if !defined(STM32F410Tx)
    USART_RTS_PIN_BUS2_PA1  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_7   ), /**< USART2 RTS pin connected to PA1 */
#endif
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F410Tx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_RTS_PIN_BUS2_PD4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_7   ), /**< USART2 RTS pin connected to PD4 */
#endif

#if defined(USART3)
#if !defined(STM32F412Cx)
    USART_RTS_PIN_BUS3_PB14 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_14  , GPIO_ALT_FUNC_7   ), /**< USART3 RTS pin connected to PB14 */
#endif
#if !defined(STM32F412Cx) && \
    !defined(STM32F412Rx)
    USART_RTS_PIN_BUS3_PD12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_7   ), /**< USART3 RTS pin connected to PD12 */
#endif
#endif /* USART3 */

#if defined(UART4)
#if defined(STM32F446xx)
    USART_RTS_PIN_BUS4_PA15 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_A   , GPIO_PIN_ID_15  , GPIO_ALT_FUNC_8   ), /**< UART4 RTS pin connected to PA15 */
#endif
#endif /* UART4 */

#if defined(UART5)
#if defined(STM32F446xx)
    USART_RTS_PIN_BUS5_PC8  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_C   , GPIO_PIN_ID_8   , GPIO_ALT_FUNC_7   ), /**< UART5 RTS pin connected to PC8 */
#endif
#endif /* UART5 */

#if defined(USART6)
#if !defined(STM32F410Cx) && \
    !defined(STM32F410Rx) && \
    !defined(STM32F412Cx) && \
    !defined(STM32F412Rx) && \
    !defined(STM32F412Vx) && \
    !defined(STM32F401xC) && \
    !defined(STM32F401xE) && \
    !defined(STM32F411xE)
    USART_RTS_PIN_BUS6_PG8  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_G   , GPIO_PIN_ID_8   , GPIO_ALT_FUNC_8   ), /**< USART6 RTS pin connected to PG8 */
    USART_RTS_PIN_BUS6_PG12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_G   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_8   ), /**< USART6 RTS pin connected to PG12 */
#endif
#endif /* USART6 */

    USART_RTS_PIN_UNUSED    = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin */
}   usart_RtsPin_t;


/** \brief USART data word width (including parity bit) */
typedef enum
{
    USART_DATA_WIDTH_7 = 0x00000001u,           /**< 7 bits word length - not supported by STM32F4, configuration request returns error */
    USART_DATA_WIDTH_8 = LL_USART_DATAWIDTH_8B, /**< 8 bits word length : Start bit, 8 data bits, n stop bits */
    USART_DATA_WIDTH_9 = LL_USART_DATAWIDTH_9B  /**< 9 bits word length : Start bit, 9 data bits, n stop bits */
}   usart_DataWidth_t;


/** \brief USART stop bits configuration */
typedef enum
{
    USART_STOP_BITS_0_5 = LL_USART_STOPBITS_0_5, /**< 0.5 stop bit (not available on UART4 / UART5 / UART7 / UART8)  */
    USART_STOP_BITS_1   = LL_USART_STOPBITS_1,   /**< 1 stop bit                                                     */
    USART_STOP_BITS_1_5 = LL_USART_STOPBITS_1_5, /**< 1.5 stop bits (not available on UART4 / UART5 / UART7 / UART8) */
    USART_STOP_BITS_2   = LL_USART_STOPBITS_2    /**< 2 stop bits                                                    */
}   usart_StopBits_t;


/** \brief USART parity options enumeration */
typedef enum
{
    USART_PARITY_NONE = LL_USART_PARITY_NONE, /**< Parity control disabled                            */
    USART_PARITY_EVEN = LL_USART_PARITY_EVEN, /**< Parity control enabled and Even Parity is selected */
    USART_PARITY_ODD  = LL_USART_PARITY_ODD   /**< Parity control enabled and Odd Parity is selected  */
}   usart_Parity_t;


/** \brief USART transfer mode enumeration */
typedef enum
{
    USART_TRANSFER_MODE_NONE  = LL_USART_DIRECTION_NONE, /**< Transmitter and Receiver are disabled           */
    USART_TRANSFER_MODE_RX    = LL_USART_DIRECTION_RX,   /**< Transmitter is disabled and Receiver is enabled */
    USART_TRANSFER_MODE_TX    = LL_USART_DIRECTION_TX,   /**< Transmitter is enabled and Receiver is disabled */
    USART_TRANSFER_MODE_TX_RX = LL_USART_DIRECTION_TX_RX /**< Transmitter and Receiver are enabled            */
}   usart_TransferMode_t;


/** \brief USART hardware flow control options enumeration (not available on UART4 / UART5 / UART7 / UART8) */
typedef enum
{
    USART_FLOW_CONTROL_NONE    = LL_USART_HWCONTROL_NONE,   /**< CTS and RTS hardware flow control disabled                                             */
    USART_FLOW_CONTROL_RTS     = LL_USART_HWCONTROL_RTS,    /**< RTS output enabled, data is only requested when there is space in the receive buffer   */
    USART_FLOW_CONTROL_CTS     = LL_USART_HWCONTROL_CTS,    /**< CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0) */
    USART_FLOW_CONTROL_RTS_CTS = LL_USART_HWCONTROL_RTS_CTS /**< CTS and RTS hardware flow control enabled                                              */
}   usart_FlowControl_t;


/** \brief USART over-sampling options enumeration */
typedef enum
{
    USART_OVERSAMPLING_16 = LL_USART_OVERSAMPLING_16, /**< Over-sampling by 16 */
    USART_OVERSAMPLING_8  = LL_USART_OVERSAMPLING_8   /**< Over-sampling by 8  */
}   usart_Oversampling_t;


/** \brief USART single-wire half-duplex mode enumeration */
typedef enum
{
    USART_HALF_DUPLEX_INACTIVE = 0u, /**< Single-Wire mode disabled */
    USART_HALF_DUPLEX_ACTIVE         /**< Single-Wire mode enabled  */
}   usart_HalfDuplex_t;


/** RX pin level configuration (STM32F4 supports standard levels only) */
typedef enum
{
    USART_RX_PIN_STANDARD = 0u, /**< RX pin use standard logic levels                 */
    USART_RX_PIN_INVERTED       /**< RX pin use inverted logic levels - not supported */
}   usart_RxPinLevel_t;


/** TX pin level configuration (STM32F4 supports standard levels only) */
typedef enum
{
    USART_TX_PIN_STANDARD = 0u, /**< TX pin use standard logic levels                 */
    USART_TX_PIN_INVERTED       /**< TX pin use inverted logic levels - not supported */
}   usart_TxPinLevel_t;


/** Reception errors bit-mask (bits of status register) */
typedef enum
{
    USART_ERROR_NONE           = 0x0,             /**< No error detected                                                    */
    USART_ERROR_PARITY_ERROR   = LL_USART_SR_PE,  /**< The number of bits in data does not match parity settings (odd/even) */
    USART_ERROR_FRAMING_ERROR  = LL_USART_SR_FE,  /**< The stop bit is not recognized at the expected time                  */
    USART_ERROR_NOISE_DETECTED = LL_USART_SR_NE,  /**< Noise detected in frame                                              */
    USART_ERROR_OVERRUN        = LL_USART_SR_ORE, /**< New data available but the RXNE is not cleared yet                   */
}   usart_Error_t;


/** \brief USART/UART Driver Enable (DE) output activation state (STM32F4 has no DE output) */
typedef enum
{
    USART_DE_DISABLED = 0u,/**< Driver Enable output disabled                */
    USART_DE_ENABLED       /**< Driver Enable output enabled - not supported */
}   usart_DeFeatureState_t;


/** \brief USART/UART Driver Enable (DE) polarity configuration (STM32F4 has no DE output) */
typedef enum
{
    USART_DE_ACTIVE_HIGH = 0u, /**< Driver Enable pin is active with level HIGH                 */
    USART_DE_ACTIVE_LOW        /**< Driver Enable pin is active with level LOW - not supported */
}   usart_DePolarity_t;


/** List of Interrupt Requests (IRQ) applicable for USART/UART bus */
typedef enum
{
    USART_IRQ_RX_NOT_EMPTY = 0u, /**< RX register not empty Interrupt Request (IRQ)          */
    USART_IRQ_TX_EMPTY,          /**< TX register empty Interrupt Request (IRQ)              */
    USART_IRQ_TX_COMPLETE,       /**< Transmit complete Interrupt Request (IRQ)              */
    USART_IRQ_IDLE,              /**< Idle frame detected Interrupt Request (IRQ)            */
    USART_IRQ_RX_TIMEOUT,        /**< Receive timeout Interrupt Request (IRQ) - not on F4    */
    USART_IRQ_ERROR,             /**< Error Interrupt Request (IRQ)                          */
    USART_IRQ_CNT
}   usart_IrqList_t;


/** DMA peripherals enumeration list */
typedef enum
{
    USART_DMA_PERIPH_1 = DMA_PERIPH_1, /**< DMA peripheral 1 identification */
#if defined(DMA2)
    USART_DMA_PERIPH_2 = DMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    USART_DMA_PERIPH_CNT
}   usart_DmaPeriphId_t;


/**
 * \brief Enumeration of available channels (DMA streams) for all DMA peripherals
 *
 * STM32F4 DMA has no request multiplexer - the stream has to be one of the streams connected to
 * the USART request (request mapping table of the reference manual, e.g. USART2_TX: DMA1 stream 6,
 * USART2_RX: DMA1 stream 5). The streams usable by the USART / UART buses are given by the lists
 * \ref usart_TxDma_t and \ref usart_RxDma_t.
 */
typedef enum
{
    USART_DMA_CHANNEL_0  = DMA_STREAM_0, /**< DMA stream 0                    */
    USART_DMA_CHANNEL_1  = DMA_STREAM_1, /**< DMA stream 1                    */
    USART_DMA_CHANNEL_2  = DMA_STREAM_2, /**< DMA stream 2                    */
    USART_DMA_CHANNEL_3  = DMA_STREAM_3, /**< DMA stream 3                    */
    USART_DMA_CHANNEL_4  = DMA_STREAM_4, /**< DMA stream 4                    */
    USART_DMA_CHANNEL_5  = DMA_STREAM_5, /**< DMA stream 5                    */
    USART_DMA_CHANNEL_6  = DMA_STREAM_6, /**< DMA stream 6                    */
    USART_DMA_CHANNEL_7  = DMA_STREAM_7, /**< DMA stream 7                    */
    USART_DMA_CHANNEL_CNT                /**< Count of available DMA streams  */
}   usart_DmaChannelId_t;


/**
 * \brief List of DMA streams able to serve the USART / UART TX request of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    USART_TX_DMA_BUS1_DMA2_STREAM7     = USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, 4u ), /**< USART1 TX request on DMA2 stream 7 (channel selection 4) */
    USART_TX_DMA_BUS2_DMA1_STREAM6     = USART_DMA_ENCODE( USART_BUS_2, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_6, 4u ), /**< USART2 TX request on DMA1 stream 6 (channel selection 4) */
#if defined(USART3)
    USART_TX_DMA_BUS3_DMA1_STREAM3     = USART_DMA_ENCODE( USART_BUS_3, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_3, 4u ), /**< USART3 TX request on DMA1 stream 3 (channel selection 4) */
    USART_TX_DMA_BUS3_DMA1_STREAM4     = USART_DMA_ENCODE( USART_BUS_3, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_4, 7u ), /**< USART3 TX request on DMA1 stream 4 (channel selection 7) */
#endif
#if defined(UART4)
    USART_TX_DMA_BUS4_DMA1_STREAM4     = USART_DMA_ENCODE( USART_BUS_4, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_4, 4u ), /**< UART4 TX request on DMA1 stream 4 (channel selection 4) */
#endif
#if defined(UART5) && \
    !defined(STM32F413xx) && \
    !defined(STM32F423xx)
    USART_TX_DMA_BUS5_DMA1_STREAM7     = USART_DMA_ENCODE( USART_BUS_5, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_7, 4u ), /**< UART5 TX request on DMA1 stream 7 (channel selection 4) */
#endif
#if defined(STM32F413xx) || \
    defined(STM32F423xx)
    USART_TX_DMA_BUS5_DMA1_STREAM7     = USART_DMA_ENCODE( USART_BUS_5, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_7, 8u ), /**< UART5 TX request on DMA1 stream 7 (channel selection 8) */
#endif
#if defined(USART6)
    USART_TX_DMA_BUS6_DMA2_STREAM6     = USART_DMA_ENCODE( USART_BUS_6, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_6, 5u ), /**< USART6 TX request on DMA2 stream 6 (channel selection 5) */
    USART_TX_DMA_BUS6_DMA2_STREAM7     = USART_DMA_ENCODE( USART_BUS_6, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_7, 5u ), /**< USART6 TX request on DMA2 stream 7 (channel selection 5) */
#endif
#if defined(UART7)
    USART_TX_DMA_BUS7_DMA1_STREAM1     = USART_DMA_ENCODE( USART_BUS_7, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_1, 5u ), /**< UART7 TX request on DMA1 stream 1 (channel selection 5) */
#endif
#if defined(UART8)
    USART_TX_DMA_BUS8_DMA1_STREAM0     = USART_DMA_ENCODE( USART_BUS_8, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_0, 5u ), /**< UART8 TX request on DMA1 stream 0 (channel selection 5) */
#endif
    USART_TX_DMA_UNUSED                = USART_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   usart_TxDma_t;


/**
 * \brief List of DMA streams able to serve the USART / UART RX request of the peripherals (STM32CubeMX database / reference
 *        manual DMA request mapping, the channel selection of the stream is part of the value, streams
 *        existing only on some STM32F4 lines are guarded by the CMSIS device line)
 */
typedef enum
{
    USART_RX_DMA_BUS1_DMA2_STREAM2     = USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_2, 4u ), /**< USART1 RX request on DMA2 stream 2 (channel selection 4) */
    USART_RX_DMA_BUS1_DMA2_STREAM5     = USART_DMA_ENCODE( USART_BUS_1, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_5, 4u ), /**< USART1 RX request on DMA2 stream 5 (channel selection 4) */
    USART_RX_DMA_BUS2_DMA1_STREAM5     = USART_DMA_ENCODE( USART_BUS_2, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_5, 4u ), /**< USART2 RX request on DMA1 stream 5 (channel selection 4) */
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
    USART_RX_DMA_BUS2_DMA1_STREAM7     = USART_DMA_ENCODE( USART_BUS_2, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_7, 6u ), /**< USART2 RX request on DMA1 stream 7 (channel selection 6) */
#endif
#if defined(USART3)
    USART_RX_DMA_BUS3_DMA1_STREAM1     = USART_DMA_ENCODE( USART_BUS_3, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_1, 4u ), /**< USART3 RX request on DMA1 stream 1 (channel selection 4) */
#endif
#if defined(UART4)
    USART_RX_DMA_BUS4_DMA1_STREAM2     = USART_DMA_ENCODE( USART_BUS_4, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_2, 4u ), /**< UART4 RX request on DMA1 stream 2 (channel selection 4) */
#endif
#if defined(UART5)
    USART_RX_DMA_BUS5_DMA1_STREAM0     = USART_DMA_ENCODE( USART_BUS_5, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_0, 4u ), /**< UART5 RX request on DMA1 stream 0 (channel selection 4) */
#endif
#if defined(USART6)
    USART_RX_DMA_BUS6_DMA2_STREAM1     = USART_DMA_ENCODE( USART_BUS_6, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_1, 5u ), /**< USART6 RX request on DMA2 stream 1 (channel selection 5) */
    USART_RX_DMA_BUS6_DMA2_STREAM2     = USART_DMA_ENCODE( USART_BUS_6, USART_DMA_PERIPH_2, USART_DMA_CHANNEL_2, 5u ), /**< USART6 RX request on DMA2 stream 2 (channel selection 5) */
#endif
#if defined(UART7)
    USART_RX_DMA_BUS7_DMA1_STREAM3     = USART_DMA_ENCODE( USART_BUS_7, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_3, 5u ), /**< UART7 RX request on DMA1 stream 3 (channel selection 5) */
#endif
#if defined(UART8)
    USART_RX_DMA_BUS8_DMA1_STREAM6     = USART_DMA_ENCODE( USART_BUS_8, USART_DMA_PERIPH_1, USART_DMA_CHANNEL_6, 5u ), /**< UART8 RX request on DMA1 stream 6 (channel selection 5) */
#endif
    USART_RX_DMA_UNUSED                = USART_DMA_CODE_UNUSED  /**< DMA stream is not selected */
}   usart_RxDma_t;


/** Channel priority options enumeration */
typedef enum
{
    USART_DMA_PRIORITY_LOW      = DMA_PRIORITY_LOW     , /**< Priority level : Low       */
    USART_DMA_PRIORITY_MEDIUM   = DMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    USART_DMA_PRIORITY_HIGH     = DMA_PRIORITY_HIGH    , /**< Priority level : High      */
    USART_DMA_PRIORITY_VERYHIGH = DMA_PRIORITY_VERYHIGH, /**< Priority level : Very_High */
}   usart_DmaPriority_t;


/* -------------------------------------------------------------------------- */
/* ---------------------- Data handling configuration ----------------------- */
/* -------------------------------------------------------------------------- */

/**
 * \brief List of data transfer modes (selected separately for transmission and reception)
 *
 * All modes use the same buffers and report the same events through the same callbacks - they
 * differ only in the context moving the data:
 * - DMA:  DMA stream (callbacks from DMA / USART interrupt)
 * - ISR:  USART interrupt service routine (callbacks from USART interrupt)
 * - POLL: Usart_Task() polling USART flags (callbacks from Usart_Task() context)
 */
typedef enum
{
    USART_XFER_MODE_NONE = 0u, /**< Direction is not used by data handling                        */
    USART_XFER_MODE_DMA,       /**< Data are transferred by DMA                                   */
    USART_XFER_MODE_ISR,       /**< Data are transferred by USART interrupt service routine       */
    USART_XFER_MODE_POLL,      /**< Data are transferred by Usart_Task() (polling of USART flags) */
    USART_XFER_MODE_CNT        /**< Count of data transfer modes                                  */
}   usart_XferMode_t;


/** \brief List of receive buffer handling modes */
typedef enum
{
    USART_BUFFER_MODE_ONE_SHOT = 0u, /**< Reception stops when the buffer is full or the message ended */
    USART_BUFFER_MODE_CIRCULAR,      /**< Reception continues from the buffer start when it is full   */
    USART_BUFFER_MODE_CNT            /**< Count of buffer modes                                        */
}   usart_BufferMode_t;


/** \brief List of end of received message detection methods */
typedef enum
{
    USART_RX_END_NONE = 0u, /**< End of message is not detected (buffer size driven reception)      */
    USART_RX_END_IDLE,      /**< Idle line (one frame without activity) after received data          */
    USART_RX_END_TIMEOUT,   /**< Receiver timeout - not available on STM32F4 (configuration error)  */
    USART_RX_END_CNT        /**< Count of end of message detection methods                           */
}   usart_RxEndMode_t;


/**
 * \brief List of data transfer errors reported through \ref usart_XferErrCallback_t
 *
 * \note  STM32F4 DMA reports only transfer errors (\ref USART_XFER_ERROR_DMA_TRANSFER), other DMA
 *        errors are kept for compatibility of the public interface.
 */
typedef enum
{
    USART_XFER_ERROR_PARITY = 0u,        /**< Parity error (PE)                                   */
    USART_XFER_ERROR_FRAMING,            /**< Framing error (FE)                                  */
    USART_XFER_ERROR_NOISE,              /**< Noise detected (NE)                                 */
    USART_XFER_ERROR_OVERRUN,            /**< Overrun - received data were not read in time (ORE) */
    USART_XFER_ERROR_DMA_TRANSFER,       /**< DMA transfer error (bus error during transfer)      */
    USART_XFER_ERROR_DMA_CONFIG,         /**< DMA configuration error                             */
    USART_XFER_ERROR_DMA_CONFIG_UPDATE,  /**< DMA configuration (linked list) update error        */
    USART_XFER_ERROR_DMA_TRIGGER_OVERRUN,/**< DMA trigger overrun                                 */
    USART_XFER_ERROR_CNT                 /**< Count of data transfer errors                       */
}   usart_XferErrorId_t;


/** \brief Data transfer event callback (transmission complete, receive buffer half / full) */
typedef void ( usart_XferCallback_t )( void );

/** \brief End of received message callback, count of received bytes in RxBuffer is given as parameter */
typedef void ( usart_RxEndCallback_t )( usart_RxDataCnt_t rxCnt );

/** \brief Data transfer error callback, error identification is given as parameter */
typedef void ( usart_XferErrCallback_t )( usart_XferErrorId_t errorId );


/**
 * \brief Data handling configuration (common for DMA, ISR and POLL mode)
 *
 * Transmission and reception use independent modes. Callback events (equal in all modes):
 * - TxCompleteCallback: all bytes given to Usart_Set_TxStart() were transmitted (last stop bit
 *                       sent - transmission complete flag)
 * - RxHalfCallback:     RxBufferSize / 2 bytes were stored into RxBuffer
 * - RxCompleteCallback: RxBufferSize bytes were stored into RxBuffer (in circular mode the next
 *                       byte is stored to RxBuffer[ 0 ])
 * - RxEndCallback:      end of message detected (RxEndMode), parameter is count of bytes stored in
 *                       RxBuffer from the reception start (one shot) / write position (circular)
 * - ErrorCallback:      reception error or DMA error (\ref usart_XferErrorId_t)
 *
 * In one shot buffer mode the reception stops when the buffer is full or the end of message is
 * detected and is restarted by Usart_Set_RxStart(). Unused callback shall be set to
 * USART_NULL_PTR. DMA streams / priorities are used only in USART_XFER_MODE_DMA of the given
 * direction (TxDma / RxDma are items of the lists \ref usart_TxDma_t / \ref usart_RxDma_t of the
 * configured bus, USART_TX_DMA_UNUSED / USART_RX_DMA_UNUSED if the direction does not use DMA),
 * IrqPriority is used if any direction uses DMA or ISR mode.
 *
 * \note  Data are handled as 8-bit values (usart_TxData_t / usart_RxData_t) - 9-bit frames
 *        without parity are not supported.
 */
typedef struct
{
    usart_XferMode_t          TxMode;             /**< Transmission mode (NONE / DMA / ISR / POLL)                    */
    usart_XferMode_t          RxMode;             /**< Reception mode (NONE / DMA / ISR / POLL)                       */
    usart_RxData_t           *RxBuffer;           /**< Receive buffer (reception used). Must stay valid.              */
    usart_RxDataCnt_t         RxBufferSize;       /**< Receive buffer size in bytes (> 0)                             */
    usart_BufferMode_t        RxBufferMode;       /**< One shot / circular receive buffer                             */
    usart_RxEndMode_t         RxEndMode;          /**< End of received message detection                              */
    usart_TxDma_t             TxDma;              /**< DMA stream (transmission in DMA mode)                          */
    usart_DmaPriority_t       TxDmaPriority;      /**< DMA stream priority (transmission in DMA mode)                 */
    usart_RxDma_t             RxDma;              /**< DMA stream (reception in DMA mode)                             */
    usart_DmaPriority_t       RxDmaPriority;      /**< DMA stream priority (reception in DMA mode)                    */
    usart_IrqPrio_t           IrqPriority;        /**< USART interrupt priority (DMA / ISR mode)                      */
    usart_XferCallback_t     *TxCompleteCallback; /**< Transmission complete. USART_NULL_PTR if not used.             */
    usart_XferCallback_t     *RxHalfCallback;     /**< Receive buffer half filled. USART_NULL_PTR if not used.        */
    usart_XferCallback_t     *RxCompleteCallback; /**< Receive buffer filled. USART_NULL_PTR if not used.             */
    usart_RxEndCallback_t    *RxEndCallback;      /**< End of received message. USART_NULL_PTR if not used.          */
    usart_XferErrCallback_t  *ErrorCallback;      /**< Data transfer error. USART_NULL_PTR if not used.               */
}   usart_DataConfig_t;


/** \brief USART/UART bus configuration structure */
typedef struct
{
    usart_PeriphId_t              PeriphId;             /**< Specifies the USART peripheral ID. */
    usart_Baudrate_t              BaudRate;             /**< This field defines expected USART/UART communication baud rate. */
    usart_DataWidth_t             DataWidth;            /**< Specifies the number of data bits transmitted or received in a frame. */
    usart_StopBits_t              StopBits;             /**< Specifies the number of stop bits transmitted. */
    usart_Parity_t                Parity;               /**< Specifies the parity mode. */
    usart_TransferMode_t          TransferMode;         /**< Specifies whether the Receiver and/or Transmitter will be enabled or disabled after USART initialization. */
    usart_FlowControl_t           HwFlowControl;        /**< Specifies whether the hardware flow control mode is enabled or disabled.*/
    usart_DeFeatureState_t        DriverEnableMode;     /**< Driver Enable mode used by RS485 configuration (shall be disabled on STM32F4) */
    usart_DePolarity_t            DriverEnablePolarity; /**< Configuration of driver enable output pin active logic state (shall be active high on STM32F4) */
    usart_Oversampling_t          Oversampling;         /**< Specifies whether USART over-sampling mode is 16 or 8. */
    usart_HalfDuplex_t            HalfDuplex;           /**< Specifies the Half-duplex state */
    usart_RxTimeout_t             RxTimeoutValue;       /**< Receiver timeout threshold value. If set to '0', feature will be disabled (shall be '0' on STM32F4) */
    usart_RxPinLevel_t            RxPinOperationLevels; /**< Receive pin operation mode (shall be standard on STM32F4) */
    usart_TxPinLevel_t            TxPinOperationLevels; /**< Transmit pin operation mode (shall be standard on STM32F4) */

    const usart_DataConfig_t     *DataConfig;           /**< Data handling configuration (copied). USART_NULL_PTR - data handling is not initialized */

    usart_RxPin_t                 BusRxPin;             /**< RX GPIO pin used by peripheral                 */
    usart_TxPin_t                 BusTxPin;             /**< TX GPIO pin used by peripheral                 */
    usart_DePin_t                 BusDePin;             /**< Driver Enable (DE) GPIO pin used by peripheral (STM32F4: USART_DE_PIN_UNUSED only) */
    usart_CtsPin_t                BusCtsPin;            /**< Clear To Send (CTS) GPIO pin of the hardware flow control used by peripheral  */
    usart_RtsPin_t                BusRtsPin;            /**< Request To Send (RTS) GPIO pin of the hardware flow control used by peripheral */
}   usart_BusConfig_t;


/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_TYPES_H */
