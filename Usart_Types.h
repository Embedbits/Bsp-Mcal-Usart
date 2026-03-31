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
 */

#ifndef USART_USART_TYPES_H
#define USART_USART_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "stdint.h"                         /* Module types definition        */
#include "Gpio_Types.h"                     /* GPIO types definitions         */
#include "Gpdma_Types.h"                    /* DMA types definitions          */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Null pointer definition */
#define USART_NULL_PTR                        ( ( void* ) 0u )

/** Maximum count of bits used for RX timeout feature */
#define USART_RX_TIMEOUT_MAX                  ( 0x00FFFFFF )

/** Minimum count of bits used for RX timeout feature */
#define USART_RX_TIMEOUT_MIN                  ( 0x00 )

/** Peripheral identification bit offset in encoded value */
#define USART_BIT_MASK_PERIPH_BIT_OFFSET      ( 15u )

/** Port identification bit offset in encoded value */
#define USART_BIT_MASK_PORT_BIT_OFFSET        ( 10u )

/** Pin identification bit offset in encoded value */
#define USART_BIT_MASK_PIN_BIT_OFFSET         ( 5u )

/** Alternative function identification bit offset in encoded value */
#define USART_BIT_MASK_AF_BIT_OFFSET          ( 0u )

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
 * no activity on the RX line. */
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


/** \brief USART clock prescaler enumeration. */
typedef enum
{
    USART_PRESCALER_1 = 0u, /**< USART clock not divided    */
    USART_PRESCALER_2,      /**< USART clock divided by 2   */
    USART_PRESCALER_4,      /**< USART clock divided by 4   */
    USART_PRESCALER_6,      /**< USART clock divided by 6   */
    USART_PRESCALER_8,      /**< USART clock divided by 8   */
    USART_PRESCALER_10,     /**< USART clock divided by 10  */
    USART_PRESCALER_12,     /**< USART clock divided by 12  */
    USART_PRESCALER_16,     /**< USART clock divided by 16  */
    USART_PRESCALER_32,     /**< USART clock divided by 32  */
    USART_PRESCALER_64,     /**< USART clock divided by 64  */
    USART_PRESCALER_128,    /**< USART clock divided by 128 */
    USART_PRESCALER_256,    /**< USART clock divided by 256 */
    USART_PRESCALER_CNT     /**< Count of options           */
}   usart_Prescaler_t;

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
    USART_BUS_4,      /**< USART bus 4 ID */
#endif
#ifdef UART5
    USART_BUS_5,      /**< USART bus 5 ID */
#endif
#ifdef USART6
    USART_BUS_6,      /**< USART bus 5 ID */
#endif
    USART_BUS_CNT
}   usart_PeriphId_t;




/** \brief List of RX pins available for USART/UART peripherals */
typedef enum
{
#ifdef USART1
#ifdef GPIOA
    USART_RX_PIN_BUS1_PA10 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART1 RX pin connected to PA10 */
#endif
#ifdef GPIOB
    USART_RX_PIN_BUS1_PB7  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_B   , GPIO_PIN_ID_7   , GPIO_ALT_FUNC_7   ), /**< USART1 RX pin connected to PB7  */
#endif
#ifdef GPIOG
    USART_RX_PIN_BUS1_PG10 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_G   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART1 RX pin connected to PG10 */
#endif
#endif

#ifdef USART2
#ifdef GPIOA
    USART_RX_PIN_BUS2_PA3  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PA3  */
#endif
#ifdef GPIOA
    USART_RX_PIN_BUS2_PA15 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_15  , GPIO_ALT_FUNC_3   ), /**< USART2 RX pin connected to PA15 */
#endif
#ifdef GPIOD
    USART_RX_PIN_BUS2_PD6  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_6   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PD6  */
#endif
#endif

#ifdef USART3
#ifdef GPIOA
    USART_RX_PIN_BUS3_PA5  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_A   , GPIO_PIN_ID_5   , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PA5  */
#endif
#ifdef GPIOB
    USART_RX_PIN_BUS3_PB11 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PB11 */
#endif
#ifdef GPIOC
    USART_RX_PIN_BUS3_PC5  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_C   , GPIO_PIN_ID_5   , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PC5  */
#endif
#ifdef GPIOC
    USART_RX_PIN_BUS3_PC11 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_C   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PC11 */
#endif
#ifdef GPIOD
    USART_RX_PIN_BUS3_PD9  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< USART3 RX pin connected to PD9  */
#endif
#endif

#ifdef UART4
#ifdef GPIOA
    USART_RX_PIN_BUS4_PA1  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_A   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_8   ), /**< UART4 RX pin connected to PA1   */
#endif
#ifdef GPIOC
    USART_RX_PIN_BUS4_PC11 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_C   , GPIO_PIN_ID_11  , GPIO_ALT_FUNC_8   ), /**< UART4 RX pin connected to PC11  */
#endif
#endif

#ifdef UART5
#ifdef GPIOD
    USART_RX_PIN_BUS5_PD2  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_D   , GPIO_PIN_ID_2   , GPIO_ALT_FUNC_8   ), /**< UART5 RX pin connected to PD2   */
#endif
#ifdef GPIOF
    USART_RX_PIN_BUS5_PF4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_F   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_8   ), /**< UART5 RX pin connected to PF4   */
#endif
#endif

#ifdef USART6
#ifdef GPIOC
    USART_RX_PIN_BUS6_PC2  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_C   , GPIO_PIN_ID_2   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PC2  */
#endif
#ifdef GPIOC
    USART_RX_PIN_BUS6_PC8  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_C   , GPIO_PIN_ID_8   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PC8  */
#endif
#ifdef GPIOE
    USART_RX_PIN_BUS6_PE0  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_E   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PE0  */
#endif
#ifdef GPIOF
    USART_RX_PIN_BUS6_PF1  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_F   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PF1  */
#endif
#ifdef GPIOJ
    USART_RX_PIN_BUS6_PJ4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_J   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_7   ), /**< USART2 RX pin connected to PJ4  */
#endif
#endif

    USART_RX_PIN_UNUSED    = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin    */
}   usart_RxPin_t;


/** \brief List of TX pins available for USART/UART peripherals */
typedef enum
{
#ifdef USART1
#ifdef GPIOA
    USART_TX_PIN_BUS1_PA9  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< USART1 TX pin connected to PA9  */
#endif
#ifdef GPIOB
    USART_TX_PIN_BUS1_PB6  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_B   , GPIO_PIN_ID_6   , GPIO_ALT_FUNC_7   ), /**< USART1 TX pin connected to PB6  */
#endif
#ifdef GPIOG
    USART_TX_PIN_BUS1_PG9  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_G   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< USART1 TX pin connected to PG9  */
#endif
#endif

#ifdef USART2
#ifdef GPIOA
    USART_TX_PIN_BUS2_PA2  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_2   , GPIO_ALT_FUNC_7   ), /**< USART2 TX pin connected to PA2  */
#endif
#ifdef GPIOD
    USART_TX_PIN_BUS2_PD5  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_5   , GPIO_ALT_FUNC_7   ), /**< USART2 TX pin connected to PD5  */
#endif
#endif

#ifdef USART3
#ifdef GPIOA
    USART_TX_PIN_BUS3_PA7  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_A   , GPIO_PIN_ID_7   , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PA7  */
#endif
#ifdef GPIOB
    USART_TX_PIN_BUS3_PB10 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PB10 */
#endif
#ifdef GPIOC
    USART_TX_PIN_BUS3_PC4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_C   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PC4  */
#endif
#ifdef GPIOC
    USART_TX_PIN_BUS3_PC10 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_C   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PC10 */
#endif
#ifdef GPIOD
    USART_TX_PIN_BUS3_PD8  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_8   , GPIO_ALT_FUNC_7   ), /**< USART3 TX pin connected to PD8  */
#endif
#endif

#ifdef UART4
#ifdef GPIOA
    USART_TX_PIN_BUS4_PA0  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_A   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_8   ), /**< UART4 TX pin connected to PA0   */
#endif
#ifdef GPIOC
    USART_TX_PIN_BUS4_PC10 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_C   , GPIO_PIN_ID_10  , GPIO_ALT_FUNC_8   ), /**< UART4 TX pin connected to PC10  */
#endif
#endif

#ifdef UART5
#ifdef GPIOC
    USART_TX_PIN_BUS5_PC12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_C   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_8   ), /**< UART5 TX pin connected to PC12  */
#endif
#ifdef GPIOF
    USART_TX_PIN_BUS5_PF3  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_F   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_8   ), /**< UART5 TX pin connected to PF3   */
#endif
#endif

#ifdef USART6
#ifdef GPIOC
    USART_TX_PIN_BUS6_PC3  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_C   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_7   ), /**< USART6 TX pin connected to PC3  */
#endif
#ifdef GPIOC
    USART_TX_PIN_BUS6_PC9  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_C   , GPIO_PIN_ID_9   , GPIO_ALT_FUNC_7   ), /**< USART6 TX pin connected to PC9  */
#endif
#ifdef GPIOE
    USART_TX_PIN_BUS6_PE1  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_E   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_7   ), /**< USART6 TX pin connected to PE1  */
#endif
#ifdef GPIOF
    USART_TX_PIN_BUS6_PF0  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_F   , GPIO_PIN_ID_0   , GPIO_ALT_FUNC_7   ), /**< USART6 TX pin connected to PF0  */
#endif
#ifdef GPIOJ
    USART_TX_PIN_BUS6_PJ3  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_J   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_7   ), /**< USART6 TX pin connected to PJ3  */
#endif
#endif

    USART_TX_PIN_UNUSED    = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin    */
}   usart_TxPin_t;


/** \brief List of Driver Enable (DE) pins available for USART/UART peripherals */
typedef enum
{
#ifdef USART1
#ifdef GPIOA
    USART_DE_PIN_BUS1_PA12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_A   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_7   ), /**< USART1 DE pin connected to PA12 */
#endif
#ifdef GPIOB
    USART_DE_PIN_BUS1_PB3  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_B   , GPIO_PIN_ID_3   , GPIO_ALT_FUNC_7   ), /**< USART1 DE pin connected to PB3  */
#endif
#ifdef GPIOG
    USART_DE_PIN_BUS1_PG12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_1   , GPIO_PORT_G   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_7   ), /**< USART1 DE pin connected to PG12 */
#endif
#endif

#ifdef USART2
#ifdef GPIOA
    USART_DE_PIN_BUS2_PA1  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_A   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_7   ), /**< USART2 DE pin connected to PA1  */
#endif
#ifdef GPIOD
    USART_DE_PIN_BUS2_PD4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_2   , GPIO_PORT_D   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_7   ), /**< USART2 DE pin connected to PD4  */
#endif
#endif

#ifdef USART3
#ifdef GPIOA
    USART_DE_PIN_BUS3_PA15 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_A   , GPIO_PIN_ID_15  , GPIO_ALT_FUNC_7   ), /**< USART3 DE pin connected to PA15 */
#endif
#ifdef GPIOB
    USART_DE_PIN_BUS3_PB1  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_1   , GPIO_ALT_FUNC_7   ), /**< USART3 DE pin connected to PB1  */
#endif
#ifdef GPIOB
    USART_DE_PIN_BUS3_PB14 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_B   , GPIO_PIN_ID_14  , GPIO_ALT_FUNC_7   ), /**< USART3 DE pin connected to PB14 */
#endif
#ifdef GPIOD
    USART_DE_PIN_BUS3_PD2  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_2   , GPIO_ALT_FUNC_7   ), /**< USART3 DE pin connected to PD2  */
#endif
#ifdef GPIOD
    USART_DE_PIN_BUS3_PD12 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_3   , GPIO_PORT_D   , GPIO_PIN_ID_12  , GPIO_ALT_FUNC_7   ), /**< USART3 DE pin connected to PD12 */
#endif
#endif

#ifdef UART4
#ifdef GPIOA
    USART_DE_PIN_BUS4_PA15 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_4   , GPIO_PORT_A   , GPIO_PIN_ID_15  , GPIO_ALT_FUNC_8   ), /**< UART4 DE pin connected to PA15  */
#endif
#endif

#ifdef UART5
#ifdef GPIOB
    USART_DE_PIN_BUS5_PB4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_5   , GPIO_PORT_B   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_8   ), /**< UART5 DE pin connected to PB4   */
#endif
#endif

#ifdef USART6
#ifdef GPIOD
    USART_DE_PIN_BUS6_PD15 = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_D   , GPIO_PIN_ID_15  , GPIO_ALT_FUNC_7   ), /**< USART6 DE pin connected to PD15 */
#endif
#ifdef GPIOE
    USART_DE_PIN_BUS6_PE4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_E   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_7   ), /**< USART6 DE pin connected to PE4  */
#endif
#ifdef GPIOF
    USART_DE_PIN_BUS6_PF4  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_F   , GPIO_PIN_ID_4   , GPIO_ALT_FUNC_7   ), /**< USART6 DE pin connected to PF4  */
#endif
#ifdef GPIOJ
    USART_DE_PIN_BUS6_PJ5  = USART_PIN_BIT_MASK_ENCODE( USART_BUS_6   , GPIO_PORT_J   , GPIO_PIN_ID_5   , GPIO_ALT_FUNC_7   ), /**< USART6 DE pin connected to PJ5  */
#endif
#endif

    USART_DE_PIN_UNUSED    = USART_PIN_BIT_MASK_ENCODE( USART_BUS_CNT , GPIO_PORT_CNT , GPIO_PIN_ID_CNT , GPIO_ALT_FUNC_CNT ), /**< Identification of unused pin    */
}   usart_DePin_t;


/** \brief USART data word width */
typedef enum
{
    USART_DATA_WIDTH_7 = LL_USART_DATAWIDTH_7B, /**< 7 bits word length : Start bit, 7 data bits, n stop bits */
    USART_DATA_WIDTH_8 = LL_USART_DATAWIDTH_8B, /**< 8 bits word length : Start bit, 8 data bits, n stop bits */
    USART_DATA_WIDTH_9 = LL_USART_DATAWIDTH_9B  /**< 9 bits word length : Start bit, 9 data bits, n stop bits */
}   usart_DataWidth_t;


/** \brief USART stop bits configuration */
typedef enum
{
    USART_STOP_BITS_0_5 = LL_USART_STOPBITS_0_5, /**< 0.5 stop bit  */
    USART_STOP_BITS_1   = LL_USART_STOPBITS_1,   /**< 1 stop bit    */
    USART_STOP_BITS_1_5 = LL_USART_STOPBITS_1_5, /**< 1.5 stop bits */
    USART_STOP_BITS_2   = LL_USART_STOPBITS_2    /**< 2 stop bits   */
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


/** \brief USART hardware flow control options enumeration */
typedef enum
{
    USART_FLOW_CONTROL_NONE    = LL_USART_HWCONTROL_NONE,   /**< CTS and RTS hardware flow control disabled                                             */
    USART_FLOW_CONTROL_RTS     = LL_USART_HWCONTROL_RTS,    /**< RTS output enabled, data is only requested when there is space in the receive buffer   */
    USART_FLOW_CONTROL_CTS     = LL_USART_HWCONTROL_CTS,    /**< CTS mode enabled, data is only transmitted when the nCTS input is asserted (tied to 0) */
    USART_FLOW_CONTROL_RTS_CTS = LL_USART_HWCONTROL_RTS_CTS /**< CTS and RTS hardware flow control enabled                                              */
}   usart_FlowControl_t;


typedef enum
{
    USART_OVERSAMPLING_16 = LL_USART_OVERSAMPLING_16, /**< Over-sampling by 16 */
    USART_OVERSAMPLING_8  = LL_USART_OVERSAMPLING_8   /**< Over-sampling by 8  */
}   usart_Oversampling_t;


typedef enum
{
    USART_HALF_DUPLEX_INACTIVE = 0u, /**< Single-Wire mode disabled */
    USART_HALF_DUPLEX_ACTIVE         /**< Single-Wire mode enabled  */
}   usart_HalfDuplex_t;


/** RX pin level configuration */
typedef enum
{
    USART_RX_PIN_STANDARD = LL_USART_RXPIN_LEVEL_STANDARD, /**< RX pin use standard logic levels */
    USART_RX_PIN_INVERTED = LL_USART_RXPIN_LEVEL_INVERTED  /**< RX pin use inverted logic levels */
}   usart_RxPinLevel_t;


/** TX pin level configuration */
typedef enum
{
    USART_TX_PIN_STANDARD = LL_USART_TXPIN_LEVEL_STANDARD, /**< TX pin use standard logic levels */
    USART_TX_PIN_INVERTED = LL_USART_TXPIN_LEVEL_INVERTED  /**< TX pin use inverted logic levels */
}   usart_TxPinLevel_t;


typedef enum
{
    USART_ERROR_NONE           = 0x0,              /**< No error detected                                                    */
    USART_ERROR_PARITY_ERROR   = LL_USART_ISR_PE,  /**< The number of bits in data does not match parity settings (odd/even) */
    USART_ERROR_FRAMING_ERROR  = LL_USART_ISR_FE,  /**< The stop bit is not recognized at the expected time                  */
    USART_ERROR_NOISE_DETECTED = LL_USART_ISR_NE,  /**< Noise detected in frame                                              */
    USART_ERROR_OVERRUN        = LL_USART_ISR_ORE, /**< New data available but the RXNE is not cleared yet                   */
}   usart_Error_t;

/** \brief USART/UART Driver Enable (DE) output activation state */
typedef enum
{
    USART_DE_DISABLED = 0u,/**< Driver Enable output disabled */
    USART_DE_ENABLED       /**< Driver Enable output enabled  */
}   usart_DeFeatureState_t;


/** \brief USART/UART Driver Enable (DE) polarity configuration */
typedef enum
{
    USART_DE_ACTIVE_HIGH = LL_USART_DE_POLARITY_HIGH, /**< Driver Enable pin is active with level HIGH */
    USART_DE_ACTIVE_LOW  = LL_USART_DE_POLARITY_LOW   /**< Driver Enable pin is active with level LOW  */
}   usart_DePolarity_t;


/** List of Interrupt Requests (IRQ) applicable for USART/UART bus */
typedef enum
{
    USART_IRQ_RX_NOT_EMPTY = 0u, /**< RX register not empty Interrupt Request (IRQ) */
    USART_IRQ_TX_EMPTY,          /**< TX register empty Interrupt Request (IRQ)     */
    USART_IRQ_TX_COMPLETE,       /**< Transmit complete Interrupt Request (IRQ)     */
    USART_IRQ_IDLE,              /**< Idle frame detected Interrupt Request (IRQ)   */
    USART_IRQ_RX_TIMEOUT,        /**< Receive timeout Interrupt Request (IRQ)       */
    USART_IRQ_ERROR,             /**< Error Interrupt Request (IRQ)                 */
    USART_IRQ_CNT
}   usart_IrqList_t;


/** Data transfer handling style enumeration */
typedef enum
{
    USART_TRANSFER_BLOCKING = 0u, /**< Blocking style is used for data transfer. */
    USART_TRANSFER_INTERRUPT,     /**< Interrupts are used for data transfer.    */
    USART_TRANSFER_DMA,           /**< DMA is used for data transfer             */
    USART_TRANSFER_CNT
}   usart_TransferStyle_t;


/** DMA peripherals enumeration list */
typedef enum
{
#if defined(GPDMA1)
    USART_DMA_PERIPH_1 = GPDMA_PERIPH_1, /**< DMA peripheral 1 identification */
#endif
#if defined(GPDMA2)
    USART_DMA_PERIPH_2 = GPDMA_PERIPH_2, /**< DMA peripheral 2 identification */
#endif
    USART_DMA_PERIPH_CNT
}   usart_DmaPeriphId_t;


/** Enumeration of available channels for all DMA peripherals */
typedef enum
{
    USART_DMA_CHANNEL_0  = GPDMA_CHANNEL_0,  /**< DMA transfer channel 0 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_1  = GPDMA_CHANNEL_1,  /**< DMA transfer channel 1 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_2  = GPDMA_CHANNEL_2,  /**< DMA transfer channel 2 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_3  = GPDMA_CHANNEL_3,  /**< DMA transfer channel 3 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_4  = GPDMA_CHANNEL_4,  /**< DMA transfer channel 4 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_5  = GPDMA_CHANNEL_5,  /**< DMA transfer channel 5 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_6  = GPDMA_CHANNEL_6,  /**< DMA transfer channel 6 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_7  = GPDMA_CHANNEL_7,  /**< DMA transfer channel 7 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_8  = GPDMA_CHANNEL_8,  /**< DMA transfer channel 8 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_9  = GPDMA_CHANNEL_9,  /**< DMA transfer channel 9 (FIFO size 8 bytes) linear addressing mode         */
    USART_DMA_CHANNEL_10 = GPDMA_CHANNEL_10, /**< DMA transfer channel 10 (FIFO size 8 bytes) linear addressing mode        */
    USART_DMA_CHANNEL_11 = GPDMA_CHANNEL_11, /**< DMA transfer channel 11 (FIFO size 8 bytes) linear addressing mode        */
    USART_DMA_CHANNEL_12 = GPDMA_CHANNEL_12, /**< DMA transfer channel 12 (FIFO size 32 bytes) linear or 2D addressing mode */
    USART_DMA_CHANNEL_13 = GPDMA_CHANNEL_13, /**< DMA transfer channel 13 (FIFO size 32 bytes) linear or 2D addressing mode */
    USART_DMA_CHANNEL_14 = GPDMA_CHANNEL_14, /**< DMA transfer channel 14 (FIFO size 32 bytes) linear or 2D addressing mode */
    USART_DMA_CHANNEL_15 = GPDMA_CHANNEL_15, /**< DMA transfer channel 15 (FIFO size 32 bytes) linear or 2D addressing mode */
    USART_DMA_CHANNEL_CNT /*= DMA_CHANNEL_CNT*/,
}   usart_DmaChannelId_t;


/** Channel priority options enumeration */
typedef enum
{
    USART_DMA_PRIORITY_LOW      = GPDMA_PRIORITY_LOW     , /**< Priority level : Low       */
    USART_DMA_PRIORITY_MEDIUM   = GPDMA_PRIORITY_MEDIUM  , /**< Priority level : Medium    */
    USART_DMA_PRIORITY_HIGH     = GPDMA_PRIORITY_HIGH    , /**< Priority level : High      */
    USART_DMA_PRIORITY_VERYHIGH = GPDMA_PRIORITY_VERYHIGH, /**< Priority level : Very_High */
}   usart_DmaPriority_t;


/** Receive register Not Empty (RxNe) interrupt callback. Received data are given as parameter. */
typedef void ( usart_RxNeIrqCallback_t )( uint16_t rxData );
/** Error (Err) interrupt callback. Error ID is given as parameter. */
typedef void ( usart_ErrIrqCallback_t )( usart_Error_t errMask );
/** Receive register Not Empty (RxNe) interrupt callback */
typedef void ( usart_TxeIrqCallback_t )( void );
/** Transmit Complete (Tc) interrupt callback */
typedef void ( usart_TcIrqCallback_t )( void );
/** Idle line detected (IdleNe) interrupt callback */
typedef void ( usart_IdleIrqCallback_t )( void );
/** Receiver Timeout detection (RxTimeout) interrupt callback */
typedef void ( usart_RxTimeoutIrqCallback_t )( void );

typedef void (usart_DmaCallback)(void);

typedef enum
{
    USART_DMA_ERROR_TRANSFER      = GPDMA_ERROR_TRANSFER     , /**< Error during transfer             */
    USART_DMA_ERROR_CONFIG_UPDATE = GPDMA_ERROR_CONFIG_UPDATE, /**< Error during configuration update */
    USART_DMA_ERROR_CONFIG_ERROR  = GPDMA_ERROR_CONFIG_ERROR , /**< Error in transfer configuration   */
    USART_DMA_ERROR_TRIG_OVERRUN  = GPDMA_ERROR_TRIG_OVERRUN , /**< Trigger overrun error             */
}   usart_DmaErrId_t;

typedef void (usart_DmaErrCallback_t)(usart_DmaErrId_t);


typedef struct
{
    /* Reception configuration. If unused set TxDmaPeriphId to USART_DMA_PERIPH_CNT */
    usart_DmaPeriphId_t           TxDmaPeriphId;        /**< DMA peripheral identification used for data transmission */
    usart_DmaChannelId_t          TxDmaChannelId;       /**< DMA channel identification used for data transmission */
    usart_DmaPriority_t           TxDmaPriority;

    usart_DmaCallback            *TxTransferCompleteCallback;
    usart_DmaCallback            *TxHalfTransferCallback;
    usart_DmaErrCallback_t       *TxErrorCallback;

    /* Reception configuration. If unused set RxDmaPeriphId to USART_DMA_PERIPH_CNT */
    usart_DmaPeriphId_t           RxDmaPeriphId;        /**< DMA peripheral identification used for data reception */
    usart_DmaChannelId_t          RxDmaChannelId;       /**< DMA channel identification used for data reception */
    usart_DmaPriority_t           RxDmaPriority;

    usart_DmaCallback            *RxTransferCompleteCallback;
    usart_DmaCallback            *RxHalfTransferCallback;
    usart_DmaErrCallback_t       *RxErrorCallback;
}   usart_DmaConfig_t;


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
    usart_DeFeatureState_t        DriverEnableMode;     /**< Driver Enable mode used by RS485 configuration */
    usart_DePolarity_t            DriverEnablePolarity; /**< Configuration of driver enable output pin active logic state */
    usart_Oversampling_t          Oversampling;         /**< Specifies whether USART over-sampling mode is 16 or 8. */
    usart_HalfDuplex_t            HalfDuplex;           /**< Specifies the Half-duplex state */
    usart_RxTimeout_t             RxTimeoutValue;       /**< Receiver timeout threshold value. If set to '0', feature will be disabled. Otherwise activated */
    usart_RxPinLevel_t            RxPinOperationLevels; /**< Receive pin operation mode. Can be normal or using inverted logical levels */
    usart_TxPinLevel_t            TxPinOperationLevels; /**< Transmit pin operation mode. Can be normal or using inverted logical levels */

    usart_TransferStyle_t         OperationMode;

    usart_DmaConfig_t            *DmaConfiguration;

    usart_IrqPrio_t               IrqPriority;          /**< Interrupt Request (IRQ) priority */
    usart_RxNeIrqCallback_t      *RxNotEmpty_ISR;       /**< Receive Buffer Not Empty (a byte has arrived) interrupt handler */
    usart_TxeIrqCallback_t       *TransmitEmpty_ISR;    /**< Transmit Buffer Empty interrupt handler */
    usart_TcIrqCallback_t        *TransferComplete_ISR; /**< Transmit Complete (a byte transmit finished) interrupt handler */
    usart_ErrIrqCallback_t       *Error_ISR;            /**< Receiver/Transmitter error handler */
    usart_IdleIrqCallback_t      *Idle_ISR;             /**< IDLE state handler */
    usart_RxTimeoutIrqCallback_t *RxTimeout_ISR;        /**< Receive timeout detected interrupt handler */

    usart_RxPin_t                 BusRxPin;             /**< RX GPIO pin used by peripheral                 */
    usart_TxPin_t                 BusTxPin;             /**< TX GPIO pin used by peripheral                 */
    usart_DePin_t                 BusDePin;             /**< Driver Enable (DE) GPIO pin used by peripheral */
}   usart_BusConfig_t;


/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_TYPES_H */
