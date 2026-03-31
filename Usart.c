/**
 * \author Mr.Nobody
 * \file Usart.h
 * \ingroup Usart
 * \brief Usart module common functionality
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart.h"                          /* Self include                   */
#include "Usart_Port.h"                     /* Own port file include          */
#include "Usart_Types.h"                    /* Module types definitions       */
#include "Stm32_usart.h"                    /* USART RAL functionality        */
#include "Rcc_Port.h"                       /* RCC port functionality         */
#include "Nvic_Port.h"                      /* Nvic port functionality        */
#include "Gpio_Port.h"                      /* Gpio port functionality        */
//#include "Dma_Port.h"                       /* Dma port functionality         */
/* ============================== TYPEDEFS ================================== */

/** Structure type used for USART/UART configuration array */
typedef struct
{
    USART_TypeDef*       UsartReg;  /**< USART configuration register */
    rcc_PeriphId_t       UsartRcc;  /**< USART RCC configuration ID   */
    nvic_PeriphIrqList_t UsartNvic; /**< USART NVIC configuration ID  */
    nvic_IsrCallback_t   UsartIsr;  /**< USART ISR callback routines  */
}   usart_ConfigStruct_t;

/** Structure type used to store users USART/UART ISR callback pointers */
typedef struct
{
    usart_BusId_t              UsartPeriphId;
    usart_TransferStyle_t         TransferStyle;
    usart_RxNeIrqCallback_t      *RxNotEmptyIsr;
    usart_ErrIrqCallback_t       *ErrorIsr;
    usart_TxeIrqCallback_t       *TxEmptyIsr;
    usart_TcIrqCallback_t        *TransferCompleteIsr;
    usart_IdleIrqCallback_t      *IdleIsr;
    usart_RxTimeoutIrqCallback_t *RxTimeoutIsr;
}   usart_IsrCallback_t;


/** USART/UART RX GPIO's configuration structure type */
typedef struct
{
    gpio_PinId_t       GpioRxPinId;
    gpio_PortId_t      GpioRxPortId;
    gpio_AltFunction_t GpioRxAltFunctionId;
    usart_BusId_t      BusId;
    usart_RxPin_t      RxPinId;
}   usart_GpioRxPinConfig_t;


/** USART/UART TX GPIO's configuration structure type */
typedef struct
{
    gpio_PinId_t       GpioTxPinId;
    gpio_PortId_t      GpioTxPortId;
    gpio_AltFunction_t GpioTxAltFunctionId;
    usart_BusId_t      BusId;
    usart_TxPin_t      TxPinId;
}   usart_GpioTxPinConfig_t;


/** USART/UART DE GPIO's configuration structure type */
typedef struct
{
    gpio_PinId_t       GpioDePinId;
    gpio_PortId_t      GpioDePortId;
    gpio_AltFunction_t GpioDeAltFunctionId;
    usart_BusId_t      BusId;
    usart_DePin_t      DePinId;
}   usart_GpioDePinConfig_t;


/** USART/UART GPIO's configuration structure type */
typedef struct
{
    usart_GpioRxPinConfig_t GpioRxConfig[ USART_RX_PIN_CNT ];
    usart_GpioTxPinConfig_t GpioTxConfig[ USART_TX_PIN_CNT ];
    usart_GpioDePinConfig_t GpioDeConfig[ USART_DE_PIN_CNT ];
}   usart_GpioConfig_t;

/* ======================== FORWARD DECLARATIONS ============================ */

#ifdef USART1
static void Usart_Usart1_IsrHandler(void);
#endif /* USART1 */
#ifdef USART2
static void Usart_Usart2_IsrHandler(void);
#endif /* USART2 */
#ifdef USART3
static void Usart_Usart3_IsrHandler(void);
#endif /* USART3 */
#ifdef UART4
static void Uart_Usart4_IsrHandler(void);
#endif /* UART4 */
#ifdef UART5
static void Uart_Usart5_IsrHandler(void);
#endif /* UART5 */

static inline void Usart_GlobalIsrHandler( usart_BusId_t usartId );

static usart_RequestState_t Usart_Set_Prescaler( usart_BusId_t usartId, usart_Prescaler_t prescaler );
static usart_RequestState_t Usart_Get_Prescaler( usart_BusId_t usartId, usart_Prescaler_t *prescaler );

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define USART_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define USART_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define USART_PATCH_VERSION           ( 0u )

/** Maximum timeout for system reaction to request */
#define USART_TIMEOUT_RAW                       ( 0x84FCB )

/** Bit-mask of all possible errors in ISR register */
#define USART_ISR_ERROR_MASK                    ( ( LL_USART_ISR_PE | LL_USART_ISR_FE | LL_USART_ISR_NE | LL_USART_ISR_ORE ) )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** USART/UART peripherals runtime data array */
static volatile usart_IsrCallback_t     usart_RuntimeData[ USART_BUS_CNT ] =
{
    { .UsartPeriphId = USART_BUS_1 , .TransferStyle = USART_TRANSFER_BLOCKING , .RxNotEmptyIsr = USART_NULL_PTR , .ErrorIsr = USART_NULL_PTR , .TxEmptyIsr = USART_NULL_PTR , .TransferCompleteIsr = USART_NULL_PTR , .IdleIsr = USART_NULL_PTR , .RxTimeoutIsr = USART_NULL_PTR },
    { .UsartPeriphId = USART_BUS_2 , .TransferStyle = USART_TRANSFER_BLOCKING , .RxNotEmptyIsr = USART_NULL_PTR , .ErrorIsr = USART_NULL_PTR , .TxEmptyIsr = USART_NULL_PTR , .TransferCompleteIsr = USART_NULL_PTR , .IdleIsr = USART_NULL_PTR , .RxTimeoutIsr = USART_NULL_PTR },
    { .UsartPeriphId = USART_BUS_3 , .TransferStyle = USART_TRANSFER_BLOCKING , .RxNotEmptyIsr = USART_NULL_PTR , .ErrorIsr = USART_NULL_PTR , .TxEmptyIsr = USART_NULL_PTR , .TransferCompleteIsr = USART_NULL_PTR , .IdleIsr = USART_NULL_PTR , .RxTimeoutIsr = USART_NULL_PTR },
    { .UsartPeriphId = USART_BUS_4 , .TransferStyle = USART_TRANSFER_BLOCKING , .RxNotEmptyIsr = USART_NULL_PTR , .ErrorIsr = USART_NULL_PTR , .TxEmptyIsr = USART_NULL_PTR , .TransferCompleteIsr = USART_NULL_PTR , .IdleIsr = USART_NULL_PTR , .RxTimeoutIsr = USART_NULL_PTR },
    { .UsartPeriphId = USART_BUS_5 , .TransferStyle = USART_TRANSFER_BLOCKING , .RxNotEmptyIsr = USART_NULL_PTR , .ErrorIsr = USART_NULL_PTR , .TxEmptyIsr = USART_NULL_PTR , .TransferCompleteIsr = USART_NULL_PTR , .IdleIsr = USART_NULL_PTR , .RxTimeoutIsr = USART_NULL_PTR },
};

/** USART/UART peripherals configuration array */
static usart_ConfigStruct_t const       usart_PeriphConf[ USART_BUS_CNT ] =
{
#ifdef USART1
    { .UsartReg = USART1, .UsartRcc = RCC_PERIPH_USART1, .UsartNvic = NVIC_PERIPH_IRQ_USART1, .UsartIsr = Usart_Usart1_IsrHandler },
#endif
#ifdef USART2
    { .UsartReg = USART2, .UsartRcc = RCC_PERIPH_USART2, .UsartNvic = NVIC_PERIPH_IRQ_USART2, .UsartIsr = Usart_Usart2_IsrHandler },
#endif
#ifdef USART3
    { .UsartReg = USART3, .UsartRcc = RCC_PERIPH_USART3, .UsartNvic = NVIC_PERIPH_IRQ_USART3, .UsartIsr = Usart_Usart3_IsrHandler },
#endif
#ifdef UART4
    { .UsartReg = UART4,  .UsartRcc = RCC_PERIPH_UART4,  .UsartNvic = NVIC_PERIPH_IRQ_UART4,  .UsartIsr = Uart_Usart4_IsrHandler  },
#endif
#ifdef UART5
    { .UsartReg = UART5,  .UsartRcc = RCC_PERIPH_UART5,  .UsartNvic = NVIC_PERIPH_IRQ_UART5,  .UsartIsr = Uart_Usart5_IsrHandler  },
#endif
};

/** USART/UART peripherals possible GPIO's pins configuration array */
static usart_GpioConfig_t const        usart_GpioConfig =
{
  .GpioRxConfig = {
#ifdef USART1
    { .RxPinId = USART_RX_PIN_BUS1_PA10, .GpioRxPinId = GPIO_PIN_ID_10, .GpioRxPortId = GPIO_PORT_A, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
    { .RxPinId = USART_RX_PIN_BUS1_PB7 , .GpioRxPinId = GPIO_PIN_ID_7 , .GpioRxPortId = GPIO_PORT_B, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
    { .RxPinId = USART_RX_PIN_BUS1_PG10, .GpioRxPinId = GPIO_PIN_ID_10, .GpioRxPortId = GPIO_PORT_G, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
#endif
#ifdef USART2
    { .RxPinId = USART_RX_PIN_BUS2_PA3 , .GpioRxPinId = GPIO_PIN_ID_3 , .GpioRxPortId = GPIO_PORT_A, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_2 },
    { .RxPinId = USART_RX_PIN_BUS2_PA15, .GpioRxPinId = GPIO_PIN_ID_15, .GpioRxPortId = GPIO_PORT_A, .GpioRxAltFunctionId = GPIO_ALT_FUNC_3, .BusId = USART_BUS_2 },
    { .RxPinId = USART_RX_PIN_BUS2_PD6 , .GpioRxPinId = GPIO_PIN_ID_6 , .GpioRxPortId = GPIO_PORT_D, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_2 },
#endif
#ifdef USART3
    { .RxPinId = USART_RX_PIN_BUS3_PB11, .GpioRxPinId = GPIO_PIN_ID_11, .GpioRxPortId = GPIO_PORT_B, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .RxPinId = USART_RX_PIN_BUS3_PC5 , .GpioRxPinId = GPIO_PIN_ID_5 , .GpioRxPortId = GPIO_PORT_C, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .RxPinId = USART_RX_PIN_BUS3_PC11, .GpioRxPinId = GPIO_PIN_ID_11, .GpioRxPortId = GPIO_PORT_C, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .RxPinId = USART_RX_PIN_BUS3_PD9 , .GpioRxPinId = GPIO_PIN_ID_9 , .GpioRxPortId = GPIO_PORT_D, .GpioRxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
#endif
#ifdef UART4
    { .RxPinId = USART_RX_PIN_BUS4_PA1 , .GpioRxPinId = GPIO_PIN_ID_1 , .GpioRxPortId = GPIO_PORT_A, .GpioRxAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_4 },
    { .RxPinId = USART_RX_PIN_BUS4_PC11, .GpioRxPinId = GPIO_PIN_ID_11, .GpioRxPortId = GPIO_PORT_C, .GpioRxAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_4 },
#endif
#ifdef UART5
    { .RxPinId = USART_RX_PIN_BUS5_PD2,  .GpioRxPinId = GPIO_PIN_ID_2 , .GpioRxPortId = GPIO_PORT_D, .GpioRxAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_5 },
#endif
  },

  .GpioTxConfig = {
#ifdef USART1
    { .TxPinId = USART_TX_PIN_BUS1_PA9, .GpioTxPinId = GPIO_PIN_ID_9 , .GpioTxPortId = GPIO_PORT_A, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
    { .TxPinId = USART_TX_PIN_BUS1_PB6, .GpioTxPinId = GPIO_PIN_ID_6 , .GpioTxPortId = GPIO_PORT_B, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
    { .TxPinId = USART_TX_PIN_BUS1_PG9, .GpioTxPinId = GPIO_PIN_ID_9 , .GpioTxPortId = GPIO_PORT_G, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
#endif
#ifdef USART2
    { .TxPinId = USART_TX_PIN_BUS2_PA2, .GpioTxPinId = GPIO_PIN_ID_2 , .GpioTxPortId = GPIO_PORT_A, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_2 },
    { .TxPinId = USART_TX_PIN_BUS2_PD5, .GpioTxPinId = GPIO_PIN_ID_5 , .GpioTxPortId = GPIO_PORT_D, .GpioTxAltFunctionId = GPIO_ALT_FUNC_3, .BusId = USART_BUS_2 },
#endif
#ifdef USART3
    { .TxPinId = USART_TX_PIN_BUS3_PB10, .GpioTxPinId = GPIO_PIN_ID_10, .GpioTxPortId = GPIO_PORT_B, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .TxPinId = USART_TX_PIN_BUS3_PC4 , .GpioTxPinId = GPIO_PIN_ID_4 , .GpioTxPortId = GPIO_PORT_C, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .TxPinId = USART_TX_PIN_BUS3_PC10, .GpioTxPinId = GPIO_PIN_ID_10, .GpioTxPortId = GPIO_PORT_C, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .TxPinId = USART_TX_PIN_BUS3_PD8 , .GpioTxPinId = GPIO_PIN_ID_8 , .GpioTxPortId = GPIO_PORT_D, .GpioTxAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
#endif
#ifdef UART4
    { .TxPinId = USART_TX_PIN_BUS4_PA0 , .GpioTxPinId = GPIO_PIN_ID_0 , .GpioTxPortId = GPIO_PORT_A, .GpioTxAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_4 },
    { .TxPinId = USART_TX_PIN_BUS4_PC10, .GpioTxPinId = GPIO_PIN_ID_10, .GpioTxPortId = GPIO_PORT_C, .GpioTxAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_4 },
#endif
#ifdef UART5
    { .TxPinId = USART_TX_PIN_BUS5_PC12, .GpioTxPinId = GPIO_PIN_ID_12, .GpioTxPortId = GPIO_PORT_C, .GpioTxAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_5 },
#endif
  },

  .GpioDeConfig = {
#ifdef USART1
    { .DePinId = USART_DE_PIN_BUS1_PA12, .GpioDePinId = GPIO_PIN_ID_12, .GpioDePortId = GPIO_PORT_A, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
    { .DePinId = USART_DE_PIN_BUS1_PB3 , .GpioDePinId = GPIO_PIN_ID_3 , .GpioDePortId = GPIO_PORT_B, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
    { .DePinId = USART_DE_PIN_BUS1_PG12, .GpioDePinId = GPIO_PIN_ID_12, .GpioDePortId = GPIO_PORT_G, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_1 },
#endif
#ifdef USART2
    { .DePinId = USART_DE_PIN_BUS2_PA1, .GpioDePinId = GPIO_PIN_ID_1 , .GpioDePortId = GPIO_PORT_A, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_2 },
    { .DePinId = USART_DE_PIN_BUS2_PD4, .GpioDePinId = GPIO_PIN_ID_4 , .GpioDePortId = GPIO_PORT_D, .GpioDeAltFunctionId = GPIO_ALT_FUNC_3, .BusId = USART_BUS_2 },
#endif
#ifdef USART3
    { .DePinId = USART_DE_PIN_BUS3_PA15, .GpioDePinId = GPIO_PIN_ID_15, .GpioDePortId = GPIO_PORT_A, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .DePinId = USART_DE_PIN_BUS3_PB1 , .GpioDePinId = GPIO_PIN_ID_1 , .GpioDePortId = GPIO_PORT_B, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .DePinId = USART_DE_PIN_BUS3_PB14, .GpioDePinId = GPIO_PIN_ID_14, .GpioDePortId = GPIO_PORT_B, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .DePinId = USART_DE_PIN_BUS3_PD2 , .GpioDePinId = GPIO_PIN_ID_2 , .GpioDePortId = GPIO_PORT_D, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
    { .DePinId = USART_DE_PIN_BUS3_PD12, .GpioDePinId = GPIO_PIN_ID_12, .GpioDePortId = GPIO_PORT_D, .GpioDeAltFunctionId = GPIO_ALT_FUNC_7, .BusId = USART_BUS_3 },
#endif
#ifdef UART4
    { .DePinId = USART_DE_PIN_BUS4_PA15, .GpioDePinId = GPIO_PIN_ID_15, .GpioDePortId = GPIO_PORT_A, .GpioDeAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_4 },
#endif
#ifdef UART5
    { .DePinId = USART_DE_PIN_BUS5_PB4, .GpioDePinId = GPIO_PIN_ID_4 , .GpioDePortId = GPIO_PORT_B, .GpioDeAltFunctionId = GPIO_ALT_FUNC_8, .BusId = USART_BUS_5 },
#endif
  }
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
 * \brief Main task of module Usart
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's 
 * requirements.
 */
void Usart_Task( void )
{

}


/**
 * \brief Initialization of configuration array to default values.
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
        usartConfig->USART_Number         = USART_BUS_1;
        usartConfig->BaudRate             = 115200u;
        usartConfig->DataWidth            = USART_DATA_WIDTH_8;
        usartConfig->StopBits             = USART_STOP_BITS_1;
        usartConfig->Parity               = USART_PARITY_NONE;
        usartConfig->TransferMode         = USART_TRANSFER_MODE_TX_RX;
        usartConfig->HwFlowControl        = USART_FLOW_CONTROL_NONE;
        usartConfig->Oversampling         = USART_OVERSAMPLING_8;
        usartConfig->IrqPriority         = 10u;
        usartConfig->RxNotEmpty_ISR       = USART_NULL_PTR;
        usartConfig->TransmitEmpty_ISR    = USART_NULL_PTR;
        usartConfig->TransferComplete_ISR = USART_NULL_PTR;
        usartConfig->Error_ISR            = USART_NULL_PTR;
        usartConfig->Idle_ISR             = USART_NULL_PTR;
        usartConfig->RxTimeout_ISR        = USART_NULL_PTR;
        usartConfig->HalfDuplex           = USART_HALF_DUPLEX_INACTIVE;

        returnState = USART_REQUEST_OK;
    }
    else
    {
        returnState = USART_REQUEST_ERROR;
    }

    return ( returnState );
}


/**
 * \brief USART/UART bus initialization through configuration structure
 *
 * \param usartConfig [in]: Pointer to configuration structure
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Init( usart_BusConfig_t * const usartConfig )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_NULL_PTR != usartConfig )
    {
        rcc_FunctionState_t rccActivationState = RCC_FUNCTION_INACTIVE;
        rcc_RequestState_t  rccRequestState    = RCC_REQUEST_ERROR;

        /*------------- USART peripheral clock activation section ------------*/
        rccRequestState = Rcc_Get_PeriphState( usart_PeriphConf[ usartConfig->USART_Number ].UsartRcc, &rccActivationState );

        if( ( RCC_REQUEST_ERROR     != rccRequestState    ) &&
            ( RCC_FUNCTION_INACTIVE != rccActivationState )    )
        {
            rccRequestState = Rcc_Set_PeriphActive( usart_PeriphConf[ usartConfig->USART_Number ].UsartRcc );

            if( RCC_REQUEST_ERROR == rccRequestState )
            {
                retState = USART_REQUEST_ERROR;
                return ( retState );
            }
        }
        else
        {
            /* No clock activation needed */
        }

        /*---------- USART peripheral GPIO initialization section ------------*/
        if( ( usart_GpioConfig.GpioRxConfig[ usartConfig->BusRxPin ].BusId == usartConfig->USART_Number ) &&
            ( USART_RX_PIN_CNT                                              > usartConfig->BusRxPin     )    )
        {
            Usart_InitRxGpio( usartConfig->BusRxPin );
        }
        else
        {
            /* RX pin configuration is not used */
        }

        if( ( usart_GpioConfig.GpioTxConfig[ usartConfig->BusTxPin ].BusId == usartConfig->USART_Number ) &&
            ( USART_TX_PIN_CNT                                              > usartConfig->BusTxPin     )    )
        {
            Usart_InitTxGpio( usartConfig->BusTxPin );
        }
        else
        {
            /* TX pin configuration is not used */
        }

        if( ( usart_GpioConfig.GpioDeConfig[ usartConfig->BusDePin ].BusId == usartConfig->USART_Number ) &&
            ( USART_DE_PIN_CNT                                              > usartConfig->BusDePin     )    )
        {
            Usart_InitDeGpio( usartConfig->BusDePin );
        }
        else
        {
            /* DE pin configuration is not used */
        }

        /*------------ USART peripheral initialization section ---------------*/
        LL_USART_Disable( usart_PeriphConf[ usartConfig->USART_Number ].UsartReg );


        rccRequestState = Rcc_Set_ResetActive( usart_PeriphConf[ usartConfig->USART_Number ].UsartRcc );
        if( RCC_REQUEST_OK != rccRequestState )
        {
            retState = USART_REQUEST_ERROR;
            return ( retState );
        }

        rccRequestState = Rcc_Set_ResetInactive( usart_PeriphConf[ usartConfig->USART_Number ].UsartRcc );
        if( RCC_REQUEST_OK != rccRequestState )
        {
            retState = USART_REQUEST_ERROR;
            return ( retState );
        }

        /* Over-sampling configuration must be executed before baud-rate configuration */
        retState = Usart_Set_Oversampling( usartConfig->USART_Number, usartConfig->Oversampling );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_Baudrate( usartConfig->USART_Number, usartConfig->BaudRate );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_DataWidth( usartConfig->USART_Number, usartConfig->DataWidth );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_StopBits( usartConfig->USART_Number, usartConfig->StopBits );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_Parity( usartConfig->USART_Number, usartConfig->Parity );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_TransferMode( usartConfig->USART_Number, usartConfig->TransferMode );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_FlowControl( usartConfig->USART_Number, usartConfig->HwFlowControl );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_HalfDuplexState( usartConfig->USART_Number, usartConfig->HalfDuplex );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_DriverEnableState( usartConfig->USART_Number, usartConfig->DriverEnableMode );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_DriverEnablePolarity( usartConfig->USART_Number, usartConfig->DriverEnablePolarity );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        retState = Usart_Set_PinLevels( usartConfig->USART_Number, usartConfig->RxPinOperationLevels, usartConfig->TxPinOperationLevels );
        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }


        if( USART_RX_TIMEOUT_MIN != usartConfig->RxTimeoutValue )
        {
            retState = Usart_Set_RxTimeoutActive( usartConfig->USART_Number, usartConfig->RxTimeoutValue );
        }
        else
        {
            retState = Usart_Set_RxTimeoutInactive( usartConfig->USART_Number );
        }

        if( USART_REQUEST_OK != retState )
        {
            return ( retState );
        }

        LL_USART_Enable( usart_PeriphConf[ usartConfig->USART_Number ].UsartReg );

        Usart_Set_IrqPriority          ( usartConfig->USART_Number, usartConfig->IrqPriority          );

        /* Return states of following functions are not checked. User do not need to strictly configure those options */
        ( void ) Usart_Set_RxNotEmptyIsrCallback( usartConfig->USART_Number, usartConfig->RxNotEmpty_ISR       );
        ( void ) Usart_Set_TxEmptyIsrCallback   ( usartConfig->USART_Number, usartConfig->TransmitEmpty_ISR    );
        ( void ) Usart_Set_TxCompleteIsrCallback( usartConfig->USART_Number, usartConfig->TransferComplete_ISR );
        ( void ) Usart_Set_IdleIsrCallback      ( usartConfig->USART_Number, usartConfig->Idle_ISR             );
        ( void ) Usart_Set_RxTimeoutIsrCallback ( usartConfig->USART_Number, usartConfig->RxTimeout_ISR        );
        ( void ) Usart_Set_ErrorIsrCallback     ( usartConfig->USART_Number, usartConfig->Error_ISR            );

        retState = USART_REQUEST_OK;
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
 * \param usartConfig [in]: Pointer to configuration structure
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Deinit( usart_BusConfig_t const * usartConfig )
{
    usart_RequestState_t returnState = USART_REQUEST_ERROR;

    if( USART_NULL_PTR != usartConfig )
    {
        Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartConfig->USART_Number ].UsartNvic );

        LL_USART_Disable( usart_PeriphConf[ usartConfig->USART_Number ].UsartReg );

        Rcc_Set_PeriphInactive( usart_PeriphConf[ usartConfig->USART_Number ].UsartRcc );

        returnState = USART_REQUEST_OK;
    }
    else
    {
        returnState = USART_REQUEST_ERROR;
    }

    return ( returnState );
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
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Baudrate( usart_BusId_t usartId, usart_Baudrate_t baudrate )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_Oversampling_t oversampling   = USART_OVERSAMPLING_16;
    uint32_t             usartPeriphClk = 0;
    usart_Prescaler_t    usartPrescaler = 0u;
    uint32_t             baudrateReg    = 0u;

    retValue = Usart_Get_Oversampling( usartId, &oversampling );

    rcc_RequestState_t   rccRequestState    = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].UsartRcc, &usartPeriphClk );
    usart_RequestState_t prescCalcState     = Usart_Get_ExpectedPrescaler( usartPeriphClk, oversampling, baudrate, &usartPrescaler);
    usart_RequestState_t prescCalcConfState = Usart_Set_Prescaler( usartId, usartPrescaler );

    if( ( 0u                  != baudrate           ) &&
        ( RCC_REQUEST_ERROR   != rccRequestState    ) &&
        ( USART_REQUEST_ERROR != prescCalcState     ) &&
        ( USART_REQUEST_ERROR != prescCalcConfState ) &&
        ( USART_REQUEST_ERROR != retValue           ) &&
        ( USART_BUS_CNT        > usartId            )    )
    {
        LL_USART_SetBaudRate( usart_PeriphConf[ usartId ].UsartReg,
                              usartPeriphClk,
                              usartPrescaler,
                              oversampling,
                              baudrate );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            baudrateReg = LL_USART_GetBaudRate( usart_PeriphConf[ usartId ].UsartReg,
                                                usartPeriphClk,
                                                usartPrescaler,
                                                oversampling );

            if( baudrate == baudrateReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the baud-rate configuration for selected USART/UART bus.
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param baudrate [out]: Value of required baud-rate
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Baudrate( usart_BusId_t usartId, usart_Baudrate_t * const baudrate )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    usart_Oversampling_t oversampling   = USART_OVERSAMPLING_16;
    uint32_t             usartPeriphClk = 0;
    usart_Prescaler_t    usartPrescaler = 0u;

    usart_RequestState_t oversamplingState = Usart_Get_Oversampling( usartId, &oversampling );
    rcc_RequestState_t   rccRequestState   = Rcc_Get_PeriphClk( usart_PeriphConf[ usartId ].UsartRcc, &usartPeriphClk );
    usart_RequestState_t prescState        = Usart_Get_Prescaler( usartPeriphClk, &usartPrescaler);

    if( ( USART_NULL_PTR      != baudrate          ) &&
        ( RCC_REQUEST_ERROR   != rccRequestState   ) &&
        ( USART_REQUEST_ERROR != prescState        ) &&
        ( USART_REQUEST_ERROR != oversamplingState )    )
    {
        *baudrate = LL_USART_GetBaudRate( usart_PeriphConf[ usartId ].UsartReg,
                                          usartPeriphClk,
                                          usartPrescaler,
                                          oversampling );

        retValue = USART_REQUEST_OK;
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
 * \param usartId   [in]: USART/UART bus identification.
 * \param dataWidth [in]: Required data width configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DataWidth( usart_BusId_t usartId, usart_DataWidth_t dataWidth )
{
    usart_RequestState_t retValue     = USART_REQUEST_ERROR;
    uint32_t             dataWidthReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetDataWidth( usart_PeriphConf[ usartId ].UsartReg, dataWidth );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            dataWidthReg = LL_USART_GetDataWidth( usart_PeriphConf[ usartId ].UsartReg );

            if( dataWidth == dataWidthReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the data width configuration for selected USART/UART bus
 *
 * \param usartId    [in]: USART/UART bus identification.
 * \param dataWidth [out]: Value of data width configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DataWidth( usart_BusId_t usartId, usart_DataWidth_t * const dataWidth )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId   ) &&
        ( USART_NULL_PTR != dataWidth )    )
    {
        *dataWidth = LL_USART_GetDataWidth( usart_PeriphConf[ usartId ].UsartReg );
        retValue = USART_REQUEST_OK;
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
 * \param usartId  [in]: USART/UART bus identification.
 * \param stopBits [in]: Required stop-bits configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_StopBits( usart_BusId_t usartId, usart_StopBits_t stopBits )
{
    usart_RequestState_t retValue    = USART_REQUEST_ERROR;
    uint32_t             stopBitsReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetStopBitsLength( usart_PeriphConf[ usartId ].UsartReg, stopBits );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            stopBitsReg = LL_USART_GetStopBitsLength( usart_PeriphConf[ usartId ].UsartReg );

            if( stopBits == stopBitsReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the stop-bits configuration for selected USART/UART bus
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param stopBits [out]: Value of stop-bits configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_StopBits( usart_BusId_t usartId, usart_StopBits_t * const stopBits )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != stopBits )    )
    {
        *stopBits = LL_USART_GetStopBitsLength( usart_PeriphConf[ usartId ].UsartReg );
        retValue = USART_REQUEST_OK;
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
 * \param usartId [in]: USART/UART bus identification.
 * \param parity  [in]: Required parity configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Parity( usart_BusId_t usartId, usart_Parity_t parity )
{
    usart_RequestState_t retValue  = USART_REQUEST_ERROR;
    uint32_t             parityReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetParity( usart_PeriphConf[ usartId ].UsartReg, parity );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            parityReg = LL_USART_GetParity( usart_PeriphConf[ usartId ].UsartReg );

            if( parity == parityReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the parity configuration for selected USART/UART bus
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param parity  [out]: Value of parity configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Parity( usart_BusId_t usartId, usart_Parity_t * const parity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId ) &&
        ( USART_NULL_PTR != parity  )    )
    {
        *parity = LL_USART_GetParity( usart_PeriphConf[ usartId ].UsartReg );
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
 * \param usartId      [in]: USART/UART bus identification.
 * \param transferMode [in]: Required transfer mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TransferMode( usart_BusId_t usartId, usart_TransferMode_t transferMode )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             transferModeReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetTransferDirection( usart_PeriphConf[ usartId ].UsartReg, transferMode );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            transferModeReg = LL_USART_GetTransferDirection( usart_PeriphConf[ usartId ].UsartReg );

            if( transferMode == transferModeReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the transfer mode configuration for selected USART/UART bus
 *
 * \param usartId       [in]: USART/UART bus identification.
 * \param transferMode [out]: Value of transfer mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TransferMode( usart_BusId_t usartId, usart_TransferMode_t * const transferMode )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId      ) &&
        ( USART_NULL_PTR != transferMode )    )
    {
        *transferMode = LL_USART_GetTransferDirection( usart_PeriphConf[ usartId ].UsartReg );
        retValue = USART_REQUEST_OK;
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
 * \param usartId     [in]: USART/UART bus identification.
 * \param flowControl [in]: Required hardware flow control configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_FlowControl( usart_BusId_t usartId, usart_FlowControl_t flowControl )
{
    usart_RequestState_t retValue       = USART_REQUEST_ERROR;
    uint32_t             flowControlReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetHWFlowCtrl( usart_PeriphConf[ usartId ].UsartReg, flowControl );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            flowControlReg = LL_USART_GetHWFlowCtrl( usart_PeriphConf[ usartId ].UsartReg );

            if( flowControl == flowControlReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
usart_RequestState_t Usart_Get_FlowControl( usart_BusId_t usartId, usart_FlowControl_t * const flowControl )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId     ) &&
        ( USART_NULL_PTR != flowControl )    )
    {
        *flowControl = LL_USART_GetHWFlowCtrl( usart_PeriphConf[ usartId ].UsartReg );
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
 * \param usartId [in]: USART/UART bus identification.
 * \param deState [in]: Required Driver Enable (DE) mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DriverEnableState( usart_BusId_t usartId, usart_DeFeatureState_t deState )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             driverEnableReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_DE_DISABLED != deState )
        {
            LL_USART_EnableDEMode( usart_PeriphConf[ usartId ].UsartReg );
        }
        else
        {
            LL_USART_DisableDEMode( usart_PeriphConf[ usartId ].UsartReg );
        }

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            driverEnableReg = LL_USART_IsEnabledDEMode( usart_PeriphConf[ usartId ].UsartReg );

            if( deState == driverEnableReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the Driver Enable (DE) feature activation state
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param deState [out]: Value of Driver Enable (DE) mode configuration
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DriverEnableState( usart_BusId_t usartId, usart_DeFeatureState_t * const deState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId ) &&
        ( USART_NULL_PTR != deState  )    )
    {
        *deState  = LL_USART_IsEnabledDEMode( usart_PeriphConf[ usartId ].UsartReg );
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
 * \param usartId    [in]: USART/UART bus identification.
 * \param dePolarity [in]: Logic level for active state of Driver Enable (DE) pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DriverEnablePolarity( usart_BusId_t usartId, usart_DePolarity_t dePolarity )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             driverEnableReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetDESignalPolarity( usart_PeriphConf[ usartId ].UsartReg, dePolarity );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            driverEnableReg = LL_USART_GetDESignalPolarity( usart_PeriphConf[ usartId ].UsartReg );

            if( dePolarity == driverEnableReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns the Driver Enable (DE) active logic level
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param dePolarity [out]: Logic level for active state of Driver Enable (DE) pin
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DriverEnablePolarity( usart_BusId_t usartId, usart_DePolarity_t * const dePolarity )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId    ) &&
        ( USART_NULL_PTR != dePolarity )    )
    {
        *dePolarity  = LL_USART_GetDESignalPolarity( usart_PeriphConf[ usartId ].UsartReg );
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * @brief Configures assertion and de-assertion times of Driver Enable (DE) pin.
 *
 * This table provides the DE assertion time for various baud rates
 * and oversampling settings (16x and 8x).
 *
 *  ------------------------------------------------------------------------------------------
 * | Baud Rate (bps) | Oversampling | Min DE Assertion Time (µs) | Max DE Assertion Time (µs) |
 * |-----------------|--------------|----------------------------|----------------------------|
 * | 9600            | 16x          | 6.51                       | 208.33                     |
 * | 9600            | 8x           | 13.02                      | 416.67                     |
 * | 115200          | 16x          | 0.54                       | 27.08                      |
 * | 115200          | 8x           | 1.09                       | 54.17                      |
 * | 1000000         | 16x          | 0.0625                     | 2                          |
 * | 1000000         | 8x           | 0.125                      | 4                          |
 *  ------------------------------------------------------------------------------------------
 *
 * DE Assertion Time Calculation Formulas:
 *
 * For 16x Oversampling (OVER8 = 0):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 16}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 16}
 *
 * For 8x Oversampling (OVER8 = 1):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 8}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 8}
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param assertTime   [in]: Pin Driver Enable (DE) assertion time in us
 * \param deassertTime [in]: Pin Driver Enable (DE) de-assertion time in us
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_AssertDeassertTimes( usart_BusId_t usartId, usart_AssertTime_us_t assertTime, usart_DeassertTime_us_t deassertTime )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_Baudrate_t     baudrateVal     = 0u;
        usart_Oversampling_t oversamplingVal = 0u;

        usart_RequestState_t baudRetValue       = Usart_Get_Baudrate( usartId, &baudrateVal );
        usart_RequestState_t oversampleRetValue = Usart_Get_Oversampling( usartId, &oversamplingVal );

        if( ( USART_REQUEST_ERROR != baudRetValue       ) &&
            ( USART_REQUEST_ERROR != oversampleRetValue )    )
        {
            retValue = USART_REQUEST_OK;

            const uint8_t assertTargetVal   = Usart_Get_AssertDeassertRegValues( assertTime, baudrateVal, oversamplingVal );
            const uint8_t deassertTargetVal = Usart_Get_AssertDeassertRegValues( deassertTime, baudrateVal, oversamplingVal );

            LL_USART_SetDEAssertionTime( usart_PeriphConf[ usartId ].UsartReg, assertTargetVal );
            LL_USART_SetDEDeassertionTime( usart_PeriphConf[ usartId ].UsartReg, deassertTargetVal );

            for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
            {
                uint8_t assertTimeReg   = LL_USART_GetDEAssertionTime( usart_PeriphConf[ usartId ].UsartReg );
                uint8_t deassertTimeReg = LL_USART_GetDEDeassertionTime( usart_PeriphConf[ usartId ].UsartReg );

                if( ( assertTargetVal   == assertTimeReg   ) &&
                    ( deassertTargetVal == deassertTimeReg )    )
                {
                    retValue = USART_REQUEST_OK;
                    break;
                }
                else
                {
                    /* Clock source has not yet been changed, keep return state as error */
                    retValue = USART_REQUEST_ERROR;
                }
            }
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
 * @brief Returns assertion and de-assertion times of Driver Enable (DE) pin.
 *
 * This table provides the DE assertion time for various baud rates
 * and oversampling settings (16x and 8x).
 *
 *  ------------------------------------------------------------------------------------------
 * | Baud Rate (bps) | Oversampling | Min DE Assertion Time (µs) | Max DE Assertion Time (µs) |
 * |-----------------|--------------|----------------------------|----------------------------|
 * | 9600            | 16x          | 6.51                       | 208.33                     |
 * | 9600            | 8x           | 13.02                      | 416.67                     |
 * | 115200          | 16x          | 0.54                       | 27.08                      |
 * | 115200          | 8x           | 1.09                       | 54.17                      |
 * | 1000000         | 16x          | 0.0625                     | 2                          |
 * | 1000000         | 8x           | 0.125                      | 4                          |
 *  ------------------------------------------------------------------------------------------
 *
 * DE Assertion Time Calculation Formulas:
 *
 * For 16x Oversampling (OVER8 = 0):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 16}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 16}
 *
 * For 8x Oversampling (OVER8 = 1):
 * Min DE Assertion Time (DEAT = 0):
 * \text{Min DE Assertion Time} = \frac{0 + 1}{\text{Baud Rate} \times 8}
 * Max DE Assertion Time (DEAT = 31):
 * \text{Max DE Assertion Time} = \frac{31 + 1}{\text{Baud Rate} \times 8}
 *
 * \param usartId       [in]: USART/UART bus identification.
 * \param assertTime   [out]: Pin Driver Enable (DE) assertion time in us
 * \param deassertTime [out]: Pin Driver Enable (DE) de-assertion time in us
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_AssertDeassertTimes( usart_BusId_t usartId, usart_AssertTime_us_t * const assertTime, usart_DeassertTime_us_t * const deassertTime )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId      ) &&
        ( USART_NULL_PTR != assertTime   ) &&
        ( USART_NULL_PTR != deassertTime )    )
    {
        usart_Baudrate_t     baudrateVal     = 0u;
        usart_Oversampling_t oversamplingVal = 0u;

        usart_RequestState_t baudRetValue       = Usart_Get_Baudrate( usartId, &baudrateVal );
        usart_RequestState_t oversampleRetValue = Usart_Get_Oversampling( usartId, &oversamplingVal );

        if( ( USART_REQUEST_ERROR != baudRetValue       ) &&
            ( USART_REQUEST_ERROR != oversampleRetValue )    )
        {
            retValue = USART_REQUEST_OK;

            uint8_t assertRegVal   = LL_USART_GetDEAssertionTime( usart_PeriphConf[ usartId ].UsartReg );
            uint8_t deassertRegVal = LL_USART_GetDEDeassertionTime( usart_PeriphConf[ usartId ].UsartReg );

            *assertTime   = Usart_Get_AssertDeassertTime( assertRegVal, baudrateVal, oversamplingVal );
            *deassertTime = Usart_Get_AssertDeassertTime( deassertRegVal, baudrateVal, oversamplingVal );

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
 * \brief Configures over-sampling mode for required USART/UART bus
 *
 * \param usartId          [in]: USART/UART bus identification.
 * \param oversamplingMode [in]: Required over-sampling mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_Oversampling( usart_BusId_t usartId, usart_Oversampling_t oversamplingMode )
{
    usart_RequestState_t retValue        = USART_REQUEST_ERROR;
    uint32_t             oversamplingReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetOverSampling( usart_PeriphConf[ usartId ].UsartReg,
                                  oversamplingMode );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            oversamplingReg = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].UsartReg );

            if( oversamplingMode == oversamplingReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns configured over-sampling mode for required USART/UART bus
 *
 * \param usartId           [in]: USART/UART bus identification.
 * \param oversamplingMode [out]: Configured over-sampling mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_Oversampling( usart_BusId_t usartId, usart_Oversampling_t * const oversamplingMode )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        *oversamplingMode = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].UsartReg );
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
 * \param usartId         [in]: USART/UART bus identification.
 * \param halfDuplexState [in]: Required half-duplex state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_HalfDuplexState( usart_BusId_t usartId, usart_HalfDuplex_t halfDuplexState )
{
    usart_RequestState_t retValue           = USART_REQUEST_ERROR;
    uint32_t             halfDuplexStateReg = 0u;

    if( USART_BUS_CNT > usartId )
    {
        if( USART_HALF_DUPLEX_INACTIVE != halfDuplexState )
        {
            LL_USART_EnableHalfDuplex( usart_PeriphConf[ usartId ].UsartReg );
        }
        else
        {
            LL_USART_DisableHalfDuplex( usart_PeriphConf[ usartId ].UsartReg );
        }

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            halfDuplexStateReg = LL_USART_IsEnabledHalfDuplex( usart_PeriphConf[ usartId ].UsartReg );

            if( halfDuplexState == halfDuplexStateReg )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns half-duplex state for required USART/UART bus
 *
 * \param usartId          [in]: USART/UART bus identification.
 * \param halfDuplexState [out]: Half-duplex state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_HalfDuplexState( usart_BusId_t usartId, usart_HalfDuplex_t * const halfDuplexState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        uint32_t halfDuplexStateReg = LL_USART_GetOverSampling( usart_PeriphConf[ usartId ].UsartReg );

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
 * This feature gives the Receiver timeout value in terms of number of bits
 * during which there is no activity on the RX line. In standard mode, the RTOF
 * flag is set if, after the last received character, no new start bit is
 * detected for more than the RTO value.
 *
 * \param usartId        [in]: USART/UART bus identification.
 * \param timeoutBitsCnt [in]: Count of bits triggering receiver timeout flag
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutActive( usart_BusId_t usartId, usart_RxTimeout_t timeoutBitsCnt )
{
    usart_RequestState_t retValue     = USART_REQUEST_ERROR;
    uint32_t             regValue     = 0u;
    uint32_t             timeoutValue = 0u;

    if( ( USART_BUS_CNT        > usartId        ) &&
        ( USART_RX_TIMEOUT_MAX > timeoutBitsCnt )    )
    {
        LL_USART_EnableRxTimeout( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_SetRxTimeout( usart_PeriphConf[ usartId ].UsartReg, timeoutBitsCnt );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue     = LL_USART_IsEnabledRxTimeout( usart_PeriphConf[ usartId ].UsartReg );
            timeoutValue = LL_USART_GetRxTimeout( usart_PeriphConf[ usartId ].UsartReg );

            if( ( 0u              != regValue     ) &&
                ( timeoutBitsCnt  == timeoutValue )    )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief De-activates receive timeout feature.
 *
 * This feature gives the Receiver timeout value in terms of number of bits
 * during which there is no activity on the RX line. In standard mode, the RTOF
 * flag is set if, after the last received character, no new start bit is
 * detected for more than the RTO value.
 *
 * \param usartId [in]: USART/UART bus identification.
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableRxTimeout( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledRxTimeout( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of receiver timeout feature
 *
 * \param usartId   [in]: USART/UART bus identification.
 * \param reqState [out]: Receiver timeout feature activation status
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxTimeoutState( usart_BusId_t usartId, usart_FlagState_t * const reqState )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        uint32_t regValue = LL_USART_IsEnabledRxTimeout( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Configures pins operation modes
 *
 * Transmit and receive pins can be independently configured to use normal or
 * inverted mode.
 *
 * \param usartId     [in]: USART/UART bus identification.
 * \param rxPinLevels [in]: RX pin operation mode
 * \param txPinLevels [in]: TX pin operation mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_PinLevels( usart_BusId_t usartId, usart_RxPinLevel_t rxPinLevels, usart_TxPinLevel_t txPinLevels )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_SetRXPinLevel( usart_PeriphConf[ usartId ].UsartReg, rxPinLevels );
        LL_USART_SetTXPinLevel( usart_PeriphConf[ usartId ].UsartReg, txPinLevels );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t rxPinLevelReg = LL_USART_GetRXPinLevel( usart_PeriphConf[ usartId ].UsartReg );
            uint32_t txPinLevelReg = LL_USART_GetTXPinLevel( usart_PeriphConf[ usartId ].UsartReg );

            if( ( rxPinLevels == rxPinLevelReg ) &&
                ( txPinLevels == txPinLevelReg )    )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Receives pins operation modes
 *
 * Transmit and receive pins can be independently configured to use normal or
 * inverted mode.
 *
 * \param usartId      [in]: USART/UART bus identification.
 * \param rxPinLevels [out]: RX pin operation mode
 * \param txPinLevels [out]: TX pin operation mode
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_PinLevels( usart_BusId_t usartId, usart_RxPinLevel_t * const rxPinLevels, usart_TxPinLevel_t * const txPinLevels )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        *rxPinLevels = LL_USART_GetRXPinLevel( usart_PeriphConf[ usartId ].UsartReg );
        *txPinLevels = LL_USART_GetTXPinLevel( usart_PeriphConf[ usartId ].UsartReg );

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Reads transmission register address
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param regAddr [out]: Address of transmission register
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxRegisterAddr( usart_BusId_t usartId, usart_RxRegAddr_t * const regAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != regAddr )    )
    {
        *regAddr = usart_PeriphConf[ usartId ].UsartReg->TDR;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Reads reception register address
 *
 * \param usartId  [in]: USART/UART bus identification.
 * \param regAddr [out]: Address of reception register
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxRegisterAddr( usart_BusId_t usartId, usart_RxRegAddr_t * const regAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != regAddr )    )
    {
        *regAddr = usart_PeriphConf[ usartId ].UsartReg->RDR;

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
 * \param txData  [in]: Data to be transmitted
 */
void Usart_SendData( usart_BusId_t usartId, usart_TxData_t txData )
{
    usart_PeriphConf[ usartId ].UsartReg->TDR = txData;
}


/**
 * \brief Reads data from reception register
 *
 * \param usartId [in]: USART/UART bus identification.
 * \return Received data value
 */
usart_RxData_t Usart_ReadData( usart_BusId_t usartId )
{
    return ( usart_PeriphConf[ usartId ].UsartReg->RDR );
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
usart_RequestState_t Usart_Set_InterruptsActive( usart_BusId_t usartId )
{
    usart_RequestState_t retState               = USART_REQUEST_ERROR;
    nvic_RequestState_t  nvicActivationState    = NVIC_REQUEST_ERROR;
    nvic_RequestState_t  nvicHandlerConfigState = NVIC_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicHandlerConfigState = Nvic_Set_PeriphIrq_Handler( usart_PeriphConf[ usartId ].UsartNvic,
                                                             usart_PeriphConf[ usartId ].UsartIsr );

        nvicActivationState = Nvic_Set_PeriphIrq_Active( usart_PeriphConf[ usartId ].UsartNvic );

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
 * \brief De-activates global Interrupt Requests (IRQ) for USART/UART peripheral
 *        and configures Interrupt Service Routine (ISR).
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_InterruptsInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicState = Nvic_Set_PeriphIrq_Inactive( usart_PeriphConf[ usartId ].UsartNvic );

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
usart_RequestState_t Usart_Set_IrqPriority( usart_BusId_t usartId, usart_IrqPrio_t irqPrio )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        nvicState = Nvic_Set_PeriphIrq_Prio( usart_PeriphConf[ usartId ].UsartNvic, irqPrio );

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
usart_RequestState_t Usart_Get_IrqPriority( usart_BusId_t usartId, usart_IrqPrio_t * const irqPrio )
{
    usart_RequestState_t retState  = USART_REQUEST_ERROR;
    nvic_RequestState_t  nvicState = NVIC_REQUEST_ERROR;

    if( ( USART_BUS_CNT   > usartId ) &&
        ( USART_NULL_PTR != irqPrio )    )
    {
        nvicState = Nvic_Get_PeriphIrq_Prio( usart_PeriphConf[ usartId ].UsartNvic, irqPrio );

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
usart_RequestState_t Usart_Set_DmaTxRequestActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableDMAReq_TX( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief De-activates USART/UART DMA transfer requests for data transmission
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
usart_RequestState_t Usart_Set_DmaTxRequestInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableDMAReq_TX( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation status of USART/UART DMA transfer request of data transmission
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For transmission is this trigger activated by HW, when transmit data register
 * is empty.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation status of transfer request state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DmaTxReqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledDMAReq_TX( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Activates USART/UART DMA transfer requests for data reception
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaRxRequestActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableDMAReq_RX( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief De-activates USART/UART DMA transfer requests for data reception
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_DmaRxRequestInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableDMAReq_RX( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation status of USART/UART DMA RX trigger request
 *
 * When DMA peripheral is used for data transfer, the DMA trigger has to be
 * activated, to trigger transfer of data from/to memory from/to peripheral.
 * For reception is this trigger activated by HW, when receive data register
 * is not empty.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation of transfer request state
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_DmaRxReqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledDMAReq_RX( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Activates Receive register Not Empty (RXNE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIrqActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ReceiveData8(  usart_PeriphConf[ usartId ].UsartReg );

        LL_USART_EnableIT_RXNE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Deactivates Receive register Not Empty (RXNE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIrqInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ReceiveData8(  usart_PeriphConf[ usartId ].UsartReg );

        LL_USART_DisableIT_RXNE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of Receive register Not Empty (RXNE) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of RX Not Empty (RXNE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxNotEmptyIrqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_RXNE( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Configures Receive register Not Empty (RXNE) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxNotEmptyIsrCallback( usart_BusId_t usartId, usart_RxNeIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].RxNotEmptyIsr = callback;

        retValue = USART_REQUEST_OK;
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
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIrqActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_EnableIT_TXE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Deactivates Transmit register Empty (TXE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIrqInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_TXE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of Transmit Register Empty (TXE) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of TX Empty (TXE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxEmptyIrqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_TXE( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Configures Transmit Register Empty (TXE) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxEmptyIsrCallback( usart_BusId_t usartId, usart_TxeIrqCallback_t * const callback )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].TxEmptyIsr = callback;

        retValue = USART_REQUEST_OK;
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
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIrqActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_TC( usart_PeriphConf[ usartId ].UsartReg );

        LL_USART_EnableIT_TC( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Deactivates Transmission Complete (TC) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIrqInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_TC( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of Transmit Complete (TC) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of TX Complete (TC) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_TxCompleteIrqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Configures Transmit Complete (TC) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_TxCompleteIsrCallback( usart_BusId_t usartId, usart_TcIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].TransferCompleteIsr = callback;

        retValue = USART_REQUEST_OK;
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
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIrqActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_IDLE( usart_PeriphConf[ usartId ].UsartReg );

        LL_USART_EnableIT_IDLE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_IDLE( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Deactivates Idle detection (IDLE) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIrqInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_IDLE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_TC( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of Idle detection (IDLE) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of Idle detection (IDLE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_IdleIrqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_IDLE( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Configures Idle detection (IDLE) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_IdleIsrCallback( usart_BusId_t usartId, usart_IdleIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].IdleIsr = callback;

        retValue = USART_REQUEST_OK;
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
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIrqActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_RTO( usart_PeriphConf[ usartId ].UsartReg );

        LL_USART_EnableIT_RTO( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RTO( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u != regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Deactivates Receive Timeout (RTO) interrupt request.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIrqInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    uint32_t             regValue = 0u;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_RTO( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            regValue = LL_USART_IsEnabledIT_RTO( usart_PeriphConf[ usartId ].UsartReg );

            if( 0u == regValue )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of Receive Timeout (RTO) interrupt request.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state Receive Timeout (RTO) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_RxTimeoutIrqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regValue = LL_USART_IsEnabledIT_RTO( usart_PeriphConf[ usartId ].UsartReg );

        if( 0u != regValue )
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
 * \brief Configures Receive Timeout (RTO) interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_RxTimeoutIsrCallback( usart_BusId_t usartId, usart_RxTimeoutIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].RxTimeoutIsr = callback;

        retValue = USART_REQUEST_OK;
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
 * error interrupt is handled together with other errors.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_ErrorIrqActive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_ClearFlag_FE( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_ORE( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_NE( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_PE( usart_PeriphConf[ usartId ].UsartReg );

        LL_USART_EnableIT_ERROR( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_EnableIT_PE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t regErrValue = LL_USART_IsEnabledIT_ERROR( usart_PeriphConf[ usartId ].UsartReg );
            uint32_t regPEValue  = LL_USART_IsEnabledIT_PE( usart_PeriphConf[ usartId ].UsartReg );

            if( ( 0u != regErrValue ) &&
                ( 0u != regPEValue  )    )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Deactivates Error interrupt request.
 *
 * When set, Error Interrupt Enable Bit is enabling interrupt generation in case
 * of a framing error, overrun error or noise flag. De-activation of the parity
 * error interrupt is handled together with other errors.
 *
 * \param usartId [in]: USART/UART peripheral ID
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_ErrorIrqInactive( usart_BusId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        LL_USART_DisableIT_ERROR( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_DisableIT_PE( usart_PeriphConf[ usartId ].UsartReg );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            uint32_t regErrValue = LL_USART_IsEnabledIT_ERROR( usart_PeriphConf[ usartId ].UsartReg );
            uint32_t regPEValue  = LL_USART_IsEnabledIT_PE( usart_PeriphConf[ usartId ].UsartReg );

            if( ( 0u == regErrValue ) &&
                ( 0u == regPEValue  )    )
            {
                retValue = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retValue = USART_REQUEST_ERROR;
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
 * \brief Returns activation state of Error interrupt request.
 *
 * \note  When set, Error Interrupt Enable Bit is enabling interrupt generation
 *        in case of a framing error, overrun error or noise flag. Activation
 *        and de-activation of the parity error interrupt is handled together
 *        with other errors.
 *
 * \param usartId   [in]: USART/UART peripheral ID
 * \param reqState [out]: Activation state of TX Empty (TXE) interrupt request
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Get_ErrorIrqState( usart_BusId_t usartId, usart_FlagState_t * const reqState)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId  ) &&
        ( USART_NULL_PTR != reqState )    )
    {
        uint32_t regErrValue = LL_USART_IsEnabledIT_ERROR( usart_PeriphConf[ usartId ].UsartReg );
        uint32_t regPEValue  = LL_USART_IsEnabledIT_PE( usart_PeriphConf[ usartId ].UsartReg );

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
 * \brief Configures error interrupt callback
 *
 * \param usartId  [in]: USART/UART peripheral ID
 * \param callback [in]: Pointer to interrupt callback
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_Set_ErrorIsrCallback( usart_BusId_t usartId, usart_ErrIrqCallback_t * const callback)
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        usart_RuntimeData[ usartId ].ErrorIsr = callback;

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
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitRxGpio( usart_RxPin_t pinId )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        pinConfig;

    pinConfig.PortId         = usart_GpioConfig.GpioRxConfig[ pinId ].GpioRxPortId;
    pinConfig.PinId          = usart_GpioConfig.GpioRxConfig[ pinId ].GpioRxPinId;
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = usart_GpioConfig.GpioRxConfig[ pinId ].GpioRxAltFunctionId;

    /* Initialize GPIO */
    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
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
 * \brief Initializes GPIO TX pin used by peripheral
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitTxGpio( usart_TxPin_t pinId )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        pinConfig;

    pinConfig.PortId         = usart_GpioConfig.GpioTxConfig[ pinId ].GpioTxPortId;
    pinConfig.PinId          = usart_GpioConfig.GpioTxConfig[ pinId ].GpioTxPinId;
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = usart_GpioConfig.GpioTxConfig[ pinId ].GpioTxAltFunctionId;

    /* Initialize GPIO */
    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
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
 * \brief Initializes GPIO Driver Enable (DE) pin used by peripheral
 *
 * \param pinId [in]: Pin identification
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_InitDeGpio( usart_DePin_t pinId )
{
    usart_RequestState_t retValue      = USART_REQUEST_ERROR;
    gpio_RequestState_t  gpioInitState = GPIO_REQUEST_ERROR;
    gpio_Config_t        pinConfig;

    pinConfig.PortId         = usart_GpioConfig.GpioDeConfig[ pinId ].GpioDePortId;
    pinConfig.PinId          = usart_GpioConfig.GpioDeConfig[ pinId ].GpioDePinId;
    pinConfig.PinMode        = GPIO_PIN_MODE_ALTERNATE;
    pinConfig.PinPull        = GPIO_PIN_PULL_NONE;
    pinConfig.PinSpeed       = GPIO_PIN_SPEED_MEDIUM;
    pinConfig.PinOutType     = GPIO_PIN_OUTPUT_PUSHPULL;
    pinConfig.PinAltFunction = usart_GpioConfig.GpioDeConfig[ pinId ].GpioDeAltFunctionId;

    /* Initialize GPIO */
    gpioInitState = Gpio_Init( &pinConfig );

    if( GPIO_REQUEST_ERROR != gpioInitState )
    {
        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/* =========================== LOCAL FUNCTIONS ============================== */

/**
 * \brief Sets the prescaler value for the required USART/UART bus
 *
 * \param usartId   [in]: USART/UART bus identification
 * \param prescaler [in]: Value of prescaler
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static usart_RequestState_t Usart_Set_Prescaler( usart_BusId_t usartId, usart_Prescaler_t prescaler )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;
    uint32_t             prescReg = 0u;

    if( ( USART_BUS_CNT           > usartId   ) &&
        ( USART_PRESCALER_CNT > prescaler )    )
    {
        LL_USART_SetPrescaler( usart_PeriphConf[ usartId ].UsartReg,
                               prescaler );

        for( uint32_t iterationCnt = 0u; USART_TIMEOUT_RAW > iterationCnt; iterationCnt ++ )
        {
            prescReg = LL_USART_GetPrescaler( usart_PeriphConf[ usartId ].UsartReg );

            if( prescaler == prescReg )
            {
                retState = USART_REQUEST_OK;
                break;
            }
            else
            {
                /* Clock source has not yet been changed, keep return state as error */
                retState = USART_REQUEST_ERROR;
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
 * \brief Reads the value of prescaler from required USART/UART bus
 *
 * \param usartId    [in]: USART/UART bus identification
 * \param prescaler [out]: Value of prescaler
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static usart_RequestState_t Usart_Get_Prescaler( usart_BusId_t usartId, usart_Prescaler_t *prescaler )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( ( USART_BUS_CNT       > usartId   ) &&
        ( USART_NULL_PTR != prescaler )    )
    {
        *prescaler = LL_USART_GetPrescaler( usart_PeriphConf[ usartId ].UsartReg );

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}

/* =========================== INTERRUPT HANDLERS =========================== */

/**
 * \brief Global interrupt handler.
 *
 * \param usartId [in]: USART/UART bus identification
 */
inline void Usart_GlobalIsrHandler( usart_BusId_t usartId )
{
    /*----------------------------- Error interrupt --------------------------*/
    if( ( 0u != LL_USART_IsActiveFlag_PE( usart_PeriphConf[ usartId ].UsartReg )  ) ||
        ( 0u != LL_USART_IsActiveFlag_FE( usart_PeriphConf[ usartId ].UsartReg )  ) ||
        ( 0u != LL_USART_IsActiveFlag_NE( usart_PeriphConf[ usartId ].UsartReg )  ) ||
        ( 0u != LL_USART_IsActiveFlag_ORE( usart_PeriphConf[ usartId ].UsartReg ) )    )
    {
        /* Bit 3 ORE: Overrun error
         * This bit is set by hardware when the data currently being received in the shift register is
         * ready to be transferred into the USART_RDR register while RXNE = 1. It is cleared by a
         * software, writing 1 to the ORECF, in the USART_ICR register.
         * An interrupt is generated if RXNEIE = 1 or EIE = 1 in the USART_CR1 register.
         * 0: No overrun error
         * 1: Overrun error is detected
         * Note: When this bit is set, the USART_RDR register content is not lost but the shift register is
         * overwritten. An interrupt is generated if the ORE flag is set during multi buffer
         * communication if the EIE bit is set.
         * This bit is permanently forced to 0 (no overrun detection) when the bit OVRDIS is set in
         * the USART_CR3 register.
         *
         * Bit 2 NE: Noise detection flag
         * This bit is set by hardware when noise is detected on a received frame. It is cleared by
         * software, writing 1 to the NECF bit in the USART_ICR register.
         * 0: No noise is detected
         * 1: Noise is detected
         * Note: This bit does not generate an interrupt as it appears at the same time as the RXNE bit
         * which itself generates an interrupt. An interrupt is generated when the NE flag is set
         * during multi buffer communication if the EIE bit is set.
         * When the line is noise-free, the NE flag can be disabled by programming the ONEBIT
         * bit to 1 to increase the USART tolerance to deviations (Refer to Section 50.5.8:
         * Tolerance of the USART receiver to clock deviation on page 1731).
         *
         * Bit 1 FE: Framing error
         * This bit is set by hardware when a de-synchronization, excessive noise or a break character
         * is detected. It is cleared by software, writing 1 to the FECF bit in the USART_ICR register.
         * When transmitting data in Smartcard mode, this bit is set when the maximum number of
         * transmit attempts is reached without success (the card NACKs the data frame).
         * An interrupt is generated if EIE = 1 in the USART_CR1 register.
         * 0: No Framing error is detected
         * 1: Framing error or break character is detected
         *
         * Bit 0 PE: Parity error
         * This bit is set by hardware when a parity error occurs in receiver mode. It is cleared by
         * software, writing 1 to the PECF in the USART_ICR register.
         * An interrupt is generated if PEIE = 1 in the USART_CR1 register.
         * 0: No parity error
         * 1: Parity error */
        LL_USART_ReceiveData9( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_PE( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_FE( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_NE( usart_PeriphConf[ usartId ].UsartReg );
        LL_USART_ClearFlag_ORE( usart_PeriphConf[ usartId ].UsartReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].ErrorIsr )
        {
            usart_RuntimeData[ usartId ].ErrorIsr( (uint16_t) usart_PeriphConf[ usartId ].UsartReg->ISR & USART_ISR_ERROR_MASK );
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*------------------ Receive timeout (RTO) interrupt ---------------------*/
    if( 0u != LL_USART_IsActiveFlag_RTO( usart_PeriphConf[ usartId ].UsartReg ) )
    {
        /* Bit 11 RTOF: Receiver timeout
         * This bit is set by hardware when the timeout value, programmed in the RTOR register has
         * lapsed, without any communication. It is cleared by software, writing 1 to the RTOCF bit in
         * the USART_ICR register.
         * An interrupt is generated if RTOIE = 1 in the USART_CR2 register.
         * In Smartcard mode, the timeout corresponds to the CWT or BWT timings.
         * 0: Timeout value not reached
         * 1: Timeout value reached without any data reception
         * Note: If a time equal to the value programmed in RTOR register separates 2 characters,
         * RTOF is not set. If this time exceeds this value + 2 sample times (2/16 or 2/8,
         * depending on the oversampling method), RTOF flag is set.
         * The counter counts even if RE = 0 but RTOF is set only when RE = 1. If the timeout has
         * already elapsed when RE is set, then RTOF is set.
         * If the USART does not support the Receiver timeout feature, this bit is reserved and
         * kept at reset value. */
        LL_USART_ClearFlag_RTO( usart_PeriphConf[ usartId ].UsartReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].TransferCompleteIsr )
        {
            usart_RuntimeData[ usartId ].TransferCompleteIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*----------------------- IDLE Line detected interrupt -------------------*/
    if( 0u != LL_USART_IsActiveFlag_IDLE( usart_PeriphConf[ usartId ].UsartReg ) )
    {
        /* Bit 4 IDLE: Idle line detected
         * This bit is set by hardware when an Idle Line is detected. An interrupt is generated if
         * IDLEIE = 1 in the USART_CR1 register. It is cleared by software, writing 1 to the IDLECF in
         * the USART_ICR register.
         * 0: No Idle line is detected
         * 1: Idle line is detected
         * Note: The IDLE bit is not set again until the RXNE bit has been set (i.e. a new idle line
         * occurs).
         * If Mute mode is enabled (MME = 1), IDLE is set if the USART is not mute (RWU = 0),
         * whatever the Mute mode selected by the WAKE bit. If RWU = 1, IDLE is not set. */
        LL_USART_ClearFlag_IDLE( usart_PeriphConf[ usartId ].UsartReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].IdleIsr )
        {
            usart_RuntimeData[ usartId ].IdleIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }

    }

    /*-------------- Receiver buffer Not Empty (RXNE) interrupt --------------*/
    if( 0u != LL_USART_IsActiveFlag_RXNE( usart_PeriphConf[ usartId ].UsartReg ) )
    {
        /* Bit 5 RXNE: Read data register not empty
         * RXNE bit is set by hardware when the content of the USART_RDR shift register has been
         * transferred to the USART_RDR register. It is cleared by reading from the USART_RDR
         * register. The RXNE flag can also be cleared by writing 1 to the RXFRQ in the USART_RQR
         * register.
         * An interrupt is generated if RXNEIE = 1 in the USART_CR1 register.
         * 0: Data is not received
         * 1: Received data is ready to be read. */
        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].RxNotEmptyIsr )
        {
            usart_RuntimeData[ usartId ].RxNotEmptyIsr( LL_USART_ReceiveData9( usart_PeriphConf[ usartId ].UsartReg ) );
        }
    }

    /*-------------- Transmission buffer Empty (TXE) interrupt ---------------*/
    if( 0u != LL_USART_IsActiveFlag_TXE( usart_PeriphConf[ usartId ].UsartReg ) )
    {
        /* Bit 7 TXE: Transmit data register empty
         * TXE is set by hardware when the content of the USART_TDR register has been transferred
         * into the shift register. It is cleared by writing to the USART_TDR register. The TXE flag can
         * also be set by writing 1 to the TXFRQ in the USART_RQR register, in order to discard the
         * data (only in Smartcard T = 0 mode, in case of transmission failure).
         * An interrupt is generated if the TXEIE bit = 1 in the USART_CR1 register.
         * 0: Data register full
         * 1: Data register not full */
        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].TxEmptyIsr )
        {
            usart_RuntimeData[ usartId ].TxEmptyIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }

    /*---------------- Transmission Complete (TC) interrupt ------------------*/
    if( 0u != LL_USART_IsActiveFlag_TC( usart_PeriphConf[ usartId ].UsartReg ) )
    {
        /* Bit 6 TC: Transmission complete
         * This bit indicates that the last data written in the USART_TDR has been transmitted out of
         * the shift register.
         * It is set by hardware when the transmission of a frame containing data is complete and
         * when TXE is set.
         * An interrupt is generated if TCIE = 1 in the USART_CR1 register.
         * TC bit is is cleared by software, by writing 1 to the TCCF in the USART_ICR register or by a
         * write to the USART_TDR register.
         * 0: Transmission is not complete
         * 1: Transmission is complete
         * Note: If TE bit is reset and no transmission is on going, the TC bit is set immediately. */
        LL_USART_ClearFlag_TC( usart_PeriphConf[ usartId ].UsartReg );

        if( USART_NULL_PTR != usart_RuntimeData[ usartId ].TransferCompleteIsr )
        {
            usart_RuntimeData[ usartId ].TransferCompleteIsr();
        }
        else
        {
            /* Interrupt callback was not configured */
        }
    }
}


#ifdef USART1
/**
 * \brief USART1 Interrupt handler
 */
void Usart_Usart1_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_1 );
}
#endif /* USART1 */

#ifdef USART2
/**
 * \brief USART2 Interrupt handler
 */
void Usart_Usart2_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_2 );
}
#endif /* USART2 */

#ifdef USART3
/**
 * \brief USART3 Interrupt handler
 */
void Usart_Usart3_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_3 );
}
#endif /* USART3 */

#ifdef UART4
/**
 * \brief UART4 Interrupt handler
 */
void Uart_Usart4_IsrHandler()
{
    Usart_GlobalIsrHandler( USART_BUS_4 );
}
#endif /* UART4 */

#ifdef UART5
/**
 * \brief UART5 Interrupt handler
 */
void Uart_Usart5_IsrHandler(void)
{
    Usart_GlobalIsrHandler( USART_BUS_5 );
}
#endif /* UART5 */


/* ================================ TASKS =================================== */
