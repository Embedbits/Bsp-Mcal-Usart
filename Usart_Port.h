/**
 * \author Mr.Nobody
 * \file Usart_Port.h
 * \ingroup Usart
 * \brief Universal Synchronous/Asynchronous Receiver/Transmitter (USART) module
 *        public functionality.
 *
 * This file contains all available public functionality, any other files shall 
 * not used outside of the module.
 *
 */

#ifndef USART_USART_PORT_H
#define USART_USART_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================== INCLUDES ================================== */
#include "Usart_Types.h"                    /* Module types definition        */
/* ============================== TYPEDEFS ================================== */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/* ========================== EXPORTED MACROS =============================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* ========================= EXPORTED FUNCTIONS ============================= */

usart_ModuleVersion_t   Usart_Get_ModuleVersion         ( void );

void                    Usart_Task                      ( void );

usart_RequestState_t    Usart_Get_DefaultConfig         ( usart_BusConfig_t *usartConfig );
usart_RequestState_t    Usart_Init                      ( usart_BusConfig_t * const usartConfig );
usart_RequestState_t    Usart_Deinit                    ( usart_BusConfig_t const *usartConfig );

/*-------------------------- Primitive functionality -------------------------*/

usart_RequestState_t    Usart_StartTransmit             ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_StopTransmit              ( usart_PeriphId_t usartId );

usart_RequestState_t    Usart_Set_TransmitBytes         ( usart_PeriphId_t usartId, usart_TxDataLen_t dataCnt );

usart_RequestState_t    Usart_StartReceive              ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_StopReceive               ( usart_PeriphId_t usartId );

usart_RequestState_t    Usart_Get_ReceivedDataCnt       ( usart_PeriphId_t usartId, usart_RxDataLen_t dataCnt );

/*-------------------------- Primitive functionality -------------------------*/

void                    Usart_Set_PeriphActive          ( usart_PeriphId_t usartId );
void                    Usart_Set_PeriphInactive        ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_PeriphState           ( usart_PeriphId_t usartId, usart_FunctionState_t * const periphState );

usart_RequestState_t    Usart_Set_Baudrate              ( usart_PeriphId_t usartId, usart_Baudrate_t baudrate );
usart_RequestState_t    Usart_Get_Baudrate              ( usart_PeriphId_t usartId, usart_Baudrate_t * const baudrate );

usart_RequestState_t    Usart_Set_DataWidth             ( usart_PeriphId_t usartId, usart_DataWidth_t dataWidth );
usart_RequestState_t    Usart_Get_DataWidth             ( usart_PeriphId_t usartId, usart_DataWidth_t * const dataWidth );

usart_RequestState_t    Usart_Set_StopBits              ( usart_PeriphId_t usartId, usart_StopBits_t stopBits );
usart_RequestState_t    Usart_Get_StopBits              ( usart_PeriphId_t usartId, usart_StopBits_t * const stopBits );

usart_RequestState_t    Usart_Set_Parity                ( usart_PeriphId_t usartId, usart_Parity_t parity );
usart_RequestState_t    Usart_Get_Parity                ( usart_PeriphId_t usartId, usart_Parity_t * const parity );

usart_RequestState_t    Usart_Set_TransferMode          ( usart_PeriphId_t usartId, usart_TransferMode_t transferMode );
usart_RequestState_t    Usart_Get_TransferMode          ( usart_PeriphId_t usartId, usart_TransferMode_t * const transferMode );

usart_RequestState_t    Usart_Set_FlowControl           ( usart_PeriphId_t usartId, usart_FlowControl_t flowControl );
usart_RequestState_t    Usart_Get_FlowControl           ( usart_PeriphId_t usartId, usart_FlowControl_t * const flowControl );

usart_RequestState_t    Usart_Set_DriverEnableState     ( usart_PeriphId_t usartId, usart_DeFeatureState_t deState );
usart_RequestState_t    Usart_Get_DriverEnableState     ( usart_PeriphId_t usartId, usart_DeFeatureState_t * const deState );

usart_RequestState_t    Usart_Set_DriverEnablePolarity  ( usart_PeriphId_t usartId, usart_DePolarity_t dePolarity );
usart_RequestState_t    Usart_Get_DriverEnablePolarity  ( usart_PeriphId_t usartId, usart_DePolarity_t * const dePolarity );

usart_RequestState_t    Usart_Set_AssertDeassertTimes   ( usart_PeriphId_t usartId, usart_AssertTime_us_t assertTime, usart_DeassertTime_us_t deassertTime );
usart_RequestState_t    Usart_Get_AssertDeassertTimes   ( usart_PeriphId_t usartId, usart_AssertTime_us_t * const assertTime, usart_DeassertTime_us_t * const deassertTime );

usart_RequestState_t    Usart_Set_Oversampling          ( usart_PeriphId_t usartId, usart_Oversampling_t oversamplingMode );
usart_RequestState_t    Usart_Get_Oversampling          ( usart_PeriphId_t usartId, usart_Oversampling_t * const oversamplingMode );

usart_RequestState_t    Usart_Set_HalfDuplexState       ( usart_PeriphId_t usartId, usart_HalfDuplex_t halfDuplexState );
usart_RequestState_t    Usart_Get_HalfDuplexState       ( usart_PeriphId_t usartId, usart_HalfDuplex_t * const halfDuplexState );

usart_RequestState_t    Usart_Set_RxTimeoutActive       ( usart_PeriphId_t usartId, usart_RxTimeout_t timeoutBitsCnt );
usart_RequestState_t    Usart_Set_RxTimeoutInactive     ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_RxTimeoutState        ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState );

usart_RequestState_t    Usart_Set_PinLevels             ( usart_PeriphId_t usartId, usart_RxPinLevel_t rxPinLevels, usart_TxPinLevel_t txPinLevels );
usart_RequestState_t    Usart_Get_PinLevels             ( usart_PeriphId_t usartId, usart_RxPinLevel_t * const rxPinLevels, usart_TxPinLevel_t * const txPinLevels );

usart_RequestState_t    Usart_Get_TxRegisterAddr        ( usart_PeriphId_t usartId, usart_RxRegAddr_t * const regAddr );
usart_RequestState_t    Usart_Get_RxRegisterAddr        ( usart_PeriphId_t usartId, usart_RxRegAddr_t * const regAddr );

void                    Usart_SendData                  ( usart_PeriphId_t usartId, usart_TxData_t txData );
usart_RxData_t          Usart_ReadData                  ( usart_PeriphId_t usartId );

/*-------------------- Non-blocking transfer functionality -------------------*/

usart_RequestState_t    Usart_Set_DmaTxRequestActive    ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_DmaTxRequestInactive  ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_DmaTxReqState         ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);

usart_RequestState_t    Usart_Set_DmaRxRequestActive    ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_DmaRxRequestInactive  ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_DmaRxReqState         ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);

usart_RequestState_t    Usart_Set_InterruptsActive      ( usart_PeriphId_t usartBus );
usart_RequestState_t    Usart_Set_InterruptsInactive    ( usart_PeriphId_t usartBus );

usart_RequestState_t    Usart_Set_IrqPriority           ( usart_PeriphId_t usartId, usart_IrqPrio_t irqPrio );
usart_RequestState_t    Usart_Get_IrqPriority           ( usart_PeriphId_t usartId, usart_IrqPrio_t * const irqPrio );

usart_RequestState_t    Usart_Set_RxNotEmptyIrqActive   ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_RxNotEmptyIrqInactive ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_RxNotEmptyIrqState    ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);
usart_RequestState_t    Usart_Set_RxNotEmptyIsrCallback ( usart_PeriphId_t usartId, usart_RxNeIrqCallback_t * const callback);

usart_RequestState_t    Usart_Set_TxEmptyIrqActive      ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_TxEmptyIrqInactive    ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_TxEmptyIrqState       ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);
usart_RequestState_t    Usart_Set_TxEmptyIsrCallback    ( usart_PeriphId_t usartId, usart_TxeIrqCallback_t * const callback);

usart_RequestState_t    Usart_Set_TxCompleteIrqActive   ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_TxCompleteIrqInactive ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_TxCompleteIrqState    ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);
usart_RequestState_t    Usart_Set_TxCompleteIsrCallback ( usart_PeriphId_t usartId, usart_TcIrqCallback_t * const callback);

usart_RequestState_t    Usart_Set_IdleIrqActive         ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_IdleIrqInactive       ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_IdleIrqState          ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);
usart_RequestState_t    Usart_Set_IdleIsrCallback       ( usart_PeriphId_t usartId, usart_IdleIrqCallback_t * const callback);

usart_RequestState_t    Usart_Set_RxTimeoutIrqActive    ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_RxTimeoutIrqInactive  ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_RxTimeoutIrqState     ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);
usart_RequestState_t    Usart_Set_RxTimeoutIsrCallback  ( usart_PeriphId_t usartId, usart_RxTimeoutIrqCallback_t * const callback);

usart_RequestState_t    Usart_Set_ErrorIrqActive        ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Set_ErrorIrqInactive      ( usart_PeriphId_t usartId );
usart_RequestState_t    Usart_Get_ErrorIrqState         ( usart_PeriphId_t usartId, usart_FlagState_t * const reqState);
usart_RequestState_t    Usart_Set_ErrorIsrCallback      ( usart_PeriphId_t usartId, usart_ErrIrqCallback_t * const callback);

/*--------------------- GPIO configuration functionality ---------------------*/

usart_RequestState_t    Usart_InitRxGpio                ( usart_RxPin_t pinId );
usart_RequestState_t    Usart_InitTxGpio                ( usart_TxPin_t pinId );
usart_RequestState_t    Usart_InitDeGpio                ( usart_DePin_t pinId );

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_PORT_H */

