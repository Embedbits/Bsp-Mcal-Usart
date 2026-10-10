# USART Peripheral Driver

This module provides an abstraction layer for configuring and managing **USART peripherals** on STM32 MCUs.  
It supports initialization, configuration of communication parameters, DMA transfers, interrupt handling, and GPIO setup.

---

## Features

- Basic initialization and deinitialization
- Configuration of baudrate, data width, stop bits, parity, transfer mode, flow control
- Driver enable feature and polarity
- Oversampling and half-duplex support
- RX timeout configuration
- Pin levels management
- Direct register access for TX/RX
- Data handling of buffers - transmission and reception configured independently in **DMA**,
  **ISR** (USART interrupt) or **POLL** (`Usart_Task()`) mode, one shot or circular receive buffer,
  end of message detection by idle line or receiver timeout, callbacks for transmission complete, half / full receive
  buffer, end of message and errors (`usart_DataConfig_t`)
- Interrupt and DMA request control
- GPIO pin initialization for USART signals (encoded pins `USART_*_PIN_BUSx_Pyy`, `USART_*_PIN_UNUSED`)

The public interface is identical with the STM32H5 USART module (applications and middlewares,
e.g. ModBus / CRSF, use the same interface).

### STM32F7 specifics

- USART1 / USART2 / USART3 / USART6 and UART4 / UART5 / UART7 / UART8 have the register model of
  STM32H5 (ISR / ICR / RDR / TDR) - all features of the interface are available: receiver
  timeout (end of message also by `USART_RX_END_TIMEOUT`), Driver Enable (DE) with its pins (RTS
  pins), polarity and assertion / deassertion times, pin level inversion, 7 / 8 / 9-bit words.
- No FIFO and no clock prescaler (PRESC) - the baud-rate divider `kernel clock / baud-rate`
  (over-sampling 16) or `2 x kernel clock / baud-rate` (over-sampling 8) has to be in the range
  16 - 65535 (e.g. 1200 Bd minimum with 64 MHz kernel clock).
- DMA: `TxDma` / `RxDma` of `usart_DataConfig_t` select the DMA stream from the lists `usart_TxDma_t` /
  `usart_RxDma_t` - one item per USART / UART bus, DMA peripheral and stream, named
  `USART_TX_DMA_BUSx_DMAy_STREAMz` / `USART_RX_DMA_BUSx_DMAy_STREAMz` (e.g. `USART_TX_DMA_BUS1_DMA2_STREAM7`).
  The items are the streams connected to the USART request (RM0385 / RM0410 / RM0431 request mapping, the table
  below), the channel selection of the stream is part of the item. Items of another bus, items of the other
  direction and `USART_TX_DMA_UNUSED` / `USART_RX_DMA_UNUSED` are refused for a direction in the DMA mode:

| Peripheral | TX streams          | RX streams          |
|------------|---------------------|---------------------|
| USART1     | DMA2 S7             | DMA2 S2, DMA2 S5    |
| USART2     | DMA1 S6             | DMA1 S5             |
| USART3     | DMA1 S3, DMA1 S4    | DMA1 S1             |
| UART4      | DMA1 S4             | DMA1 S2             |
| UART5      | DMA1 S7             | DMA1 S0             |
| USART6     | DMA2 S6, DMA2 S7    | DMA2 S1, DMA2 S2    |
| UART7      | DMA1 S1             | DMA1 S3             |
| UART8      | DMA1 S0             | DMA1 S6             |

- Reception error flags are cleared by the interrupt clear register (ICR). In DMA reception RDR
  is read by the DMA stream, errors and end of message (idle line / receiver timeout) are reported
  from the USART interrupt. DMA errors are reported as `USART_XFER_ERROR_DMA_TRANSFER`.
- DMA buffers must be accessible by DMA (not held in the Cortex-M7 data cache).
- Data are handled as 8-bit values - 9-bit frames without parity are not supported by the data
  handling.
- Receiver timeout is counted in bits from the end of the first stop bit. With two stop bits the
  hardware starts the counter one bit later (device errata ES0334 "Receiver timeout counter wrong
  start in two-stop-bit configuration") - the module programs RTOR one bit shorter and corrects it
  after a stop bits change.
- RTS flow control: the hardware drives RTS active as soon as RTSE is set, also while the USART
  or its receiver is disabled (device errata ES0334 "RTS is active while RE = 0 or UE = 0"). RTSE
  is writable only with UE = 0, so the RTS pin initialized by `Usart_InitRtsGpio()` (`BusRtsPin`) or `Usart_InitDeGpio()` (`BusDePin`) is
  held as GPIO output at high (inactive) level while RTS flow control is requested and UE or RE is
  0, and is switched to the alternate function after UE and RE are set. The pin stays held after
  `Usart_Deinit()`. Without RTS flow control (Driver Enable on the same pin) the pin is not changed.
- Device errata ES0334 without workaround: "Data corruption due to noisy receive line" - a glitch
  to zero shorter than half a bit in the second half of the stop bit may corrupt the received
  data (over-sampling 8 and 16, one and two stop bits).

---

## Public API

### Module Management
- `usart_ModuleVersion_t   Usart_Get_ModuleVersion(void);`
- `usart_RequestState_t    Usart_Init(usart_BusConfig_t * const usartConfig);`
- `usart_RequestState_t    Usart_Deinit(usart_PeriphId_t usartId);`
- `void                    Usart_Task(void);`
- `usart_RequestState_t    Usart_Get_DefaultConfig(usart_BusConfig_t *usartConfig);`

### Peripheral Control
- `usart_RequestState_t    Usart_Set_PeriphActive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Set_PeriphInactive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Get_PeriphState(usart_PeriphId_t usartId, usart_FlagState_t * const reqState);`

### Communication Settings
- `usart_RequestState_t    Usart_Set_Baudrate(usart_PeriphId_t usartId, usart_Baudrate_t baudrate);`
- `usart_RequestState_t    Usart_Get_Baudrate(usart_PeriphId_t usartId, usart_Baudrate_t * const baudrate);`
- `usart_RequestState_t    Usart_Set_DataWidth(usart_PeriphId_t usartId, usart_DataWidth_t dataWidth);`
- `usart_RequestState_t    Usart_Get_DataWidth(usart_PeriphId_t usartId, usart_DataWidth_t * const dataWidth);`
- `usart_RequestState_t    Usart_Set_StopBits(usart_PeriphId_t usartId, usart_StopBits_t stopBits);`
- `usart_RequestState_t    Usart_Get_StopBits(usart_PeriphId_t usartId, usart_StopBits_t * const stopBits);`
- `usart_RequestState_t    Usart_Set_Parity(usart_PeriphId_t usartId, usart_Parity_t parity);`
- `usart_RequestState_t    Usart_Get_Parity(usart_PeriphId_t usartId, usart_Parity_t * const parity);`
- `usart_RequestState_t    Usart_Set_TransferMode(usart_PeriphId_t usartId, usart_TransferMode_t transferMode);`
- `usart_RequestState_t    Usart_Get_TransferMode(usart_PeriphId_t usartId, usart_TransferMode_t * const transferMode);`
- `usart_RequestState_t    Usart_Set_FlowControl(usart_PeriphId_t usartId, usart_FlowControl_t flowControl);`
- `usart_RequestState_t    Usart_Get_FlowControl(usart_PeriphId_t usartId, usart_FlowControl_t * const flowControl);`

### Driver Enable and Timing
- `usart_RequestState_t    Usart_Set_DriverEnableState(usart_PeriphId_t usartId, usart_DeFeatureState_t deState);`
- `usart_RequestState_t    Usart_Get_DriverEnableState(usart_PeriphId_t usartId, usart_DeFeatureState_t * const deState);`
- `usart_RequestState_t    Usart_Set_DriverEnablePolarity(usart_PeriphId_t usartId, usart_DePolarity_t dePolarity);`
- `usart_RequestState_t    Usart_Get_DriverEnablePolarity(usart_PeriphId_t usartId, usart_DePolarity_t * const dePolarity);`
- `usart_RequestState_t    Usart_Set_AssertDeassertTimes(usart_PeriphId_t usartId, usart_AssertTime_us_t assertTime, usart_DeassertTime_us_t deassertTime);`
- `usart_RequestState_t    Usart_Get_AssertDeassertTimes(usart_PeriphId_t usartId, usart_AssertTime_us_t * const assertTime, usart_DeassertTime_us_t * const deassertTime);`

### Advanced Communication Settings
- `usart_RequestState_t    Usart_Set_Oversampling(usart_PeriphId_t usartId, usart_Oversampling_t oversamplingMode);`
- `usart_RequestState_t    Usart_Get_Oversampling(usart_PeriphId_t usartId, usart_Oversampling_t * const oversamplingMode);`
- `usart_RequestState_t    Usart_Set_HalfDuplexState(usart_PeriphId_t usartId, usart_HalfDuplex_t halfDuplexState);`
- `usart_RequestState_t    Usart_Get_HalfDuplexState(usart_PeriphId_t usartId, usart_HalfDuplex_t * const halfDuplexState);`
- `usart_RequestState_t    Usart_Set_RxTimeoutActive(usart_PeriphId_t usartId, usart_RxTimeout_t timeoutBitsCnt);`
- `usart_RequestState_t    Usart_Set_RxTimeoutInactive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Get_RxTimeoutState(usart_PeriphId_t usartId, usart_FlagState_t * const reqState);`

### Pin Management
- `usart_RequestState_t    Usart_Set_PinLevels(usart_PeriphId_t usartId, usart_RxPinLevel_t rxPinLevels, usart_TxPinLevel_t txPinLevels);`
- `usart_RequestState_t    Usart_Get_PinLevels(usart_PeriphId_t usartId, usart_RxPinLevel_t * const rxPinLevels, usart_TxPinLevel_t * const txPinLevels);`
- `usart_RequestState_t    Usart_Get_TxRegisterAddr(usart_PeriphId_t usartId, usart_TxRegAddr_t * const regAddr);`
- `usart_RequestState_t    Usart_Get_RxRegisterAddr(usart_PeriphId_t usartId, usart_RxRegAddr_t * const regAddr);`

### Data Transfer
- `void                    Usart_SendData(usart_PeriphId_t usartId, usart_TxData_t txData);`
- `usart_RxData_t          Usart_ReadData(usart_PeriphId_t usartId);`

---

## Data Handling (DMA / ISR / POLL)
- `usart_RequestState_t    Usart_Set_DataConfig(usart_PeriphId_t usartId, const usart_DataConfig_t * const dataConfig);`
- `usart_RequestState_t    Usart_Get_DataConfig(usart_PeriphId_t usartId, usart_DataConfig_t * const dataConfig);`
- `usart_RequestState_t    Usart_Set_TxStart(usart_PeriphId_t usartId, const usart_TxData_t * const txData, usart_TxDataCnt_t txSize);`
- `usart_RequestState_t    Usart_Set_TxStop(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Get_TxState(usart_PeriphId_t usartId, usart_FunctionState_t * const txState);`
- `usart_RequestState_t    Usart_Set_RxStart(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Set_RxStop(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Get_RxState(usart_PeriphId_t usartId, usart_FunctionState_t * const rxState);`
- `usart_RequestState_t    Usart_Get_RxCount(usart_PeriphId_t usartId, usart_RxDataCnt_t * const rxCnt);`

Data handling is configured by `usart_BusConfig_t::DataConfig` in `Usart_Init()` or later by
`Usart_Set_DataConfig()`. POLL mode requires periodic call of `Usart_Task()`.

---

## DMA Requests
- `usart_RequestState_t    Usart_Set_DmaTxRequestActive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Set_DmaTxRequestInactive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Get_DmaTxReqState(usart_PeriphId_t usartId, usart_FlagState_t * const reqState);`
- `usart_RequestState_t    Usart_Set_DmaRxRequestActive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Set_DmaRxRequestInactive(usart_PeriphId_t usartId);`
- `usart_RequestState_t    Usart_Get_DmaRxReqState(usart_PeriphId_t usartId, usart_FlagState_t * const reqState);`

---

## Interrupts
- General:
  - `usart_RequestState_t    Usart_Set_InterruptsActive(usart_PeriphId_t usartBus);`
  - `usart_RequestState_t    Usart_Set_InterruptsInactive(usart_PeriphId_t usartBus);`
  - `usart_RequestState_t    Usart_Set_IrqPriority(usart_PeriphId_t usartId, usart_IrqPrio_t irqPrio);`
  - `usart_RequestState_t    Usart_Get_IrqPriority(usart_PeriphId_t usartId, usart_IrqPrio_t * const irqPrio);`
- Interrupt sources (`Usart_Set_<Irq>IrqActive / Usart_Set_<Irq>IrqInactive / Usart_Get_<Irq>IrqState`,
  `<Irq>` = `RxNotEmpty`, `TxEmpty`, `TxComplete`, `Idle`, `RxTimeout`, `Error`). The USART interrupt
  handler serves the data handling - these functions are intended for diagnostics.

---

## GPIO Configuration
- `usart_RequestState_t    Usart_InitRxGpio(usart_RxPin_t pinId);`
- `usart_RequestState_t    Usart_InitTxGpio(usart_TxPin_t pinId);`
- `usart_RequestState_t    Usart_InitDeGpio(usart_DePin_t pinId);`
- `usart_RequestState_t    Usart_InitCtsGpio(usart_CtsPin_t pinId);`
- `usart_RequestState_t    Usart_InitRtsGpio(usart_RtsPin_t pinId);`

The pins of the hardware flow control are configured by `BusCtsPin` (CTS input) and `BusRtsPin` (RTS output)
of `usart_BusConfig_t`; the RTS output has the same pads and alternate functions as the Driver Enable output (`BusDePin`).

---

## 🛠 CMake Integration

1. Include `Usart_Lib` in your CMake library.
2. Include `Usart_Port.h` in your project.
3. Link against the Usart module implementation files.
4. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** — [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.

---

## 🌐 Useful Links

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [Azure DevOps](https://azure.microsoft.com/en-us/services/devops/)
- [Embedbits Github](https://github.com/Embedbits)
- [CC BY-NC 4.0 License](https://creativecommons.org/licenses/by-nc/4.0/)
