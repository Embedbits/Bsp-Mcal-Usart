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
  end of message detection by idle line, callbacks for transmission complete, half / full receive
  buffer, end of message and errors (`usart_DataConfig_t`)
- Interrupt and DMA request control
- GPIO pin initialization for USART signals (encoded pins `USART_*_PIN_BUSx_Pyy`,
  `USART_*_PIN_LPBUS1_Pyy`, `USART_*_PIN_UNUSED`)

The public interface is identical with the STM32H5 / STM32F4 USART modules (applications and
middlewares, e.g. ModBus / CRSF, use the same interface). The STM32H7 implementation is based on
the STM32G4 module (same USART / LPUART IP).

### STM32H7 specifics

- Peripherals: USART1, USART2, USART3, UART4, UART5, USART6, UART7, UART8 (`USART_BUS_1` ...
  `USART_BUS_8`), UART9 / USART10 (`USART_BUS_9` / `USART_BUS_10`, only STM32H72x / H73x / H7A3 /
  H7B0 / H7B3) and LPUART1 (`USART_BUS_LPUART1`). Pin enumerations are generated from ST open pin
  data (STM32_open_pin_data) - pins missing on some device lines are guarded by the device macros;
  a pin with different alternate function on different device lines keeps one name.
- Kernel clocks (RCC module): USART1 / USART6 / UART9 / USART10 - PCLK2, USART2 / USART3 / UART4 /
  UART5 / UART7 / UART8 - PCLK1, LPUART1 - PCLK4 (D3 / SmartRun domain).
- All features of the USART interface are available: receiver timeout (end of message also by
  `USART_RX_END_TIMEOUT`), Driver Enable (DE) with its pins, polarity and assertion / deassertion
  times, pin level inversion, 7 / 8 / 9-bit words, kernel clock prescaler (PRESC) selected by the
  module for the required baud-rate. FIFO mode is not used.
- LPUART1 limitations (low-power UART IP):
  - over-sampling by 16 only - `Usart_Set_Oversampling()` accepts `USART_OVERSAMPLING_16`,
    `Usart_Get_Oversampling()` returns it (the default configuration uses over-sampling by 8 and
    has to be changed for LPUART1),
  - 1 or 2 stop bits,
  - no receiver timeout - `RxTimeoutValue` shall be 0, `Usart_Set_RxTimeoutActive()`,
    `Usart_Set_RxTimeoutIrqActive()` and `USART_RX_END_TIMEOUT` return error,
  - baud-rate divider LPUARTDIV = 256 x kernel clock / baud-rate (20-bit BRR, kernel clock 3 -
    4096 x baud-rate),
  - DE assertion / deassertion times are expressed in LPUART kernel clock cycles,
  - no DMA mode - LPUART1 requests are routed by DMAMUX2 to BDMA (not supported by the Dma
    module), `USART_XFER_MODE_DMA` is refused (ISR / POLL mode available).
- LPUART errata "Possible LPUART transmitter issue when using low BRR[15:0] value" (STM32G4
  ES0430 2.17.1, same LPUART IP - to be confirmed for STM32H7 by the errata audit AB#844):
  baud-rates with non-integer ratio of kernel clock and baud-rate in the range 3 - 4 (LPUARTDIV
  0x301 - 0x3FF) are refused by `Usart_Set_Baudrate()`.
- DMA: `usart_DmaPeriphId_t` / `usart_DmaChannelId_t` select any free stream of DMA1 / DMA2
  (`USART_DMA_CHANNEL_0` ... `USART_DMA_CHANNEL_7` = streams 0 - 7), the DMAMUX1 request of the
  peripheral is selected by the module (Dma module, direct mode, 8-bit). Transmit and receive
  direction shall use different streams. Buffers shall be accessible by DMA1 / DMA2 (AXI SRAM,
  SRAM1 - SRAM4 - not DTCM / ITCM).
- Reception error flags are cleared by the interrupt clear register (ICR). DMA errors are reported
  as `USART_XFER_ERROR_DMA_TRANSFER`.
- Interrupt service routines end without DSB (Cortex-M4 erratum 838869 of STM32G4 does not apply
  to Cortex-M7).
- Defects of the STM32H5 reference implementation fixed in this implementation (taken over from
  STM32G4): NULL output pointer of `Usart_Get_Oversampling()` / `Usart_Get_RxTimeoutState()` /
  `Usart_Get_PinLevels()` (AB#968), overflow of the DE time calculation at high baud-rates and
  division by zero without configured baud-rate (AB#970), `USART_RX_TIMEOUT_MAX` refused by
  `Usart_Set_RxTimeoutActive()` (AB#971).

**STM32H7R3 / H7R7 / H7S3 / H7S7** (Ral family STM32H7RS, macro `STM32H7RS`): no DMA1 / DMA2 streams - DMA data
handling uses GPDMA1 / HPDMA1 channels (`Gpdma_Lib`, `Usart_Dma.c` of the STM32H5 module compiled under `STM32H7RS`,
GPDMA1 requests `GPDMA_REQ_<USART>_RX / _TX`, LPUART1 included). Peripherals USART1 - USART3, UART4 / UART5 / UART7 /
UART8, LPUART1; pin enumerations from the STM32H7RS open pin data. Unit tests: the STM32H7 tests run on STM32H7R / H7S
except the DMA stream tests (ignored there - the STM32H5 module has no unit tests of the GPDMA data handling), the
GPDMA data handling is verified by the integration tests on NUCLEO-H7S3L8 with the IT phase.

Not yet tested on hardware (integration tests prepared for STM32H7 Nucleo boards).

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
