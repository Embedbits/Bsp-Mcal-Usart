/**
 * \author Mr.Nobody
 * \file Usart_CommDma.h
 * \ingroup Usart
 * \brief UART/USART DMA communication handler functionality
 *
 */
/* ============================== INCLUDES ================================== */
#include "Usart_CommDma.h"                  /* Self include                   */
#include "Usart_Port.h"                     /* Own port file include          */
#include "Usart_Types.h"                    /* Module types definitions       */
#include "Dma_Port.h"                       /* DMA functionality include      */
/* ============================== TYPEDEFS ================================== */

/** USART/UART DMA communication style configuration structure type definition */
typedef struct
{
    usart_PeriphId_t  UsartId;
    dma_PeriphReqId_t TxReq;
    dma_PeriphReqId_t RxReq;
}   usart_CommDma_Config_t;


/** USART/UART DMA communication style runtime data structure type definition */
typedef struct
{
    const usart_PeriphId_t UsartId;
    dma_PeriphId_t         TxDmaId;
    dma_ChannelId_t        TxDmaChannelId;
    dma_PeriphId_t         RxDmaId;
    dma_ChannelId_t        RxDmaChannelId;
}   usart_CommDma_RuntimeData_t;

/* ======================== FORWARD DECLARATIONS ============================ */

/* ========================== SYMBOLIC CONSTANTS ============================ */

/** Value of major version of SW module */
#define USART_MAJOR_VERSION           ( 1u )

/** Value of minor version of SW module */
#define USART_MINOR_VERSION           ( 0u )

/** Value of patch version of SW module */
#define USART_PATCH_VERSION           ( 0u )

/* =============================== MACROS =================================== */

/* ========================== EXPORTED VARIABLES ============================ */

/* =========================== LOCAL VARIABLES ============================== */

/** Configuration of USART/UART DMA communication style. It contains information
 * about DMA requests for each USART/UART peripheral, and it is not updated
 * during the process execution. */
static usart_CommDma_Config_t const     usart_CommDma_Config[ USART_BUS_CNT ] =
{
 { .UsartId = USART_PERIPH_1, .TxReq = DMA_REQ_USART1_TX, .RxReq = DMA_REQ_USART1_RX },
 { .UsartId = USART_PERIPH_2, .TxReq = DMA_REQ_USART2_TX, .RxReq = DMA_REQ_USART2_RX },
 { .UsartId = USART_PERIPH_3, .TxReq = DMA_REQ_USART3_TX, .RxReq = DMA_REQ_USART3_RX },
 { .UsartId = USART_PERIPH_4, .TxReq = DMA_REQ_UART4_TX , .RxReq = DMA_REQ_UART4_RX  },
 { .UsartId = USART_PERIPH_5, .TxReq = DMA_REQ_UART5_TX , .RxReq = DMA_REQ_UART5_RX  },
};


/** Runtime data of USART/UART DMA communication style. It contains information
 * about DMA channels used for USART/UART communication style, and it is
 * updated during the process execution. */
static usart_CommDma_RuntimeData_t      usart_CommDma_RuntimeData[ USART_BUS_CNT ] =
{
 { .UsartId = USART_PERIPH_1, .TxDmaId = DMA_PERIPH_CNT, .TxDmaChannelId = DMA_CHANNEL_CNT, .RxDmaId = DMA_PERIPH_CNT, .RxDmaChannelId = DMA_CHANNEL_CNT },
 { .UsartId = USART_PERIPH_2, .TxDmaId = DMA_PERIPH_CNT, .TxDmaChannelId = DMA_CHANNEL_CNT, .RxDmaId = DMA_PERIPH_CNT, .RxDmaChannelId = DMA_CHANNEL_CNT },
 { .UsartId = USART_PERIPH_3, .TxDmaId = DMA_PERIPH_CNT, .TxDmaChannelId = DMA_CHANNEL_CNT, .RxDmaId = DMA_PERIPH_CNT, .RxDmaChannelId = DMA_CHANNEL_CNT },
 { .UsartId = USART_PERIPH_4, .TxDmaId = DMA_PERIPH_CNT, .TxDmaChannelId = DMA_CHANNEL_CNT, .RxDmaId = DMA_PERIPH_CNT, .RxDmaChannelId = DMA_CHANNEL_CNT },
 { .UsartId = USART_PERIPH_5, .TxDmaId = DMA_PERIPH_CNT, .TxDmaChannelId = DMA_CHANNEL_CNT, .RxDmaId = DMA_PERIPH_CNT, .RxDmaChannelId = DMA_CHANNEL_CNT },
};

/* ========================= EXPORTED FUNCTIONS ============================= */

/**
 * \brief Returns module SW version
 *
 * \return Module SW version
 */
usart_ModuleVersion_t Usart_CommDma_Get_ModuleVersion( void )
{
    usart_ModuleVersion_t retVersion;

    retVersion.Major = USART_MAJOR_VERSION;
    retVersion.Minor = USART_MINOR_VERSION;
    retVersion.Patch = USART_PATCH_VERSION;

    return (retVersion);
}


/**
 * \brief Initializes DMA communication for UART/USART peripheral
 *
 * \warning The peripheral has to be configured before communication initialization.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Init( usart_PeriphId_t usartId, usart_XferConfig_t * const xferConfig )
{
    usart_RequestState_t retValue  = USART_REQUEST_ERROR;
    usart_RxRegAddr_t    rxRegAddr = 0u;
    usart_TxRegAddr_t    txRegAddr = 0u;
    usart_DataWidth_t    dataWidth = USART_DATA_WIDTH_7;
    dma_ConfigStruct_t   dmaConfig;

    retValue = Usart_Get_TxRegisterAddr( usartId, &rxRegAddr );
    retValue = Usart_Get_TxRegisterAddr( usartId, &txRegAddr );

    retValue = Usart_Get_DataWidth( usartId, &dataWidth );

    if( USART_DATA_WIDTH_9 == dataWidth )
    {
        dmaConfig.PeriphTransferSize = DMA_TRANSFER_SIZE_16BIT;
        dmaConfig.MemoryTransferSize = DMA_TRANSFER_SIZE_16BIT;
    }
    else
    {
        dmaConfig.PeriphTransferSize = DMA_TRANSFER_SIZE_8BIT;
        dmaConfig.MemoryTransferSize = DMA_TRANSFER_SIZE_8BIT;
    }

    dmaConfig.DmaPeriphId              = DMA_PERIPH_1;
    dmaConfig.DmaChannel               = DMA_CHANNEL_1;
    dmaConfig.Direction                = DMA_DIR_PERIPH_TO_MEMORY;
    dmaConfig.TransferMode             = DMA_TRANSFER_MODE_NORMAL;
    dmaConfig.PeriphAddress            = txRegAddr;
    dmaConfig.MemoryAddress            = xferConfig->RxDataAddr;
    dmaConfig.PeriphAddrIncrement      = DMA_PERIPH_ADDR_STATIC;
    dmaConfig.MemoryAddrIncrement      = DMA_MEMORY_ADDR_INCREMENT;
    dmaConfig.DataCount                = xferConfig->RxDataLen;
    dmaConfig.PeripheralReqId          = usart_CommDma_Config[ usartId ].RxReq;
    dmaConfig.Priority                 = (dma_Priority_t)xferConfig->Prio;
    dmaConfig.TransferCompleteCallback = USART_NULL_PTR;
    dmaConfig.HalfTransferCallback     = USART_NULL_PTR;
    dmaConfig.TransferErrorCallback    = USART_NULL_PTR;

    Dma_Init( &dmaConfig );

    return ( retValue );
}


/**
 * \brief USART/UART DMA communication style de-intialization.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Deinit( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        Usart_CommDma_Set_TxStop( usartId );
        Usart_CommDma_Set_RxStop( usartId );

        usart_CommDma_RuntimeData[ usartId ].TxDmaId        = DMA_PERIPH_CNT;
        usart_CommDma_RuntimeData[ usartId ].TxDmaChannelId = DMA_CHANNEL_CNT;
        usart_CommDma_RuntimeData[ usartId ].RxDmaId        = DMA_PERIPH_CNT;
        usart_CommDma_RuntimeData[ usartId ].RxDmaChannelId = DMA_CHANNEL_CNT;

        retValue = USART_REQUEST_OK;
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Main task of module Usart
 *
 * This function shall be called in the main loop of the application or the task
 * scheduler. It shall be called periodically, depending on the module's 
 * requirements.
 */
void Usart_CommDma_Task( void )
{

}


/**
 * \brief Starts USART/UART DMA receive communication style.
 *
 * \param usartId [in]: USART/UART peripheral identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_TxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_TransferActive( usart_CommDma_RuntimeData[ usartId ].TxDmaId,
                                           usart_CommDma_RuntimeData[ usartId ].TxDmaChannelId );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Stops USART/UART DMA transmit communicaiton style.
 *
 * \param usartId [in]: USART/UART peripheral identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_TxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_TransferInactive( usart_CommDma_RuntimeData[ usartId ].TxDmaId,
                                             usart_CommDma_RuntimeData[ usartId ].TxDmaChannelId );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Starts USART/UART DMA receive communication style.
 *
 * \param usartId [in]: USART/UART peripheral identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_RxStart( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_TransferActive( usart_CommDma_RuntimeData[ usartId ].RxDmaId,
                                           usart_CommDma_RuntimeData[ usartId ].RxDmaChannelId );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Stops USART/UART DMA receive communication style.
 *
 * \param usartId [in]: USART/UART peripheral identification.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_RxStop( usart_PeriphId_t usartId )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_TransferInactive( usart_CommDma_RuntimeData[ usartId ].RxDmaId,
                                             usart_CommDma_RuntimeData[ usartId ].RxDmaChannelId );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets address of memory buffer where data to be transmitted are stored
 *        for USART/UART DMA communication style.
 *
 * \param usartId    [in]: USART/UART peripheral identification.
 * \param txDataAddr [in]: Address of memory buffer where data to be transmitted
 *                         are stored for USART/UART DMA communication style.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_TxDataAddr( usart_PeriphId_t usartId, usart_TxData_t txDataAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_MemoryAddr( usart_CommDma_RuntimeData[ usartId ].TxDmaId,
                                       usart_CommDma_RuntimeData[ usartId ].TxDmaChannelId,
                                       txDataAddr );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets address of memory buffer where received data will be stored by
 *        USART/UART DMA communication style.
 *
 * \param usartId    [in]: USART/UART peripheral identification.
 * \param rxDataAddr [in]: Address of memory buffer where received data will be
 *                         stored by USART/UART DMA communication style.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_RxDataAddr( usart_PeriphId_t usartId, usart_TxData_t rxDataAddr )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_MemoryAddr( usart_CommDma_RuntimeData[ usartId ].RxDmaId,
                                       usart_CommDma_RuntimeData[ usartId ].RxDmaChannelId,
                                       rxDataAddr );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets number of data to be transmitted by USART/UART peripheral using DMA.
 *
 * \param usartId   [in]: USART/UART peripheral identification.
 * \param txDataLen [in]: Number of data to be transmitted.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_TxDataLen( usart_PeriphId_t usartId, usart_TxDataLen_t txDataLen )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_DataCount( usart_CommDma_RuntimeData[ usartId ].TxDmaId,
                                      usart_CommDma_RuntimeData[ usartId ].TxDmaChannelId,
                                      txDataLen );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}


/**
 * \brief Sets number of data to be received by USART/UART DMA communication style.
 *
 * \param usartId   [in]: USART/UART peripheral identification.
 * \param rxDataLen [in]: Number of data to be received by DMA communication style.
 *
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
usart_RequestState_t Usart_CommDma_Set_RxDataLen( usart_PeriphId_t usartId, usart_RxDataLen_t rxDataLen )
{
    usart_RequestState_t retValue = USART_REQUEST_ERROR;
    dma_RequestState_t   dmaState = DMA_REQUEST_ERROR;

    if( USART_BUS_CNT > usartId )
    {
        dmaState = Dma_Set_DataCount( usart_CommDma_RuntimeData[ usartId ].TxDmaId,
                                      usart_CommDma_RuntimeData[ usartId ].TxDmaChannelId,
                                      rxDataLen );

        if( DMA_REQUEST_ERROR == dmaState )
        {
            retValue = USART_REQUEST_ERROR;
        }
        else
        {
            retValue = USART_REQUEST_OK;
        }
    }
    else
    {
        retValue = USART_REQUEST_ERROR;
    }

    return ( retValue );
}

/* =========================== LOCAL FUNCTIONS ============================== */

/* =========================== INTERRUPT HANDLERS =========================== */

/* ================================ TASKS =================================== */
