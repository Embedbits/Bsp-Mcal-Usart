/**
 * \author Mr.Nobody
 * \file Usart.h
 * \ingroup Usart
 * \brief Usart module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 *
 */

#ifndef USART_USART_H
#define USART_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usart_Types.h"                    /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

typedef USART_TypeDef usart_PeriphRegAddr_t;

/* ========================= SYMBOLIC CONSTANTS ============================= */

#define USART_DE_ASSERT_MAX_VALUE               ( 31u )

/* ========================= EXPORTED MACROS ================================ */

/* ========================= EXPORTED VARIABLES ============================= */

/* ======================== EXPORTED FUNCTIONS ============================== */

/**
 * \brief Calculation of assertion and de-assertion values of Driver Enable (DE) pin
 *
 * \param targetTime   [in]: Required time in us
 * \param baudrate     [in]: Actual baudrate
 * \param oversampling [in]: Oversampling configuration
 * \return Register value of assertion/de-assertion feature
 */
static inline uint8_t Usart_Get_AssertDeassertRegValues( uint32_t targetTime,
                                                         usart_Baudrate_t baudrate,
                                                         usart_Oversampling_t oversampling )
{
    uint8_t returnValue          = 0u;
    uint8_t oversampleMultiplier = 0u;

    if( USART_OVERSAMPLING_8 == oversampling )
    {
        oversampleMultiplier = 8u;
    }
    else
    {
        oversampleMultiplier = 16u;
    }

    uint32_t time = ( targetTime * (uint32_t)baudrate * (uint32_t)oversampleMultiplier ) / ( 1000000u - 1u );

    if( USART_DE_ASSERT_MAX_VALUE < time )
    {
        returnValue = USART_DE_ASSERT_MAX_VALUE;
    }
    else
    {
        returnValue = time;
    }

    return ( returnValue );
}



/**
 * \brief Calculation of assertion and de-assertion values of Driver Enable (DE) pin
 *
 * \param targetTime   [in]: Required time in us
 * \param baudrate     [in]: Actual baudrate
 * \param oversampling [in]: Oversampling configuration
 * \return Register value of assertion/de-assertion feature
 */
static inline uint32_t Usart_Get_AssertDeassertTime( uint8_t regValue,
                                                     usart_Baudrate_t baudrate,
                                                     usart_Oversampling_t oversampling )
{
    uint32_t returnValue          = 0u;
    uint8_t  oversampleMultiplier = 0u;

    if( USART_OVERSAMPLING_8 == oversampling )
    {
        oversampleMultiplier = 8u;
    }
    else
    {
        oversampleMultiplier = 16u;
    }

    returnValue = ( ( regValue + 1u ) * 1000000u ) / ( baudrate * oversampleMultiplier );

    return ( returnValue );
}


/**
 * \brief Calculate expected prescaler value for USART/UART peripheral
 *
 * \param periphClock  [in]: Peripheral clock in Hz
 * \param oversampling [in]: Peripheral over-sampling configuration
 * \param baudrate     [in]: Required peripheral baud-rate
 * \param prescaler   [out]: Calculated prescaler value
 * \return State of request execution. Returns "OK" if request was success,
 *         otherwise return error.
 */
static inline usart_RequestState_t Usart_Get_ExpectedPrescaler( usart_FreqHz_t periphClock,
                                                                usart_Oversampling_t oversampling,
                                                                usart_Baudrate_t baudrate,
                                                                usart_Prescaler_t *prescaler )
{
    usart_RequestState_t retState = USART_REQUEST_ERROR;

    if( USART_NULL_PTR != prescaler )
    {
        if( USART_OVERSAMPLING_8 == oversampling )
        {
            *prescaler = periphClock / ( baudrate * 8u );
        }
        else
        {
            *prescaler = periphClock / ( baudrate * 16u );
        }

        retState = USART_REQUEST_OK;
    }
    else
    {
        retState = USART_REQUEST_ERROR;
    }

    return ( retState );
}

#ifdef __cplusplus
}
#endif

#endif /* USART_USART_H */
