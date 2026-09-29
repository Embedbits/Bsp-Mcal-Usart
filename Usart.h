/**
 * \author Mr.Nobody
 * \file Usart.h
 * \ingroup Usart
 * \brief Universal Synchronous/Asynchronous Receiver-Transmitter (USART) MCAL
 *        module common functionality header file.
 *
 * This file contains the common functionality used internally by the module,
 * and shall provide interface between the module and the application.
 */

#ifndef USART_USART_H
#define USART_USART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================= INCLUDES =================================== */
#include "Usart_Types.h"                    /* Module types definition        */
/* ============================= TYPEDEFS =================================== */

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


#ifdef __cplusplus
}
#endif

#endif /* USART_USART_H */
