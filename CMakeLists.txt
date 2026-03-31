# Include current directory for provide headers include
set( Usart_IncludeDirs
    ${CMAKE_CURRENT_SOURCE_DIR} 
)


# Set source files for library generation
# All used function files shall be added here.
set( Usart_LibSrcs
    Usart.c )


# Set public header files for the library
# All header files used by other libraries shall be added here.
set( Usart_PublicHeaders 
    Usart_Port.h 
)


# Set dependent libraries
# All libraries used by current target shall be added here.
set( Usart_DependentLibs
    Ral_Lib
    Gpio_Lib
    Gpdma_Lib
    Nvic_Lib
    Rcc_Lib
)


# Create library
add_library(Usart_Lib STATIC ${Usart_LibSrcs})


# Specify the public headers for the library
set_target_properties( Usart_Lib PROPERTIES PUBLIC_HEADER "${Usart_PublicHeaders}" )


# Include necessary libraries
target_link_libraries( Usart_Lib 
    PUBLIC 
    ${Usart_DependentLibs}
)


target_include_directories( Usart_Lib 
    INTERFACE 
    ${Usart_IncludeDirs}
)


message(STATUS "Usart_Lib library created")

#========================= Documentation configuration ========================#
Doxygen_AddPath("${CMAKE_CURRENT_SOURCE_DIR}/")
