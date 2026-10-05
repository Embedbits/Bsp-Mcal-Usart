#======================== Unit tests configuration ============================#
# Unit tests of Usart module. Registered in module CMakeLists.txt by
# UnitTesting_AddPath() and created by UnitTesting_Generate() in "UnitTest"
# build (see EmBi_Platform/CMake/UnitTesting/README.md).

get_filename_component(USART_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(MCAL_DIR  "${USART_DIR}/.." ABSOLUTE)

UnitTesting_Add_Test(
    NAME            Usart
    TEST_SOURCE     Test_Usart.c
    UUT_SOURCES     ${USART_DIR}/Usart.c
                    ${USART_DIR}/Usart_Dma.c
                    ${USART_DIR}/Usart_Isr.c
                    ${USART_DIR}/Usart_Poll.c
    MOCK_HEADERS    ${MCAL_DIR}/Rcc/Rcc_Port.h
                    ${MCAL_DIR}/Nvic/Nvic_Port.h
                    ${MCAL_DIR}/Gpio/Gpio_Port.h
                    ${MCAL_DIR}/Gpdma/Gpdma_Port.h
    INCLUDE_DIRS    ${USART_DIR}
                    ${MCAL_DIR}/Rcc
                    ${MCAL_DIR}/Nvic
                    ${MCAL_DIR}/Gpio
                    ${MCAL_DIR}/Gpdma
    LINK_LIBS       Ral_Lib
)
