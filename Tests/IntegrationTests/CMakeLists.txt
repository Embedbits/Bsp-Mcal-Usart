#===================== Integration tests configuration ========================#
# Integration tests of Usart module on target (Nucleo-144 / 32F746GDISCOVERY). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).
#
# The test set is independent - target interface (entry point, initialization,
# fault handlers, system reset) is implemented by ItTarget_Usart.c and
# all libraries of the firmware are listed here.

# StartUp calls BspMain() - the entry point of the test firmware is provided by
# the test set (BspMain.h, ItTarget_Usart.c)
if(NOT TARGET BspMain_Lib)
    add_library(BspMain_Lib INTERFACE)
    target_include_directories(BspMain_Lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
endif()

if(MCU_FAMILY_ID STREQUAL "STM32H7RSxx")

    # STM32H7R / STM32H7S (NUCLEO-H7S3L8)
    # The firmware needs 154 % of the 64 KB flash at -O0 and does not link, optimization is required.
    if(CMAKE_C_FLAGS_INTEGRATIONTEST MATCHES "-O0")
        message(WARNING "Integration test IT_Usart of STM32H7R / STM32H7S does not fit into 64 KB flash at -O0 - configure with -DINTEGRATION_TEST_OPTIMIZATION=-Os")
    endif()

    IntegrationTesting_Add_Test(
        NAME            Usart
        TEST_SOURCE     ItTest_Usart_H7RS.c
        SOURCES         ItTarget_Usart.c    # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        Usart_Lib
                        Tim_Lib             # reference time base (TIM2)
        TIMEOUT         90
    )

else()

    # Classic STM32H7 (NUCLEO-H723ZG, NUCLEO-H7A3ZI-Q)
    IntegrationTesting_Add_Test(
        NAME            Usart
        TEST_SOURCE     ItTest_Usart.c
        SOURCES         ItTarget_Usart.c    # Target interface of the test set
        LINK_LIBS       StartUp_Lib         # Startup code (calls BspMain)
                        Nvic_Lib            # Fault handlers, system reset (ItTarget)
                        Usart_Lib           # Module under test
                        Dwt_Lib             # Reference time base (DWT cycle counter)
        TIMEOUT         90
    )

endif()
