#===================== Integration tests configuration ========================#
# Integration tests of Usart module on target (Nucleo board). Registered in module
# CMakeLists.txt by IntegrationTesting_AddPath() and created by
# IntegrationTesting_Generate() in "IntegrationTest" build
# (see EmBi_Platform/CMake/IntegrationTesting/README.md).

IntegrationTesting_Add_Test(
    NAME            Usart
    TEST_SOURCE     ItTest_Usart.c
    LINK_LIBS       Usart_Lib
                    Tim_Lib             # reference time base (TIM2)
    TIMEOUT         90
)
