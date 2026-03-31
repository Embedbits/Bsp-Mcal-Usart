# Template version: 1.0.1
# Sets the version of CMakeLists.txt file.
# Never, under any circumstances, change this line. It is used by
# the module generation script to identify the template version.
#======================== Library configuration ===============================#
# Publicly included directories to provide headers include
set( Usart_PublicIncludeDirs
    
)


# Privately included directories to provide headers include
set( Usart_PrivateIncludeDirs
    ${CMAKE_CURRENT_SOURCE_DIR}
)


# Source files list for current library generation
set( Usart_SourceFiles
    Usart.c
    Usart_CommDma.c
    Usart_CommInt.c
    Usart_CommPoll.c
)


# List of public header files provided by the current library
set( Usart_PublicHeaders 
    Usart_Types.h
    Usart_Port.h 
)


# List of dependent libraries that are publicly used by the current library
set( Usart_PublicDependentLibs

)


# List of dependent libraries that are privately used by the current library
set( Usart_PrivateDependentLibs
    Ral_Lib
    Gpio_Lib
    Nvic_Lib
    Dma_Lib
)


#========================== Library generation ================================#
# Create library
add_library(Usart_Lib STATIC ${Usart_SourceFiles})


# Include necessary libraries to the target
target_link_libraries( Usart_Lib
    PUBLIC 
        ${Usart_PublicDependentLibs} 
    PRIVATE 
        ${Usart_PrivateDependentLibs}
)


# Set public headers directory path in build folder
set(PUBLIC_HEADERS_DIR ${CMAKE_CURRENT_BINARY_DIR}/PublicHeaders)

# Generates the public headers directory in build folder
file(MAKE_DIRECTORY ${PUBLIC_HEADERS_DIR})

# Process all public headers
foreach(PublicHeader IN LISTS Usart_PublicHeaders)
    
    # Copy (or configure) only the public header there
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/${PublicHeader} 
        ${PUBLIC_HEADERS_DIR}/${PublicHeader}
        COPYONLY
    )
    
endforeach()


# Set include directories for the created target
target_include_directories( Usart_Lib
    PUBLIC
        $<BUILD_INTERFACE:${PUBLIC_HEADERS_DIR}>
        $<INSTALL_INTERFACE:include>
        Usart_PublicIncludeDirs
    PRIVATE
        ${Usart_IncludeDirs}
        Usart_PrivateIncludeDirs
)


# Print status message
message(STATUS "Usart_Lib library created")

#========================= Documentation configuration ========================#
if(DOXYGEN_AVAILABLE STREQUAL "ON")
    Doxygen_AddPath("${CMAKE_CURRENT_SOURCE_DIR}/")
endif()
