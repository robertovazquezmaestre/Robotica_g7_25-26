INCLUDE($ENV{ROBOCOMP}/cmake/modules/opencv4.cmake)

# Sources set
LIST(APPEND SOURCES
        ../src/specificworker.cpp
        ../src/hungarian.cpp
        ../src/ransac_line_detector.cpp
        ../src/room_detector.cpp

)

# Headers set
LIST(APPEND HEADERS
        ../src/specificworker.h
)


LIST(APPEND LIBS ${LIBS})

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

# Usar TBB para paralelización (requerido por std::execution::par)
find_package(TBB REQUIRED)
if(TBB_FOUND)
    message(STATUS "TBB found, linking with target")
    set(LIBS ${LIBS} TBB::tbb)
endif()


