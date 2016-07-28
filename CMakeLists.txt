set(SOURCES
    "src/parking.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/parking.h"
)

include_directories(
        include
        /usr/local/include
        /usr/include
        /usr/local/include/eigen3
        /usr/include/eigen3
        /usr/include/qt5

)

add_definitions(-DEIGEN_NO_DEBUG)

include_directories(include)

include_directories(include)
add_library(parking MODULE ${SOURCES} ${HEADERS})
target_link_libraries(parking PRIVATE lmscore street_environment phoenix_CC2016_service sensor_utils local_course)
