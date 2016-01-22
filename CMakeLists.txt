set(SOURCES
    "src/parking.cpp"
    "src/interface.cpp"
)

set(HEADERS
    "include/parking.h"
)

include_directories(include)
add_library(parking MODULE ${SOURCES} ${HEADERS})
target_link_libraries(parking PRIVATE lmscore phoenix_CC2016_service sensor_utils local_course)
