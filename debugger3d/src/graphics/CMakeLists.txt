get_filename_component(libname ${CMAKE_CURRENT_SOURCE_DIR} NAME)

file(GLOB c_sources "*.cpp")

add_library(${libname} STATIC ${c_sources})
