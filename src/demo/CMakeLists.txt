# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)

# Add executables
## demo
add_executable(cbba_demo cbba_demo.cpp)
target_link_libraries(cbba_demo cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(map_vis_demo map_vis_demo.cpp)
target_link_libraries(map_vis_demo cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(syn_demo syn_demo.cpp)
target_link_libraries(syn_demo cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(cbga_demo cbga_demo.cpp)
target_link_libraries(cbga_demo cbga cbta cbba vis ltl map utility ${OpenCV_LIBS})

add_executable(cbba_cbta_demo cbba_cbta_demo.cpp)
target_link_libraries(cbba_cbta_demo cbta cbba vis map ltl ${OpenCV_LIBS})

add_executable(syn_cbta_demo syn_cbta_demo.cpp)
target_link_libraries(syn_cbta_demo cbta cbba vis map ltl ${OpenCV_LIBS})


