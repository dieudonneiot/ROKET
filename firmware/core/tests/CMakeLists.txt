set(UNITY_DIR ${PROJECT_SOURCE_DIR}/firmware/third_party/unity)

add_library(unity STATIC ${UNITY_DIR}/unity.c)
target_include_directories(unity PUBLIC ${UNITY_DIR})

add_executable(test_att7 test_att7.c)
target_link_libraries(test_att7 PRIVATE fc_core unity)
add_test(NAME att7 COMMAND test_att7)
