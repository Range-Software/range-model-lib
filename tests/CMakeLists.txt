# Unit tests for range-model-lib (QTest based).
#
# Enabled by configuring the project with -DRANGE_BUILD_TESTS=ON.
# Run with `ctest` from the build directory.

find_package(Qt6 REQUIRED COMPONENTS Test)

set(CMAKE_AUTOMOC ON)

set(range_model_lib_tests
    tst_node
    tst_element
    tst_segment
    tst_triangle
    tst_tetrahedron
    tst_sparse_vector
    tst_sparse_matrix
)

foreach(test_name IN LISTS range_model_lib_tests)
    qt_add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name}
        PRIVATE
            range-model-lib
            Qt6::Test
    )
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
