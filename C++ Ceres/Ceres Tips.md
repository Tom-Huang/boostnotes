# Ceres Tips

Please check this link for details of using function and class: [How to model optimization problems in ceres](http://ceres-solver.org/nnls_modeling.html?highlight=addparameterblock#_CPPv2N5ceres7Problem17AddParameterBlockEPdiP21LocalParameterization)

## Undefined reference to Ceres::xxx

the capital in difference `ceres`s

```txt
find_package(Ceres 1.14 REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(my_lib ${CERES_LIBRARIES})
```
