# Eigen

## Usage

1.Eigen::Map<output_Type> output_name(input_name); // input_name is type T, output_name is type

## add column to matrix

```c++
Eigen::MatrixX<double, dynamic, dynamic> mat;
mat.conservativeResize(mat.rows(), mat.cols()+1);
mat.col(mat.cols()-1) = vec;
```