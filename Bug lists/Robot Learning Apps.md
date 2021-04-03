# Robot Learning Apps

## 1. Can not reach init position
initial gains is too small. Remember that the last joint should have a larger gain, because the damping of the last joint is very large.

## 2. Initialize VectorXd or MatrixXd before assigning value to them

```cpp
// init the vector
Eigen::VectorXd a(6); // init the length
// or
Eigen::VectorXd b = Eigen::VectorXd::Zero(6); // set to zero
// or
Eigen::VectorXd c;
c.setZero(6); // set c as a 6 dim zero vector
```