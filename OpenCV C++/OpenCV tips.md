# OpenCV tips

## visualize image

```
#include <opencv2/opencv.hpp>
cv::Mat img = cv::imread(path_to_image);
cv::imshow("title of window", img);
cv::waitKey();
```

## access pixel of image
```c++
Mat im(3,3,CV_8U3); // 3 channels
im.at<Vec3b>(0,0)[0] = 2; // access (0,0) pixel r channel
im.at<Vec3b>(0,0)[1] = 2; // access (0,0) pixel g channel
im.at<Vec3b>(0,0)[2] = 2; // access (0,0) pixel b channel

Mat img(3,3,CV_8U); // 1 channel
im.at<uchar>(0,0) = 2;
```
Mat_<uchar>---------CV_8U
Mat<char>-----------CV_8S
Nat_<short>---------CV_16S
Mat_<ushort>--------CV_16U
Mat_<int>-----------CV_32S
Mat_<float>----------CV_32F
Mat_<double>--------CV_64F
--------------------- 
作者：liulina603 
来源：CSDN 
原文：https://blog.csdn.net/liulina603/article/details/47277793 
版权声明：本文为博主原创文章，转载请附上博文链接！