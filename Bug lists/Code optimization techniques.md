# Code optimization techniques

## 1. expand for loop manually when the number of iteration is fixed

## 2. early ending in the loop

if the end criterium of the loop is that some values should exceed some thresholds. And if the value is changed according to some rule(like exponential and so on), we can compare the previous value and the current value. If the change is smaller than 0.0000001, we can say that it will not change a lot if we further iterate. So we can end the loop early.

## 3. be aware of the cache misses

cache misses will drop the performance significantly. Find a way to reduce the number of times one thread needs to access the memory. One load from the thread can load a cache line. 

