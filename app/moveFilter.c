#include "moveFilter.h"

#define N 5  // 滤波窗口大小

float filter(float newData) {
    static float buf[N];
    static int index = 0;
    static float sum = 0;
    static int count = 0;
    
    if(count == N) {
        sum -= buf[index];
    } else {
        count++;
    }
    
    buf[index] = newData;
    sum += newData;
    index = (index + 1) % N;
    
    return sum / count;
}
