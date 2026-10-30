#include <stdbool.h>
int test(void){
    volatile _Bool b = TRUE;
    return (int)b;
}
