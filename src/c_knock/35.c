#include <stdio.h>

int main(){

    int array[10] = {3, 7, 0, 8, 4, 1, 9, 6, 5, 2};

    printf("input index: ");
    int d;
    scanf("%d",&d);

    printf("array[%d] = %d\n", d,array[d]);

    return 0;
}