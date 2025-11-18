#include <stdio.h>

int main(){

    int array[10] = {3, 7, 0, 8, 4, 1, 9, 6, 5, 2};

    int a,b;
    printf("input index1: ");
    scanf("%d",&a);
    printf("input index2: ");
    scanf("%d",&b);

    printf("%d * %d = %d\n", array[a], array[b], array[a]*array[b]);

    return 0;
}