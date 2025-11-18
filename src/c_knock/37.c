#include <stdio.h>

int main(){
    
    int array[10] = {3, 7, 0, 8, 4, 1, 9, 6, 5, 2};

    printf("input index: ");
    int d;
    scanf("%d",&d);

    int num = array[d-1]; // array[8] = 6 
    
    printf("value = %d\n",array[num-1]);
    return 0;
}