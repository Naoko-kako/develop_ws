#include <stdio.h>

int main(){
    int array[10];
    printf("input number :");
    int d;
    scanf("%d",&d);
    for(int i=0;i<10;i++){
        array[i]=d;
        printf("%d\n",array[i]);
    }
    return 0;
}