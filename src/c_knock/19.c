#include <stdio.h>

int main(){
    int array[5];
    int d[5];
    for(int i=0; i<5; i++){
        printf("input number :");
        scanf("%d",&d[i]);}
    
    for(int i=0; i<5; i++){
        array[i] = d[i];
        printf("%d\n",array[i]);
    }
    return 0;
}