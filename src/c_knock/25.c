#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    if(d < -10){
        printf("range 1\n");
    }
    else if(d >= -10 && d < 0){
        printf("range 2\n");
    }
    else if(d >= 0){
        printf("range 3 \n");
    }
    else{}
    return 0;
}