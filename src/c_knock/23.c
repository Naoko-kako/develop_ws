#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    if(d >= -5 && d < 10){
        printf("OK\n");
    }
    else{
        printf("NG\n");
    }
    return 0;
}