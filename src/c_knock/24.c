#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    if((-10 <= d && d < 0) || d >= 10){
        printf("OK\n");
    }
    else{
        printf("NG\n");
    }
    return 0;
}