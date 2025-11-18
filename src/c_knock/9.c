#include <stdio.h>

int main(){
    int d;
    printf("input number: ");
    scanf("%d",&d);
    if(d > 0){
        printf("positive\n");
    }
    else if(d == 0){
        printf("zero\n");
    }
    else if(d < 0){
        printf("negative\n");
    }
    else{
    }
    return 0;
}