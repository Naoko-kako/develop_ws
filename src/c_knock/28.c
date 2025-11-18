#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    int factorial = 1;
    if(d > 0){
        for(int i=1; i<=d; i++){
            factorial = factorial * i;
        }

        printf("factorial = %d\n",factorial);

    }
    else if(d <= 0){

        printf("factorial = 1\n");

    }
    else{}
    return 0;
}