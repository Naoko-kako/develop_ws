#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);

    if(d > 0){
        for(int i=0; i<d; i++){
            printf("*");
        }
    }
    else{}
    
    printf("\n");

    return 0;
}