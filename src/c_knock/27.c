#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    int sum = 0;

    if(d > 0){
        for(int i = 1; i <= d; i++){
                sum = sum + i;
        }
        printf("sum = %d\n",sum);
    }
    else{
        printf("sum = 0\n");
    }
    
    return 0;
}