#include <stdio.h>

int main(){
    int d;
    int sum = 0;
    if(d>0){
        for(int i=0;i<5;i++){
            printf("input number: ");
            scanf("%d",&d);
            sum += d;
        }
        printf("sum = %d\n",sum);
    }
    else{}

    return 0;
}