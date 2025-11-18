#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    for(int i=1;i<=9;i++){
        if(i == d){
        }
        else{
            printf("%d\n",i);
        }
    }
    return 0;
}