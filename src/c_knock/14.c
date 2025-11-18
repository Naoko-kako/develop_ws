#include <stdio.h>

int main(){
    int d;
    printf("input number: ");
    scanf("%d",&d);
    for(int i=d;0<=i;i=i-1){
        printf("%d\n",i);
    }
    return 0;
}