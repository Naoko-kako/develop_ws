#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    if(d % 2 == 0){
        printf("%d is even\n",d);
    }
    else if(d % 2 == 1){
        printf("%d is odd\n",d);
    }
    else{}
    return 0;

}