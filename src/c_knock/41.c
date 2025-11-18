#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    if(d > 0 && d <= 9){
        printf("%d is a single figure.\n",d);
    }
    else {
        printf("%d is not a single figure.\n",d);
    }

    return 0;
}