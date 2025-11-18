#include <stdio.h>
#include <math.h>

int main(){
    int a,b;
    printf("input 1st value: ");
    scanf("%d",&a);
    printf("input 2nd value: ");
    scanf("%d",&b);

    int sum = a/b;
    printf("result: %d\n",sum);
    printf("result: %d\n",sum*b);

    return 0;
}