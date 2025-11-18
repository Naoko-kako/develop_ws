#include <stdio.h>
#include <math.h>

int main(){
    int a,b;
    printf("input 1st number: ");
    scanf("%d",&a);
    printf("input 2nd number: ");
    scanf("%d",&b);
    printf("\n和: %d",a+b);
    printf("\n差: %d",a-b);
    printf("\n積: %d",a*b);
    printf("\n商: %d, 余り: %d\n",a/b,a%b);
    return 0;
}