#include <stdio.h>
#include <math.h>

int main(){
    int d;
    printf("input number: ");
    scanf("%d",&d);
    for(int i=0;i<d;i=i+2){
        printf("%d\n",i);
    }
    return 0;
}