#include <stdio.h>

int main(){
    int a,b,c;
    printf("input a: ");
    scanf("%d",&a);
    printf("input b: ");
    scanf("%d",&b);
    printf("input c: ");
    scanf("%d",&c);

    int d = b*b - 4*a*c;

    if(d>0){
        printf("２つの実数解\n");
    }
    else if(d==0){
        printf("重解\n");
    }
    else if(d<0){
        printf("２つの虚数解\n");
    }
    else{}
    return 0;
}