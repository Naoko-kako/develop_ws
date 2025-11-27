#include <stdio.h>

int main(){
    int a,b;
    int *pa,*pb;
    int temp;
    printf("input a: ");
    scanf("%d",&a);
    printf("input b: ");
    scanf("%d",&b);
    
    // アドレス割当    
    pa = &a;
    pb = &b;

    // 一旦aを空にする
    temp = *pa;
    *pa = *pb;
    *pb = temp;
    
    printf("a = %d\n",a);
    printf("b = %d\n",b);
    return 0;
}