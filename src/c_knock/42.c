#include <stdio.h>

int main(){
    int a,b,c;
    printf("input number 1: ");
    scanf("%d",&a); 
    printf("input number 2: ");
    scanf("%d",&b);  
    printf("input number 3: ");
    scanf("%d",&c); 

    if(a <= b && b <=c){
        printf("OK\n");
    }
    else{
        printf("NG\n");
    }
    return 0;
}