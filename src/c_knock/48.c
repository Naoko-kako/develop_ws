#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    
    int i=0;
    
    while(d != 1){
        i++;   
        if(d % 2 == 0){
            d = d / 2;
        }
        else{
            d = 3 * d + 1;
        }
        printf("%d: %d\n",i,d);     
    }    
    return 0;
}