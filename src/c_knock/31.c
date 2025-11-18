#include <stdio.h>

int main(){
    printf("input number :");
    int d;
    scanf("%d",&d);

    if(d > 0){
        for(int i = 0; i < d; i++){
                int count = i % 5;
                    if(count == 4){
                        printf("* ");
                    }
                    else{
                        printf("*");
                    }
        }
    }
    else{
    }
    
    printf("\n");  

    return 0;
}