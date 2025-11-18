#include <stdio.h>

int main(){
    for(int i=1;i<=20;i++){
        int count = i % 5;
        if(count == 0){
            printf("bar\n");
        }
        else{
            printf("%d\n",i);
        }
    }
    return 0;
}