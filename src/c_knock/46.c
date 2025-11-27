#include <stdio.h>

int main(){
    int visitor;
    printf("人数 ");
    scanf("%d",&visitor);

    int sum;
    int i = visitor;
    if(i < 5 && i > 0){
        sum = visitor * 600;
    }
    else if(i < 20 && i >=5){
        sum = visitor * 550;
    }
    else if(i >= 20){
        sum = visitor * 500;
    }
    else{
        sum = 0;
    }
    printf("料金 %d\n",sum);
    return 0;
}