#include <stdio.h>

int main(){
    printf("距離 ");
    int k,cost;
    scanf("%d",&k);
    
    if(k >= 0&& k<=1700){
        cost = 610;
        printf("金額 %d\n",cost);
    }
    else if(k > 1700){
        int count = (k-1700) / 313;       //繰り返し
        int i = (k-1700) % 313;            //制限
            if(i == 0){
                count = count;
            }
            else if(i =! 0){
                count = count + 1;
            }
            else{}

        cost = 610 + 80 * count;
        printf("金額 %d\n",cost);
        
    }
    else{}
    return 0;
}
