#include <stdio.h>

int main(){
    int array[10] = {3, 7, 0, 8, 4, 1, 9, 6, 5, 2};

    int num = 0;
    int dif;
   
    for(int i;i<9;i++){
        dif = array[num] - array[num+1];
        printf("%d\n",dif);
        num = num
    }
    return 0;
}