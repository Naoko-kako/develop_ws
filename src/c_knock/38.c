#include <stdio.h>

int main(){
    int array[10] ={3, 7, 0, 8, 4, 1, 9, 6, 5, 2};

    int num = 0;
    int num_plus = 0;
   
    for(int i=0;i<10;i++){
        num_plus = array[num];
        printf("%d\n",array[num]);
        num = num_plus;
    }
       
    return 0;
}