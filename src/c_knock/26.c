#include <stdio.h>

int main(){
    printf("input number: ");
    int d;
    scanf("%d",&d);
    switch(d){
        case 1:
            printf("one\n");
            break;
        case 2:
            printf("two\n");
            break;
        case 3:
            printf("three\n");
            break;
        default:
            printf("others\n");
    }
    return 0;
}
