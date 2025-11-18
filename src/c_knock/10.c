   #include <stdio.h>

int main(){
    int d;
    printf("input number: ");
    scanf("%d",&d);
    if(d > 0){
        printf("absolute value is %d\n",d);
    }
    else if(d == 0){
        printf("absolute value is %d\n",d);
    }
    else if(d < 0){
        printf("absolute value is %d\n",d*-1);
    }
    else{
    }
    return 0;
}