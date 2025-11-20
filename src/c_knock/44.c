#include <stdio.h>

int main(){
    printf("何円? ");
    int yen;
    scanf("%d",&yen);
    printf("1ドルは何円? ");
    int dollar;
    scanf("%d",&dollar);
    
    int dol,cent;
    dol = yen / dollar;
    int remain;
    remain = yen % dollar;
    
    cent = remain * 100 / dollar;

    printf("%d円は%dドル%dセント\n", yen, dol, cent);
    return 0;
}