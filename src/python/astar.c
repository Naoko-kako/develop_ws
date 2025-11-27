#include <stdio.h>

#define NODE_MAX 1000
/*
g(n) = スタートから n までの累積コスト
h(n) = n からゴールまでの推定コスト（ヒューリスティック）
f(n) = g(n) + h(n)
*/
typedef struct Node{
    int row,col;
    int f;
    int g;
    int h;
    struct Node *parent;
}Node;

typedef struct List{
    int index;
    Node *node[NODE_MAX];
}List;

void search(List *open,List *close){


}



int main(){
    return 0;
}