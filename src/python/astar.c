#include astar.h


/* NHKロボコン 2026 
 R2
 経路生成　A*(C言語) */

#define MAX_NODE 500

//初期設定
struct NODE{
    int row,col;
    int g,h,f;
    struct NODE *parent;
};
typedef struct NODE NODE;

//NODEを保管するリスト
struct LIST{
    NODE *node[MAX_NODE]; //node[]をmax個数つくる
    int num; //繰り返す回数
};
typedef struct LIST LIST;

void astar(){
    //初期化
    NODE start_node = {0,0,0,0,0,NULL};
    NODE end_node = {0,0,0,0,0,NULL};
    LIST *open_list = {.num = 0};

    for(num = 1, num < MAX_NODE, num++){
       
    }

    if(open == 1){

    }
}

int main() {
    return 0;
}