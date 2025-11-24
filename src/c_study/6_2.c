#include <stdio.h>

typedef struct{
    char maker[250]; //製造会社
    int year;        //製造年
    double speed;    //最高速度
}Engine;

typedef struct{
    int num;         //ナンバー
    double gas;      //ガス
    Engine engine;   //エンジン
}Car;

int main(){
    Engine e[4] = {{"YAMAHA",2015,16},
                   {"HONDA",2018,20},
                   {"SUZUKI",2024,23},
                   {"HITATI",2005,25}};

    Car car[4] = {{1234,25.5,e[0]},
                  {5678,15.0,e[1]},
                  {9101,10.0,e[2]},
                  {1122,30.0,e[3]}};


    for(int i=0; i<4; i++){
            printf("車のナンバー: %d\n",car[i].num);
            printf("車のガゾリン料: %.2fL\n",car[i].gas);
            printf("エンジンのメーカー: %s\n",car[i].engine.maker);
            printf("エンジンの製造年: %d年\n",car[i].engine.year);
            printf("エンジンの最高速度: %.2fkm\n",car[i].engine.speed);
            printf("-----------------------------\n");
    }
    

    return 0;
}
