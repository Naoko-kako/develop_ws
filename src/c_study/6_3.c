#include <stdio.h>

typedef struct Dept{
    int dept_id;
    char name[256];
    char address[100];
}Dept;

typedef struct Employee{
    int employee_id;
    char name[256];
    char mail[100];
    Dept* p_dept;
}Employee;

int main(){

    Dept dep[3] = {{1, "営業部", "東京都千代田区"},
                   {2, "開発部", "大阪府難波"},
                   {3, "人事部", "東京都渋谷区"}};

    Employee emp[6] = {{1,"佐藤 太郎","taro.sato@example.com",&dep[0]},
                      {2, "鈴木 次郎", "jiro.suzuki@example.com", &dep[1]},
                      {3, "高橋 花子", "hanako.takahashi@example.com", &dep[2]},
                      {4, "田中 三郎", "saburo.tanaka@example.com", &dep[0]},
                      {5, "渡辺 美里", "misato.watanabe@example.com", &dep[1]},
                      {6, "伊藤 志乃", "shino.ito@example.com", &dep[2]}};

    for(int i=0;i<6;i++){
        printf("社員: %s\t, 部署: %s\n", emp[i].name, (emp[i].p_dept)->name);
    }

    return 0;
}