#include <stdio.h>

typedef struct {
    char name[50];
    int age;
    double score; 
}Student;

int main(){
    
    Student stu[3];

    for(int i=0;i<3;i++){
        int num = i +1;
        printf("学生 %d の名前は？: ", num); scanf("%49s",stu[num].name); 
        printf("学生 %d の年齢は？: ", num); scanf("%d",&stu[num].age); 
        printf("学生 %d の成績は？: ", num); scanf("%lf",&stu[num].score);
    }

    printf("学生データ一覧\n");
    for(int i=0;i<3;i++){ 
        int num = i + 1;
        printf("学生 %d : 名前 = %s, 年齢 = %d, 成績 = %lf\n"
                , i, stu[num].name, stu[num].age, stu[num].score);
    }

    return 0;
}