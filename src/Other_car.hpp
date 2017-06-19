#ifndef OTHER_CAR_H
#define OTHER_CAR_H

class Other_car{
private:
    int id;
    float x;
    float y;
    float vx;
    float vy;
    float s;
    float d;

public:
    Other_car(int id, float x,float y,float vx,float vy,float s,float d):id(id)
      ,x(x),y(y),vx(vx),vy(vy),s(s),d(d){}
};

#endif // OTHER_CAR_H

/*
typedef struct OtherCars{
        int id;
        float x;
        float y;
        float vx;
        float vy;
        float s;
        float d;
}Other_cars;
 */
