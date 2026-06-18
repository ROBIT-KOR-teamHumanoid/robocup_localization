#ifndef BALL_H
#define BALL_H

using namespace std;

class BALL
{
public:

    int x = 0;
    int y = 0;

    int set_x = 0;
    int set_y = 0;

    int d = 999999;

    double speed_x = 0.0;
    double speed_y = 0.0;

    int noballcnt = 0;
    bool from_udp = false;

private:

};


#endif // BALL_H
