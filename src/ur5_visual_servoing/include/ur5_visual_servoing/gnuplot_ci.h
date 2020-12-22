//
// Created by vcr on 2020-12-21.
//

#ifndef IBVS_WS_GNUPLOT_CI_H
#define IBVS_WS_GNUPLOT_CI_H

#include<cstdio>
#include<cstdlib>
#include<cstring>
#include<unistd.h>
#include<csignal>
#include<cmath>
#include <cstring>

namespace gnuplot_ci
{
    class GP_handle
    {
    private:
        char *gp_binary;
        char *set_term;
        char *xt;
        char *yt;
        char *zt;

        FILE *fp;

        int arrow_cnt;    // To keep count of arrows

    public:
        GP_handle(const char *gp_bin, const char *xl, const char *yl, const char *zl=NULL);
        GP_handle(const char *gp_bin);
        ~GP_handle();
        void gen_random_string(char *s, const int len);
        void gen_random_color(char *s);
        void gnuplot_cmd(const char *command);
        void draw_circle(float x, float y, float r);
        void draw_sphere(float x, float y, float z, float r, int iso, int ltc=0);
        void draw_cuboid(float x1, float y1, float z1, float x2, float y2, float z2);
        void draw_cuboid2(double x1[3], double dx[3], int replot_flag=0);
        void draw_cuboid_solid(double x1[3], double dx[3], int replot_flag=0);
        void draw3dcoordAxis(double xp[],double dx[], bool replot_flag=false, int cFlag = 1, float scale=1.0);
        void draw3dcoordAxis(double x[], double xp[], double yp[], double zp[], bool replot_flag=false, int cFlag = 1);
        int get_arrow_cnt() const;
    };
}


#endif //IBVS_WS_GNUPLOT_CI_H
