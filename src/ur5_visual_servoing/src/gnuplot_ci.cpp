//
// Created by vcr on 2020-12-21.
//

/* C++ interface to GNUPLOT
 * File: gnuplot_ci.cpp
 *
 * Date: 09 August 2007 Thursday
 * Status: success
 * ---------------------------------------------------- */

/* -----------
 * Updates:
 *
 * Date: 12 October 2007 Friday
 *
 * Constructor GP_handle::GP_handle(char *bin_path) added
 * --------------------------------------- */

#include<iostream>
#include<fstream>
#include<cstdio>
#include<ur5_visual_servoing/gnuplot_ci.h>

using namespace std;
using namespace gnuplot_ci;

//-------------------------------------------
// This function generates random strings of a desired length
void GP_handle::gen_random_string(char *s, const int len)
{
    static const char alphanum[] =
            "0123456789"
            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
            "abcdefghijklmnopqrstuvwxyz";

    for (int i = 0; i < len; ++i) {
        s[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    s[len] = 0;
}
//-------------------------------------------------------------
// This function generates random rgb color hexadecimal strings between (#000000 to #FFFFFF)
void GP_handle::gen_random_color(char *color)
{
    const char *hex_digits = "0123456789ABCDEF";
    unsigned char str[7] = {0};
    for(int i = 0 ; i < 6; i++ ) {
        str[i] = hex_digits[ ( rand() % 16 ) ];
    }
    sprintf(color,"#%s",str);
}

GP_handle::GP_handle(const char *bin_path, const char *xl, const char *yl, const char *zl)
{
    xt = new char[200];
    yt = new char[200];
    zt = new char[200];
    gp_binary = new char[200];

    set_term = (char*) "set terminal X11\n";
    strcpy(gp_binary, bin_path);
    strcat(gp_binary, "gnuplot");

    sprintf(xt, "set xlabel \"%s\"\n", xl);
    sprintf(yt, "set ylabel \"%s\"\n", yl);

    if(zl != NULL)
        sprintf(zt, "set zlabel \"%s\"\n", zl);

    fp = popen(gp_binary, "w");

    if(fp == NULL)
    {
        cout << "Error opening a pipe to gnuplot\n";
        exit(-1);
    }
    fprintf(fp, "%s", set_term);
    fflush(fp);
    fprintf(fp, "%s", xt);
    fflush(fp);
    fprintf(fp, "%s", yt);
    fflush(fp);
    if(zl != NULL)
    {
        fprintf(fp, "%s", zt);
        fflush(fp);
        fprintf(fp, "set border 4095\n");
        fflush(fp);
        fprintf(fp, "set ticslevel 0\n");
        fflush(fp);
    }

    //initialize the arrow_cnt
    arrow_cnt = 0;
}
//-----------------------------------------
// Overloaded Constructor
//
// This one is used when one needs to load a file
// -----------------------------------------------
GP_handle::GP_handle(const char *bin_path)
{
    gp_binary = new char[200];

    set_term = (char*)"set terminal X11\n";
    strcpy(gp_binary, bin_path);
    strcat(gp_binary, "gnuplot");

    fp = popen(gp_binary, "w");

    if(fp == NULL)
    {
        cout << "Error opening a pipe to gnuplot\n";
        exit(-1);
    }
    fprintf(fp, "%s", set_term);
    fflush(fp);

    //initialize arrow_cnt
    arrow_cnt = 0;
}
//--------------------------------
GP_handle::~GP_handle()
{
    pclose(fp);
    delete [] xt;
    delete [] yt;
    delete [] zt;
    delete [] gp_binary;

    cout << "gnuplot_ci destructor called" << endl;
}
//--------------------------------------------
void GP_handle::gnuplot_cmd(const char *cmd)
{

    char *command = new char[400];
    sprintf(command, "%s\n", cmd);
    fprintf(fp, "%s", command);
    fflush(fp);
    delete [] command;
}
//-------------------------------------------------
// Draw a circle
void GP_handle::draw_circle(float x, float y, float r)
{
    fprintf(fp, "set parametric\n");
    fflush(fp);
    fprintf(fp, "set trange [0:2*pi]\n");
    fflush(fp);
    fprintf(fp, "replot %f+%f*sin(t), %f+%f*cos(t) notitle\n", x,r,y,r);
    fflush(fp);
    //fprintf(fp, "unset parametric\n");
    //fflush(fp);
}
//----------------------------------------------------
// Draw a sphere
// ltc : line type (color)
// {x,y,z} : center of sphere
// iso : grid
// ----------------------------------------------------
void GP_handle::draw_sphere(float x, float y, float z, float r, int iso, int ltc)
{
    fprintf(fp, "set parametric\n");
    fflush(fp);
    fprintf(fp, "set urange [0:2*pi]\n");
    fflush(fp);
    fprintf(fp, "set vrange [0:2*pi]\n");
    fflush(fp);
    fprintf(fp, "set isosamples %d,%d\n", iso, iso);
    fflush(fp);
    if(ltc == 0)
    {
        fprintf(fp, "splot %f+%f*cos(u)*cos(v), %f+%f*sin(u)*cos(v), %f+%f*sin(v) notitle\n", x,r,y,r,z,r);
        fflush(fp);
    }
    else
    {
        fprintf(fp, "replot %f+%f*cos(u)*cos(v), %f+%f*sin(u)*cos(v), %f+%f*sin(v) lt %d notitle\n", x,r,y,r,z,r,ltc);
        fflush(fp);
    }
    //fprintf(fp, "unset parametric\n");
    //fflush(fp);
}
//-----------------------------------------------
// Draw a cuboid
// (x1,y1,z1) --> left-bottom point
// (x2,y2,z2) --> top-right point
//
// Date: 15 Feb 2007 Friday
// Tested : OK
// -----------------------------------------------
void GP_handle::draw_cuboid(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;

    fprintf(fp, "splot '-' index 0:4 u 1:2:3 w l t 'cuboid'\n");
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1, z1);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1+dy, z1);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1+dy, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1, z1+dz);
    fflush(fp);
    fprintf(fp, "\n\n");
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1, z1);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1, z1);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1, z1);
    fflush(fp);
    fprintf(fp, "\n\n");
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1+dy, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1+dy, z1);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1, z1);
    fflush(fp);
    fprintf(fp, "\n\n");
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1+dy, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1+dy, z1+dz);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1+dx, y1+dy, z1);
    fflush(fp);
    fprintf(fp, "%f %f %f\n", x1, y1+dy, z1);
    fflush(fp);
    fprintf(fp, "e\n");
    fflush(fp);
}

//--------------------------------------------
// Draw a cuboid which left-bottom corner is given along with the
// length, breadth and height of the cuboid.
//
// Date: 30 August 2008 Saturday
// Status: checked
//-----------------------------------------

void GP_handle::draw_cuboid2(double xinit[], double dx[],int flag)
{

    char name[10];
    gen_random_string(name, 10);


    ofstream f1(name);

    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl << endl;

    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl << endl;

    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl << endl;

    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl << endl;


    f1.close();



    if(flag == 0)
    {
        fprintf(fp, "splot '%s' u 1:2:3 w l lw 2 notitle\n", name);
        fflush(fp);
    }
    else
    {
        fprintf(fp, "replot '%s' u 1:2:3  w l lw 2 lt %d notitle\n",name, flag);
        fflush(fp);
    }
}//EOF
//----------------------------------------------------------------
// Draw a Solid Cuboid
// This works only with one coloured box. It can not plot two colored boxes one after another.
// Only the last command remains in effect and overrides the previous plots. It works if a getchar is placed between
// two consecutive calls.
// - Only Last color palette is rendered by the plot.
// Date: January 31, 2016
// ------------------------------------------------
void GP_handle::draw_cuboid_solid(double xinit[], double dx[], int flag)
{

    srand(time(NULL));
    char name[10], color[8];
    gen_random_string(name, 10);
    gen_random_color(color);
    int pi = rand() % 10;


    ofstream f1(name);

    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl << endl;

    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl << endl;

    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2] << endl << endl;

    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0]+dx[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1]+dx[1] << "\t" << xinit[2]+dx[2] << endl;
    f1 << xinit[0] << "\t" << xinit[1] << "\t" << xinit[2]+dx[2] << endl << endl;

    f1.close();


    if(flag == 0)
    {
        fprintf(fp, "set cbrange [0.9:1]\n");
        fflush(fp);
        fprintf(fp, "set palette defined (%d '%s')\n", pi, color);
        fflush(fp);
        fprintf(fp, "set pm3d depthorder hidden3d explicit\n");
        fflush(fp);
        fprintf(fp, "unset colorbox\n");
        fflush(fp);
        fprintf(fp, "splot '%s' u 1:2:3:(%d) w pm3d lw 2 notitle\n", name, pi);
        fflush(fp);
        fprintf(fp, "unset hidden3d\n");
        fflush(fp);
        fprintf(fp,"unset pm3d\n");
        fflush(fp);
    }
    else
    {
        fprintf(fp, "set cbrange [0.9:1]\n");
        fflush(fp);
        fprintf(fp, "set palette defined (%d '%s')\n", pi, color);
        fflush(fp);
        fprintf(fp, "set pm3d depthorder hidden3d explicit\n");
        fflush(fp);
        fprintf(fp, "unset colorbox\n");
        fflush(fp);
        fprintf(fp, "replot '%s' u 1:2:3:(%d)  w pm3d lw 2 lt %d notitle\n",name, pi, flag);
        fflush(fp);
        fprintf(fp, "unset hidden3d\n");
        fflush(fp);
        fprintf(fp,"unset pm3d\n");
        fflush(fp);

    }

}


//--------------------------------
// This functions draws 3D axes at a given a 3d point xp [] with a
// displacement vector dx[3]
// You should have an existing splot before calling this function
// -------------------------------------------------------
void GP_handle::draw3dcoordAxis(double xp[], double dx[], bool replot_flag, int cFlag, float scale)
{

    // Check if SPLOT exists or not.
    //fprintf(fp, "if(GPVAL_SPLOT == 1){\n");
    //fflush(fp);

    if(cFlag == 1)
    {
        //arrow styles
        fprintf(fp,"set style arrow 1 head filled size 0.03,10,0 lt 1 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 2 head filled size 0.03,10,0 lt 2 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 3 head filled size 0.03,10,0 lt 3 lw 2\n");
        fflush(fp);
    }
    else if(cFlag == 2)
    {
        //arrow styles
        fprintf(fp,"set style arrow 1 head filled size 0.03,10,0 lt 4 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 2 head filled size 0.03,10,0 lt 5 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 3 head filled size 0.03,10,0 lt 6 lw 2\n");
        fflush(fp);
    }
    else
    {
        fprintf(fp,"set style arrow 1 head filled size 0.03,10,0 lt 7 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 2 head filled size 0.03,10,0 lt 8 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 3 head filled size 0.03,10,0 lt 9 lw 2\n");
        fflush(fp);
    }

    arrow_cnt++;
    fprintf(fp, "set arrow %d from %f,%f,%f to %f, %f, %f as 1\n", arrow_cnt, xp[0], xp[1], xp[2], xp[0]+scale*dx[0],xp[1],xp[2]);
    fflush(fp);

    arrow_cnt++;
    fprintf(fp, "set arrow %d from %f,%f,%f to %f,%f,%f as 2\n", arrow_cnt, xp[0], xp[1], xp[2], xp[0],xp[1]+scale*dx[1],xp[2]);
    fflush(fp);

    arrow_cnt++;
    fprintf(fp, "set arrow %d from %f,%f,%f to %f,%f,%f as 3\n", arrow_cnt, xp[0], xp[1], xp[2], xp[0],xp[1],xp[2]+scale*dx[2]);
    fflush(fp);

    if(replot_flag == false)
    {
        fprintf(fp, "splot '<echo %f %f %f' u 1:2:3 w p  notitle \n", xp[0], xp[1], xp[2]);
        fflush(fp);
    }
    else
    {
        fprintf(fp, "replot\n");
        fflush(fp);
    }

    //fprintf(fp, "}else{print 'SPLOT does not exist. Coordinates axes are NOT drawn.'}\n");
    //fflush(fp);

    cout << "arrow_cnt = " << arrow_cnt << endl;
    cout << "call 'get_arrow_cnt()' to get the current arrow count" << endl;
}
//-----------------------------
// Overloading function draw3dcoordAxis
// In this origin is provided in x[]
// x-axis, y-axis and z-axis points are provided in xp, yp and zp
// respectively
//==================================================
void GP_handle::draw3dcoordAxis(double x[], double xp[], double yp[], double zp[], bool replot_flag, int cFlag)
{

    if(cFlag == 1)
    {
        //arrow styles
        fprintf(fp,"set style arrow 1 head filled size 0.03,10,0 lt 1 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 2 head filled size 0.03,10,0 lt 2 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 3 head filled size 0.03,10,0 lt 3 lw 2\n");
        fflush(fp);
    }
    else if(cFlag == 2)
    {
        //arrow styles
        fprintf(fp,"set style arrow 1 head filled size 0.03,10,0 lt 4 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 2 head filled size 0.03,10,0 lt 5 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 3 head filled size 0.03,10,0 lt 6 lw 2\n");
        fflush(fp);
    }
    else
    {
        //arrow styles
        fprintf(fp,"set style arrow 1 head filled size 0.03,10,0 lt 7 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 2 head filled size 0.03,10,0 lt 8 lw 2\n");
        fflush(fp);
        fprintf(fp,"set style arrow 3 head filled size 0.03,10,0 lt 9 lw 2\n");
        fflush(fp);
    }

    arrow_cnt++;
    fprintf(fp, "set arrow %d from %f,%f,%f to %f, %f,%f as 1\n", arrow_cnt,
            x[0], x[1], x[2], xp[0], xp[1], xp[2]);
    fflush(fp);

    arrow_cnt++;
    fprintf(fp, "set arrow %d from %f,%f,%f to %f,%f,%f as 2\n", arrow_cnt,
            x[0], x[1], x[2], yp[0], yp[1], yp[2]);
    fflush(fp);

    arrow_cnt++;
    fprintf(fp, "set arrow %d from %f,%f,%f to %f,%f,%f as 3\n", arrow_cnt,
            x[0],x[1],x[2], zp[0], zp[1], zp[2]);
    fflush(fp);

    if(replot_flag == false)
    {
        fprintf(fp, "splot '<echo %f %f %f' u 1:2:3 w p  notitle \n", x[0], x[1], x[2]);
        fflush(fp);
    }
    else
    {
        fprintf(fp, "replot\n");
        fflush(fp);
    }

    //close the if loop
    //fprintf(fp, "}else{print 'SPLOT does not exist. Coordinates axes are NOT drawn.'}\n");
    //fflush(fp);

    cout << "arrow_cnt = " << arrow_cnt << endl;
    cout << "call 'get_arrow_cnt()' to get the current arrow count" << endl;
}
//-------------------------------------
// Return the current arrow cnt
int GP_handle::get_arrow_cnt() const
{
    return arrow_cnt;
}


