#include "fatigue.h"
#include "fatigue_font.h"
#include <math.h>
#include <stdio.h>
namespace fatigue {
/* framebuffer is 0x00BBGGRR, exactly as in the upstream display color bar.
   Write only after inference and before handing the completed buffer to display. */
static void pixel(volatile uint32_t* p,int w,int h,int x,int y,uint32_t c) {
    if(x>=0&&x<w&&y>=0&&y<h) p[y*w+x]=c;
}
static void box(volatile uint32_t* p,int w,int h,int x0,int y0,int x1,int y1,uint32_t c) {
    for(int x=x0;x<=x1;++x) {pixel(p,w,h,x,y0,c);pixel(p,w,h,x,y1,c);}
    for(int y=y0;y<=y1;++y) {pixel(p,w,h,x0,y,c);pixel(p,w,h,x1,y,c);}
}
static void text(volatile uint32_t* p,int w,int h,int x,int y,const char* s,uint32_t c,int scale=2) {
    for(;*s;++s,x+=8*scale) {
        unsigned ch=(unsigned char)*s;
        if(ch<32||ch>90) continue;
        for(int row=0;row<12;++row) for(int col=0;col<8;++col)
            if(FATIGUE_FONT[ch-32][row] & (1<<col))
                for(int dy=0;dy<scale;++dy) for(int dx=0;dx<scale;++dx)
                    pixel(p,w,h,x+col*scale+dx,y+row*scale+dy,c);
    }
}
void render(volatile uint32_t* p,int w,int h,const Point* pts,int n,const Result& r) {
    if(!p||w<1||h<1) return;
    uint32_t color=r.state==RISK?0x000000ff:r.state==AWAKE?0x0000ff00:0x0000ffff;
    if(r.face_valid && r.state!=STALE && pts && n>=468) {
        float xmin=1,ymin=1,xmax=0,ymax=0;
        for(int i=0;i<468;++i) {
            Point q=pts[i];
            if(!isfinite(q.x)||!isfinite(q.y)||q.x<0||q.x>1||q.y<0||q.y>1) continue;
            if(q.x<xmin) xmin=q.x;
            if(q.x>xmax) xmax=q.x;
            if(q.y<ymin) ymin=q.y;
            if(q.y>ymax) ymax=q.y;
        }
        if(xmax>xmin&&ymax>ymin) box(p,w,h,int(xmin*(w-1)),int(ymin*(h-1)),int(xmax*(w-1)),int(ymax*(h-1)),color);
        const int eyes[]={362,385,387,263,373,380,33,160,158,133,153,144};
        for(int i=0;i<12;++i) {
            Point q=pts[eyes[i]];
            if(!isfinite(q.x)||!isfinite(q.y)||q.x<0||q.x>1||q.y<0||q.y>1) continue;
            int x=int(q.x*(w-1)),y=int(q.y*(h-1));
            box(p,w,h,x-1,y-1,x+1,y+1,color);
        }
    }
    /* Opaque status panel deliberately covers its own rectangle, not the face interior. */
    int panel_w=w<420?w:420, panel_h=h<112?h:112;
    for(int y=0;y<panel_h;++y) for(int x=0;x<panel_w;++x) p[y*w+x]=0x00181818;
    box(p,w,h,0,0,panel_w-1,panel_h-1,color);
    text(p,w,h,8,4,"FATIGUE DEMO",0x00ffffff);
    text(p,w,h,8,30,state_name(r.state),color);
    char line[48];
    snprintf(line,sizeof(line),"CLOSED %lu MS",(unsigned long)r.closed_ms);
    text(p,w,h,8,58,line,0x00ffffff,1);
    int l=r.left_ear>=0&&r.left_ear<=1?int(r.left_ear*1000):0;
    int rr=r.right_ear>=0&&r.right_ear<=1?int(r.right_ear*1000):0;
    snprintf(line,sizeof(line),"EAR L %03d R %03d / 1000",l,rr);
    text(p,w,h,8,74,line,0x00ffffff,1);
    text(p,w,h,8,92,"SINGLE FACE - KEEP EYES VISIBLE",0x00bbbbbb,1);
}
}
