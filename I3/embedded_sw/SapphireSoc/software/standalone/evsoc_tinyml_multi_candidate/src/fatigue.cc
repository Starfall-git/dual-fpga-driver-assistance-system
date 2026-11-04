#include "fatigue.h"
#include <math.h>
namespace fatigue {
/* canonical MediaPipe mesh eye pairs. Thresholds are exposed for demo calibration. */
static const int L[6]={362,385,387,263,373,380};
static const int R[6]={33,160,158,133,153,144};
static bool valid(Point p) { return isfinite(p.x)&&isfinite(p.y)&&p.x>=0&&p.x<=1&&p.y>=0&&p.y<=1; }
static float distance(Point a,Point b) { float x=a.x-b.x,y=a.y-b.y;return sqrtf(x*x+y*y); }
float eye_ratio(const Point* p,const int ids[6]) {
    for(int i=0;i<6;++i) if(!valid(p[ids[i]])) return -1;
    float w=distance(p[ids[0]],p[ids[3]]);
    if(w<0.015f||w>0.4f) return -1;
    float ear=(distance(p[ids[1]],p[ids[5]])+distance(p[ids[2]],p[ids[4]]))/(2*w);
    return ear<=0.8f ? ear : -1;
}
Result Detector::update(const Point* p,int n,float confidence,uint32_t ms,bool ok,int eye_decision) {
    Result out;
    bool gap=have_time_ && uint32_t(ms-last_)>config.max_gap_ms;
    last_=ms;have_time_=true;
    if(gap) {closing_=opening_=alarm_=false;}
    if(!ok || !isfinite(confidence)) out.state=UNCERTAIN;
    else if(confidence<config.confidence) out.state=NO_FACE;
    else if(!p||n<468) out.state=UNCERTAIN;
    else {
        out.left_ear=eye_ratio(p,L);out.right_ear=eye_ratio(p,R);
        out.face_valid=out.left_ear>=0 && out.right_ear>=0;
        out.state=out.face_valid?AWAKE:UNCERTAIN;
    }
    if(!out.face_valid) {closing_=opening_=alarm_=false;return out;}
    /* No accumulation across a wink, missing face, invalid sample or long capture gap.
       Between thresholds only a previously confirmed CLOSED state is retained. */
    bool closed=(out.left_ear<config.close_ear && out.right_ear<config.close_ear) ||
       (closing_ && out.left_ear<config.open_ear && out.right_ear<config.open_ear);
    // Both eyes must be measurable; tolerate a small left/right landmark bias.
    // Large asymmetry stays uncertain instead of clearing a warning on a wink.
    float min_open=config.close_ear-0.020f;
    bool open=(out.left_ear+out.right_ear)>=2*config.open_ear &&
              out.left_ear>=min_open && out.right_ear>=min_open;
    if (eye_decision>=0) { closed=(eye_decision==1); open=(eye_decision==0); }
    if(closed) {
        opening_=false;
        if(!closing_) {closed_since_=ms;closing_=true;}
        out.closed_ms=uint32_t(ms-closed_since_);
        if(out.closed_ms>=config.risk_ms) alarm_=true;
        out.state=alarm_?RISK:CLOSED;
    } else if(open) {
        closing_=false;
        if(!opening_) {opening_=true;opened_since_=ms;}
        if(uint32_t(ms-opened_since_)>=config.recover_ms) alarm_=false;
        out.state=alarm_?RISK:AWAKE;
    } else {
        closing_=opening_=false;
        out.state=alarm_?RISK:UNCERTAIN;
    }
    if(gap) out.state=STALE;
    return out;
}
const char* state_name(State s) {
    switch(s) {
    case AWAKE:return "AWAKE";case CLOSED:return "EYES CLOSED";case RISK:return "FATIGUE RISK";
    case NO_FACE:return "NO FACE";case UNCERTAIN:return "UNCERTAIN";case STALE:return "STALE";
    default:return "STARTING";
    }
}
}
