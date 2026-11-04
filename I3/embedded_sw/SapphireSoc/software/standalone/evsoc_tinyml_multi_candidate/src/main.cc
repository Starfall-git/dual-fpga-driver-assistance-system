////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013-2026 Efinix Inc. All rights reserved.
// See https://github.com/Efinix-Inc/tinyml/blob/main/LICENSE.txt for details.
////////////////////////////////////////////////////////////////////////////////
/* VF board uses hardware SC130GS initialization and continuous HDMI.
   CPU reads an immutable RGB snapshot through APB; no PiCam/DSI/DMASG accesses.
   Original face model and TinyML operator implementation are retained. */
#include <stdint.h>
#include <math.h>
#include "riscv.h"
#include "soc.h"
#include "bsp.h"
#include "clint.h"
#include "intc.h"
#include "tinyml.h"
#include "ops/ops_api.h"
#include "model/arena.h"
#include "fatigue.h"
#include "uart.h"
#include "fatigue_control.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/debug_log.h"
#include "platform/tinyml/profiler.h"
#include "model/mediapipe_face_landmark_model_data.h"
/* Experimental network-video ROI. These values must match the endpoint in
   cnn_soc_subsystem.v; the matching FPGA bit and ELF must be loaded together. */
extern const unsigned char eye_classifier_data[];
#define FACE_ROI_X 552u
#define FACE_ROI_Y 0u
#define FACE_ROI_EXTENT 767u
#define FACE_LANDMARK_INPUT_BYTES (192*192*3)
#define NUM_LANDMARK 468
namespace {
   tflite::ErrorReporter* error_reporter = nullptr;
   const tflite::Model* model = nullptr;
   tflite::MicroInterpreter* interpreter = nullptr;
   TfLiteTensor* model_input = nullptr;
   
   //Create an area of memory to use for input, output, and other TensorFlow
   //arrays. You'll need to adjust this by combiling, running, and looking
   //for errors.
   constexpr int kTensorArenaSize = 10000 * 1024;
   uint8_t tensor_arena[kTensorArenaSize];
   int total_output_layers = 0;
   constexpr int kEyeArenaSize = 2 * 1024 * 1024;
   alignas(16) uint8_t eye_arena[kEyeArenaSize];
   tflite::MicroInterpreter* eye_interpreter = nullptr;
   TfLiteTensor* eye_input = nullptr;

}

void tinyml_init() {
   //Set up logging
   static tflite::MicroErrorReporter micro_error_reporter;
   error_reporter = &micro_error_reporter;
   
   //Map the model into a usable data structure
   model = tflite::GetModel(mediapipe_face_landmark_model_data);
   
   if (model->version() != TFLITE_SCHEMA_VERSION) {
      MicroPrintf("Model version does not match Schema\n\r");
      while(1);
   }
   
   
   ////AllOpsResolver may be used for generalization
   static tflite::AllOpsResolver resolver;
   tflite::MicroOpResolver* op_resolver = nullptr;
   op_resolver = &resolver;
   
   //Build an interpreter to run the model
   
   static FullProfiler prof;
   static tflite::MicroInterpreter static_interpreter(
      model, *op_resolver, tensor_arena, kTensorArenaSize,
      error_reporter, nullptr); //Without profiler
      //error_reporter, &prof); //With profiler
   interpreter = &static_interpreter;
   prof.setInterpreter(interpreter);
   prof.setDump(false);
   
   //Allocate memory from the tensor_arena for the model's tensors
   TfLiteStatus allocate_status = interpreter->AllocateTensors();
   if (allocate_status != kTfLiteOk) {
      MicroPrintf("AllocateTensors() failed\n\r");
      while(1);
   }
   
   const tflite::Model* eye_model=tflite::GetModel(eye_classifier_data);
   if(eye_model->version()!=TFLITE_SCHEMA_VERSION)while(1);
   static tflite::MicroInterpreter eye_static(eye_model,*op_resolver,eye_arena,kEyeArenaSize,error_reporter,nullptr);
   eye_interpreter=&eye_static;
   if(eye_interpreter->AllocateTensors()!=kTfLiteOk)while(1);
   eye_input=eye_interpreter->input(0);
   if(eye_input->type!=kTfLiteInt8 || eye_input->bytes!=24*24 || eye_interpreter->outputs_size()!=1 || eye_interpreter->output(0)->bytes!=2 || eye_interpreter->output(0)->type!=kTfLiteInt8)while(1);
   //Assign model input buffer (tensor) to pointer
   model_input = interpreter->input(0);
   
   //Print loaded model input and output shape
   total_output_layers = interpreter->outputs_size();
   MicroPrintf("\n\rTotal output layers: %d\n\r", total_output_layers);
   MicroPrintf(
      "Input shape: %d %d %d %d %d, type: %d\n", model_input->dims->size, model_input->dims->data[0], model_input->dims->data[1], model_input->dims->data[2], model_input->dims->data[3], model_input->type
   );
   
   /* validate tensor contract without reading beyond tensor dimensions. */
   if (model_input->type != kTfLiteInt8 || model_input->bytes != FACE_LANDMARK_INPUT_BYTES ||
       total_output_layers != 2 || interpreter->output(0)->type != kTfLiteInt8 ||
       interpreter->output(0)->bytes != NUM_LANDMARK*3 ||
       interpreter->output(1)->type != kTfLiteInt8 || interpreter->output(1)->bytes != 1) {
      MicroPrintf("Unsupported face model tensor contract\n");
      while(1);
   }

}

static volatile uint32_t* const regs=(volatile uint32_t*)IO_APB_SLAVE_1_INPUT;
static uint32_t read_reg(unsigned offset) { return regs[offset/4]; }
static void write_reg(unsigned offset,uint32_t v) { regs[offset/4]=v;asm volatile("fence rw,rw" ::: "memory"); }
static uint32_t ms_now() {return (uint32_t)(clint_getTime(BSP_CLINT)/(SYSTEM_CLINT_HZ/1000));}
/* JTAG-visible diagnostics; published only after a complete inference result. */
struct Debug {uint32_t magic,sequence,state,closed_ms,left_ear,right_ear,inference_ms,capture_errors,invoke_errors,input_checksum;};
volatile Debug fatigue_debug={0x46415431,0,0,0,0,0,0,0,0,0};
static fatigue::Point points[468];
static fatigue::Detector detector;
// Debounce ordinary one-sample changes; never delay risk, no-face or stale.
static fatigue::State stable_state=fatigue::STARTING,pending_state=fatigue::STARTING;
static unsigned pending_count=0;
static fatigue::State smooth_state(fatigue::State next,bool strong_open) {
    if((next==fatigue::AWAKE&&strong_open)||next==fatigue::RISK||next==fatigue::NO_FACE||next==fatigue::STALE||stable_state==fatigue::STARTING||stable_state==fatigue::NO_FACE||stable_state==fatigue::STALE||(stable_state==fatigue::RISK&&next==fatigue::AWAKE)) {
        stable_state=next;pending_count=0;return next;
    }
    if(next==stable_state){pending_count=0;return stable_state;}
    if(next!=pending_state){pending_state=next;pending_count=1;return stable_state;}
    if(++pending_count>=2){stable_state=next;pending_count=0;}
    return stable_state;
}
static FatigueControl control_parser;
static bool fatigue_enabled=true;
// Display controls: face box, behavior, warning, feature points, part boxes.
static uint8_t display_mask=0x07;
struct BehaviorState {bool yawn=false,head=false;uint32_t mouth_since=0,head_since=0,last_yawn=0;unsigned yawns=0;bool mouth_pending=false,head_pending=false;};
static BehaviorState behavior;
static float point_distance(const fatigue::Point& a,const fatigue::Point& b) {float x=a.x-b.x,y=a.y-b.y;return sqrtf(x*x+y*y);}
static void update_behavior(const fatigue::Point* p,uint32_t now,bool valid) {
 if(!valid) {behavior=BehaviorState();return;}
 float width=point_distance(p[78],p[308]);
 float mouth=width>.01f?point_distance(p[13],p[14])/width:0;
 float ey=(p[33].y+p[263].y)*.5f,denom=p[152].y-ey;
 float head=denom>.02f?(p[1].y-ey)/denom:0;
 bool mo=mouth>(behavior.yawn?.16f:.22f),hd=head>(behavior.head?.40f:.45f);
 if(!mo){behavior.yawn=false;behavior.mouth_pending=false;}else {if(!behavior.mouth_pending){behavior.mouth_pending=true;behavior.mouth_since=now;} if(now-behavior.mouth_since>=900&&!behavior.yawn){behavior.yawn=true;if(behavior.yawns&&now-behavior.last_yawn<=60000)++behavior.yawns;else behavior.yawns=1;behavior.last_yawn=now;}}
 if(!hd){behavior.head=false;behavior.head_pending=false;}else {if(!behavior.head_pending){behavior.head_pending=true;behavior.head_since=now;} if(now-behavior.head_since>=900)behavior.head=true;}
}
static uint32_t point_pixel(float x,float y) {
 if(!isfinite(x)||!isfinite(y)||x<0||x>1||y<0||y>1)return 0;
 return ((FACE_ROI_Y+(uint32_t)(y*FACE_ROI_EXTENT))<<16)|(FACE_ROI_X+(uint32_t)(x*FACE_ROI_EXTENT));
}
static void poll_control() {
    while(uart_readOccupancy(SYSTEM_UART_0_IO_CTRL)) {
        if(!control_parser.feed((uint8_t)uart_read(SYSTEM_UART_0_IO_CTRL))) continue;
        uint8_t seq=control_parser.b[2],cmd=control_parser.b[3],value=control_parser.b[4];
        uint8_t applied=0xff;
        if(cmd==2 && value<=63){display_mask=value;applied=value;}
        if(cmd==1 && value<=1) { fatigue_enabled=(value!=0); applied=value; if(!fatigue_enabled){detector=fatigue::Detector();stable_state=fatigue::STARTING;pending_count=0;} }
        uint8_t reply[6]={0xc4,0x4c,seq,cmd,applied,0};reply[5]=FatigueControl::crc(reply+2);
        for(int i=0;i<6;++i)uart_write(SYSTEM_UART_0_IO_CTRL,reply[i]);
    }
}
static bool capture() {
    if(read_reg(4)&2)return false; // a timed-out capture must not issue a busy APB command
    write_reg(8,1);uint32_t start=ms_now();
    while(read_reg(4)&2) {poll_control();if(!fatigue_enabled||uint32_t(ms_now()-start)>10000)return false;}
    return (read_reg(4)&5)==1 && read_reg(0x1c)==192*192;
}
// Candidate classifier: estimate each square eye crop from face landmarks,
// read the SAME immutable captured frame, and classify on I3. No host inference.
static float eye_debug_open[2]={0,0};
static int classify_eyes(const fatigue::Point *pt) {
    const int corners[2][2]={{362,263},{33,133}};
    float open[2];
    for(int eye=0;eye<2;++eye) {
        float x0=pt[corners[eye][0]].x*192.f,y0=pt[corners[eye][0]].y*192.f;
        float x1=pt[corners[eye][1]].x*192.f,y1=pt[corners[eye][1]].y*192.f;
        float dx=x1-x0,dy=y1-y0,side=sqrtf(dx*dx+dy*dy)*1.3f;
        float cx=(x0+x1)*.5f,cy=(y0+y1)*.5f;
        if(!isfinite(side)||side<5||side>80||cx-side*.5f<0||cy-side*.5f<0||cx+side*.5f>=192||cy+side*.5f>=192)return 2;
        for(int j=0;j<24;++j)for(int i=0;i<24;++i) {
            int x=(int)(cx-side*.5f+(i+.5f)*side/24.f),y=(int)(cy-side*.5f+(j+.5f)*side/24.f);
            write_reg(0x0c,(uint32_t)(y*192+x));uint32_t rgb=read_reg(0x10);
            int gray=(77*(int)((rgb>>16)&255)+150*(int)((rgb>>8)&255)+29*(int)(rgb&255)+128)>>8;
            int q=(int)((gray/255.f)/eye_input->params.scale+eye_input->params.zero_point+.5f);
            if(q<-128)q=-128;if(q>127)q=127;eye_input->data.int8[j*24+i]=(int8_t)q;
        }
        if(eye_interpreter->Invoke()!=kTfLiteOk)return 2;
        const TfLiteTensor* o=eye_interpreter->output(0);
        open[eye]=1.f-(o->data.int8[1]-o->params.zero_point)*o->params.scale;
    }
    eye_debug_open[0]=open[0];eye_debug_open[1]=open[1];
    if(open[0]<.35f && open[1]<.35f)return 1;
    if(open[0]>.90f || open[1]>.90f || ((open[0]>.55f || open[1]>.55f) && open[0]>.20f && open[1]>.20f))return 0;
    return 2;
}
/* CRC8-framed state; host accepts only complete fresh results.
   Debug logs may share this UART and are ignored by the frame parser. */
static void send_fatigue_state(uint8_t seq, const fatigue::Result &r) {
    uint8_t frame[6]={0xd3,0x6c,seq,(uint8_t)r.state,(uint8_t)r.face_valid,0};
    uint8_t crc=0;
    for(int i=2;i<5;i++) {crc^=frame[i];for(int j=0;j<8;j++) crc=(crc&0x80)?(uint8_t)((crc<<1)^7):(uint8_t)(crc<<1);}
    frame[5]=crc;
    for(int i=0;i<6;i++) uart_write(SYSTEM_UART_0_IO_CTRL,frame[i]);
}
/* Separate diagnostic packet: not a control or fatigue-state packet. */
static void send_fatigue_diagnostic(uint8_t seq, const fatigue::Result &r, float confidence, uint32_t source_id, uint8_t behavior_flags) {
    int left=(int)(r.left_ear*1000),right=(int)(r.right_ear*1000);
    int conf=isfinite(confidence)?(int)(confidence*1000):0;
    if(conf<0)conf=0;if(conf>1000)conf=1000;
    uint8_t frame[16]={0xe2,0x52,seq,(uint8_t)r.state,
        (uint8_t)((uint16_t)left>>8),(uint8_t)left,
        (uint8_t)((uint16_t)right>>8),(uint8_t)right,
        (uint8_t)(conf>>8),(uint8_t)conf,(uint8_t)((r.face_valid?1:0)|behavior_flags),
        (uint8_t)(source_id>>24),(uint8_t)(source_id>>16),(uint8_t)(source_id>>8),(uint8_t)source_id,0};
    for(int i=2;i<15;++i)frame[15]^=frame[i];
    for(int i=0;i<16;++i)uart_write(SYSTEM_UART_0_IO_CTRL,frame[i]);
}
// Timings are CPU elapsed durations in milliseconds; not camera-to-screen latency.
static void send_performance(uint32_t capture,uint32_t prepare,uint32_t face,uint32_t eye,uint32_t publish) {
 uint32_t values[5]={capture,prepare,face,eye,publish};uint8_t pkt[13]={0xe3,0x53};
 for(int i=0;i<5;++i){uint32_t v=values[i]>65535?65535:values[i];pkt[2+i*2]=v>>8;pkt[3+i*2]=v;}
 for(int i=2;i<12;++i)pkt[12]^=pkt[i];for(int i=0;i<13;++i)uart_write(SYSTEM_UART_0_IO_CTRL,pkt[i]);
}
int main() {
    /* 100 MHz peripheral clock, 8x oversampling, approximately 115200 baud. */
    Uart_Config serial_config={BITS_8,NONE,ONE,(SYSTEM_CLINT_HZ/(115200u*8u))-1u};
    uart_applyConfig(SYSTEM_UART_0_IO_CTRL,&serial_config);
    uint32_t hart=csr_read(mhartid);
    arena[hart]=arena_create(500000);
    dma_init(); // enables only the custom-instruction accelerator IRQ in this BSP
    tinyml_init();init_accel(hart);print_accel(hart);
    if(read_reg(0)!=0x46415431 || read_reg(0x38)!=((192<<16)|4))while(1);
    uint32_t last_paused_message=0;
    uint32_t last_source_id=0,last_source_change_ms=0;
    bool have_source_id=false;
    for(;;) {
        poll_control();
        if(!fatigue_enabled) {
            uint32_t now=ms_now();
            if(uint32_t(now-last_paused_message)>=500) {fatigue::Result paused;paused.state=fatigue::STALE;send_fatigue_state((uint8_t)fatigue_debug.sequence,paused);last_paused_message=now;}
            continue;
        }
        uint32_t perf_begin=ms_now();
        if(!capture()) {++fatigue_debug.capture_errors;continue;}
        uint32_t perf_captured=ms_now();
        uint32_t stamp=read_reg(0x18),source_id=read_reg(0x3c),display_count=read_reg(0x40),checksum=0;
        if(!have_source_id || source_id!=last_source_id) {
            have_source_id=true;last_source_id=source_id;last_source_change_ms=stamp;
        }
        bool source_stale=uint32_t(stamp-last_source_change_ms)>2000u;
        for(unsigned i=0;i<192*192;++i) {
            write_reg(0x0c,i);
            uint32_t rgb=read_reg(0x10);
            model_input->data.int8[3*i]=(int)(rgb>>16 &255)-128;
            model_input->data.int8[3*i+1]=(int)(rgb>>8 &255)-128;
            model_input->data.int8[3*i+2]=(int)(rgb&255)-128;
            checksum+=(rgb>>16 &255)+(rgb>>8 &255)+(rgb&255);
        }
        uint32_t start=ms_now();TfLiteStatus status=interpreter->Invoke();
        uint32_t elapsed=ms_now()-start;float confidence=0;
        if(status==kTfLiteOk) {
            const TfLiteTensor* lm=interpreter->output(0);
            const TfLiteTensor* flag=interpreter->output(1);
            for(int i=0;i<468;++i) {
                points[i].x=(lm->data.int8[i*3]-lm->params.zero_point)*lm->params.scale/192;
                points[i].y=(lm->data.int8[i*3+1]-lm->params.zero_point)*lm->params.scale/192;
            }
            float logit=(flag->data.int8[0]-flag->params.zero_point)*flag->params.scale;
            confidence=1/(1+expf(-logit));
        } else ++fatigue_debug.invoke_errors;
        int eye_decision=2;
        if(status==kTfLiteOk&&!source_stale&&confidence>=0.5f)eye_decision=classify_eyes(points);
        uint32_t perf_eyes=ms_now();
        fatigue::Result result=detector.update(points,468,confidence,stamp,status==kTfLiteOk&&!source_stale,eye_decision);
        if(source_stale)result.state=fatigue::STALE;
        // Eye behavior is independent of the latched fatigue alarm: during
        // recovery the eyes may already be open while warning is still active.
        fatigue::State eye_visual=result.state;
        if(result.face_valid&&!source_stale)
            eye_visual=eye_decision==0?fatigue::AWAKE:eye_decision==1?fatigue::CLOSED:fatigue::UNCERTAIN;
        eye_visual=smooth_state(eye_visual,eye_decision==0&&eye_debug_open[0]>.95f&&eye_debug_open[1]>.95f);
        update_behavior(points,stamp,result.face_valid&&!source_stale);
        bool warning=result.state==fatigue::RISK || (behavior.head&&behavior.head_pending&&(stamp-behavior.head_since>=5000)) || (behavior.yawns>=2&&(stamp-behavior.last_yawn<=10000));
        float xmin=1,ymin=1,xmax=0,ymax=0;
        if(result.face_valid)for(int i=0;i<468;++i) {
            float x=points[i].x,y=points[i].y;
            if(!isfinite(x)||!isfinite(y)||x<0||x>1||y<0||y>1)continue;
            if(x<xmin)xmin=x;if(x>xmax)xmax=x;
            if(y<ymin)ymin=y;if(y>ymax)ymax=y;
        }
        uint32_t b0=0,b1=0;
        if(result.face_valid&&xmax>xmin&&ymax>ymin) {
            b0=((FACE_ROI_Y+(uint32_t)(ymin*FACE_ROI_EXTENT))<<16)|(FACE_ROI_X+(uint32_t)(xmin*FACE_ROI_EXTENT));
            b1=((FACE_ROI_Y+(uint32_t)(ymax*FACE_ROI_EXTENT))<<16)|(FACE_ROI_X+(uint32_t)(xmax*FACE_ROI_EXTENT));
        }
        while(read_reg(4)&8); // previous result must reach a display frame boundary
        uint32_t visual=(uint32_t)eye_visual|(result.face_valid?256u:0u)|(behavior.yawn?512u:0u)|(behavior.head?1024u:0u)|(warning?2048u:0u)|((uint32_t)display_mask<<16);
        write_reg(0x20,visual);
        write_reg(0x44,point_pixel((points[33].x+points[133].x)*.5f,(points[33].y+points[133].y)*.5f));
        write_reg(0x48,point_pixel((points[362].x+points[263].x)*.5f,(points[362].y+points[263].y)*.5f));
        write_reg(0x4c,point_pixel(points[1].x,points[1].y));
        write_reg(0x50,point_pixel((points[13].x+points[14].x)*.5f,(points[13].y+points[14].y)*.5f));
        write_reg(0x54,point_pixel(points[152].x,points[152].y));
        write_reg(0x58,point_pixel(points[33].x,points[33].y));
        write_reg(0x5c,point_pixel(points[133].x,points[133].y));
        write_reg(0x60,point_pixel(points[362].x,points[362].y));
        write_reg(0x64,point_pixel(points[263].x,points[263].y));
        write_reg(0x68,point_pixel(points[159].x,points[159].y));
        write_reg(0x6c,point_pixel(points[145].x,points[145].y));
        write_reg(0x70,point_pixel(points[386].x,points[386].y));
        write_reg(0x74,point_pixel(points[374].x,points[374].y));
        write_reg(0x78,point_pixel(points[78].x,points[78].y));
        write_reg(0x7c,point_pixel(points[308].x,points[308].y));

        write_reg(0x24,result.closed_ms);
        uint32_t left=result.left_ear>0?(uint32_t)(result.left_ear*1000):0;
        uint32_t right=result.right_ear>0?(uint32_t)(result.right_ear*1000):0;
        write_reg(0x28,(right<<16)|left);write_reg(0x2c,b0);write_reg(0x30,b1);write_reg(0x34,1);
        fatigue_debug.sequence=read_reg(0x14);fatigue_debug.state=result.state;
        fatigue_debug.closed_ms=result.closed_ms;fatigue_debug.left_ear=left;fatigue_debug.right_ear=right;
        fatigue_debug.inference_ms=elapsed;fatigue_debug.input_checksum=checksum;
        fatigue::Result relay_result=result;if(warning&&result.face_valid)relay_result.state=fatigue::RISK;
        send_fatigue_state((uint8_t)fatigue_debug.sequence,relay_result);
        fatigue::Result diagnostic=result;diagnostic.left_ear=eye_debug_open[0];diagnostic.right_ear=eye_debug_open[1];
        send_fatigue_diagnostic((uint8_t)display_count,diagnostic,confidence,source_id,(behavior.yawn?2:0)|(behavior.head?4:0)|(warning?8:0)|(eye_visual==fatigue::AWAKE?16:eye_visual==fatigue::CLOSED?32:64));
        send_performance(perf_captured-perf_begin,start-perf_captured,elapsed,perf_eyes-start-elapsed,ms_now()-perf_eyes);
        arena_clear(arena[hart]);
    }
}


