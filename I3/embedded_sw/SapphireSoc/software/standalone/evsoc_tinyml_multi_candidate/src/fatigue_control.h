#ifndef FATIGUE_CONTROL_H
#define FATIGUE_CONTROL_H
#include <stdint.h>
// C3 3C seq command value crc8(seq,command,value); reply C4 4C ... .
struct FatigueControl {
 uint8_t n,b[6];
 FatigueControl():n(0),b{0,0,0,0,0,0}{}
 static uint8_t crc(const uint8_t *p){uint8_t c=0;for(int i=0;i<3;++i){c^=p[i];for(int j=0;j<8;++j)c=(c&128)?(uint8_t)((c<<1)^7):(uint8_t)(c<<1);}return c;}
 bool feed(uint8_t x){
  if(n==0){if(x==0xc3){b[n++]=x;}return false;}
  if(n==1 && x!=0x3c){n=0;if(x==0xc3)b[n++]=x;return false;}
  b[n++]=x;if(n<6)return false;n=0;return b[5]==crc(b+2);
 }
};
#endif
