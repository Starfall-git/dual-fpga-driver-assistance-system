#ifndef FATIGUE_DEMO_H
#define FATIGUE_DEMO_H
#include <stdint.h>
/* hardware-independent single-face demonstration, not a medical classifier.
   Coordinates are normalized to a SQUARE RGB model input. Both eyes must be measurable. */
namespace fatigue {
struct Point { float x,y; };
enum State { STARTING, AWAKE, CLOSED, RISK, NO_FACE, UNCERTAIN, STALE };
struct Config {
    float confidence, close_ear, open_ear;
    uint32_t risk_ms, recover_ms, max_gap_ms;
    // Calibrated only for the provided recording and RGB332/768-pixel ROI.
    // Validate again for other people, camera views, or image pipelines.
    Config(): confidence(0.5f), close_ear(0.235f), open_ear(0.24f),
              risk_ms(1500), recover_ms(300), max_gap_ms(5000) {}
};
struct Result {
    State state;
    float left_ear, right_ear;
    uint32_t closed_ms;
    bool face_valid;
    Result():state(STARTING),left_ear(0),right_ear(0),closed_ms(0),face_valid(false){}
};
class Detector {
    bool have_time_, closing_, opening_, alarm_;
    uint32_t last_, closed_since_, opened_since_;
public:
    Config config;
    Detector():have_time_(false),closing_(false),opening_(false),alarm_(false),
               last_(0),closed_since_(0),opened_since_(0){}
    Result update(const Point* p, int count, float confidence, uint32_t ms, bool inference_ok=true, int eye_decision=-1);
};
float eye_ratio(const Point* p, const int indices[6]);
const char* state_name(State s);
void render(volatile uint32_t* pixels, int width, int height,
            const Point* landmarks, int count, const Result& result);
}
#endif

