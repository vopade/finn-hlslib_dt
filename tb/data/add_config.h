// placeholder, until parameters are passed from TCL
#define TI_ float
#define TO_ float
#define NUM_CHANNELS_ 32
#define NUM_WORDS_ 5
#define NUM_REPEAT_ 1
#define OFFSET_ 2

using TI = TI_;
//using TO = ap_ufixed<OUTPUT_WIDTH, OUTPUT_WIDTH, AP_TRN>;
using TO = TO_;
constexpr unsigned NUM_CHANNELS = NUM_CHANNELS_;
constexpr unsigned NUM_WORDS = NUM_WORDS_;
constexpr unsigned NUM_REPEAT = NUM_REPEAT_;
constexpr unsigned OFFSET = OFFSET_;
