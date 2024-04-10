#include <hls_stream.h>
using namespace hls;
#include "ap_int.h"
#include "bnn-library.h"
#include "data/stream_limiter_config.h"

void Testbench_streamLimiter(stream<hls::vector<T, NUM_TOTAL>> &in, stream<hls::vector<T, NUM_ALLOWED>> &out, unsigned numReps) {
  StreamLimiter_Batch<T, NUM_ALLOWED, NUM_TOTAL>(in, out, numReps);
}