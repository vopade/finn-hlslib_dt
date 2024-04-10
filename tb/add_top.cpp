#include <hls_stream.h>
using namespace hls;
#include "ap_int.h"
#include "bnn-library.h"
#include "data/add_config.h"

void Testbench_add(stream<hls::vector<TI, NUM_CHANNELS>> &in1, stream<hls::vector<TI, NUM_CHANNELS>> &in2, stream<hls::vector<TO, NUM_CHANNELS>> &out, unsigned int numReps) {
  AddStreams_Batch<NUM_CHANNELS, TI, TI, TO, NUM_WORDS, OFFSET>(in1, in2, out, numReps);
}