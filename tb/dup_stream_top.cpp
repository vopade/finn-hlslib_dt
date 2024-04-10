#include <hls_stream.h>
using namespace hls;
#include "ap_int.h"
#include "bnn-library.h"
#include "data/dup_stream_config.h"

void Testbench_dup_stream(stream<T> & in, stream<T> & out1, stream<T> & out2, unsigned int numReps){
	DuplicateStreams_Batch<T, NUM_REPEAT>(in, out1, out2, numReps);
}
