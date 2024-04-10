#include <iostream>
#include <cmath>
#include <ctime>
#include <cstring>
#include <hls_stream.h>
#include <cstdlib>
#define AP_INT_MAX_W 8191
#include "ap_int.h"
#include "weights.hpp"
#include "bnn-library.h"
#include "activations.hpp"
#include "interpret.hpp"
#include "data/stream_limiter_config.h"

using namespace hls;
using namespace std;

void Testbench_streamLimiter(stream<hls::vector<T, NUM_TOTAL>> &in, stream<hls::vector<T, NUM_ALLOWED>> &out, unsigned numReps);

int main() {
  hls::stream<hls::vector<T, NUM_TOTAL>> input_stream("input_stream");
  hls::stream<hls::vector<T, NUM_ALLOWED>> output_stream("output_stream");

  hls::vector<T, NUM_TOTAL> input;
  hls::vector<T, NUM_ALLOWED> output;
  for(int i = 0; i < NUM_TOTAL; i++) {
      input[i] = i;
  }
  input_stream.write(input);
  Testbench_streamLimiter(input_stream, output_stream, numReps);

  output = output_stream.read();
  for(int i = 0; i < NUM_ALLOWED; i++) {
    if(input[i] != output[i]) {
      std::cout << "ERRoR!" << std::endl;
      return(1);
    }
  }
}


