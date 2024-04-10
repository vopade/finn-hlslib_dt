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
#include "data/add_config.h"
#include "activations.hpp"
#include "interpret.hpp"

using namespace hls;
using namespace std;

void Testbench_add(stream<hls::vector<TI, NUM_CHANNELS>> &in1, stream<hls::vector<TI, NUM_CHANNELS>> &in2,
	stream<hls::vector<TO, NUM_CHANNELS>> &out, unsigned int numReps);

int main()
{
  hls::vector<TI, NUM_CHANNELS> input;
  hls::vector<TO, NUM_CHANNELS> output;
	stream<hls::vector<TI, NUM_CHANNELS>> input_stream1("input_stream1");
	stream<hls::vector<TI, NUM_CHANNELS>> input_stream2("input_stream2");
	stream<hls::vector<TO, NUM_CHANNELS>> output_stream("output_stream");
	static hls::vector<TO, NUM_CHANNELS> expected[NUM_REPEAT*NUM_WORDS];
	unsigned int count_out = 0;
	unsigned int count_in = 0;
	for (unsigned int i = 0; i < NUM_REPEAT*NUM_WORDS; i++) {
    for (int j = 0; j < NUM_CHANNELS; j++) {
      TI value = TI (j*i+j);
      input[j] = value;
      expected[i][j] = value + value + OFFSET;
    }
    input_stream1.write(input);
    input_stream2.write(input);
  }

  Testbench_add(input_stream1, input_stream2, output_stream, NUM_REPEAT);

  for (unsigned int i = 0; i < NUM_REPEAT*NUM_WORDS; i++) {
    hls::vector<TO, NUM_CHANNELS> valOut = output_stream.read();
    for (unsigned int j = 0; j < NUM_CHANNELS; j++) {
      if(valOut[j] != expected[i][j]) {
        cout << "ERROR at sample " << i << std::hex << " expected: " << expected[i][j] << ", valOut: "
           << valOut[j] << std::dec << endl;
        //return(1);
      }
    }
	}
}


