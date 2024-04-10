/******************************************************************************
 *
 *  Authors:  erling on 5/10/21.
 *  			Giulio Gambardella <giuliog@xilinx.com>
 *  			Jonas Kuehle <jonas.kuehle@cs.hs-fulda.de>
 *
 *  \file upsample_top.cpp
 *  
 *  Testbench function for unit testing of the Upsample with Nearest Neighbour 
 *  
 *****************************************************************************/

#include <iostream>
#include <hls_stream.h>
#include "ap_int.h"
#include "data/upsample_config.h"
#include <hls_vector.h>

using namespace hls;
using namespace std;

void Testbench_upsample(stream<hls::vector<T,FM_CHANNELS>> &in, stream<hls::vector<T,FM_CHANNELS>> &out);

void Golden_upsample(T in[IFMDIM][IFMDIM][FM_CHANNELS], T out[OFMDIM][OFMDIM][FM_CHANNELS]);

int main(){
  static T golden_in[IFMDIM][IFMDIM][FM_CHANNELS];
  static T golden_out[OFMDIM][OFMDIM][FM_CHANNELS];

  stream<hls::vector<T,FM_CHANNELS>> test_in("test_input");
  stream<hls::vector<T,FM_CHANNELS>> test_out("test_ouput");

  for (int i = 0; i<IFMDIM; i++) {
    for (int j = 0; j<IFMDIM; j++) {
      hls::vector<T,FM_CHANNELS> input_channel;
      for (int k = 0; k<FM_CHANNELS; k++) {
        T input = i*IFMDIM + j;
        input_channel[k] = input;
        golden_in[i][j][k] = input;
      }
      test_in.write(input_channel);
    }
  }

  Golden_upsample(golden_in, golden_out);
  Testbench_upsample(test_in, test_out);

  T out_channel;
  int err_counter = 0;
  for (int i = 0; i<OFMDIM; i++) {
    for (int j = 0; j<OFMDIM; j++) {
      hls::vector<T,FM_CHANNELS> out_elem = test_out.read();
      for (int k = 0; k<FM_CHANNELS; k++) {
        T expect = golden_out[i][j][k];
        out_channel = out_elem[k];
        if (expect != out_channel) {
          cout << "ERROR: Expected["<<i<<"]["<<j<<"]["<<k<<"]=" <<expect <<" actual " <<out_channel <<endl;
          err_counter++;
        }
      }
    }
  }
  return err_counter;
}





void Golden_upsample(T in[IFMDIM][IFMDIM][FM_CHANNELS], T out[OFMDIM][OFMDIM][FM_CHANNELS]) {
  const int scaling = OFMDIM / IFMDIM;
  const int padding = OFMDIM % IFMDIM;
  for (int i = 0; i<OFMDIM; i++) {
    for (int j = 0; j<OFMDIM; j++) {

      int dst_i = i-padding;
      if (dst_i<0) dst_i = 0;
      int dst_j = j - padding;
      if (dst_j<0) dst_j = 0;

      int src_i = dst_i/scaling;
      int src_j = dst_j/scaling;
      for (int k = 0; k<FM_CHANNELS; k++) {
        out[i][j][k] = in[src_i][src_j][k];
      }
	  //std::cout << out[i][j][0] << " " ;
    }
	//std::cout << std::endl;
  }
}
