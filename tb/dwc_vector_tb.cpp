/******************************************************************************
 *  Copyright (c) 2019, Xilinx, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.  Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2.  Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *  3.  Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 *  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/
/******************************************************************************
 *
 *  Authors: Jonas Kuhle <jonkuhle@amd.com>
 *
 *  \file dwc_vector_tb.cpp
 *
 *  Testbench for the data-width converter HLS block using vectors
 *
 *****************************************************************************/
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstring>
#include <hls_stream.h>
#include <hls_vector.h>
#include <cstdlib>
#define AP_INT_MAX_W 8191
#include "ap_int.h"
#include "weights.hpp"
#include "bnn-library.h"
#include "activations.hpp"
#include "interpret.hpp"

using namespace hls;
using namespace std;

constexpr unsigned MAX_IMAGES = 1;
using T = ap_uint<8>; 
constexpr unsigned NUM_WORDS_IN_TMP = 20;
//constexpr unsigned NI = 8; // 8,8,4
//constexpr unsigned NO = 8; // 8,4,8

void Testbench_dwc_vector(hls::stream<hls::vector<T, NI_>>& in ,hls::stream<hls::vector<T, NO_>>& out, unsigned int numReps){
	StreamingDataWidthConverterVector_Batch	<NUM_WORDS_IN_TMP, T, NI_, NO_> (in, out, numReps);
}

int main()
{
	static_assert((NI_ % NO_ == 0) || (NO_ % NI_ == 0), "");
	hls::stream<hls::vector<T, NI_>> input_stream;
	hls::stream<hls::vector<T, NO_>> output_stream;
	
	constexpr unsigned numElementsOut = NI_*NUM_WORDS_IN_TMP;
	unsigned numVectorsOut = 0;
	if (NI_ >= NO_){
		numVectorsOut = (NUM_WORDS_IN_TMP)*NI_/NO_;
	}
	else {
		numVectorsOut = NUM_WORDS_IN_TMP/(NO_/NI_);
	}

	T expected[MAX_IMAGES*numElementsOut];

	unsigned iVecCtr = 0;
	for(int i = 0; i < NUM_WORDS_IN_TMP; i++){
	// generate input stream
		hls::vector<T, NI_> input;
		for(int j = 0; j < NI_; j++){
			unsigned valIn = i*NI_+j;
			unsigned largestValue = (pow(2, NI_));
			valIn %= largestValue; 
			input[j] = valIn;
			expected[i*NI_+j] = valIn; 
		}
		input_stream.write(input);
		iVecCtr++;
	}

	Testbench_dwc_vector(input_stream, output_stream, MAX_IMAGES);	
	
	// verification
	unsigned oVecCtr = 0;
	for(int i = 0; i < numVectorsOut; i++){
		hls::vector<T, NO_> value = output_stream.read();
		oVecCtr++;
		for(int j = 0; j < NO_; j++){
			if (expected[i*NO_+j] != value[j]){ 
				std::cerr << "ERROR! Expected: " << expected[i*j+j] << " is: " << value[j] << ", idx=: " << i*j+j << std::endl;
				return 1;
			}
		}
	}
}


