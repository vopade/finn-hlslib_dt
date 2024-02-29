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
 *  Authors: Jonas Kuehle <jonkuhle@amd.com>
 *
 *  \file threshold_tb.cpp
 *
 *  Testbench for Thresholding batch
 *
 *****************************************************************************/

#include "threshold_stream_top.hpp"  

using namespace hls;
using namespace std;

template<typename TO, typename TI, typename TT>
TO computeThresholds(TI in, TT ths[stp]) {

    std::sort(ths, ths + stp); 
    TO out = 0;
    for(int i = 0; i < stp; i++) {
        if(in >= ths[i]) {
            out = i+1;
        }
    }
    return out;
}

int main() {  
    hls::stream<hls::vector<TI, PE>> s_in("input_stream");
    hls::stream<hls::vector<TO, PE>> s_out("output_stream");
    hls::stream<hls::vector<TT, PE*stp>> s_ths("stream_thresholds"); 

    TI in[dim][chn];
    TT ths[dim][chn][stp];
    TO prod[dim][chn];
    TO exp[dim][chn];
    
    // generate input
    for(int d = 0; d < dim; d++) {
        for(int i = 0; i < chn; i++) {
            in[d][i] = rand()%20;
        }
    }

    // generate thresholds
    for(int d = 0; d < dim; d++) {    
        for(int i = 0; i < chn; i++) {
            for(int j = 0; j < stp; j++) {
                ths[d][i][j] = rand()%20;
            }
        }
    }

    // compute expected results
    for(int d = 0; d < dim; d++) {  
        for(int i = 0; i < chn; i++) {
            TT act_thr[stp];
            for(int j = 0; j < stp; j++) {
                act_thr[j] = ths[d][i][j];
            }
            exp[d][i] = computeThresholds<TO>(in[d][i], act_thr);
        }
    }

    // prepare input stream
    for(int d = 0; d < dim; d++) {  
        for(int i = 0; i < chn/PE; i++) {
            hls::vector<TI, PE> actual_input;
            for(int j = 0; j < PE; j++) {
                actual_input[j] = in[d][i*PE+j];
            }
            s_in.write(actual_input);
        }
    }

    // prepare thresholds streams
    for(int d = 0; d < dim; d++) {  
        for(int i = 0; i < chn/PE; i++) {
            hls::vector<TT, PE*stp> act_thr;
            for(int j = 0; j < PE; j++) {
                for(int k = 0; k < stp; k++) {
                    act_thr[j*stp+k] = ths[d][i*PE+j][k];
                }
            }
            s_ths.write(act_thr);
        }
    }

    Testbench_threshold_stream(s_in, s_out, s_ths, 1);

    for(int d = 0; d < dim; d++) {  
        for(int i = 0; i < chn/PE; i++) {
            hls::vector<TO, PE> act_out = s_out.read();
            for(int j = 0; j < PE; j++) {
                prod[d][i*PE+j] = act_out[j];
            }
        }
    }

    // Verification
    bool error_occured = false;
    for(int d = 0; d < dim; d++) {  
        for(int i = 0; i < chn; i++) {
            if(prod[d][i] != exp[d][i]) {
                std::cerr << "ERROR!!! produced: " << prod[d][i] << ", expected: " << exp[d][i] << ".\t\t input: " << in[d][i] << ", thresholds: ";
                for(int j = 0; j < stp; j++) {
                    std::cerr << ths[d][i][j] << ", ";
                }
                error_occured = true;
                std::cout << std::endl;
            }
        }
    }

    if(error_occured) {
        std::cerr << std::endl << "Test FAILED!" << std::endl << std::endl << std::endl;
    }
    else {
        std::cout << std::endl << "Test PASSED!" << std::endl << std::endl << std::endl;
    }
}