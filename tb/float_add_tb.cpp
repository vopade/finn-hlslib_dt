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
 *  Authors: Giulio Gambardella <giuliog@xilinx.com>
 *           Tobias Alonso <tobiasa@xilinx.com>
 *
 *  \file channelwise_op_tb.cpp
 *
 *  Testbench for the ChannelWiseOperation activation
 *
 *****************************************************************************/

#include <iostream>
#include <cmath>
#include <cstring>
#include <hls_stream.h>
#include <cstdlib>
#define AP_INT_MAX_W 8191
#include "ap_int.h"
#include "bnn-library.h"
#include "activations.hpp"
#include "float_add_top.h"  

using namespace hls;
using namespace std;

#define ROUNDS 2 // ROUNDS >1 to get cosim to measure II

int main()
{   
    // input and output of DUT
    hls::stream<hls::vector<TI,IFM_CH>> sIn[ROUNDS];
    hls::stream<hls::vector<TO,OFM_CH>> sOut[ROUNDS];
    hls::vector<TI, IFM_CH> vIn;
    vIn[0] = 2.4; 
    
    for(int i = 0; i < ROUNDS; i++) {
        sIn[i].write(vIn);
    }

    // call DUT, same operation two times
    for(int i = 0; i < ROUNDS; i++) {
        Testbench_float_add(sIn[i], sOut[i], 1);
    }

    // compute expected    
    TO exp = vIn[0] + add_init[0];
    bool errorOccured = false;
    for(int i = 0; i < ROUNDS; i++) {
        auto prod = sOut[i].read();
        if(exp != prod) {
            std::cerr << "ERROR! expected=" << exp <<", produced=" << prod[0] << std::endl;
            errorOccured = true;
        }
    }

    if(errorOccured) {
        std::cerr << "ERROR OCCURED! TEST NOT SUCCESFULLY!" << std::endl; 
    }
    else {
        std::cerr << "TEST SUCCESFULLY!" << std::endl;
    } 
}