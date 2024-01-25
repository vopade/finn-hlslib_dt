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
#pragma once
#include <hls_stream.h>
using namespace hls;
#include "ap_int.h"
#include "bnn-library.h"

#include "activations.hpp"
#include "weights.hpp"
#include "interpret.hpp"
#include "hls_vector.h"
#include "interpret_bipolar.hpp"

#define PE 4
#define IFM_CH 8 // IFM channels
#define OFM_CH IFM_CH

#define IFM_DIM 3
#define OFMDim IFM_DIM

//#define BIP_PDT ap_uint<1> // param type for BIP
#define BIP_PDT Bipolar
#define ADD_PDT ap_int<3> 
#define MUL_PDT ap_int<3>
#define BIP_INIT {{(Bipolar)1,(Bipolar)1},{(Bipolar)1,(Bipolar)0},{(Bipolar)0,(Bipolar)1},{(Bipolar)1,(Bipolar)1}}
#define ADD_INIT {{2, 1},{ 0,-1},{-1,-3},{ 1, 1}} 
#define MUL_INIT {{3, 1}, { 2,-1}, {-1, 1}, { 1,-2}} 

#define INPUT_BITS 4
//#define IDT ap_uint<INPUT_BITS>
#define IDT Bipolar
#define BIP_OUT_BITS  (INPUT_BITS+1)
#define ADD_OUT_BITS  (BIP_OUT_BITS+1)
#define MUL_OUT_BITS  (ADD_OUT_BITS+3)
#define OUTPUT_BITS MUL_OUT_BITS
#define ODT ap_int<OUTPUT_BITS>

//#define IN_T ap_uint
#define IN_T Bipolar
//#define BIP_ODT ap_int<BIP_OUT_BITS>
#define BIP_ODT Bipolar
#define ADD_ODT ap_int<ADD_OUT_BITS>
#define MUL_ODT ap_int<MUL_OUT_BITS>
#define OUT_T ap_int

#define FOLD (OFM_CH/PE)

const int bipolar_init[PE][FOLD] = BIP_INIT;
const int add_init[PE][FOLD] = ADD_INIT;
const int mult_init[PE][FOLD] = MUL_INIT;

template<typename T>
struct per_channel_neg
{
    constexpr T operator()(const ap_uint<1> &lhs, const T &rhs) const {
        return lhs? static_cast<decltype(-rhs)>(rhs):-rhs;
    } 
};

void Testbench_channelwise_op(hls::stream<hls::vector<IDT,IFM_CH> > & in, 
                     hls::stream<hls::vector<ODT,OFM_CH> > & out, unsigned int numReps);