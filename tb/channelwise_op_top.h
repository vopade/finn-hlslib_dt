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

constexpr int PE = 4;
constexpr int IFM_CH = 8;
constexpr int OFM_CH = IFM_CH;
constexpr int FOLD = (OFM_CH/PE);
constexpr int IFM_DIM = 3;
constexpr int OFMDim = IFM_DIM;

constexpr int INPUT_BITS = 4;
constexpr int BIP_OUT_BITS = INPUT_BITS+1;
constexpr int ADD_OUT_BITS = BIP_OUT_BITS+1;
constexpr int MUL_OUT_BITS = ADD_OUT_BITS+3;
constexpr int OUTPUT_BITS = MUL_OUT_BITS;

using IDT = Bipolar;
using BIP_PDT = Bipolar;
using ADD_PDT = ap_int<3>;
using MUL_PDT = ap_int<3>;
using BIP_ODT = Bipolar;
using ADD_ODT = ap_int<ADD_OUT_BITS>;
using MUL_ODT = ap_int<MUL_OUT_BITS>;
using ODT = ap_int<OUTPUT_BITS>;

//constexpr int BIP_INIT[PE][FOLD] = {{(Bipolar)1,(Bipolar)1},{(Bipolar)1,(Bipolar)0},{(Bipolar)0,(Bipolar)1},{(Bipolar)1,(Bipolar)1}};
//constexpr BIP_PDT BIP_INIT[PE][FOLD] = {{1,1},{1,0},{0,1},{1,1}};
//constexpr int ADD_INIT[PE][FOLD] = {{2, 1},{0, -1},{-1, -3},{1, 1}};  
//constexpr MUL_PDT MUL_INIT[PE][FOLD] = {{3, 1}, { 2,-1}, {-1, 1}, { 1,-2}};
#define BIP_INIT {{1,1},{1,0},{0,1},{1,1}}
#define ADD_INIT {{2, 1},{ 0,-1},{-1,-3},{ 1, 1}} 
#define MUL_INIT {{3, 1}, { 2,-1}, {-1, 1}, { 1,-2}} 

const BIP_PDT bip_init[PE][FOLD] = BIP_INIT;
//const int add_init[PE][FOLD] = ADD_INIT;
const ADD_PDT add_init[PE][FOLD] = ADD_INIT;
const MUL_PDT mul_init[PE][FOLD] = MUL_INIT;

void Testbench_channelwise_op(hls::stream<hls::vector<IDT,IFM_CH> > & in, 
                     hls::stream<hls::vector<ODT,OFM_CH> > & out, unsigned int numReps);