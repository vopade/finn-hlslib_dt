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
#ifndef THRESHOLD_TOP_HPP
#define THRESHOLD_TOP_HPP
#define AP_INT_MAX_W 8191

#include <hls_stream.h>
#include "ap_int.h"
#include "bnn-library.h"
#include "activations.hpp"
#include "weights.hpp"
#include "interpret.hpp"
#include "hls_vector.h"
#include <cstdlib>
#include "interpret_bipolar.hpp"

constexpr unsigned PE = PE_;
constexpr unsigned chn = CHN_; // channels
constexpr unsigned stp = STP_; // number of steps (thresholds)
constexpr unsigned mw = MW_; // matrix width
constexpr unsigned mh = MH_; // matrix height
constexpr unsigned dim = mh*mw;

using TI = ap_int<16>;
using TO = ap_int<33>;
using TT = ap_int<13>;
using namespace hls;

void Testbench_threshold_stream(hls::stream<hls::vector<TI, PE>> &in,
                                hls::stream<hls::vector<TO, PE>> &out,
                                hls::stream<hls::vector<TT, PE*stp>> &weights,
                                int const reps);
#endif