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
 *  \file conv_mvau_top.cpp
 *
 *  HLS Top function with a single MVAU for unit testing
 *
 *****************************************************************************/
#include "mvau_top.h"



template
<unsigned N, typename T>
void move(hls::stream<T> & src, hls::stream<T> & dst) {
    for(int i = 0; i < N; i++) {
#pragma HLS pipeline II=1 style=flp
        dst.write(src.read());
    }
}

#ifdef DECOUPLED_MODE
void Testbench_mvau(hls::stream<hls::vector<IDT, SIMD>> & in, hls::stream<hls::vector<ODT, PE>> & out, hls::stream<hls::vector<WDT, SIMD*PE>> & weights) {
#pragma HLS interface AXIS port=in
#pragma HLS interface AXIS port=weights
#pragma HLS interface AXIS port=out
#pragma HLS interface ap_ctrl_none port=return
#pragma HLS aggregate variable=in compact=bit
#pragma HLS aggregate variable=weights compact=bit
#pragma HLS aggregate variable=out compact=bit
#pragma HLS dataflow disable_start_propagation

    static hls::stream<hls::vector<IDT, SIMD>> in_0("in_0");
    static hls::stream<hls::vector<ODT, PE>> out_0("out_0");
    static hls::stream<hls::vector<WDT, SIMD*PE>> weights_0("weights_0");
    move<MatrixW/SIMD>(in, in_0);
    move<(MatrixH / PE) * (MatrixW / SIMD)>(weights, weights_0);

    Matrix_Vector_Activate_Stream_Vector_Batch
    <
    MatrixW, MatrixH, SIMD, PE, WDT, IDT, ODT
    >
    (
        in_0, out_0, weights_0, 1
    );

    move<MatrixH/PE>(out_0, out);
}
#endif

#ifdef CONST_MODE
void Testbench_mvau(hls::stream<hls::vector<IDT, SIMD>> & in, hls::stream<hls::vector<ODT, PE>> & out, hls::vector<WDT, SIMD*PE> const (&weights)[NUMTILES]) {
#pragma HLS interface AXIS port=in
#pragma HLS interface mode=ap_none port=weights 
#pragma HLS interface AXIS port=out
#pragma HLS interface ap_ctrl_none port=return
#pragma HLS aggregate variable=in compact=bit
//#pragma HLS aggregate variable=weights compact=bit
#pragma HLS aggregate variable=out compact=bit
#pragma HLS dataflow disable_start_propagation

    static hls::stream<hls::vector<IDT, SIMD>> in_0("in_0");
    static hls::stream<hls::vector<ODT, PE>> out_0("out_0");
    move<MatrixW/SIMD>(in, in_0);
    
    Matrix_Vector_Activate_Stream_Vector_Batch
    <
    MatrixW, MatrixH, SIMD, PE, NUMTILES, WDT, IDT, ODT
    >
    (
        in_0, out_0, weights, 1
    );

    move<MatrixH/PE>(out_0, out);
}
#endif
