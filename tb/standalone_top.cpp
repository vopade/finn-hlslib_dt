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

#include "standalone_top.h"

template
<unsigned N, typename T>
void move(hls::stream<T> & src, hls::stream<T> & dst) {
    for(int i = 0; i < N; i++) {
#pragma HLS pipeline II=1 style=flp
        dst.write(src.read());
    }
}

void Testbench_standalone
(
    hls::stream<hls::vector<TI, SIMD1>> & in, 
    hls::stream<hls::vector<TO4, PE4>> & out4th, 
    hls::stream<hls::vector<TW1, SIMD1*PE1>> & weights1, 
    hls::stream<hls::vector<TW2, SIMD2*PE2>> & weights2, 
    hls::stream<hls::vector<TW3, SIMD3*PE3>> & weights3, 
    hls::stream<hls::vector<TW4, SIMD4*PE4>> & weights4, 
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS1>> & thr1,
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS2>> & thr2,
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS3>> & thr3,
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS4>> & thr4,
    unsigned int numReps
) {
#pragma HLS dataflow
#pragma HLS dataflow disable_start_propagation
#pragma HLS interface AXIS port=in
#pragma HLS interface AXIS port=weights1
#pragma HLS interface AXIS port=weights2
#pragma HLS interface AXIS port=weights3
#pragma HLS interface AXIS port=weights4
#pragma HLS interface AXIS port=thr1
#pragma HLS interface AXIS port=thr2
#pragma HLS interface AXIS port=thr3
#pragma HLS interface AXIS port=thr4
#pragma HLS interface AXIS port=out4th
#pragma HLS aggregate variable=in compact=bit
#pragma HLS aggregate variable=weights1 compact=bit
#pragma HLS aggregate variable=weights2 compact=bit
#pragma HLS aggregate variable=weights3 compact=bit
#pragma HLS aggregate variable=weights4 compact=bit
#pragma HLS aggregate variable=thr1 compact=bit
#pragma HLS aggregate variable=thr2 compact=bit
#pragma HLS aggregate variable=thr3 compact=bit
#pragma HLS aggregate variable=thr4 compact=bit
#pragma HLS aggregate variable=out4th compact=bit
//#pragma HLS interface ap_ctrl_none port=return // TODO: what is the problem? criteria is matched

    static hls::stream<hls::vector<TI, SIMD1>> in_0("in_0");
    static hls::stream<hls::vector<TW1, SIMD1*PE1>> weights1_0("weights1_0");
    static hls::stream<hls::vector<TW2, SIMD2*PE2>> weights2_0("weights2_0");
    static hls::stream<hls::vector<TW3, SIMD3*PE3>> weights3_0("weights3_0");
    static hls::stream<hls::vector<TW4, SIMD4*PE4>> weights4_0("weights4_0");
    static hls::stream<hls::vector<TT, PE_THR*NUM_STEPS1>> thr1_0("thr1_0");
    static hls::stream<hls::vector<TT, PE_THR*NUM_STEPS2>> thr2_0("thr2_0");
    static hls::stream<hls::vector<TT, PE_THR*NUM_STEPS3>> thr3_0("thr3_0");
    static hls::stream<hls::vector<TT, PE_THR*NUM_STEPS4>> thr4_0("thr4_0");
    static hls::stream<hls::vector<TO4, PE4>> out4th_0("out4th_0");

    // copy streams to avoid FIFO problems
    move<MatrixW/SIMD1>(in, in_0);
    move<(MatrixH / PE1) * (MatrixW / SIMD1)>(weights1, weights1_0); 
    move<(HIDDEN2 / PE2) * (HIDDEN1 / SIMD2)>(weights2, weights2_0);
    move<(HIDDEN3 / PE3) * (HIDDEN2 / SIMD3)>(weights3, weights3_0);
    move<(NUMCLASSES / PE4) * (HIDDEN3 / SIMD4)>(weights4, weights4_0);
    move<(NUM_THRESHOLDS1)>(thr1, thr1_0); 
    move<(NUM_THRESHOLDS2)>(thr2, thr2_0);
    move<(NUM_THRESHOLDS3)>(thr3, thr3_0);
    move<(NUM_THRESHOLDS4)>(thr4, thr4_0);

    // layer 1
    hls::stream<hls::vector<TO1, PE1>> out1("out1_stream");
    hls::stream<hls::vector<TO1, PE_THR>> out1th("out1th_stream"); // after thresholding
    Matrix_Vector_Activate_Stream_Vector_Batch<MatrixW, MatrixH, SIMD1, PE1, TW1, TI, TO1>
    (
        in_0, out1, weights1_0, numReps
    );

    // convert to PE=1 since thresholding assumes that if chn=1
    hls::stream<hls::vector<TO1, PE_THR>> in1th("in1th_stream");
    StreamingDataWidthConverterVector_Batch<HIDDEN1/PE1, TO1, PE1, 1>(out1, in1th, 1);
    Thresholding_Stream_Batch_Vector<NUM_THRESHOLDS1, 1, PE_THR, 0, TT, NUM_STEPS1, TO1, TO1> 
    (
        in1th, out1th, thr1_0, numReps
    );

    // layer 2
    // Thresholding provides vector of size <T, PE> but MVAU needs <T, SIMD>, <T, PE> is always <T, 1>
    hls::stream<hls::vector<TO1, SIMD2>> mvauIn2("mvauIn2_stream");
    StreamingDataWidthConverterVector_Batch<HIDDEN1, TO1, PE_THR, SIMD2>(out1th, mvauIn2, 1);
    hls::stream<hls::vector<TO2, PE2>> out2("out2_stream");
    Matrix_Vector_Activate_Stream_Vector_Batch<MatrixH, HIDDEN2, SIMD2, PE2, TW2, TO1, TO2>
    (
        mvauIn2, out2, weights2_0, numReps
    );

    hls::stream<hls::vector<TO2, PE_THR>> in2th("in2th_stream");
    StreamingDataWidthConverterVector_Batch<HIDDEN2/PE2, TO2, PE2, 1>(out2, in2th, 1);
    hls::stream<hls::vector<TO2, PE_THR>> out2th("out2th_stream");
    Thresholding_Stream_Batch_Vector<NUM_THRESHOLDS2, 1, PE_THR, 0, TT, NUM_STEPS2, TO2, TO2> 
    (
        in2th, out2th, thr2_0, numReps
    );

    // layer 3
    hls::stream<hls::vector<TO2, SIMD3>> mvauIn3("mvauIn3_stream");
    StreamingDataWidthConverterVector_Batch<HIDDEN2, TO2, PE_THR, SIMD3>(out2th, mvauIn3, 1);
    hls::stream<hls::vector<TO3, PE3>> out3("out3_stream");
    Matrix_Vector_Activate_Stream_Vector_Batch<HIDDEN2, HIDDEN3, SIMD3, PE3, TW3, TO2, TO3>
    (
        mvauIn3, out3, weights3_0, numReps
    );

    hls::stream<hls::vector<TO3, PE_THR>> in3th("in3th_stream");
    StreamingDataWidthConverterVector_Batch<HIDDEN3/PE3, TO3, PE3, 1>(out3, in3th, 1);
    hls::stream<hls::vector<TO3, PE_THR>> out3th("out3th_stream");
    Thresholding_Stream_Batch_Vector<NUM_THRESHOLDS3, 1, PE_THR, 0, TT, NUM_STEPS3, TO3, TO3> 
    (
        in3th, out3th, thr3_0, numReps
    );

    // layer 4
    hls::stream<hls::vector<TO3, SIMD4>> mvauIn4("mvauIn4_stream");
    StreamingDataWidthConverterVector_Batch<HIDDEN3, TO3, PE_THR, SIMD4>(out3th, mvauIn4, 1);
    hls::stream<hls::vector<TO4, PE_THR>> out4("out4_stream");
    Matrix_Vector_Activate_Stream_Vector_Batch<HIDDEN3, NUMCLASSES, SIMD4, PE4, TW4, TO3, TO4>
    (
        mvauIn4, out4, weights4_0, numReps
    );

    hls::stream<hls::vector<TO4, PE_THR>> in4th("in4th_stream");
    StreamingDataWidthConverterVector_Batch<NUMCLASSES/PE4, TO4, PE4, 1>(out4, in4th, 1);
    Thresholding_Stream_Batch_Vector<NUM_THRESHOLDS4, 1, PE_THR, 0, TT, NUM_STEPS4, TO4, TO4> 
    (
        in4th, out4th_0, thr4_0, numReps
    );

    move<NUMCLASSES>(out4th_0, out4th);
}
