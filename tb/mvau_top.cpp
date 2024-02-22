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

void Testbench_mvau(hls::stream<hls::vector<IDT, SIMD>> & in, hls::stream<hls::vector<ODT, PE>> & out, hls::stream<hls::vector<WDT, SIMD*PE>> & weights)
{

#pragma HLS interface AXIS port=in
#pragma HLS interface AXIS port=weights
#pragma HLS interface AXIS port=out
#pragma HLS interface ap_ctrl_none port=return
#pragma HLS aggregate variable=in compact=bit
#pragma HLS aggregate variable=weights compact=bit
#pragma HLS aggregate variable=out compact=bit

#pragma HLS dataflow disable_start_propagation

//StreamingDataWidthConverter_Batch<IFM_Channels1*INPUT_PRECISION, SIMD1*INPUT_PRECISION, InpPerImage>(in, wa_in, numReps);
//StreamingDataWidthConverterVector_Batch

Matrix_Vector_Activate_Stream_Vector_Batch
<
MatrixW, MatrixH, SIMD, PE, WDT, IDT, ODT
>
(
    in, out, weights, 1
);

//StreamingDataWidthConverter_Batch<PE1*ACTIVATION_PRECISION, OFM_Channels1*ACTIVATION_PRECISION, OFMDim1 * OFMDim1 * (OFM_Channels1 / PE1)>(mvOut, out, numReps);

}
