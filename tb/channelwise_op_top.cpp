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
 *  Authors: Tobias Alonso <tobiasa@xilinx.com>
 *
 *  \file channelwise_op_top.cpp
 *  
 *  HLS Top function with three ChannelWiseOperation activation layers for unit 
 *  testing
 *  
 *****************************************************************************/

#include "channelwise_op_top.h"

// Implements:
// in -> [width adapt] -> [bipolar mult] -> [add]  -> [mult] -> [width adapt] -> out
 void Testbench_channelwise_op(hls::stream<hls::vector<IDT,IFM_CH> > & in, 
                     hls::stream<hls::vector<ODT,OFM_CH> > & out, unsigned int numReps){
#pragma HLS DATAFLOW
     //with default size can result in deadlock. Please consider resizing the stream using the d
    // [width adapt] 

    stream<hls::vector<IDT,PE>>  wa_in;   
#pragma HLS STREAM variable=wa_in depth=INPUT_BITS*PE
    StreamingDataWidthConverterVector_Batch<IFM_DIM*IFM_DIM, IDT, IFM_CH, PE>(in, wa_in, numReps); 
    // [bipolar mult]
    ChannelWiseOperation<FOLD, PE, IDT, BIP_PDT, BIP_ODT, 
            per_channel_neg<BIP_ODT> > bip_params= {.parameters = BIP_INIT};

    hls::stream<hls::vector<BIP_ODT,PE>>  bip_out;
#pragma HLS STREAM variable=bip_out depth=BIP_OUT_BITS*PE
    Thresholding_Batch<IFM_DIM*IFM_DIM, IFM_CH, PE,IDT,BIP_ODT>
        (wa_in, bip_out, bip_params, numReps);

    // [add] 
    ChannelWiseOperation<FOLD, PE, BIP_ODT, ADD_PDT, ADD_ODT, 
            comp::add<ADD_PDT, BIP_ODT, ADD_ODT> > add_params = {.parameters = ADD_INIT};

    hls::stream<hls::vector<ADD_ODT,PE>> add_out;
#pragma HLS STREAM variable=add_out depth=ADD_OUT_BITS*PE
    Thresholding_Batch<IFM_DIM*IFM_DIM, IFM_CH, PE, BIP_ODT, ADD_ODT>
        (bip_out, add_out, add_params, numReps);

    // [mult] 
    ChannelWiseOperation<FOLD, PE, ADD_ODT, MUL_PDT, MUL_ODT, 
            comp::mul<MUL_PDT, ADD_ODT, MUL_ODT> > mult_params= {.parameters = MUL_INIT};

    hls::stream<hls::vector<MUL_ODT,PE>> mul_out;
#pragma HLS STREAM variable=mul_out depth=MUL_OUT_BITS*PE
    Thresholding_Batch<IFM_DIM*IFM_DIM, IFM_CH, PE, ADD_ODT, MUL_ODT>
        (add_out, mul_out, mult_params, numReps);

    // [width adapt]
    StreamingDataWidthConverterVector_Batch<IFM_DIM*IFM_DIM*FOLD, MUL_ODT, PE, OFM_CH>
        (mul_out, out, numReps);
}
