##############################################################################
 #  Copyright (c) 2019, Xilinx, Inc.
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions are met:
 #
 #  1.  Redistributions of source code must retain the above copyright notice,
 #     this list of conditions and the following disclaimer.
 #
 #  2.  Redistributions in binary form must reproduce the above copyright
 #      notice, this list of conditions and the following disclaimer in the
 #      documentation and/or other materials provided with the distribution.
 #
 #  3.  Neither the name of the copyright holder nor the names of its
 #      contributors may be used to endorse or promote products derived from
 #      this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 #  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 #  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 #  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 #  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 #  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 #  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 #  OR BUSINESS INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 #  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 #  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 #  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
###############################################################################
###############################################################################
 #
 #  Authors: Giulio Gambardella <giuliog@xilinx.com>
 #           Jonas Kuehle <jonkuhle@amd.com>
 #
 # \file test_conv3.tcl
 #
 # Tcl script for HLS csim, synthesis and cosim of the convolutional layer
 #
###############################################################################

# matrixH, matrixW, SIMD, PE, IDT, WDT, ODT
#set params {{16 32 32 16} {16 32 16 16} {16 32 1 1} {32 16 1 1} {16 32 2 2} {32 16 2 2} {16 32 8 4} {16 32 4 8} {16 32 2 2} {4 4 2 2} {4 4 2 4} {4 4 4 2} {4 4 4 4} }  
set params {{32 32 16 16 ap_uint<9> ap_uint<9> ap_uint<16>}}
#set params {{8 4 2 2 Bipolar Bipolar auto}}
#set params {{4 4 4 4 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 32 16 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 16 16 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 1 1 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {32 16 1 1 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 2 2 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {32 16 2 2 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 8 4 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 4 8 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {16 32 2 2 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {4 4 2 2 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {4 4 2 4 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {4 4 4 2 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'} {4 4 4 4 'ap_uint<9>' 'ap_uint<9>' 'ap_uint<16>'}}  

foreach p $params {
    #set compilerFlags "-std=c++14 -I$::env(FINN_HLS_ROOT) -I$::env(FINN_HLS_ROOT)/tb -DMATRIXH_=[lindex $p 0] -DMATRIXW_=[lindex $p 1] -DSIMD_=[lindex $p 2] -DPE_=[lindex $p 3] -DIDTTCL='ap_uint<9>' -DWDTTCL=int -DODTTCL=int"
    set compilerFlags "-std=c++14 -I$::env(FINN_HLS_ROOT) -I$::env(FINN_HLS_ROOT)/tb -DMATRIXH_=[lindex $p 0] -DMATRIXW_=[lindex $p 1] -DSIMD_=[lindex $p 2] -DPE_=[lindex $p 3] -DIDTTCL=\"[lindex $p 4]\" -DWDTTCL=\"[lindex $p 5]\" -DODTTCL=\"[lindex $p 6]\""
    #set compilerFlags "-E -std=c++14 -I$::env(FINN_HLS_ROOT) -I$::env(FINN_HLS_ROOT)/tb -DMATRIXH_=[lindex $p 0] -DMATRIXW_=[lindex $p 1] -DSIMD_=[lindex $p 2] -DPE_=[lindex $p 3]"
    puts $compilerFlags
    # in case project already exists from an aborted previous run
    delete_project hls-syn-mvau-stream 
    open_project hls-syn-mvau-stream
    add_files mvau_top.cpp -cflags $compilerFlags
    add_files -tb mvau_tb.cpp -cflags $compilerFlags
    set_top Testbench_mvau
    open_solution sol1
    set_part {xczu3eg-sbva484-1-i}
    create_clock -period 5 -name default
    csim_design
    csynth_design
    cosim_design
    #delete_project hls-syn-mvau-stream
}
exit
