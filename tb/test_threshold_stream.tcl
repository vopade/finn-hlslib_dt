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
 # \file test_threshold_stream.tcl
 #
 # Tcl script for HLS csim, synthesis and cosim of thresholding stream
 #
###############################################################################

# PE, channel, steps, matrixH, matrixW, 
#set params {{8 8 6 8 4} {16 16 8 2 4} {8 8 32 2 4}}
set params {{2 2 3 2 2}}

foreach p $params {
    # in case project already exists from an aborted previous run
    delete_project hls-syn-thresholding-stream 
    set compilerFlags "-std=c++14 -I$::env(FINN_HLS_ROOT) -I$::env(FINN_HLS_ROOT)/tb -DPE_=[lindex $p 0] -DCHN_=[lindex $p 1] -DSTP_=[lindex $p 2] -DMH_=[lindex $p 3] -DMW_=[lindex $p 4]"
    open_project hls-syn-thresholding-stream
    add_files threshold_stream_top.cpp -cflags $compilerFlags
    add_files -tb threshold_stream_tb.cpp -cflags $compilerFlags
    set_top Testbench_threshold_stream
    open_solution sol1
    set_part {xczu3eg-sbva484-1-i}
    create_clock -period 5 -name default
    csim_design
    csynth_design
    cosim_design
}
exit