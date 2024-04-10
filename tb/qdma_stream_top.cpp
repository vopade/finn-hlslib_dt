#include "ap_int.h"
#include <hls_stream.h>
#include "bnn-library.h"
#include "data/qdmastream_config.h"
void qdma_conv_top(hls::stream<qdma_axis<WIDTH,0,0,0> > & in, hls::stream<qdma_axis<WIDTH,0,0,0> > & out){
#pragma HLS INTERFACE axis port=in
#pragma HLS INTERFACE axis port=out
	hls::stream<T> internal_stream;
#pragma HLS RESOURCE variable=internal_stream core=FIFO_LUTRAM
#pragma HLS STREAM variable=internal_stream depth=32
#pragma HLS DATAFLOW
	Qdma2Stream_Batch<T, WIDTH, 1000>(in,internal_stream, 1);
	Stream2Qdma_Batch<T, WIDTH, 1000>(internal_stream, out, 1);
}
