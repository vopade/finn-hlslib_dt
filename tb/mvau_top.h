#pragma once

#include "ap_int.h"
#include "weights.hpp"
#include "bnn-library.h"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstring>
#include <hls_stream.h>
#include <cstdlib>
#define AP_INT_MAX_W 8191
#include "data/memdata.h"
//#include "data/config.h"
#include "activations.hpp"
#include "weights.hpp"
#include "interpret.hpp"
#include "mvau.hpp"
#include "conv.hpp"
#include "hls_vector.h"
#include "dma.h"
#include "interpret_bipolar.hpp"

constexpr unsigned MAX_IMAGES = 1;
constexpr unsigned MatrixH = MATRIXH_;  
constexpr unsigned MatrixW = MATRIXW_; // #inputSamples, since weights matrix is transposed
constexpr unsigned SIMD = SIMD_;
constexpr unsigned PE = PE_;
constexpr unsigned numReps = 1;

//using IDT = Bipolar;
//using ODT = Bipolar;
//using WDT = Bipolar;

using IDT = ap_uint<9>;
using ODT = ap_uint<16>;
using WDT = ap_uint<9>;

void Testbench_mvau(hls::stream<hls::vector<IDT, SIMD>> & in, hls::stream<hls::vector<ODT, PE>> & out, hls::stream<hls::vector<WDT, SIMD*PE>> & weights, unsigned int numReps);










