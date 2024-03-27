#ifndef STANDALONE_TOP_H
#define STANDALONE_TOP_H

#define AP_INT_MAX_W 8191
#include "ap_int.h"
#include "weights.hpp"
#include "bnn-library.h"
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstring>
#include <hls_stream.h>
#include <cstdlib>
#include "data/memdata.h"
#include "activations.hpp"
#include "interpret.hpp"
#include "mvau.hpp"
#include "hls_vector.h"
#include "dma.h"
#include "interpret_bipolar.hpp"

constexpr unsigned MAX_IMAGES = 1;
constexpr unsigned MatrixH = 64; // #output size first layer, HIDDEN1
constexpr unsigned MatrixW = 600; // #input size first layer, since weights matrix is transposed
constexpr unsigned HIDDEN1 = MatrixH; // use this terminology to be consistent with cybersecurity notebook
constexpr unsigned INPUTSIZE = MatrixW;
constexpr unsigned numReps = 1;
constexpr unsigned SIMD1 = 8;
constexpr unsigned PE1 = 4;

constexpr unsigned HIDDEN2 = 64; 
constexpr unsigned SIMD2 = 4;
constexpr unsigned PE2 = 2;

constexpr unsigned HIDDEN3 = 64;
constexpr unsigned SIMD3 = 2;
constexpr unsigned PE3 = 4;

constexpr unsigned NUMCLASSES = 1;
constexpr unsigned SIMD4 = 2;
constexpr unsigned PE4 = 1;

constexpr unsigned NUM_THRESHOLDS1 = 64; // equals HIDDENX
constexpr unsigned NUM_THRESHOLDS2 = 64;
constexpr unsigned NUM_THRESHOLDS3 = 64;
constexpr unsigned NUM_THRESHOLDS4 = 1;

constexpr unsigned NUM_STEPS1 = 3; // thresholding steps
constexpr unsigned NUM_STEPS2 = 3;
constexpr unsigned NUM_STEPS3 = 3;
constexpr unsigned NUM_STEPS4 = 1;

constexpr unsigned PE_THR = 1;

/* 
define: 
numLayers
data structure for input (unchanged), weights and thresholds, expected and produced, NUM_THRESHOLDS1, NUM_THRESHOLDS1
data types
*/

constexpr unsigned NUM_LAYERS = 4; 
constexpr unsigned PE[NUM_LAYERS] = {4,2,4,1};
constexpr unsigned SIMD[NUM_LAYERS] = {8,4,2,2};
constexpr unsigned LAYER_SIZES[NUM_LAYERS+1] = {600, 64, 64, 64, 1}; // first is input size -> rename

/*
using TI = ap_int<2>;
using TW1 = ap_int<2>;
using TW2 = ap_int<2>;
using TW3 = ap_int<2>;
using TW4 = ap_int<2>;
using TO1 = ap_int<2>;
using TO2 = ap_int<2>;
using TO3 = ap_int<2>;
using TO4 = ap_int<2>;
using TT = ap_int<8>;
*/

using TI = ap_int<26>;
using TW1 = ap_int<27>;
using TW2 = ap_int<28>;
using TW3 = ap_int<29>;
using TW4 = ap_int<29>;
using TO1 = ap_int<30>;
using TO2 = ap_int<31>;
using TO3 = ap_int<32>;
using TO4 = ap_int<32>;
using TT = ap_int<32>;

/*
using TI = float;
using TW1 = float;
using TW2 = float;
using TW3 = float;
using TW4 = float;
using TO1 = float;
using TO2 = float;
using TO3 = float;
using TO4 = float;
using TT = float;
*/

void Testbench_standalone
(
    hls::stream<hls::vector<TI, SIMD1>> & in,
    hls::stream<hls::vector<TO3, PE4>> & out,
    hls::stream<hls::vector<TW1, SIMD1*PE1>> & weights1,
    hls::stream<hls::vector<TW2, SIMD2*PE2>> & weights2,
    hls::stream<hls::vector<TW3, SIMD3*PE3>> & weights3,
    hls::stream<hls::vector<TW4, SIMD4*PE4>> & weights4,
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS1>> & thr1, // PE must be 1, because thresoldig assumes that if chn=1
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS2>> & thr2,
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS3>> & thr3,
    hls::stream<hls::vector<TT, PE_THR*NUM_STEPS4>> & thr4,
    unsigned int numReps
);
#endif