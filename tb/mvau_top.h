#ifndef MVAU_TOP_H
#define MVAU_TOP_H

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
//#include "data/config.h"
#include "activations.hpp"
#include "interpret.hpp"
#include "mvau.hpp"
#include "conv.hpp"
#include "hls_vector.h"
#include "dma.h"
#include "interpret_bipolar.hpp"

template<unsigned  DEPTH>
class BipolarAccu {
    ap_uint<clog2(DEPTH+1)>  val; // number of +1s

public:
    BipolarAccu() : val(0) {}
    BipolarAccu(unsigned val_) : val(val_) {}
    ~BipolarAccu() {}
    BipolarAccu&  operator+=(Bipolar const& o) {
        val += o.val;
        return *this;
    }
    BipolarAccu&  operator=(BipolarAccu const& o) {
        val = o.val;
        return *this;
    }
    operator int() const {
        return 2*val-DEPTH; 
    }
};

constexpr unsigned MAX_IMAGES = 1;
constexpr unsigned MatrixH = MATRIXH_; // #output size
constexpr unsigned MatrixW = MATRIXW_; // #input size, since weights matrix is transposed
constexpr unsigned SIMD = SIMD_;
constexpr unsigned PE = PE_;
constexpr unsigned numReps = 1;

using IDT = ap_uint<5>;
using WDT = ap_uint<5>;
using ODT = ap_uint<8>;

//using IDT = Bipolar;
//using WDT = Bipolar;
//using ODT = BipolarAccu<MatrixW>;

//using IDT = IDTTCL;
//using WDT = WDTTCL;
//using ODT = ODTTCL;

void Testbench_mvau(hls::stream<hls::vector<IDT, SIMD>> & in, hls::stream<hls::vector<ODT, PE>> & out, hls::stream<hls::vector<WDT, SIMD*PE>> & weights);
#endif