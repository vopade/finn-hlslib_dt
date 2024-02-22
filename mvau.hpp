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
 *******************************************************************************/

/*******************************************************************************
 *
 *  Authors: Giulio Gambardella <giuliog@xilinx.com>
 *           Thomas B. Preusser <thomas.preusser@utexas.edu>
 *             Marie-Curie Fellow, Xilinx Ireland, Grant Agreement No. 751339
 *           Christoph Doehring <cdoehrin@xilinx.com>
             Jonas Kuehle <jonkuhle@amd.com>
 *
 *  \file mvau.hpp
 *
 *  This file lists a templated funtion used to implement  
 *  Matrix-Vector-Activation Unit
 *
 *  This project has received funding from the European Union's Framework
 *  Programme for Research and Innovation Horizon 2020 (2014-2020) under
 *  the Marie Skłodowska-Curie Grant Agreement No. 751339.
 *
 *******************************************************************************/

#ifndef MVAU_HPP
#define MVAU_HPP

#include "hls_stream.h"
#include "hls_vector.h"
#include "mac.hpp"
#include "interpret.hpp"
#include "weights.hpp"
#include "interpret_bipolar.hpp"


/**
 * \brief Matrix vector activate function
 *
 * The function performs the multiplication between a weigth matrix and the input activation vector,
 * accumulating the results and then applying an activation function on the accumulated result.
 *
 * TODO: update
 * \tparam MatrixW    Width of the input matrix
 * \tparam MatrixH    Heigth of the input matrix
 * \tparam SIMD       Number of input columns computed in parallel
 * \tparam PE         Number of output rows computed in parallel
 * \tparam MMV        Number of output pixels computed in parallel
 * \tparam TSrcI      DataType of the input activation (as used in the MAC)
 * \tparam TDstI      DataType of the output activation (as generated by the activation)
 * \tparam TWeightI   DataType of the weights and how to access them in the array
 * \tparam TI         DataType of the input stream - safely deducible from the paramaters
 * \tparam TO         DataType of the output stream - safely deducible from the paramaters
 * \tparam TW         DataType of the weights matrix - safely deducible from the paramaters
 * \tparam TA         DataType of the activation class (e.g. thresholds) - safely deducible from the paramaters
 * \tparam R          Datatype for the resource used for FPGA implementation of the MAC  - safely deducible from the paramaters
 *
 * \param in          Input stream
 * \param out         Output stream
 * \param weights     Weights matrix (currently supports BinaryWeights or FixedPointWeights)
 * \param activation  Activation class
 * \param reps        Number of time the function has to be repeatedly executed (e.g. number of images)
 * \param r           Resource type for the hardware implementation of the MAC block
 */
template<
  unsigned MatrixW, unsigned MatrixH, unsigned SIMD, unsigned PE, unsigned MMV,
  typename TI, typename TO, typename TWeightI, typename TW, typename R
>
void Matrix_Vector_Activate_Batch(
          hls::stream<hls::vector<TI, SIMD>> &in, 
				  hls::stream<hls::vector<TO, PE>> &out, 
				  const hls::vector<TW, MatrixH*MatrixW> &weights,
          //const &weights,
				  int const  reps,
				  R const &r) {

  // how many different rows each neuron will compute
  // alternatively: number of vertical matrix chunks
  unsigned const  NF = MatrixH / PE;

  // how many synapse groups each row is split into
  // alternatively: number of horizontal matrix chunks
  unsigned const  SF = MatrixW / SIMD;

  // input vector buffers
  hls::vector<TI, SIMD>  inputBuf[SF];
#pragma HLS ARRAY_PARTITION variable=inputBuf complete dim=0
#pragma HLS aggregate variable=out compact=bit
#pragma HLS aggregate variable=in compact=bit

  TO accu[MMV][PE];

#pragma HLS ARRAY_PARTITION variable=accu complete dim=0

  unsigned  nf   = 0;
  unsigned  sf   = 0;
  unsigned  tile = 0; // invariant: tile = nf*SF + sf

  // everything merged into a common iteration space (one "big" loop instead
  // of smaller nested loops) to get the pipelinening the way we want
  unsigned const TOTAL_FOLD = NF * SF;

  std::cout << "SIMD:" << SIMD << ", pe: " << PE << ", NF:" << NF << ", SF:" << SF <<", mmv: " << MMV << ", TI: " << typeid(TI).name() << ", TW: " << typeid(TW).name() <<  ", TWeightI: " << typeid(TWeightI).name() << std::endl;

  for(unsigned  i = 0; i < reps * TOTAL_FOLD; i++) {
#pragma HLS pipeline style=flp II=1

    hls::vector<TI, SIMD> inElem;

    if(nf == 0) {
      // read input from stream
      inElem = in.read();
      // store in appropriate buffer for reuse
      inputBuf[sf] = inElem;
    }
    else {
      // reuse buffered input
      inElem = inputBuf[sf];
    }

    // Threshold Initialisation
    if(sf == 0) {
      for(unsigned  pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
        for(unsigned mmv = 0; mmv < MMV; mmv++) {
#pragma HLS UNROLL
          //accu[mmv][pe] = activation.init(nf, pe);
          accu[mmv][pe] = 0; // TODO: initialise in a better way
        }
      }
    }

    // compute matrix-vector product for each processing element
    auto const &w = weights.weights(tile);
  
    //std::cout << "w is of type: " << typeid(w).name() << std::endl;
    for(unsigned  pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
      auto const  wgt = w[pe];

      //std::array<TWeightI, 1> const wgt = w[pe]; 
      std::cout << "wgt is of type: " << typeid(wgt).name() << std::endl;

      for (unsigned mmv = 0; mmv < MMV; mmv++){
        //auto const  act = TSrcI()(inElem, mmv);
        //accu[mmv][pe] = mac<SIMD>(accu[mmv][pe], wgt, act, r, mmv);
        //accu[mmv][pe] = mac_New<SIMD, hls::vector<TI, SIMD>>(accu[mmv][pe], wgt, inputBuf[sf], r); 
        
        TO res = accu[mmv][pe]; 
        for(int i = 0; i < SIMD; i++){ 
          res = res + wgt[i] * inputBuf[sf][i];
          std::cout << wgt[i] << " * " << inputBuf[sf][i] << " + " << accu[mmv][pe] << " = " << res << std::endl;
        }
        accu[mmv][pe] = res;
      }
    }

    // keep track of which folded synapse/neuron we are processing
    ++tile;
    hls::vector <TO, PE> vecOut;
    if(++sf == SF) {
      // produce output and clear accumulators
      // auto  outElem = TDstI().template operator()<TO>();
      for (unsigned  pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
        for (unsigned mmv = 0; mmv < MMV; mmv++){
#pragma HLS UNROLL
          //outElem(pe,mmv,1) = activation.activate(nf, pe, accu[mmv][pe]);
          std::cout << "accu[mmv][pe] before act: " << accu[mmv][pe];
          //vecOut[pe] = activation.activate(nf, pe, accu[mmv][pe]); // raus. Aber wird noch benotigt. Warum?
          vecOut[pe] = accu[mmv][pe];
          std::cout << ", after: " << vecOut[pe] << std::endl;
          //vecOut[pe]++; // force wrong output for debugging
        }
      }
      out.write(vecOut);
      // next folded neuron or image
      sf = 0;
      if(++nf == NF) {
	    nf   = 0;
	    tile = 0;
      }
    }
  }
}

template<typename TW, long unsigned N>
class WeightsDecoupled{
  hls::stream<hls::vector<TW, N>> &weights;

public:
  WeightsDecoupled(hls::stream<hls::vector<TW, N>> &weights_) : weights(weights_) {}
  ~WeightsDecoupled(){}

public:
  hls::vector<TW, N> operator[](unsigned tile) const {
    return weights.read();
  }
}; // class WeightsDecoupled

template<typename TW, long unsigned N, unsigned TILES>
class WeightsConst{
public:
  hls::vector<TW, N> weights[TILES];
public:
  hls::vector<TW, N> operator[](unsigned tile) const {
    return weights[tile];
  }
}; // class WeightsConst


/**
 * \brief Matrix vector activate function with streaming weights
 *
 * The function performs the multiplication between a weigth matrix, presented as an input stream, and the input activation vector. 
 * Does not support MMV.
 *
 * 
 * \tparam MatrixW    Width of the (transposed) input matrix
 * \tparam MatrixH    Heigth of the (transposed) input matrix
 * \tparam SIMD       Number of input columns computed in parallel
 * \tparam PE         Number of output rows computed in parallel
 * \tparam TW         DataType of the weights (as used in the MAC) - not deducible from the paramaters
 * \tparam TI         DataType of the input stream - safely deducible from the paramaters
 * \tparam TO         DataType of the output stream - safely deducible from the paramaters
 *
 * \param in          Input stream
 * \param out         Output stream
 * \param weight      Weight stream (currently supports BinaryWeights or FixedPointWeights)
 * \param reps        Number of time the function has to be repeatedly executed (e.g. number of images)
 */

//#define DEBUG

template<
  unsigned MatrixW, unsigned MatrixH, long unsigned SIMD, long unsigned PE,
  typename TW, typename TI, typename TO
>
void Matrix_Vector_Activate_Stream_Vector_Batch(
  hls::stream<hls::vector<TI, SIMD>> &in, 
  hls::stream<hls::vector<TO, PE>> &out,
  TW const & weights,
  int const  reps
) {
#pragma HLS pipeline II=1 style=flp

  // how many different rows each neuron will compute
  // alternatively: number of vertical matrix chunks
  unsigned const NF = MatrixH / PE;

  // how many synapse groups each row is split into
  // alternatively: number of horizontal matrix chunks
  unsigned const SF = MatrixW / SIMD;

  // input vector buffers
  hls::vector<TI, SIMD> inputBuf[SF];
#pragma HLS ARRAY_PARTITION variable=inputBuf complete

  constexpr unsigned MMV = 1;
  //TO accu[MMV][PE];
  TO accu[MMV][PE];
  //decltype(activation.init(0,0))  accu[1][PE];
#pragma HLS ARRAY_PARTITION variable=accu complete dim=0
  unsigned nf = 0;
  unsigned sf = 0;
  unsigned tileCtr = 0; // invariant: tile = nf*SF + sf

  // everything merged into a common iteration space (one "big" loop instead
  // of smaller nested loops) to get the pipelinening the way we want
  unsigned const TOTAL_FOLD = NF * SF;
  std::cout << "!!MatrixH:" << MatrixH << ", MatrixW:" << MatrixW << ", SIMD:" << SIMD << ", PE: " << PE << ", NF:" << NF << ", SF:" << SF <<", mmv: " << MMV << ", TI: " << typeid(TI).name() << ", TW: " << typeid(TW).name() << std::endl;
  for(unsigned  i = 0; i < reps; i++) {
    for(unsigned  tile = 0; tile < TOTAL_FOLD; tile++) {
      hls::vector<TI, SIMD> inElem;

      if(nf == 0) {
        // read input from stream
        //std::cout << "reading input stream, nf= "<< nf << std::endl;
        inElem = in.read();
        // store in appropriate buffer for reuse
        inputBuf[sf] = inElem;
  #ifdef DEBUG
        std::cout << std::endl << "read from input stream ";
        for(int k = 0; k < SIMD; k++)
          std::cout << inElem[k] << ",";
  #endif
      }
      else {
        // reuse buffered input
        inElem = inputBuf[sf];
  #ifdef DEBUG
        std::cout << std::endl << "read from buffer ";
        for(int k = 0; k < SIMD; k++)
          std::cout << inElem[k] << ",";
  #endif
      }

      decltype(weights[tile]) w;
  #ifdef DEBUG
      std::cout << "reading from weight stream: ";
  #endif
      w = weights[tile];


      // Threshold Initialisation
      if(sf == 0) {
        for(unsigned pe = 0; pe < PE; pe++) {
  #pragma HLS UNROLL
          accu[0][pe] = 0;
        }
      }

      // iterates over one tile
      for(unsigned pe = 0; pe < PE; pe++) {
          //auto const wgt = w[pe];
        //TW wgt[SIMD] = w[pe]; 
  #pragma HLS UNROLL
        for (unsigned mmv = 0; mmv < MMV; mmv++) {
          TO res = accu[mmv][pe]; 
          //std::cout << "accu[mmv]["<< pe << "] read: " << accu[mmv][pe] << std::endl;
          for(int s = 0; s < SIMD; s++) { 
            TO resForOutput = res;
            //res += wgt * inputBuf[sf][s];
            volatile auto t1 = w[s*PE+pe];
            volatile auto t2 = inputBuf[sf][s];
            res += w[s*PE+pe] * inputBuf[sf][s]; // weight inside tile,pe*SIMD+s
  //#ifdef DEBUG
            std::cout << "MVAU pe=" << pe << ": " << w[s*PE+pe] << "*" << inputBuf[sf][s] << "+" << resForOutput << "=" << res  << " (w["<< s*PE+pe << "]), " <<  w[s*PE+pe] << "*" << inputBuf[sf][s]  << "=" << w[s*PE+pe] * inputBuf[sf][s] << std::endl;
  //#endif
          }

          accu[mmv][pe] = res;
          //std::cout << res << " written " << accu[mmv][pe] << " to accu[mmv][" << pe << "]" << std::endl;
        }
      }
      // keep track of which folded synapse/neuron we are processing
      ++tileCtr;
      hls::vector <TO, PE> vecOut;
      if(++sf == SF) {
        // produce output and clear accumulators
        // auto  outElem = TDstI().template operator()<TO>();
        for (unsigned  pe = 0; pe < PE; pe++) {
  #pragma HLS UNROLL
          for (unsigned mmv = 0; mmv < MMV; mmv++) {
  #pragma HLS UNROLL
            vecOut[pe] = accu[mmv][pe];
  //#ifdef DEBUG
            std::cout << " written to outputstream: accu[mmv][" << pe << "]: " << vecOut[pe] <<  std::endl;
  //#endif
            //vecOut[pe]++; // force wrong output for debugging
          }
        }

        out.write(vecOut);
        // next folded neuron or image
        sf = 0;
        if(++nf == NF) {
        nf   = 0;
        tileCtr = 0;
        }
      }
    }
  }
}

template<
  unsigned MatrixW, unsigned MatrixH, long unsigned SIMD, long unsigned PE,
  typename TW, typename TI, typename TO
>
void Matrix_Vector_Activate_Stream_Vector_Batch(
  hls::stream<hls::vector<TI, SIMD>> &in, 
  hls::stream<hls::vector<TO, PE>> &out,
  hls::stream<hls::vector<TW, SIMD*PE>> &weights,
  int const  reps
) {
#pragma HLS inline
  Matrix_Vector_Activate_Stream_Vector_Batch<MatrixW, MatrixH>(in, out, WeightsDecoupled<TW, SIMD*PE>(weights), reps);
}

#endif
