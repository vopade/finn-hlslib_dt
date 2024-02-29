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
 *
 *  @file activations.hpp
 *
 *  Library of templated HLS classes for BNN deployment. 
 *  This file lists a set of classes used to implement  
 *  threshold memory in neural network. 
 *
 *  This project has received funding from the European Union's Framework
 *  Programme for Research and Innovation Horizon 2020 (2014-2020) under
 *  the Marie Skłodowska-Curie Grant Agreement No. 751339.
 *
 *******************************************************************************/

#ifndef ACTIVATIONS_HPP
#define ACTIVATIONS_HPP

//#include "interpret.hpp"
#include <hls_stream.h>
#include <functional>
#include <hls_vector.h>


namespace comp{
  using std::binary_function;

  template<typename input_type_1 = void, typename input_type_2 = void>
    struct greater;

  template<typename input_type_1 = void, typename input_type_2 = void>
    struct less;

  template<typename input_type_1 = void, typename input_type_2 = void>
    struct greater_equal;

  template<typename input_type_1 = void, typename input_type_2 = void>
    struct less_equal;

  template<typename input_type_1 = void, typename input_type_2 = void, typename result_type = void>
    struct add;	

  template<typename input_type_1 = void, typename input_type_2 = void, typename result_type = void>
    struct mul;	

  template<typename input_type_1 = void, typename input_type_2 = void, typename result_type = void>
    struct max;

  template<typename input_type_1, typename input_type_2>
    struct greater : public binary_function<input_type_1, input_type_2, ap_uint<1>> {
      ap_uint<1>
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a > b; }
    };

  template<typename input_type_1, typename input_type_2>
    struct less : public binary_function<input_type_1, input_type_2, ap_uint<1>> {
      ap_uint<1>
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a < b; }
    };

  template<typename input_type_1, typename input_type_2>
    struct greater_equal : public binary_function<input_type_1, input_type_2, ap_uint<1>> {
      ap_uint<1>
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a >= b; }
    };

  template<typename input_type_1, typename input_type_2>
    struct less_equal : public binary_function<input_type_1, input_type_2, ap_uint<1>> {
      ap_uint<1>
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a <= b; }
    };
	
  template<typename input_type_1, typename input_type_2, typename result_type>
    struct add : public binary_function<input_type_1, input_type_2, result_type> {
      result_type
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a + b; }
    };

  template<typename input_type_1, typename input_type_2, typename result_type>
    struct mul : public binary_function<input_type_1, input_type_2, result_type> {
      result_type
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a * b; }
    };

  template<typename input_type_1, typename input_type_2, typename result_type>
    struct max : public binary_function<input_type_1, input_type_2, result_type> {
      result_type
      operator()(const input_type_1& a, const input_type_2& b) const
      { return a > b ? a : b; }
    };
}

/**
 * General contract for activation functions.
 *
 * This class itself has no formal significance for the implementation
 * of the MVAU. Implementations of activation functions are encouraged
 * to implement it nonetheless to guarantee appropriate function
 * signatures.
 */
template<typename TA, typename TO>
class Activation {
public:
  TA init(__attribute__((unused)) unsigned const  nf, __attribute__((unused)) unsigned const  pe) const {
#pragma HLS inline
    return  TA(0);
  }

  /**
   * Compute the activation of the passed accumulator value accu in row idx.
   */
  TO activate(unsigned const  nf, unsigned const  pe, TA const &accu) const;
};

/**
 * A no-op activation that simply outputs the computed accumulator
 * output as the final result.
 */
template<typename T>
class PassThroughActivation : public Activation<T, T> {
public:
  T activate(__attribute__((unused)) unsigned const  nf, __attribute__((unused)) unsigned const  pe, T const &accu) const {
#pragma HLS inline
    return  accu;
  }
};

/**
 * Use a simple global threshold comparison as activation function.
 *
 * The constant threshold is initialized at construction.
 * The default comparison returns true if the threshold value is
 * smaller than the passed accumulator value.
 */
template<typename TA, typename Compare = comp::less<TA, TA>>
class ThresholdActivation : public Activation<TA, bool> {
  TA const  m_threshold;
public:
  ThresholdActivation(TA const &threshold) : m_threshold(threshold) {
#pragma HLS inline
  }

public:
  bool activate(__attribute__((unused)) unsigned const  nf, __attribute__((unused)) unsigned const  pe, TA const &accu) const {
#pragma HLS inline
    return  Compare()(m_threshold, accu);
  }
};

/*!
 * Use a simple per-row threshold comparison as activation function.
 *
 * The thresholds are taken from an array indexed by output row.
 * It is currently public to allow direct initialization and
 * to make its name accessible for top-level HLS pragmas.
 *
 * The default comparison returns true if the threshold value defined for
 * the indexed row is smaller than the passed accumulator value.
 */
template<unsigned NF, unsigned PE, unsigned NumTH, 
	 typename TA, typename TR, int ActVal = 0, typename Compare = comp::less<TA, TA>>
class ThresholdsActivation {
public:
  TA m_thresholds[PE][NF][NumTH];
  
public:
  TA init(__attribute__((unused)) unsigned const  nf, __attribute__((unused)) unsigned const  pe) const {
#pragma HLS inline
    return  TA(0);
  }

public:
  TR activate(unsigned const  nf, unsigned const  pe,  TA const &accu) const {
#pragma HLS inline
    TR result=ActVal;
	for(unsigned int i=0; i< NumTH; i++){
#pragma HLS unroll
      result+=Compare()(m_thresholds[pe][nf][i], accu);
    }
    return result;
  }
};


/*!
 * \brief Use a simple activation function with per-row parameters.
 *
 * The parameters are taken from an array indexed by output row.
 * It is currently public to allow direct initialization and
 * to make its name accessible for top-level HLS pragmas.
 * 
 * \tparam NF    First dimension of the parameter matrix
 * \tparam PE    Second dimension of the parameter matrix
 * \tparam TI    DataType of input layer values
 * \tparam TP    DataType of parameters
 * \tparam TR    DataType of return values
 * \tparam Fxn   Function to be applied on the channel input value
 */

template<unsigned NF, unsigned PE,
   typename TI, typename TP, typename TR, typename Fxn = comp::mul<TI, TP, TR>>
class ChannelWiseOperation {
public:
  TP parameters[PE][NF];
public:
  TI init(__attribute__((unused)) unsigned const  nf, __attribute__((unused)) unsigned const  pe) const {
#pragma HLS inline
    return  TI(0);
  }
public:
  TR activate(unsigned const  nf, unsigned const  pe,  TI const &in) const {
#pragma HLS inline
    TR result = Fxn()(parameters[pe][nf], in);
    return result;
  }
};





/**
 * \brief Matrix vector activate function with streaming weights
 *
 * The function performs the multiplication between a weigth matrix, presnted as an input stream, and the input activation vector,
 * accumulating the results and then applying an activation function on the accumulated result. Does not support MMV.
 *
 * 
 * \tparam MatrixW    Width of the input matrix
 * \tparam MatrixH    Heigth of the input matrix
 * \tparam SIMD       Number of input columns computed in parallel
 * \tparam PE         Number of output rows computed in parallel
 * \tparam TSrcI      DataType of the input activation (as used in the MAC)
 * \tparam TDstI      DataType of the output activation (as generated by the activation)
 * \tparam TWeightI   DataType of the weights and how to access them in the array
 * \tparam TW         DataType of the weights (as used in the MAC) - not deducible from the paramaters
 * \tparam TI         DataType of the input stream - safely deducible from the paramaters
 * \tparam TO         DataType of the output stream - safely deducible from the paramaters
 * \tparam TA         DataType of the activation class (e.g. thresholds) - safely deducible from the paramaters
 * \tparam R          Datatype for the resource used for FPGA implementation of the MAC  - safely deducible from the paramaters
 *
 * \param in          Input stream
 * \param out         Output stream
 * \param weight      Weight stream (currently supports BinaryWeights or FixedPointWeights)
 * \param activation  Activation class
 * \param reps        Number of time the function has to be repeatedly executed (e.g. number of images)
 * \param r           Resource type for the hardware implementation of the MAC block
 */

// testbench for separate TB
template<
  unsigned MatrixW, unsigned MatrixH, unsigned SIMD, unsigned PE, 
  typename TSrcI = Identity, typename TDstI = Identity, typename TWeightI = Identity, typename TW,
  typename TI, typename TO, typename TA, typename R
>
void Matrix_Vector_Activate_Stream_Batch_sepTB(hls::stream<TI> &in,
          hls::stream<TO> &out,
          hls::stream<ap_uint<PE*SIMD*TW::width>> &weight,
          TA  const &activation,
          int const  reps,
          R const &r) {

  // how many different rows each neuron will compute
  // alternatively: number of vertical matrix chunks
  unsigned const  NF = MatrixH / PE;

  // how many synapse groups each row is split into
  // alternatively: number of horizontal matrix chunks
  unsigned const  SF = MatrixW / SIMD;

  // input vector buffers
  TI  inputBuf[SF];
#pragma HLS ARRAY_PARTITION variable=inputBuf complete dim=1
  // accumulators
  decltype(activation.init(0,0))  accu[1][PE];
#pragma HLS ARRAY_PARTITION variable=accu complete dim=0
  // unpacked and packed buffers for weight stream
  Weights_Tile<SIMD, TW, PE > w;
#pragma HLS ARRAY_PARTITION variable=w.m_weights complete dim=0
  ap_uint<PE * SIMD * TW::width> W_packed;


  unsigned  nf   = 0;
  unsigned  sf   = 0;
  unsigned  tile = 0; // invariant: tile = nf*SF + sf
  
  // everything merged into a common iteration space (one "big" loop instead
  // of smaller nested loops) to get the pipelinening the way we want
  unsigned const TOTAL_FOLD = NF * SF;
  for(unsigned  i = 0; i < reps * TOTAL_FOLD; i++) {
#pragma HLS pipeline style=flp II=1
    TI  inElem;

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

    // read from the parameter stream
    W_packed = weight.read();
    for (unsigned pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
      w.m_weights[pe] = W_packed((pe+1)*SIMD*TW::width-1,pe*SIMD*TW::width);
    }

    // Threshold Initialisation
    if(sf == 0) {
      for(unsigned pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
      accu[0][pe] = activation.init(nf, pe);
      }
    }

    // compute matrix-vector product for each processing element
    for(unsigned  pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
      auto const  act = TSrcI()(inElem, 0);
      auto const  wgt = TWeightI()(w[pe]);
      //auto const  wgt = w[pe];
      accu[0][pe] = mac<SIMD>(accu[0][pe], wgt, act, r, 0);
    }

    // keep track of which folded synapse/neuron we are processing
    ++tile;
    if(++sf == SF) {
      // produce output and clear accumulators
      auto  outElem = TDstI().template operator()<TO>();
      for (unsigned  pe = 0; pe < PE; pe++) {
#pragma HLS UNROLL
      outElem(pe,0,1) = activation.activate(nf, pe, accu[0][pe]);
          }

      out.write(outElem);

      // next folded neuron or image
      sf = 0;
      if(++nf == NF) {
      nf   = 0;
      tile = 0;
      }
    }
  }
}

/*!
 * \brief Thresholding function for multiple images
 *
 * The function performs thresholds comparison with input activation vector, 
 * and generating output based on the comparison results
 *
 * \tparam ImgDim         Total spatial size of input feature map
 * \tparam NumChannels    Number of channels in input feature map
 * \tparam PE             Number of output rows computed in parallel
 * \tparam TI             DataType of the input stream - safely deducible from the paramaters
 * \tparam TO             DataType of the output stream - safely deducible from the paramaters
 * \tparam TA             DataType of the activation class (e.g. thresholds) - safely deducible from the paramaters
 *
 * \param in              Input stream
 * \param out             Output stream
 * \param activation      Activation class
 * \param reps            Number of time the function has to be repeatedly executed (e.g. number of images)
 */
template <
    unsigned ImgDim, unsigned NumChannels, unsigned PE,
    typename TI, typename TO, typename TA>
void Thresholding_Batch(hls::stream<hls::vector<TI,PE>> &in,
                        hls::stream<hls::vector<TO,PE>> &out,
                        TA const &activation,
                        int const reps)
{
#pragma HLS aggregate variable=out compact=bit
#pragma HLS aggregate variable=in compact=bit
  // how many different rows each neuron will compute
  // alternatively: number of vertical matrix chunks
  constexpr unsigned  NF = NumChannels / PE;

  // everything merged into a common iteration space (one "big" loop instead
  // of smaller nested loops) to get the pipelinening the way we want
  unsigned nf = 0;

  for (unsigned i = 0; i < reps * ImgDim * NF; i++) {
#pragma HLS pipeline style=flp II=1
    hls::vector<TI,PE> vecIn = in.read();
    hls::vector<TO,PE> vecOut;

    for (unsigned pe = 0; pe < PE; pe++){
#pragma HLS UNROLL
      vecOut[pe] = activation.activate(nf, pe, vecIn[pe]);
    }
    out.write(vecOut);
  
    if (++nf == NF){
      nf = 0;
    }
  }
}

/*!
 * \brief Thresholding function for multiple images, with streaming thresholds
 *
 * The function performs thresholds comparison with input activation vector, 
 * and generating output based on the comparison results
 *
 * \tparam ImgDim         Total spatial size of input feature map
 * \tparam NumChannels    Number of channels in input feature map
 * \tparam PE             Number of output rows computed in parallel
 * \tparam TSrcI          DataType of the input activation (as used in the MAC)
 * \tparam TDstI          DataType of the output activation (as generated by the activation)
 * \tparam ActVal         Initial value of activation at start of thresholding procedure
 * \tparam TT             DataType of the thresholds stream
 * \tparam NumSteps       Number of thresholds per activation
 * \tparam TI             DataType of the input stream - safely deducible from the paramaters
 * \tparam TO             DataType of the output stream - safely deducible from the paramaters
 *
 * \param in              Input stream
 * \param out             Output stream
 * \param weight          Weight stream
 * \param reps            Number of time the function has to be repeatedly executed (e.g. number of images)
 */
template <
    unsigned ImgDim, unsigned NumChannels, unsigned PE,
    typename TSrcI = Identity, typename TDstI = Identity,
    int ActVal=0, typename TT, unsigned int NumSteps,
    typename TI, typename TO>
void Thresholding_Stream_Batch(hls::stream<TI> &in,
                        hls::stream<TO> &out,
                        hls::stream<ap_uint<PE*NumSteps*TT::width>> &weight,
                        int const reps)
{

  // how many different rows each neuron will compute
  // alternatively: number of vertical matrix chunks
  unsigned const NF = NumChannels / PE;

  ThresholdsActivation<1, PE, NumSteps, TT, TO, ActVal, comp::less_equal<TT, TT>> internal_thr;
#pragma HLS ARRAY_PARTITION variable=internal_thr.m_thresholds complete dim=0

  // everything merged into a common iteration space (one "big" loop instead
  // of smaller nested loops) to get the pipelinening the way we want
  for (unsigned i = 0; i < reps * ImgDim * NF; i++)
  {
#pragma HLS pipeline style=flp II=1

    ap_uint<PE*NumSteps*TT::width> packed_thr;
    packed_thr = weight.read();
    // slicer to get 1 PE's worth of thresholds
    auto const pe_slicer = Slice<ap_uint<NumSteps*TT::width>>()(packed_thr);

    TI inElem;
    inElem = in.read();
    auto outElem = TDstI().template operator()<TO>();

    for (unsigned pe = 0; pe < PE; pe++)
    {
#pragma HLS UNROLL
      // slicer to get individual thresholds
      auto const thr_slicer = Slice<TT>()(pe_slicer(pe, 0));
      for (unsigned nt = 0; nt < NumSteps; nt++)
      {
#pragma HLS UNROLL
        internal_thr.m_thresholds[pe][0][nt] = thr_slicer(nt, 0);
      }

      auto const act = TSrcI()(inElem);
      outElem(pe,0,1) = internal_thr.activate(0, pe, act(pe,0));
    }
    out.write(outElem);
  }
}


/*!
 * \brief Thresholding function for multiple images, with streaming thresholds
 *
 * The function performs thresholds comparison with input activation vector, 
 * and generating output based on the comparison results
 *
 * \tparam dim             Total spatial size of input feature map
 * \tparam chn            Number of channels in input feature map
 * \tparam PE             Number of output rows computed in parallel
 * \tparam actVal         Initial value of activation at start of thresholding procedure
 * \tparam TT             DataType of the thresholds stream
 * \tparam stp            Number of thresholds per activation
 * \tparam TI             DataType of the input stream - safely deducible from the paramaters
 * \tparam TO             DataType of the output stream - safely deducible from the paramaters
 *
 * \param in              Input stream
 * \param out             Output stream
 * \param weight          Weight stream
 * \param reps            Number of time the function has to be repeatedly executed (e.g. number of images)
 */
template <
    unsigned dim, unsigned chn, unsigned PE,
    int actVal=0, typename TT, unsigned int stp,
    typename TI, typename TO>
void Thresholding_Stream_Batch_Vector(hls::stream<hls::vector<TI, PE>> &in,
                        hls::stream<hls::vector<TO, PE>> &out,
                        hls::stream<hls::vector<TT, PE*stp>> &wgt,
                        int const reps)
{
#pragma HLS pipeline style=flp II=1
  // how many different rows each neuron will compute
  // alternatively: number of vertical matrix chunks
  unsigned const NF = chn / PE;

  ThresholdsActivation<1, PE, stp, TT, TO, actVal, comp::less_equal<TT, TT>> internal_thr; 
#pragma HLS ARRAY_PARTITION variable=internal_thr.m_thresholds complete dim=0

  // everything merged into a common iteration space (one "big" loop instead
  // of smaller nested loops) to get the pipelinening the way we want
  for (unsigned i = 0; i < reps * dim * NF; i++)
  {
#pragma HLS pipeline style=flp II=1
    hls::vector<TI,PE> v_i = in.read();
    hls::vector<TO,PE> v_o;
    hls::vector<TT,PE*stp> w = wgt.read(); // get 1 PE's worth of thresholds

    for (unsigned pe = 0; pe < PE; pe++)
    {
#pragma HLS UNROLL
      // slicer to get individual thresholds
      for (unsigned nt = 0; nt < stp; nt++) 
      {
#pragma HLS UNROLL
        internal_thr.m_thresholds[pe][0][nt] = w[pe*stp+nt];
      }
      v_o[pe] = internal_thr.activate(0, pe, v_i[pe]);
    }
    out.write(v_o);
  }
}
#endif
