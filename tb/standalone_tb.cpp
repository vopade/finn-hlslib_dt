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
 *  \file mvau_tb.cpp
 *
 *  Testbench for the MVAU HLS block
 *
 *****************************************************************************/

#include "standalone_top.h"
#include "standalone_data.h"

using namespace hls;
using namespace std;

template<unsigned numSteps, typename TO, typename TI, typename TT>
TO computeThresholds(TI in, TT thr[numSteps]) {
    std::sort(thr, thr + numSteps); 
    TO out = 0;
    for(int i = 0; i < numSteps; i++) {
        if(in >= thr[i]) {
            out = i+1;
        }
    }
    return out;
}

// only supports TI = TO
template<unsigned numThresholds, unsigned numSteps, typename TO, typename TI, typename TT>
void computeThresholdsForLayer(TI input[numThresholds], TT thresholds[numThresholds][numSteps]) {
	for(int i = 0; i < numThresholds; i++) {
		input[i] = computeThresholds<numSteps, TO, TI, TT>(input[i], thresholds[i]);
	}
}

template<unsigned PE, unsigned SIMD, unsigned matrixWidth> 
void tiling(unsigned weightsIndices[PE*SIMD], int tileNumber) {
	// matrix is transposed, matrixH is width
	int weightsIndex = 0;
    int TX = matrixWidth/PE; // number of tiles X

    int offsetX = (tileNumber*PE)%matrixWidth; // access correct weight on the X axis, tileNumber%TX: reset after end of line is reached
    int offsetY = ((int(tileNumber/TX)))*matrixWidth*SIMD; // (int)tileNumber/TX is line on Y axis on matrix, mul by matrixH*PE gives weight number
    int tileOffset = offsetX + offsetY;
    for (int s = 0; s < SIMD; s++) { // vertically
        for (int p = 0; p < PE; p++) { // horizontally
            weightsIndex = s*matrixWidth+p+tileOffset;
            weightsIndices[s*PE+p] = weightsIndex;
        }
    }
}

template<typename TW>
void generateWeights(int sizeThisLayer, int sizeNextLayer, TW* weights) {
	for(int i = 0; i < sizeThisLayer; i++) {
		for(int j = 0; j < sizeNextLayer; j++) {
			weights[i*sizeNextLayer+j] = rand()%250;
		}
	}
}

template<typename TI, typename TO, typename TW>
void computeLayer(int sizeThisLayer, int sizeNextLayer, TO* res, TI* in, TW* weights) {
	std::cout << std::endl << std::endl << "TB MVAU output:" << std::endl;
	for (int i=0;i<sizeNextLayer;i++) { 
		res[i] = 0;
		for (int j=0;j<sizeThisLayer;j++) { 
			res[i] += in[j] * weights[j*sizeNextLayer+i];
		}
		std::cout << res[i]  << ", ";
	}
}

template<unsigned PE, typename TT, unsigned numSteps, unsigned numThresholds>
void writeThresholdStream(hls::stream<hls::vector<TT, PE*numSteps>> & stream, TT thresholdingParams[numThresholds][numSteps]) {
	for(int i = 0; i < numThresholds; i++) {
		hls::vector<TT, PE*numSteps> actual_thresholds;
		for(int j = 0; j < PE*numSteps; j++) {
			actual_thresholds[j] = thresholdingParams[i][j];
		}
		stream.write(actual_thresholds);
	}
}

template<unsigned sizeThisLayer, unsigned sizePrevLayer, typename WDT, unsigned PE, unsigned SIMD>
void writeWeightStream(WDT weights[sizePrevLayer*sizeThisLayer], hls::stream<hls::vector<WDT, SIMD*PE>> & weight_stream) {
	constexpr int TX = sizeThisLayer / PE; // using transposed matrix
	constexpr int TY = sizePrevLayer / SIMD; // using transposed matrix
	int counter = 0;
	for (int tx = 0; tx < TX; tx++) {
        for (int ty = 0; ty < TY; ty++) {
            unsigned weightsIndices[PE*SIMD];
            tiling<PE, SIMD, sizeThisLayer>(weightsIndices, ty * TX + tx);
            hls::vector<WDT, PE*SIMD> weigthsVecIn;
			for(int i = 0; i < PE*SIMD; i++) {
				weigthsVecIn[i] = weights[weightsIndices[i]]; // get weight at index
			}
			weight_stream.write(weigthsVecIn);

        }
    }
	std::cout << std::endl << std::endl;
}

void generateInput(TI image[MatrixW], unsigned SIMD, unsigned limit) {
	TI sub = 0.15; // used to create non-integers if TI != integer
	for (unsigned i = 0; i < MatrixW / SIMD; i++) {
		for(unsigned int j = 0; j < SIMD; j++) {
			image[i*(SIMD)+j] = TI(rand()%limit)-sub; 
		}
	}
}

template<unsigned SIMD>
void writeInputStream(TI image[MatrixW], hls::stream<hls::vector<TI, SIMD>> & input_stream) {
	for (unsigned i = 0; i < MatrixW / SIMD; i++) {
		hls::vector<TI,SIMD> vecIn; 
		for(unsigned int j = 0; j < SIMD; j++) {
			vecIn[j] = image[i*(SIMD)+j];
		}
		input_stream.write(vecIn);
	}
}

int main() {
    static_assert((HIDDEN1 % PE1 == 0), "");
	static_assert((INPUTSIZE % SIMD1 == 0), "");

	TI image[MatrixW];
	TO1 expected1[MatrixH]; 
	TO2 expected2[HIDDEN2]; 
	TO3 expected3[HIDDEN3]; 
	TO4 expected4[NUMCLASSES]; 
	TO4 produced[NUMCLASSES];

	hls::stream<hls::vector<TI, SIMD1>> input_stream;
	hls::stream<hls::vector<TW1, SIMD1*PE1>> weight_stream1("weight_stream1");
	hls::stream<hls::vector<TW2, SIMD2*PE2>> weight_stream2("weight_stream2");
	hls::stream<hls::vector<TW3, SIMD3*PE3>> weight_stream3("weight_stream3");
	hls::stream<hls::vector<TW4, SIMD4*PE4>> weight_stream4("weight_stream4");

	hls::stream<hls::vector<TT, PE_THR*NUM_STEPS1>> thr_stream1("thr_stream1"); // PE must be 1, because thresoldig assumes that if chn=1
	hls::stream<hls::vector<TT, PE_THR*NUM_STEPS2>> thr_stream2("thr_stream2");
	hls::stream<hls::vector<TT, PE_THR*NUM_STEPS3>> thr_stream3("thr_stream3");
	hls::stream<hls::vector<TT, PE_THR*NUM_STEPS4>> thr_stream4("thr_stream4");
	hls::stream<hls::vector<TO4, PE4>> output_stream("output_stream");

	generateInput(image, SIMD1, 500);
	writeInputStream<SIMD1>(image, input_stream);
	writeWeightStream<MatrixH, MatrixW, TW1, PE1, SIMD1>(weights1, weight_stream1);
	writeWeightStream<HIDDEN2, HIDDEN1, TW2, PE2, SIMD2>(weights2, weight_stream2);
	writeWeightStream<HIDDEN3, HIDDEN2, TW3, PE3, SIMD3>(weights3, weight_stream3);
	writeWeightStream<NUMCLASSES, HIDDEN3, TW4, PE4, SIMD4>(weights4, weight_stream4);
	writeThresholdStream<PE_THR, TT, NUM_STEPS1, NUM_THRESHOLDS1>(thr_stream1, thresholdingParams1); // PE_THR must be 1, because thresoldig assumes that if chn=1
	writeThresholdStream<PE_THR, TT, NUM_STEPS2, NUM_THRESHOLDS2>(thr_stream2, thresholdingParams2);
	writeThresholdStream<PE_THR, TT, NUM_STEPS3, NUM_THRESHOLDS3>(thr_stream3, thresholdingParams3);
	writeThresholdStream<PE_THR, TT, NUM_STEPS4, NUM_THRESHOLDS4>(thr_stream4, thresholdingParams4);

	Testbench_standalone(input_stream, output_stream, weight_stream1, weight_stream2, weight_stream3, weight_stream4, thr_stream1, thr_stream2, thr_stream3, thr_stream4, 1);

	computeLayer(MatrixW, MatrixH, expected1, image, weights1); 
	computeThresholdsForLayer<NUM_THRESHOLDS1,NUM_STEPS1,TO1,TO1,TT>(expected1, thresholdingParams1);
	computeLayer(HIDDEN1, HIDDEN2, expected2, expected1, weights2);
	computeThresholdsForLayer<NUM_THRESHOLDS2,NUM_STEPS2,TO2,TO2,TT>(expected2, thresholdingParams2);
	computeLayer(HIDDEN2, HIDDEN3, expected3, expected2, weights3);
	computeThresholdsForLayer<NUM_THRESHOLDS3,NUM_STEPS3,TO3,TO3,TT>(expected3, thresholdingParams3);
	computeLayer(HIDDEN3, NUMCLASSES, expected4, expected3, weights4);
	computeThresholdsForLayer<NUM_THRESHOLDS4,NUM_STEPS4,TO4,TO4,TT>(expected4, thresholdingParams4);
	
	// read results from MVAU
	std::cout << "produced:" << std::endl;
	for (int i=0;i<NUMCLASSES/PE4;i++) {
		hls::vector<TO4, PE4> prod = output_stream.read();
		for(int j = 0; j < PE4; j++) {
			std::cout << prod[j] << ",";
			produced[i*PE4+j] = prod[j];
		}
	}
	std::cout << " expected:" << expected4[0] << std::endl;
	
	// verification
	bool errorOccured = false;
	for(int i = 0; i < NUMCLASSES; i++) {
		if(produced[i] != expected4[i]) {
			std::cerr << "Error! Produced: " << produced[i] << ", expected; " << expected4[i] << std::endl;
			errorOccured = true;
		}
	}
	
	if(errorOccured) {
		std::cerr << std::endl<< std::endl <<"Test NOT passed! Parameters: HIDDEN1=" << HIDDEN1 << ", INPUTSIZE=" << INPUTSIZE<< ", SIMD=" << SIMD1 << ", PE=" << PE1 << std::endl<< std::endl << std::endl;
		return 1;
	}
	else {
		std::cout << std::endl << "Test passes with success" << std::endl;
		std::cout <<std::endl << std::endl << std::endl << std::endl;
		return 0;
	}
}