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

#include "mvau_top.h"


using namespace hls;
using namespace std;

void getWeightsIndex(unsigned weightsIndices[PE*SIMD], int tileNumber, int matrixH, int SIMD, int PE) {
	// matrix is transposed, matrixH is width
	int weightsIndex = 0;
    int TX = matrixH/PE; // number of tiles X

    int offsetX = (tileNumber*PE)%matrixH; // access correct weight on the X axis, tileNumber%TX: reset after end of line is reached
    int offsetY = ((int(tileNumber/TX)))*matrixH*SIMD; // (int)tileNumber/TX= line on Y axis on matrix, mul by matrixH*PE gives weight number
    int tileOffset = offsetX + offsetY;
    //std::cout << " tOffX:" << offsetX;
    //std::cout << " tOffY:" << offsetY;
    //std::cout << " tOff:" << tileOffset;
    for (int s = 0; s < SIMD; s++) { // vertically
        for (int p = 0; p < PE; p++) { // horizontally
            weightsIndex = s*matrixH+p+tileOffset;
            weightsIndices[s*PE+p] = weightsIndex;
            //std::cout << " w: " << (weightsIndex);
        }
    }
}

int main()
{
    static_assert((MatrixH % PE == 0), "");
	static_assert((MatrixW % SIMD == 0), "");

	static IDT IMAGE[MatrixW];
	static ODT EXPECTED[MatrixH]; 
	static ODT PRODUCED[MatrixH];

	hls::stream<hls::vector<IDT, SIMD>> input_stream;
	hls::stream<hls::vector<WDT, SIMD*PE>> weight_stream;
	hls::stream<hls::vector<ODT, PE>> output_stream;

	// generate image of size MatrixW and write it into SIMD-sized vectors
	unsigned int counter = 0;
	for (unsigned i = 0; i < MatrixW / SIMD; i++) {
		hls::vector<IDT,SIMD> vecIn; 
		for(unsigned int j = 0; j < SIMD; j++)
		{
			//IDT input = (counter+1)%2;
			IDT input = rand()%2;
			IMAGE[i*(SIMD)+j]= input;
			//std::cout << "input=" << input << ", counter=" << (counter+1)%2 << std::endl;
			vecIn[j] = input;
			counter++;
		}
		input_stream.write(vecIn);
	}

	// weights
	static WDT WEIGHTS[MatrixH*MatrixW];

	// generate weights
	//std::cout << "weights: " << std::endl;
	for(int i = 0; i < MatrixW; i++) {
		for(int j = 0; j < MatrixH; j++) {
			//WEIGHTS[i*MatrixH+j] = (i*MatrixH+j);
			WEIGHTS[i*MatrixH+j] = rand()%2;
			//std::cout << i*MatrixH+j << ", ";
		}
	}

    std::cout << std::endl;
	// tiling
	constexpr int TX = MatrixH / PE; // using transposed matrix
	constexpr int TY = MatrixW / SIMD; // using transposed matrix

	for (int tx = 0; tx < TX; tx++) {
        for (int ty = 0; ty < TY; ty++) {
            unsigned weightsIndices[PE*SIMD];
            getWeightsIndex(weightsIndices, ty * TX + tx, MatrixH, SIMD, PE);
            hls::vector<WDT, PE*SIMD> weigthsVecIn;
			for(int i = 0; i < PE*SIMD; i++) {
				weigthsVecIn[i] = WEIGHTS[weightsIndices[i]]; // get weight at index
				//std::cout << "TB: wrote weight " << WEIGHTS[weightsIndices[i]] << " to stream, index is: " << weightsIndices[i] << std::endl;
			}
			weight_stream.write(weigthsVecIn);
        }
    }

	// generate dummy weight streams for L2, L3
	

	
	Testbench_mvau(input_stream, output_stream, weight_stream);
	//std::cout << "Testbench_mvau done" << std::endl;

	// initialization
	for(int i = 0; i < MatrixH; ++i) {
		EXPECTED[i]=0;
	}

	for (int i=0;i<MatrixH;i++) { 
		for (int j=0;j<MatrixW;j++) { 
			//ODT(EXPECTED[i])+=IDT(IMAGE[j])*WDT(WEIGHTS[j*MatrixH+i]);
			EXPECTED[i] += IMAGE[j] * WEIGHTS[j*MatrixH+i];
			//std::cout << "TB: " << WEIGHTS[j*MatrixH+i] << " * " << IMAGE[j]  << " = " << EXPECTED[i] << std::endl;
		}
	}

	// temp outputs
	/*
	std::cout << "IMAGE: " << std::endl;
	for(int i = 0; i < MatrixW; i++) {
		std::cout << IMAGE[i] << " ";
	}

	std::cout << std::endl << "WEIGHTS:";
	for(int i = 0; i < MatrixW; i++) {
		std::cout << std::endl;
		for(int j = 0; j < MatrixH; j++) {
			std::cout << WEIGHTS[i*MatrixH+j] << ",";
		}
	}
	std::cout << std::endl;
	
	std::cout << std::endl;
	std::cout << "EXPECTED:" << std::endl;
	for (int i=0;i<MatrixH;i++) {
		std::cout << EXPECTED[i] << ",";
	}	
	std::cout << std::endl;
*/
	std::cout << "PRODUCED:" << std::endl;
	for (int i=0;i<MatrixH/PE;i++) {
		hls::vector<ODT, PE> prod = output_stream.read();
		for(int j = 0; j < PE; j++) {
			std::cout << prod[j] << ",";
			PRODUCED[i*PE+j] = prod[j];
		}
	}

	std::cout << std::endl;
	
	// verification
	bool errorOccured = false;
	for(int i = 0; i < MatrixH; i++) {
		if(PRODUCED[i] != EXPECTED[i] ){
			std::cerr << "Error! Produced: " << PRODUCED[i] << ", expected; " << EXPECTED[i] << std::endl;
			errorOccured = true;
		}
	}
	
	if(errorOccured) {
		std::cerr << std::endl<< std::endl <<"Test NOT passed! Parameters: MatrixH=" << MatrixH << ", MatrixW=" << MatrixW<< ", SIMD=" << SIMD<< ", PE=" << PE << std::endl<< std::endl << std::endl;
		return 1;
	}
	else
	{
		std::cout << "Test passes with success" << std::endl;
		std::cout <<std::endl << std::endl << std::endl << std::endl;
		return 0;
	}
}


