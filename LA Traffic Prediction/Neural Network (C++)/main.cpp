
//+++++++++++++++++++++++++++++++++++++++++++++++
// NEURAL NETWORK
// CODE IS PROPERTY OF AYUSH SAHA (12/18/2022)
// DO NOT DISTRIBUTE
//+++++++++++++++++++++++++++++++++++++++++++++++

#include "header.h"
#include<string>
#include<iostream>
#include<fstream>
#include<sstream>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

int main() {
	// GENERATE train AND test datafiles
	std::string filename = "data.csv";
	std::string filename_test = "datatest.csv";
	gen_datafiles(filename, filename_test, 600, 100);		// if we're generating our data

	// NORMALIZE train data
	std::string newfilename = "data_norm.csv";
	Normalizer myNorm;
	int numsamples = myNorm.init_normalization_x_y(filename);		// returns total # samples in dataset
	int numbatches = numsamples / batchsize;						// rounds down to get # batches
	myNorm.normalize_datafile(filename, newfilename);


	// INITIALIZE NETWORK n
	int layer_sizes[numLayers] = { numfeats , 5, numOutputNeurons };
	std::string layer_activations[numLayers] = { "relu","relu","relu"};

	Network n(layer_sizes);						// init layer sizes
	n.init_activations(layer_activations);		// init activations
	n.get_normalizer(myNorm);					// save normalizer (just for denormalization/display purposes)

	//+++++++++++++++++++++++++++++++++++++++
	// input train data
	n.get_data(newfilename, numsamples);
	n.shuffle_data();		// shuffle data

	// TRAIN NN
	float mincost = n.train();
	std::cout << "\n--------------------------\nMINIMAL COST AFTER TRAINING: " << mincost;
	n.showoutput();

	//+++++++++++++++++++++++++++++++++++++++
	// normalize test data
	std::string newfilename_test = "datatest_norm.csv";
	myNorm.normalize_datafile(filename_test, newfilename_test);

	// replace data stored in network
	int numsamples_test = myNorm.countRows(newfilename_test);
	n.replace_data(newfilename_test, numsamples_test);
	n.shuffle_data();

	// RUN TEST NN ROUTINE (displays predictions)
	n.test();
	return 0;
}


