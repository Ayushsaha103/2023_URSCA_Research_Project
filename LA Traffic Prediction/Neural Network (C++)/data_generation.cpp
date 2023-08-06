
#include "header.h"
#include<string>
#include<iostream>
#include<fstream>
#include<sstream>
#include<random>

// categorical split ranges
// (used if NN is categorical)
float category_ranges[numOutputNeurons + 1] = {};// { -999999, 1000, 999999 };		// categorical split ranges


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// write appropriate column headers into csv (x1, x2, ... y)
void write_data_headers(std::ofstream& wfile) {
	// WRITE THE HEADERS
	std::string headers = "";
	for (int i = 0; i < numfeats; i++) {				// write the x features
		headers += "x" + std::to_string(i + 1) + ",";
	}
	for (int j = 0; j < numOutputNeurons; j++) {		// write the y feature (one-hot encoded)
		headers += "y" + std::to_string(j + 1) + ",";
	}
	headers[headers.size() - 1] = '\n';
	wfile << headers;
}

// one-hot-encode a value into format [1,0,0,...] based on category ranges
// (1=falls in category)
void one_hot_encode_ydata(float ydata, std::string& OHE_ydata) {
	for (int i = 1; i < numOutputNeurons+1; i++) {
		if (ydata > category_ranges[i-1] and ydata < category_ranges[i])
			OHE_ydata += "1,";
		else
			OHE_ydata += "0,";
	}
}

void write_data(std::ofstream& wfile, float*& xdata, float& ydata, \
	std::uniform_real_distribution<float>& mydist, std::default_random_engine& mygen, \
	float*& x_coeffs, int numsamples) {

	// variables for writing the str
	std::string line;

	// one-hot-encoded (categorical) variables
	std::string OHE_ydata;		// one_hot_encoded ydata (only for categorical prediction -> numOutputNeurons > 1)

	// RANDOMIZE X VALUES, CALCULATE CORRESPONDING Y, WRITE EACH LINE TO FILE
	for (int n = 0; n < numsamples; n++) {

		// generate data sample
		getRandomDoubles(mydist, mygen, xdata, numfeats);
		ydata = 0;
		std::ostringstream ostream;
		for (int j = 0; j < numfeats; j++) {
			// compute ydata
			ydata += x_coeffs[j] * xdata[j];// +bvals[j];

			// send xdata value to string
			ostream << xdata[j] << ",";
			//std::cout << xdata[j] << "\t";
		}
		// send the calculated y value
		if (numOutputNeurons == 1) {
			ostream << ydata;
			// convert arr row to string
			line = (ostream.str());
		}
		else {
			OHE_ydata = "";
			one_hot_encode_ydata(ydata, OHE_ydata);
			line = (ostream.str());
			line += OHE_ydata;
		}


		line[line.size() - 1] = '\n';

		// write string to new data file
		wfile << line;
	}

}

// simulate data for train AND test datafiles
void gen_datafiles(std::string newfilename, std::string filename_test, int numsamples_train, int numsamples_test) {
	// open train file
	std::ofstream wfile(newfilename);

	// create xdata, ydata storage
	float* xdata; resizeVector(xdata, numfeats);
	float ydata;

	// init randomization variables
	std::uniform_real_distribution<float> mydist(-22.0, 56.0);
	std::default_random_engine mygen;
	mygen.seed(24);

	// WRITE THE HEADERS
	write_data_headers(wfile);

	// GENERATE RANDOM PATTERN OF x COEFF's
	float* x_coeffs; resizeVector(x_coeffs, numfeats);
	// float* bvals; resizeVector(x_coeffs, numfeats);
	for (int i = 0; i < numfeats; i++) {
		x_coeffs[i] = mydist(mygen);
		// bvals[i] = mydist(mygen);
	}

	// generate train data
	write_data(wfile, xdata, ydata, mydist, mygen, x_coeffs, numsamples_train);
	wfile.close();

	// generate test data
	std::ofstream wfile_test(filename_test);
	write_data_headers(wfile_test);
	write_data(wfile_test, xdata, ydata, mydist, mygen, x_coeffs, numsamples_test);
	wfile_test.close();

}
