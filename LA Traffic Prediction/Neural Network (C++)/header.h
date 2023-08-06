//+++++++++++++++++++++++++++++++++++++++++++++++
// NEURAL NETWORK
// CODE IS PROPERTY OF AYUSH SAHA (12/18/2022)
// DO NOT DISTRIBUTE
//+++++++++++++++++++++++++++++++++++++++++++++++


#pragma once

#include<random>
#include<map>
#include<string>

#define batchsize 10
#define numfeats 3
#define numLayers 3
#define num_epochs 60000
#define numOutputNeurons 1
#define learning_rate 0.001


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// MODIFIABLE VARIABLES

//layer.cpp
// layer::dist(_, _)
// backprop routine: cost calculation:
//		cost = (2.0 * (y_pred - y_true));
//		AKA: iptr->dcda[j] = (2.0 * (iptr->a[i][j] - layer::y_true[i][j]));

//network.cpp
// layer::gen.seed(_)

//header.h
// numfeats = _
// numLayers = _
// numOutputNeurons = _

//normalizer.cpp
// float normstart, normrange = _, _

//main.cpp
// int layer_sizes[numLayers] = { numfeats , _, _, numOutpuNeurons };
// std::string layer_activations[numLayers] = { "_", "_", "_", "N/A" };
// learning_rate = _

//network.cpp
// minimal_cost_decline = _

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// matrix math

void mat_times_mat_T(float**& x, float**& y, float**& storage, \
	int numRowsA, int numColsA, int numRowsB);
void mat_T_times_mat(float**& x, float**& y, float**& storage, \
    int numRowsA, int numColsA, int numColsB);
void mat_times_scaler(float**& x, float y, float**& storage, int numR, int numC);
void matmul_elemwise(float**& x, float**& y, float**& storage, int numR, int numC);
void deriv_relu(float**& a, float**& storage, int numR, int numC);
void deriv_sigmoid(float**& a, float**& storage, int numR, int numC);
void relu(float& a);
void sigmoid(float& a);


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class Network;
class layer;
class Normalizer;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// LAYER CLASS
class layer {
private:
	int numNeurons, numNeuronsNext;
	layer* next; layer* prev;

    // matrices
    float** a;		// [batchsize][numNeurons]
	float** w;		// [numNeurons][numNeuronsNext]
	float** b;		// [batchsize][numNeuronsNext]

	float** dcdw;		// w size
	float** dcdb;		// b size

    float** dcda; float** dcdz; float** dadz;		// a size

	std::string activation;			// specifies which activation to apply in forward prop.

public:
	// static members
	static std::uniform_real_distribution<float> dist;      // for random generator
	static std::default_random_engine gen;

	// activation function maps
	static std::map<std::string, void(*)(float& a)> activmap;		// scalar = relu(scalar)
    static std::map<std::string, void(*)(float**& a, float**& storage, \
        int numR, int numC)> deriv_activmap;						// matrix = de_relu(matrix)

	// INITIALIZATION
	friend class Normalizer; friend class Network;
	layer(int numNeurons);
	friend void link(layer* ptr1, layer* ptr2);
	void assign_activation(std::string activation) { this->activation = activation; }

    void initMatrices();
	void randomizeWB();
	int getNumNeurons() { return numNeurons; }

	// FORWARD/ BACKWARD PROP.
    friend void forward(layer* ptr, float**& input);
	friend void showpredictions(layer* ptr, float**& supposedOutput);

	friend float get_cost(layer* ptr, float**& y_true);
    friend float backward(layer* ptr, float**& supposedOutput);
    void changeWB();
};




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// NETWORK CLASS
// member var: static array of layer*
class Network {
private:
	layer* layers[numLayers];		// array of ptrs to layer objects
	float** xdataAll; float** ydataAll;		// giant matrices to store all data
	float** xbatch; float** ybatch;			// small matrices to store part of all data

	int numbatches; int numsamples;
	int layer_sizes[numLayers];			// array of ints representing layer sizes

	Normalizer* norm;				// ptr to normalizer obj (used for denormalizing/displaying output nicely)
public:
	Network(int layer_sizes[numLayers]);
	void init_network_matrices();
	// input x, y data
	void get_data(std::string filename, int numsamples);
	void replace_batch(int batchstart);
	void shuffle_data();
	void init_activations(std::string layer_activations[numLayers]);
	void get_normalizer(Normalizer& norm);

	// train/test network (returns the mean cost)
	float train();
	float test();

	// replace xdataAll, ydataAll matrices with new dataset
	void replace_data(std::string filename, int numsamples_);
	void showoutput(); float get_MAE();
};

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// Normalizer
class Normalizer {
private:
	float mins_x_cols[numfeats];
	float x_col_ranges[numfeats];
	float y_mins[numOutputNeurons];
	float y_ranges[numOutputNeurons];
	float normstart, normrange;
public:
	Normalizer();
	int init_normalization_x_y(std::string filename);
	void denormalize_y(layer* a, float**& y_true);
	void normalize_datafile(std::string filename, std::string newfilename);
	void mat_to_file(float newdata[batchsize][numfeats + 1], std::ofstream& wfile);

	void norm_func(float& i, float& min, float& range);

	int countRows(std::string filename);
};


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// general funcs

void resizeVector(float*& mat, int numElems);
void resizeMatrix(float**& mat, int numRows, int numCols);
void freeMatrix(float**& mat, int numRows);
void getRandomDoubles(std::uniform_real_distribution<float>& dist,
	std::default_random_engine& gen, float*& storage, int numElems);
void displayMat(float**& dataAll, int numRows, int numCols, std::string title = "");
void displayMat(const float**& dataAll, int numRows, int numCols, std::string title = "");
void displayVect(float*& vec, int numElems, std::string title = "");
void copyMat(float**& dest, float**& src, int numRows, int numCols);

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// dataset generation

void write_data_headers(std::ofstream& wfile);
void write_data(std::ofstream& wfile, float*& xdata, float& ydata, \
	std::uniform_real_distribution<float>& mydist, std::default_random_engine& mygen, \
	float*& x_coeffs, int numsamples);
void gen_datafiles(std::string newfilename, std::string filename_test, int numsamples_train = 100, int numsamples_test = 100);
void one_hot_encode_ydata(float ydata, std::string& OHE_ydata, float category_ranges[numOutputNeurons + 1]);
