
#include "header.h"

#include<cstring>
#include<iostream>
#include<string>
#include<random>
#include<map>


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// STATIC MEMBERS (y_true, random generator)
std::uniform_real_distribution<float> layer::dist(-0.5, 0.5);
std::default_random_engine layer::gen;

std::map<std::string, void(*)(float& a)> layer::activmap;
std::map<std::string, void(*)(float**& a, float**& storage, \
    int numR, int numC)> layer::deriv_activmap;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// LAYER INITIALIZATION

// create layer by specifying numNeurons
layer::layer(int numNeurons) : numNeurons(numNeurons) {
	layer::activmap["relu"] = relu;
	layer::activmap["sigmoid"] = sigmoid;

	layer::deriv_activmap["relu"] = deriv_relu;
	layer::deriv_activmap["sigmoid"] = deriv_sigmoid;
}

// link 2 layers
// layer 1's numNeuronsNext = layer 2's numNeurons
void link(layer* ptr1, layer* ptr2) {
	ptr1->next = ptr2;
	ptr2->prev = ptr1;
	ptr1->numNeuronsNext = ptr2->numNeurons;
	ptr2->numNeuronsNext = 0;
}

// initialize all matrices & partial terms
// w, a, b, dcda, dadz, dcdz, dcdw, dcdb
void layer::initMatrices() {
	resizeMatrix(a, batchsize, numNeurons);
	resizeMatrix(w, numNeurons, numNeuronsNext);
	resizeMatrix(b, batchsize, numNeuronsNext);

	resizeMatrix(dcdw, numNeurons, numNeuronsNext);
	resizeMatrix(dcdb, batchsize, numNeuronsNext);
    randomizeWB();

	resizeMatrix(dcda, batchsize, numNeurons);
	resizeMatrix(dcdz, batchsize, numNeurons);
	resizeMatrix(dadz, batchsize, numNeurons);
}

// fill w and b with random values
void layer::randomizeWB() {
    for (int i = 0; i < numNeurons; i++) {
        getRandomDoubles(dist, gen, w[i], numNeuronsNext);
        //for (int j = 0; j < numNeuronsNext; j++)
        //    w[i][j] = 1;
    }
    for (int i = 0; i < batchsize; i++) {
        //for (int j = 0; j < numNeuronsNext; j++)
        //    b[i][j] = 1;
        getRandomDoubles(dist, gen, b[i], numNeuronsNext);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// FORWARD PROP
// 
//	  output
//		a2
//		^			<~~activation
//		z2
//		^
// a1 * w1 + b1
//		^			<~~activation
//		z1
//		^
// a0 * w0 + b0
//		^
//	  input

// display all predictions for current batch
// display: a matrix     supposed y_true matrix
void showpredictions(layer* ptr, float**& supposedOutput) {
	std::cout << "\n\nPredictions\ny_pred\t\ty_true\n";
	for (int i = 0; i < batchsize; i++) {
		for (int j = 0; j < numOutputNeurons; j++)
			std::cout << ptr->a[i][j] << "\t" << supposedOutput[i][j] << "\t\t";
		std::cout << "\n";
	}
	std::cout << "\n";
}

// perform computations (nexta = relu(a * w + b))
void forwardMath(float**& a, float**& w, float**& b, float**& nexta, \
    std::string& activation, int numRowsA, int numColsA, int numColsW) {

    for (int i = 0; i < numRowsA; i++) {
        for (int j = 0; j < numColsW; j++) {
            // nexta = a . w
            nexta[i][j] = 0;
            for (int k = 0; k < numColsA; k++) {
                nexta[i][j] += a[i][k] * w[k][j];
            }
            // nexta += b
            nexta[i][j] += b[i][j];
            // nexta = relu(nexta)
            layer::activmap[activation]\
                (nexta[i][j]);
        }
    }
}

// FORWARD PROP DRIVER ROUTINE
void forward(layer* ptr, float**& input) {
    // a = input (passed parameter)
    copyMat(ptr->a, input, batchsize, ptr->numNeurons);
    
    // iterate thru network, applying same procedure
    while (ptr->next != nullptr) {
        forwardMath(ptr->a, ptr->w, ptr->b, ptr->next->a, ptr->activation, \
            batchsize, ptr->numNeurons, ptr->numNeuronsNext);

        ptr = ptr->next;
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// BACKWARD PROP.

// change w & b matrices by whatever's stored in dcdw, dcdb
void layer::changeWB() {
    // w -= learning_rate * dcdw
    for (int i = 0; i < numNeurons; i++) {      // changes to w
        for (int j = 0; j < next->numNeurons; j++) {
            w[i][j] -= learning_rate * dcdw[i][j];
        }
    }
    // b -= learning_rate * dcdb
    for (int i = 0; i < batchsize; i++) {       // changes to b
        for (int j = 0; j < next->numNeurons; j++) {
            b[i][j] -= learning_rate * dcdb[i][j];
        }
    }
}
// return avg cost of network predictions (compared w/ y_true)
float get_cost(layer* ptr, float**& y_true) {
    float avg_cost = 0;
    for (int i = 0; i < batchsize; i++) {
        for (int j = 0; j < ptr->numNeurons; j++) {
            // ptr->dcda[i][j] = 2.0 * (ptr->a[i][j] - y_true[i][j]);
            avg_cost += (ptr->a[i][j] - y_true[i][j]);
        }
    }
    avg_cost /= ptr->numNeurons;
    return avg_cost;
}

// BACKWARD PROP DRIVER ROUTINE
float backward(layer* ptr, float**& y_true) {

    // get final layer's dcda
    // and calc. avg_cost
    float avg_cost = 0;
    for (int i = 0; i < batchsize; i++) {
        for (int j = 0; j < ptr->numNeurons; j++) {
            ptr->dcda[i][j] = 2.0 * (ptr->a[i][j] - y_true[i][j]);
            avg_cost += 0.5 * ptr->dcda[i][j];
        }
    }
    avg_cost /= (ptr->numNeurons * batchsize);
    ptr = ptr->prev;

    // iterate thru prev. layers
    while (ptr != nullptr) {
        // next dadz = de_relu(next a)
        layer::deriv_activmap[ptr->activation]\
            (ptr->next->a, ptr->next->dadz, batchsize, ptr->next->numNeurons);
        // next dcdz = next dcda .* next dadz
        matmul_elemwise(ptr->next->dcda, ptr->next->dadz, ptr->next->dcdz, batchsize, ptr->next->numNeurons);
        // dcdw = a_T * next dcdz
        mat_T_times_mat(ptr->a, ptr->next->dcdz, ptr->dcdw, batchsize, ptr->numNeurons, ptr->next->numNeurons);
        // dcdb = next dcdz
        copyMat(ptr->dcdb, ptr->next->dcdz, batchsize, ptr->next->numNeurons);
        // dcda = next dcdz * w_T
        mat_times_mat_T(ptr->next->dcdz, ptr->w, ptr->dcda, batchsize, ptr->next->numNeurons, ptr->numNeurons);
        // modify w, b matrices (for cur. layer)
        ptr->changeWB();

        ptr = ptr->prev;
    }
    return avg_cost;
}



