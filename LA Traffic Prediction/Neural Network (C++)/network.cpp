
#include "header.h"

#include<cstring>
#include<iostream>
#include<sstream>
#include<string>
#include<fstream>


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NETWORK INITIALIZATION
void Network::init_network_matrices() {
    layer* ptr = layers[0];
    while (ptr != nullptr) {
        ptr->initMatrices();
        ptr->randomizeWB();
        ptr = ptr->next;
    }
}

Network::Network(int layersizes[numLayers]) {
    // seed the rand generator
    layer::gen.seed(5);

    // initialize and connect 'layers'
    for (int i = 0; i < numLayers; i++) {
        layers[i] = new layer(layersizes[i]);
        if (i > 0)
            link(layers[i - 1], layers[i]);
    }
    // initialize all matrices & partial terms
    // w, a, b, dcda, dadz, dcdz, dcdw, dcdb
    init_network_matrices();

    // remember layer sizes
    std::memcpy(this->layer_sizes, layersizes, sizeof(int) * numLayers);

    // init. the xbatch, ybatch matrices
    resizeMatrix(xbatch, batchsize, numfeats);
    resizeMatrix(ybatch, batchsize, numOutputNeurons);
}

void Network::init_activations(std::string layer_activations[numLayers]) {
    for (int i = 0; i < numLayers; i++) {
        layers[i]->assign_activation(layer_activations[i]);
    }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


void Network::get_data(std::string filename, int numsamples) {
    this->numsamples = numsamples;
    
    // resize xdataAll and ydataAll
    resizeMatrix(xdataAll, numsamples, numfeats);
    resizeMatrix(ydataAll, numsamples, numOutputNeurons);

    // open file
    std::string line, word; int numlines = 0;
    std::ifstream rfile(filename);
    std::getline(rfile, line);        // get the headers

    // read each line
    int col = 0; int row = 0; float num;
    while (getline(rfile, line)) {
        std::istringstream s(line);
        // get x features
        for (col = 0; col < numfeats; col++) {
            getline(s, word, ',');
            xdataAll[row][col] = std::stof(word);
        }
        // get y
        for (int i = 0; i < numOutputNeurons; i++) {
            getline(s, word, ',');
            ydataAll[row][i] = std::stof(word);
        }
        row++;
    }
    this->numbatches = numsamples / batchsize;
    rfile.close();
}


// swap two arrays
void swap_arrays(float*& v1, float*& v2, float*& temp, int numElems) {
    std::memcpy(temp, v1, sizeof(float) * numElems);
    std::memcpy(v1, v2, sizeof(float) * numElems);
    std::memcpy(v2, temp, sizeof(float) * numElems);
}

// shuffle all data in xdataAll & ydataAll
void Network::shuffle_data() {
    // initialize temp storage vectors
    float* xtemp; resizeVector(xtemp, numfeats);
    float* ytemp; resizeVector(ytemp, numOutputNeurons);

    // initialize the random generator
    std::uniform_int_distribution<> mydist(0, numsamples-1);
    std::default_random_engine mygen;
    mygen.seed(37);

    // loop thru xdataAll & ydataAll, swapping elems
    for (int j = 0; j < numsamples; j++) {
        int swapidx = mydist(mygen);
        swap_arrays(xdataAll[j], xdataAll[swapidx], xtemp, numfeats);
        swap_arrays(ydataAll[j], ydataAll[swapidx], ytemp, numOutputNeurons);
    }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NETWORK ACTION

// extract (batchsize) #rows from xdataAll, ydataAll into xbatch, ybatch
void Network::replace_batch(int batchidx) {

    int i = 0;
    int batchstart = batchidx * batchsize;      // start idx of iteration
    // iterate through xdataAll, ydataAll
    for (int m = batchstart; m < batchstart + batchsize; m++) {
        // intake x sample
        for (int j = 0; j < numfeats; j++) {
            xbatch[i][j] = xdataAll[m][j];
        }
        // intake y values
        for (int k = 0; k < numOutputNeurons; k++)
            ybatch[i][k] = ydataAll[m][k];
        i++;
    }
    // displayMat(xbatch, batchsize, numfeats, "xbatch");
    // displayMat(ybatch, batchsize, numOutputNeurons, "ybatch");
}


// returns the final cost
float Network::train() {
    // displayMat(ydataAll, numsamples, numOutputNeurons, "YDATA_ALL");
    // displayMat(xdataAll, numsamples, numfeats, "XDATA_ALL");
 

    float prev_cost = 9999999;
    float cost = 0;
    // by increasing min_cost_decline, we can make training halt earlier
    float minimal_cost_decline = 0.0;//0.001;
    int batchcnt = 0;

    // forwardprop / backprop cycle
    // train entire dataset, #cycles = num_epochs
    for (int i = 0; i < num_epochs; i++) {
        cost = 0;
        // for each batch in dataset
        for (int curbatch = 0; curbatch < numbatches; curbatch++) {

            replace_batch(curbatch);       // fill in values of current batch (xbatch, ybatch)
            
            // forward prop
            // input: xbatch, output: stored in last layer a
            forward(layers[0], xbatch);

            // backward prop (modifies all w, b matrices in network)
            // input: ybatch = the supposed y_true matrix
            // output: mean residual
            cost += backward(layers[numLayers - 1], ybatch);

            // rarely show predictions
            if (rand() % 5000 == 1)
                showoutput();
        }
        cost /= numbatches;
        //std::cout << "avg cost: " << cost << "\n";		// display cost

        // break if dcost < min_cost_decline
        if (std::abs(prev_cost - cost) < minimal_cost_decline)
            break;
        prev_cost = cost;
    }

    return cost;
}

// showing NN output
void Network::get_normalizer(Normalizer& norm) {
    this->norm = &norm;
}
float Network::get_MAE() {
    layer* ptr = layers[numLayers - 1];

    float avg_cost = 0;
    for (int i = 0; i < batchsize; i++) {
        for (int j = 0; j < ptr->numNeurons; j++) {
            // ptr->dcda[i][j] = 2.0 * (ptr->a[i][j] - y_true[i][j]);
            avg_cost += std::abs(ybatch[i][j] - ptr->a[i][j]) / (ptr->numNeurons * batchsize);
        }
    }
    //avg_cost /= ( ptr->numNeurons * batchsize);
    return avg_cost;
}
void Network::showoutput() {
    norm->denormalize_y(layers[numLayers-1], ybatch);
    showpredictions(layers[numLayers - 1], ybatch);

    float avg = get_MAE();
    std::cout << "\nMAE: " << avg << "\n\n";
}


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// store new data file into xdataAll and ydataAll
void Network::replace_data(std::string filename, int numsamples_) {
    freeMatrix(xdataAll, this->numsamples);
    freeMatrix(ydataAll, this->numsamples);

    get_data(filename, numsamples_);
}

// test NN
// same idea as train NN, except without modifying w, b matrices
float Network::test() {
    std::cout << "\n---------------\nTEST EVALUATION:\n";
    float cost = 0;
    int batchcnt = 0;
    for (int j = 0; j < numbatches; j++) {
        replace_batch(j);
        forward(layers[0], xbatch);			// forward prop

        cost += get_cost(layers[numLayers - 1], ybatch);        // calculate cost
        batchcnt++;
        
        showoutput();       // show predictions
    }
    // display avg cost
    cost /= numbatches;
    std::cout << "COST: " << cost << "\n";
    return cost;
}