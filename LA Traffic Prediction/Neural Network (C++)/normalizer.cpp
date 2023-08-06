

#include "header.h"

#include<iostream>
#include<string>
#include<sstream>
#include<fstream>

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NORMALIZER INITIALIZATION

Normalizer::Normalizer() {
    normstart = 0; normrange = 1;

    std::fill(mins_x_cols, mins_x_cols + numfeats, 999999.0);
    std::fill(x_col_ranges, x_col_ranges + numfeats, -999999.0);
    std::fill(y_mins, y_mins + numOutputNeurons, 999999.0);
    std::fill(y_ranges, y_ranges + numOutputNeurons, -999999.0);
}

// initialize normalization features
int Normalizer::init_normalization_x_y(std::string filename) {
    // THIS FUNCTION initializes:
    // x_col_ranges, mins_x_cols
    // y_range, y_min
    // based off of a file read

    // open file
    std::string line, word; int numlines = 0;
    std::ifstream rfile(filename);
    std::getline(rfile, line);        // get the headers

    int col = 0; float num;
    while (getline(rfile, line)) {
        std::istringstream s(line);
        // get x features
        for (col = 0; col < numfeats; col++) {
            getline(s, word, ',');
            num = std::stof(word);

            mins_x_cols[col] = std::min(mins_x_cols[col], num);
            x_col_ranges[col] = std::max(x_col_ranges[col], num);
        }
        // get y
        for (int i = 0; i < numOutputNeurons; i++) {
            getline(s, word, ',');
            num = std::stof(word);

            y_mins[i] = std::min(y_mins[i], num);
            y_ranges[i] = std::max(y_ranges[i], num);
        }

        numlines++;     // increment num lines
    }
    // set the x, y ranges
    for (col = 0; col < numfeats; col++) {
        x_col_ranges[col] -= mins_x_cols[col];
    }    
for (int i = 0; i < numOutputNeurons; i++) {
    y_ranges[i] -= y_mins[i];
}

// Close the file
rfile.close();
return numlines;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// NORMALIZER ACTION

// denormalize y (apply to 'a' vector of output layer AND 'y_true' vector of layer)
void Normalizer::denormalize_y(layer* ptr, float**& y_true) {
    // DENORMALIZATION
    for (int i = 0; i < batchsize; i++) {
        for (int j = 0; j < numOutputNeurons; j++) {
            ptr->a[i][j] = (((ptr->a[i][j] - normstart) / normrange) * y_ranges[j]) + y_mins[j];
            y_true[i][j] = (((y_true[i][j] - normstart) / normrange) * y_ranges[j]) + y_mins[j];
        }
    }
}

// NORMALIZE ENTIRE DATAFILE, WRITE TO NEW FILE
void Normalizer::norm_func(float& i, float& min, float& range) {
    i = (((i - min) / range) * this->normrange) + this->normstart;
}

void Normalizer::normalize_datafile(std::string filename, std::string newfilename) {
    // open files
    std::string line, word;
    std::ifstream rfile(filename);
    std::ofstream wfile(newfilename);

    // copy the headers into wfile
    std::getline(rfile, line);
    wfile << line << "\n";

    // xdata, ydata matrices to hold values
    float* xdata; resizeVector(xdata, numfeats);
    float* ydata; resizeVector(ydata, numOutputNeurons);

    // read thru entire file
    while (getline(rfile, line)) {
        std::istringstream s(line);

        // get x features & normalize each
        for (int col = 0; col < numfeats; col++) {
            getline(s, word, ',');
            xdata[col] = std::stof(word);
            // apply normalization
            norm_func(xdata[col], mins_x_cols[col], x_col_ranges[col]);
        }
        for (int i = 0; i < numOutputNeurons; i++) {
            // get y & normalize it
            getline(s, word, ',');
            ydata[i] = std::stof(word);
            norm_func(ydata[i], y_mins[i], y_ranges[i]);       // apply normalization
        }
        // generate ostream from xdata,ydata values
        std::ostringstream ostream;
        for (int j = 0; j < numfeats; j++) { ostream << xdata[j] << ","; }
        for (int j = 0; j < numOutputNeurons; j++) { ostream << ydata[j] << ","; }
        // generate line (str) from ostream
        line = (ostream.str());
        line[line.size() - 1] = '\n';
        // write line to new data file
        wfile << line;
    }
    // Close the file
    rfile.close();
    wfile.close();
}


// UNUSED FUNCTION
// send matrix to file
// matrix contains: x1,x2,...y
// matrix size: [batchsize][numfeats+1]
void Normalizer::mat_to_file(float newdata[batchsize][numfeats + 1], std::ofstream& wfile) {
    // variables for writing the str
    std::string rowline;
    int rowlen = numfeats + 1;
    std::string line;

    // write each line to wfile
    for (int i = 0; i < batchsize; i++) {
        // convert arr row to string
        std::ostringstream ostream;
        for (int j = 0; j < rowlen; j++) { ostream << newdata[i][j] << ","; }
        line = (ostream.str());
        line[line.size() - 1] = '\n';
        // write string to new data file
        wfile << line;
    }
}

int Normalizer::countRows(std::string filename) {
    std::ifstream rfile(filename);
    std::string line;
    getline(rfile, line);       // get the headers

    int numRows = 0;
    while (getline(rfile, line)) {
        numRows++;
    }
    rfile.close();
    return numRows;
}