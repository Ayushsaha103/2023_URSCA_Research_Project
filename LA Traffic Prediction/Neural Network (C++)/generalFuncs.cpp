

#include<cstring>
#include<random>
#include<string>
#include<iostream>

// GENERAL FUNCS
void resizeMatrix(float**& mat, int numRows, int numCols) {
	mat = (float**)malloc(numRows * sizeof(float*));
	for (int i = 0; i < numRows; i++) {
		mat[i] = (float*)malloc(numCols * sizeof(float));
	}
}
void resizeVector(float*& mat, int numElems) {
	mat = (float*)malloc(numElems * sizeof(float));
}

void freeMatrix(float**& mat, int numRows) {
	for (int i = 0; i < numRows; i++) {
		mat[i] = nullptr;
	}
	mat = nullptr; free(mat);
}

// fill array with random doubles
void getRandomDoubles(std::uniform_real_distribution<float>& dist,
	std::default_random_engine& gen, float*& storage, int numElems) {

	for (int i = 0; i < numElems; i++) {
		storage[i] = dist(gen);
		// storage[i] = (rand() % 2) - 1;
	}
}
#include<string>
void displayMat(float**& dataAll, int numRows, int numCols, std::string title="") {
	std::cout << "\n\nMATRIX " << title << ":\n";
	for (int i = 0; i < numRows; i++) {
		for (int j = 0; j < numCols; j++) {
			std::cout << dataAll[i][j] << " ";
		}
		std::cout << "\n";
	}
	std::string buf;
	std::getline(std::cin, buf);
}

void displayMat(const float**& dataAll, int numRows, int numCols, std::string title = "") {
	std::cout << "\n\nMATRIX " << title << ":\n";
	for (int i = 0; i < numRows; i++) {
		for (int j = 0; j < numCols; j++) {
			std::cout << dataAll[i][j] << " ";
		}
		std::cout << "\n";
	}
}

void displayVect(float*& vec, int numElems, std::string title = "") {
	std::cout << "\n\nVECT " << title << ":\n";
	for (int j = 0; j < numElems; j++) {
		std::cout << vec[j] << " ";
	}
	std::cout << "\n";
}

void copyMat(float**& dest, float**& src, int numRows, int numCols) {
	for (int i = 0; i < numRows; i++) {
		std::memcpy(dest[i], src[i], numCols * sizeof(float));
	}
}
