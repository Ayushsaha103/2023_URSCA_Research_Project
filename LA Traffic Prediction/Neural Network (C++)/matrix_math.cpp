
// MATIX MATH FUNCTIONS (FOR BACKPROP)
#include<cmath>


void mat_times_mat_T(float**& x, float**& y, float**& storage, \
	int numRowsA, int numColsA, int numRowsB) {

	for (int i = 0; i < numRowsA; i++) {
		for (int j = 0; j < numRowsB; j++) {
			// nexta = a . w
			storage[i][j] = 0;
			for (int k = 0; k < numColsA; k++) {
				storage[i][j] += x[i][k] * y[j][k];
			}
		}
	}
}


void mat_T_times_mat(float**& x, float**& y, float**& storage, \
	int numRowsA, int numColsA, int numColsB) {

	for (int i = 0; i < numColsA; i++) {
		for (int j = 0; j < numColsB; j++) {
			// nexta = a . w
			storage[i][j] = 0;
			for (int k = 0; k < numRowsA; k++) {
				storage[i][j] += x[k][i] * y[k][j];
			}
		}
	}
}

void mat_times_scaler(float**& x, float y, float**& storage, int numR, int numC) {
	for (int i = 0; i < numR; i++) {
		for (int j = 0; j < numC; j++) {
			storage[i][j] = x[i][j] * y;
		}
	}
}
void matmul_elemwise(float**& x, float**& y, float**& storage, int numR, int numC) {
	for (int i = 0; i < numR; i++) {
		for (int j = 0; j < numC; j++) {
			storage[i][j] = x[i][j] * y[i][j];
		}
	}
}


// a := relu(a)
void relu(float& a) {
	if (a < 0) a = 0;
}
void sigmoid(float& a) {
	a = 1 / (1 + pow(2.718281828, -a));
}

// deriv_relu(vec) ~~> storage
void deriv_relu(float**& a, float**& storage, int numR, int numC) {
	for (int i = 0; i < numR; i++) {
		for (int j = 0; j < numC; j++) {
			storage[i][j] = (float)(a[i][j] > 0);
		}
	}
}
void deriv_sigmoid(float**& a, float**& storage, int numR, int numC) {
	for (int i = 0; i < numR; i++) {
		for (int j = 0; j < numC; j++) {
			storage[i][j] = a[i][j] * (1 - a[i][j]);
		}
	}
}

