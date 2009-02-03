#include "BuiltinFilters.h"
#include <iostream>
//////////////////////////////////////////////////////////////////
BuiltinFilters::BuiltinFilters() {
	
}

//////////////////////////////////////////////////////////////////

void BuiltinFilters::init(double key, unsigned int* coeffs, int n) {

	(*this)[key].resize(n);

	for (unsigned int i = 0; i < (*this)[key].size(); i++) {
		(*this)[key][i] = coeffs[i];
	}
}

