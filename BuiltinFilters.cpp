#include "BuiltinFilters.h"
#include <iostream>
//////////////////////////////////////////////////////////////////
BuiltinFilters::BuiltinFilters() {

}

//////////////////////////////////////////////////////////////////

void BuiltinFilters::init(std::string key, unsigned int* coeffs, int n) {

	std::cout << "initializing " << n << " coefficients for " << key << std::endl;
	(*this)[key].resize(n);

	for (unsigned int i = 0; i < (*this)[key].size(); i++) {
		(*this)[key][i] = coeffs[i];
	}
}

