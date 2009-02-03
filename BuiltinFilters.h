#ifndef BUILTINFILTERS_H_
#define BUILTINFILTERS_H_

#include <map>
#include <vector>

/// Create a collection of filter coeeficients. each set of coeeficients 
/// is accessed by a key.
class BuiltinFilters : public std::map<double, std::vector<unsigned int> > 
{
public:
	BuiltinFilters();
protected:
	/// Initialize the coefficients for one filter
	/// @param key The filter width 
	/// @param coeffs The coefficients
    /// @param n The number of coeffs
	void init(double key, unsigned int* coeffs, int n);
};

#endif

