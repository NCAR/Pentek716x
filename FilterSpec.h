#ifndef FILTERSPECH_
#define FILTERSPECH_

#include <vector>
#include <string>

/// Specify a filter simply as a vector of integer coefficients. 
/// The coefficitnts can be read from a file, using the constructor
/// which specifies a file name, or the user can populate the
/// vector of coefficients using the alternative constructor.
class FilterSpec : public std::vector<unsigned int> {

public:
	/// Constructor
	/// Read the filter specification from 
	/// from a file.
	FilterSpec(std::string fileName ///< The file to read the values from
			);

	/// Initialize the filter with a vector of coefficients.
	/// @param coefficients The filter coefficients.
	FilterSpec(std::vector<unsigned int> coefficients);

	///  Create an empty FilterSpec
	FilterSpec();

	/// Destructor
	virtual ~FilterSpec();

	/// @return True if the filter specification was acceptible,
	/// false otherwise.
	bool ok();

	/// @return The filter name
	std::string name();

	/// list the filter characteristics to cout.
	void dump();

protected:
	bool _ok; ///< true if the filter was specified correctly
	std::string _fileName; ///< The name of the file specifying the filter
	std::string _filterName; ///< A filter name
};

#endif
