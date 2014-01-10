#ifndef FILTERSPECH_
#define FILTERSPECH_

#include <vector>
#include <string>
#include <ostream>

/// Specify a filter simply as a vector of integer coefficients. 
/// The coefficients can be read from a file, using the constructor
/// which specifies a file name, or the user can populate the
/// vector of coefficients using the alternative constructor.

class FilterSpec : public std::vector<unsigned int> {

public:
	/// Constructor
	/// Read the filter specification from 
	/// from a file.
	FilterSpec(std::string filePath ///< The file to read the values from
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

	/// @return true if symmetric
	bool isSymmetric();

	/// print the filter characteristics
        void dump(std::ostream &out);
        void dump(); // to stdout

        /// convert to string representation
        std::string toStr();

protected:
	bool _ok; ///< true if the filter was specified correctly
	std::string _filePath; ///< The path of the file specifying the filter
	std::string _filterName; ///< A filter name - normally the file name
        bool _isSymmetric;
};

#endif
