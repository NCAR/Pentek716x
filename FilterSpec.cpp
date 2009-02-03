#include "FilterSpec.h"
#include <algorithm>
#include <locale>
#include <fstream>
#include <sstream>
#include <iostream>

///////////////////////////////////////////////////////////////////////////////
FilterSpec::FilterSpec():
  _ok(true),
  _fileName("") {
}
  
///////////////////////////////////////////////////////////////////////////////
  FilterSpec::FilterSpec(std::string fileName):
    _ok(false),
    _fileName(fileName)
{
  if (_fileName.length() == 0)
    return;

  std::ifstream file;
  file.open(fileName.c_str());
  if (!file.is_open())
    return;

  // The file format:
  // (blank lines will be ignored)
  // (lines beginning with # will be ignored)
  //
  // name
  // coef0
  // coef1
  //  .  .  .
  //  .  .  .
  //  .  .  .
  //  .  .  .
  // coefn
  //

  // skip white space
  file.setf(std::ios_base::skipws);

  do {
    std::string line;
    std::getline(file, line);
    if (line.size() > 0) {
      if (line[0] != '#') {
    if (_filterName.size() == 0) {
      // get the table name
      _filterName = line;
    } else {
      // get the next filter value
      int value;
      std::stringstream s;
      s << line;
      s >> value;
      this->push_back(value);
    }
      }
    }      
  } while (!file.eof());

  if (this->size() > 0)
    _ok = true;

}

///////////////////////////////////////////////////////////////////////////////
FilterSpec::FilterSpec(std::vector<unsigned int> coefficients) {
	
    for (unsigned int i = 0; i < coefficients.size(); i++)
	    this->push_back(coefficients[i]);
	
	_ok = true;
}
///////////////////////////////////////////////////////////////////////////////
FilterSpec::~FilterSpec() {
}

///////////////////////////////////////////////////////////////////////////////
bool
FilterSpec::ok() {
  return _ok;
}

///////////////////////////////////////////////////////////////////////////////
std::string 
FilterSpec::name() {
  return _filterName;
}

///////////////////////////////////////////////////////////////////////////////
void
FilterSpec::dump() {

  std::cout << _filterName << std::endl;
  for (unsigned int i = 0; i < this->size(); i++) {
    std::cout << (*this)[i] << std::endl;
  }
}
