%module Tracker

%{
#define SWIG_FILE_WITH_INIT
#include "tracker.h"
%}

%include "numpy.i"
%include "std_vector.i"
%{
#include <vector>
%}

%init %{
import_array();
%}

%apply (unsigned char* INPLACE_ARRAY2, int DIM1, int DIM2) {(unsigned char* image, int rows, int cols)}
%template(PointVector) std::vector<point>;
%include "tracker.h"

