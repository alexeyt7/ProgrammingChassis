#pragma once

/* vectorization is broken for some reason
 * TODO: check if performance is acceptable
 */
#define EIGEN_DONT_VECTORIZE
#include "Eigen/Dense"
#include "Eigen/StdVector"


/* wpi aliases to make declarations shorter */
template <int Size>
using Vectord = Eigen::Vector<double, Size>;

template <int Rows, int Cols,
          int Options = Eigen::AutoAlign |
                        ((Rows == 1 && Cols != 1) ? Eigen::RowMajor
                         : (Cols == 1 && Rows != 1)
                             ? Eigen::ColMajor
                             : EIGEN_DEFAULT_MATRIX_STORAGE_ORDER_OPTION),
          int MaxRows = Rows, int MaxCols = Cols>
using Matrixd = Eigen::Matrix<double, Rows, Cols, Options, MaxRows, MaxCols>;
