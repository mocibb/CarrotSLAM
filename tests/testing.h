#ifndef TESTS_TESTING_H_
#define TESTS_TESTING_H_

#include "gtest/gtest.h"

#define EXPECT_MATRIX_NEAR(a, b, tolerance) \
do { \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  EXPECT_EQ(a.rows(), b.rows()) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols(), b.cols()) << "Matrix cols don't match."; \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        EXPECT_NEAR(a(r, c), b(r, c), tolerance) \
          << "r=" << r << ", c=" << c << "."; \
      } \
    } \
  } \
} while(false);

#define EXPECT_MATRIX_NEAR_ZERO(a, tolerance) \
do { \
  for (int r = 0; r < a.rows(); ++r) { \
    for (int c = 0; c < a.cols(); ++c) { \
      EXPECT_NEAR(0.0, a(r, c), tolerance) \
        << "r=" << r << ", c=" << c << "."; \
    } \
  } \
} while(false);

#define EXPECT_MATRIX_EQ(a, b) \
do { \
  bool dims_match = (a.rows() == b.rows()) && (a.cols() == b.cols()); \
  EXPECT_EQ(a.rows(), b.rows()) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols(), b.cols()) << "Matrix cols don't match."; \
  if (dims_match) { \
    for (int r = 0; r < a.rows(); ++r) { \
      for (int c = 0; c < a.cols(); ++c) { \
        EXPECT_EQ(a(r, c), b(r, c)) \
          << "r=" << r << ", c=" << c << "."; \
      } \
    } \
  } \
} while(false);



#endif  // TESTS_TESTING_H_
