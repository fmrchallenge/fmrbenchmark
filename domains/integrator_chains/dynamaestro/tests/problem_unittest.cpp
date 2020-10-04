#include "gtest/gtest.h"

#include "problem.hpp"
using namespace integrator_chains;


TEST(ProblemInit, DefaultEmpty) {
    Problem *prob = new Problem();
    EXPECT_EQ(0, prob->get_numdim_output());
}
