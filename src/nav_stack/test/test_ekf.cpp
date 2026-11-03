#include <gtest/gtest.h>
#include <cmath>
#include "nav_stack/ekf.hpp"

using nav_stack::Ekf;

TEST(Ekf, StraightLine) {
  Ekf ekf;
  for (int i = 0; i < 100; ++i) {
    ekf.predict(0.02);
    ekf.updateWheel(0.5, 0.0);
    ekf.updateGyro(0.0);
  }
  EXPECT_NEAR(ekf.x(), 1.0, 0.05);
  EXPECT_NEAR(ekf.y(), 0.0, 1e-6);
}

TEST(Ekf, TrustsGyroMoreThanWheelsForTurning) {
  Ekf ekf;
  for (int i = 0; i < 200; ++i) {
    ekf.predict(0.01);
    ekf.updateGyro(0.4);
    if (i % 2 == 0) ekf.updateWheel(0.0, 0.5);
  }
  EXPECT_NEAR(ekf.w(), 0.4, 0.02);
}

TEST(Ekf, StandingStillMeansNoMovement) {
  Ekf ekf;
  for (int i = 0; i < 100; ++i) {
    ekf.predict(0.02);
    ekf.updateWheel(0.0, 0.0, true);
  }
  EXPECT_NEAR(ekf.x(), 0.0, 1e-9);
  EXPECT_NEAR(ekf.theta(), 0.0, 1e-9);
}

TEST(Ekf, CovarianceStaysSymmetric) {
  Ekf ekf;
  for (int i = 0; i < 500; ++i) {
    ekf.predict(0.02);
    ekf.updateWheel(0.3, 0.4);
    ekf.updateGyro(0.41);
  }
  const auto& P = ekf.cov();
  EXPECT_LT((P - P.transpose()).norm(), 1e-9);
  EXPECT_GT(P.diagonal().minCoeff(), 0.0);
}
