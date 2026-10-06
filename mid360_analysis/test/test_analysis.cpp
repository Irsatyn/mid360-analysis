#include <gtest/gtest.h>
#include <limits>
#include "mid360_analysis/running_stats.hpp"
#include "mid360_analysis/imu_analysis.hpp"
#include "mid360_analysis/odom_analysis.hpp"
using namespace mid360_analysis;

TEST(RunningStats, EvictsOldSamplesAndResets) {
  RunningStats s(3);
  for (double x : {1., 2., 3., 4.}) s.push(x);
  EXPECT_EQ(s.size(), 3U); EXPECT_TRUE(s.full());
  EXPECT_DOUBLE_EQ(s.mean(), 3.); EXPECT_NEAR(s.stddev(), std::sqrt(2./3.), 1e-12);
  s.reset(); EXPECT_EQ(s.size(), 0U); EXPECT_DOUBLE_EQ(s.mean(), 0.);
}
TEST(RunningStats, StableVarianceWithLargeOffset) {
  RunningStats s(2); s.push(1e12); s.push(1e12+2.);
  EXPECT_DOUBLE_EQ(s.stddev(), 1.);
  s.push(1e12+4.); EXPECT_DOUBLE_EQ(s.stddev(), 1.);
}
TEST(RunningStats, RejectsInvalidCapacityAndSamples) {
  EXPECT_THROW(RunningStats(0), std::invalid_argument);
  RunningStats s(1); EXPECT_THROW(s.push(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
}
TEST(ImuAnalysis, DetectsStaticAndHoldsLastBiasDuringMotion) {
  ImuConfig cfg; cfg.window_size=3; ImuAnalysis a(cfg);
  a.update(0., {0,0,1}, {0.001,-0.002,0});
  EXPECT_FALSE(a.is_static());
  a.update(0.005, {0,0,1}, {0.001,-0.002,0});
  a.update(0.010, {0,0,1}, {0.001,-0.002,0});
  EXPECT_TRUE(a.is_static()); EXPECT_NEAR(a.rate(), 200., 1e-9);
  EXPECT_DOUBLE_EQ(a.bias()[1], -0.002);
  a.update(0.015, {0,0,2}, {1,0,0}); EXPECT_FALSE(a.is_static());
  EXPECT_DOUBLE_EQ(a.bias()[1], -0.002);
}
TEST(ImuAnalysis, ConvertsMetersPerSecondSquaredToG) {
  ImuConfig cfg; cfg.acc_unit_is_g=false; cfg.window_size=1; ImuAnalysis a(cfg);
  a.update(0., {0,0,9.80665}, {0,0,0});
  EXPECT_DOUBLE_EQ(a.acc_norm.mean(), 1.); EXPECT_TRUE(a.is_static());
}
TEST(ImuAnalysis, CountsGapsAndIgnoresDuplicateIntervals) {
  ImuAnalysis a; a.update(1., {0,0,1}, {0,0,0});
  a.update(1.005, {0,0,1}, {0,0,0});
  EXPECT_FALSE(a.update(1.005, {0,0,1}, {0,0,0}));
  a.update(1.015, {0,0,1}, {0,0,0});
  EXPECT_EQ(a.non_monotonic_count, 1U); EXPECT_EQ(a.drop_count, 1U);
  EXPECT_NEAR(a.rate(), 1./0.0075, 1e-9);
}
TEST(ImuAnalysis, BackwardTimeClearsOldWindowAndBias) {
  ImuConfig cfg; cfg.window_size=1; ImuAnalysis a(cfg);
  a.update(2., {0,0,1}, {0.001,0,0});
  a.update(1., {0,0,2}, {1,0,0});
  EXPECT_EQ(a.non_monotonic_count, 1U); EXPECT_FALSE(a.is_static());
  EXPECT_DOUBLE_EQ(a.bias()[0], 0.); EXPECT_DOUBLE_EQ(a.rate(), 0.);
}
TEST(ImuAnalysis, RejectsNonFiniteInputWithoutPollutingWindow) {
  ImuAnalysis a; EXPECT_FALSE(a.update(0., {0,0,std::numeric_limits<double>::infinity()}, {0,0,0}));
  EXPECT_EQ(a.acc_norm.size(), 0U);
}
TEST(OdomAnalysis, AccumulatesSlowMotionAcrossSamples) {
  OdomAnalysis a; a.update(1., {0,0,0});
  a.update(1.1, {0.003,0,0}); EXPECT_DOUBLE_EQ(a.distance, 0.);
  a.update(1.2, {0.006,0,0}); EXPECT_NEAR(a.distance, 0.006, 1e-12);
  EXPECT_NEAR(a.speed, 0.03, 1e-12); EXPECT_NEAR(a.displacement, 0.006, 1e-12);
}
TEST(OdomAnalysis, ComputesVelocityAndIgnoresDuplicateTimestamp) {
  OdomAnalysis a; a.update(1., {0,0,0}); a.update(2., {3,4,0});
  EXPECT_DOUBLE_EQ(a.speed, 5.); EXPECT_DOUBLE_EQ(a.distance, 5.);
  EXPECT_FALSE(a.update(2., {100,0,0})); EXPECT_DOUBLE_EQ(a.distance, 5.);
  a.update(3., {0,0,0}); EXPECT_DOUBLE_EQ(a.distance, 10.);
  EXPECT_DOUBLE_EQ(a.displacement, 0.); EXPECT_DOUBLE_EQ(a.max_speed, 5.);
}
TEST(OdomAnalysis, BackwardTimeStartsNewTrajectory) {
  OdomAnalysis a; a.update(2., {0,0,0}); a.update(3., {1,0,0});
  EXPECT_TRUE(a.update(1., {100,0,0})); EXPECT_TRUE(a.segment_reset);
  EXPECT_DOUBLE_EQ(a.distance, 0.); EXPECT_DOUBLE_EQ(a.speed, 0.);
}
TEST(OdomAnalysis, RejectsNonFinitePosition) {
  OdomAnalysis a; EXPECT_FALSE(a.update(0., {std::numeric_limits<double>::quiet_NaN(),0,0}));
  EXPECT_TRUE(a.update(1., {0,0,0})); EXPECT_DOUBLE_EQ(a.distance, 0.);
}
TEST(Configuration, RejectsInvalidThresholds) {
  ImuConfig imu; imu.nominal_rate_hz=0.; EXPECT_THROW(ImuAnalysis{imu}, std::invalid_argument);
  OdomConfig odom; odom.min_step_m=-1.; EXPECT_THROW(OdomAnalysis{odom}, std::invalid_argument);
}
