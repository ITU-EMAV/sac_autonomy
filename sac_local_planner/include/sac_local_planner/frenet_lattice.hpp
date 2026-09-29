// Candidates in the route's Frenet frame (Werling et al., "Optimal Trajectory Generation for
// Dynamic Street Scenarios in a Frenet Frame", ICRA 2010, the lateral part): from the car's
// offset d0 and its heading against the route, a quintic d(s) reaches each target offset
// with zero slope and curvature after a shift length, and stays there to the horizon.
// Quintics give smooth paths the car can drive; the lattice (offsets x shift lengths) is
// small enough to check every candidate each cycle.
//
// Parameters:
//   max_offset [m] (3.5): targets from -max_offset to +max_offset (the corridor)
//   offset_step [m] (0.5)
//   shift_times [s] ([1.5, 2.5, 3.5]): shift lengths = speed x time, at least
//     min_shift_length [m] (6)
//   horizon_time [s] (4), min_horizon [m] (30): how far each candidate goes
//   step [m] (0.5): between points

#pragma once

#include <vector>

#include "sac_local_planner/types.hpp"

namespace sac_local_planner
{

/// Quintic d(s) from (d0, d0', 0) at 0 to (d1, 0, 0) at length
class Quintic
{
public:
  Quintic(double d0, double slope0, double d1, double length);
  double value(double s) const;
  double slope(double s) const;

private:
  double a0_, a1_, a3_, a4_, a5_;
};

class FrenetLattice : public TrajectoryGenerator
{
public:
  void initialize(const Params & params) override;
  std::vector<Candidate> generate(const PlanningContext & context) const override;

  /// One candidate (for tests)
  Candidate make(const PlanningContext & context, double target_d, double shift_length, double horizon) const;

private:
  double max_offset_ = 3.5;
  double offset_step_ = 0.5;
  std::vector<double> shift_times_{1.5, 2.5, 3.5};
  double min_shift_length_ = 6.0;
  double horizon_time_ = 4.0;
  double min_horizon_ = 30.0;
  double step_ = 0.5;
};

}  // namespace sac_local_planner
