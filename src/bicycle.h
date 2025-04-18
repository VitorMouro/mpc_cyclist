#ifndef MJPC_TASKS_BICYCLE_BICYCLE_H_
#define MJPC_TASKS_BICYCLE_BICYCLE_H_

#include <string>

#include <mujoco/mujoco.h>
#include <fstream>

#include "path.h"
#include "metrics.h"
#include "mjpc/task.h"

using namespace std::chrono;

namespace mjpc
{
  class Bicycle : public Task
  {
  public:
    std::string Name() const override;
    std::string XmlPath() const override;
    const Path *getPath() const { return path_; }
    int current_point_i = 0;
    // Experiment execution helpers
    void printInfo();
    Metrics *metrics; // Store metrics
    std::ofstream out;
    time_point<steady_clock> last_advance = time_point<steady_clock>::min(); // Last time advanced in path
    duration<double> advance_timeout = seconds(2); // Timeout to fail task
    time_point<steady_clock> start_time = time_point<steady_clock>::min(); // Store the task start time


    class ResidualFn : public BaseResidualFn
    {
      friend class Bicycle;

    public:
      explicit ResidualFn(const Bicycle *task) : BaseResidualFn(task) {}

      void Residual(const mjModel *model, const mjData *data,
                    double *residual) const override;
    };

    Bicycle();
    ~Bicycle() override;
    void TransitionLocked(mjModel *model, mjData *data) override;
    void ModifyScene(const mjModel *model, const mjData *data,
                     mjvScene *scene) const override;

  protected:
    std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override
    {
      return std::make_unique<ResidualFn>(this);
    }
    ResidualFn *InternalResidual() override { return &residual_; }
    void ResetLocked(const mjModel *model) override;

  private:
    ResidualFn residual_;
    Path *path_;
  };
} // namespace mjpc

#endif // MJPC_TASKS_BICYCLE_BICYCLE_H_
