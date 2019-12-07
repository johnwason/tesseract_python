%{
#include <tesseract_motion_planners/core/planner.h>
%}

namespace tesseract_motion_planners
{
class MotionPlanner
{
public:  
  MotionPlanner(std::string name);
  virtual ~MotionPlanner();
  
  const std::string& getName();
  
  const PlannerRequest& getRequest() const;
  
  void setRequest(const PlannerRequest& request);
  
  %rename(_solve) solve;
  virtual tesseract_common::StatusCode solve(PlannerResponse& res, const bool verbose = false) = 0;
  %pythoncode %{
  def solve(self, verbose=False):
      response = PlannerResponse()
      status_code = self._solve(response, verbose)
      return status_code, response
  %}
  virtual tesseract_common::StatusCode isConfigured() const = 0;

  virtual bool terminate() = 0;
  
  virtual void clear() = 0;

};
}  // namespace tesseract_motion_planners