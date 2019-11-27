%{
#include <tesseract_common/status_code.h>
%}

%shared_ptr(tesseract_common::StatusCategory)
%shared_ptr(tesseract_common::GeneralStatusCategory)
%shared_ptr(tesseract_common::StatusCode)

namespace tesseract_common
{
class StatusCategory
{
public:
  using Ptr = std::shared_ptr<StatusCategory>;
  using ConstPtr = std::shared_ptr<const StatusCategory>;

  constexpr StatusCategory() noexcept;
  StatusCategory(const StatusCategory& other) = delete;
  virtual ~StatusCategory() = default;

  virtual const std::string& name() const noexcept = 0;
  virtual std::string message(int code) const = 0;

  bool operator==(const StatusCategory& rhs) const noexcept;

  bool operator!=(const StatusCategory& rhs) const noexcept;
};

class GeneralStatusCategory : public StatusCategory
{
public:
  GeneralStatusCategory(std::string name = "GeneralStatus");
  const std::string& name() const noexcept override;
  std::string message(int code) const override;

  enum
  {
    IsConfigured = 1,
    Success = 0,
    Failure = -1,
    IsNotConfigured = -2
  };
};

class StatusCode
{
public:
  using Ptr = std::shared_ptr<StatusCode>;
  using ConstPtr = std::shared_ptr<const StatusCode>;

  StatusCode(StatusCode::ConstPtr child = nullptr);
  StatusCode(int val, StatusCategory::ConstPtr cat, StatusCode::ConstPtr child = nullptr);
  ~StatusCode() = default;
  int value() const noexcept;
  const StatusCategory::ConstPtr& category() const noexcept;
  std::string message() const;
  void setChild(StatusCode::ConstPtr child);
  const StatusCode::ConstPtr& getChild() const;
  explicit operator bool() const noexcept;
  bool operator==(const StatusCode& rhs) noexcept;
  bool operator!=(const StatusCode& rhs) noexcept;
};

}  // namespace tesseract_common