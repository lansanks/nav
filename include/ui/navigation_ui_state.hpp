#ifndef NAVIGATION_UI_NAVIGATION_UI_STATE_HPP_
#define NAVIGATION_UI_NAVIGATION_UI_STATE_HPP_

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "params/navigation_params.hpp"
#include "ui/map_ui_types.hpp"

namespace navigation::ui
{

using navigation::params::ParamField;

class OnlineParamsSession
{
public:
  bool active() const;
  bool editing() const;
  std::size_t selectedIndex() const;
  const std::string & editText() const;

  void open(std::string & status_message);
  void setActive(bool active);
  void cancelEdit();
  void selectByIndex(int index, const std::vector<ParamField> & fields, std::string & status_message);
  bool handleKey(
    int key,
    const std::vector<ParamField> & fields,
    const std::function<void()> & apply_config,
    const std::function<void()> & begin_save,
    const std::function<void()> & begin_load,
    std::string & status_message);
  void applyToState(MapUiState & ui_state, const std::vector<ParamField> & fields) const;

private:
  void beginSelectedEdit(const std::vector<ParamField> & fields, std::string & status_message);
  void applySelectedEdit(
    const std::vector<ParamField> & fields,
    const std::function<void()> & apply_config,
    std::string & status_message);
  void moveSelection(int delta, const std::vector<ParamField> & fields);

  bool active_{false};
  bool editing_{false};
  int last_click_index_{-1};
  std::size_t selected_index_{0};
  std::chrono::steady_clock::time_point last_click_time_{};
  std::string edit_text_;
};

std::string trim(std::string text);
std::string formatDouble(double value);

}  // namespace navigation::ui

#endif  // NAVIGATION_UI_NAVIGATION_UI_STATE_HPP_
