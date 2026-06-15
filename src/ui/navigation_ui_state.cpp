#include "ui/navigation_ui_state.hpp"

#include <exception>
#include <iomanip>
#include <sstream>

#include "keyboards/navigation_keys.hpp"

namespace navigation::ui
{
using navigation::keyboards::isBackspaceKey;
using navigation::keyboards::isDownKey;
using navigation::keyboards::isEnterKey;
using navigation::keyboards::isEscKey;
using navigation::keyboards::isUpKey;
using navigation::keyboards::keyAscii;

bool OnlineParamsSession::active() const
{
  return active_;
}

bool OnlineParamsSession::editing() const
{
  return editing_;
}

std::size_t OnlineParamsSession::selectedIndex() const
{
  return selected_index_;
}

const std::string & OnlineParamsSession::editText() const
{
  return edit_text_;
}

void OnlineParamsSession::open(std::string & status_message)
{
  active_ = true;
  editing_ = false;
  edit_text_.clear();
  status_message = "Editing params";
}

void OnlineParamsSession::setActive(bool active)
{
  active_ = active;
  if (!active_) {
    editing_ = false;
    edit_text_.clear();
  }
}

void OnlineParamsSession::cancelEdit()
{
  editing_ = false;
  edit_text_.clear();
}

void OnlineParamsSession::selectByIndex(
  int index,
  const std::vector<ParamField> & fields,
  std::string & status_message)
{
  if (index < 0 || index >= static_cast<int>(fields.size())) {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool double_click = index == last_click_index_ &&
    (now - last_click_time_) <= std::chrono::milliseconds(450);

  selected_index_ = static_cast<std::size_t>(index);
  cancelEdit();

  last_click_index_ = index;
  last_click_time_ = now;
  if (double_click) {
    beginSelectedEdit(fields, status_message);
  } else {
    status_message = "Param selected";
  }
}

bool OnlineParamsSession::handleKey(
  int key,
  const std::vector<ParamField> & fields,
  const std::function<void()> & apply_config,
  const std::function<void()> & begin_save,
  const std::function<void()> & begin_load,
  std::string & status_message)
{
  if (!active_) {
    return false;
  }

  if (key == -1) {
    return true;
  }

  if (editing_) {
    if (isEscKey(key)) {
      cancelEdit();
      status_message = "Param edit cancelled";
      return true;
    }

    if (isEnterKey(key)) {
      applySelectedEdit(fields, apply_config, status_message);
      return true;
    }

    if (isBackspaceKey(key)) {
      if (!edit_text_.empty()) {
        edit_text_.pop_back();
      }
      return true;
    }

    const int ascii = keyAscii(key);
    if ((ascii >= '0' && ascii <= '9') || ascii == '.' || ascii == '-' || ascii == '+') {
      edit_text_.push_back(static_cast<char>(ascii));
    }
    return true;
  }

  if (isEscKey(key)) {
    setActive(false);
    status_message = "Param editor closed";
    return true;
  }

  if (isEnterKey(key)) {
    beginSelectedEdit(fields, status_message);
    return true;
  }

  if (isUpKey(key)) {
    moveSelection(-1, fields);
    return true;
  }

  if (isDownKey(key)) {
    moveSelection(1, fields);
    return true;
  }

  const int ascii = keyAscii(key);
  if (ascii == 's' || ascii == 'S') {
    begin_save();
    return true;
  }
  if (ascii == 'l' || ascii == 'L') {
    setActive(false);
    begin_load();
    return true;
  }

  return true;
}

void OnlineParamsSession::applyToState(
  MapUiState & ui_state,
  const std::vector<ParamField> & fields) const
{
  ui_state.params_active = active_;
  ui_state.params_editing = editing_;
  ui_state.params_edit_text = edit_text_;
  ui_state.param_selected_index = selected_index_;
  ui_state.param_names.reserve(fields.size());
  ui_state.param_values.reserve(fields.size());
  for (const auto & field : fields) {
    ui_state.param_names.push_back(field.name);
    ui_state.param_values.push_back(formatDouble(*field.value));
  }
}

void OnlineParamsSession::beginSelectedEdit(
  const std::vector<ParamField> & fields,
  std::string & status_message)
{
  if (selected_index_ >= fields.size()) {
    return;
  }

  editing_ = true;
  edit_text_ = formatDouble(*fields[selected_index_].value);
  status_message = "Typing param value";
}

void OnlineParamsSession::applySelectedEdit(
  const std::vector<ParamField> & fields,
  const std::function<void()> & apply_config,
  std::string & status_message)
{
  if (selected_index_ >= fields.size()) {
    cancelEdit();
    return;
  }

  try {
    const double value = std::stod(edit_text_);
    const auto & field = fields[selected_index_];
    if (field.positive && value <= 0.0) {
      status_message = "Param must be > 0";
    } else {
      *field.value = value;
      apply_config();
      status_message = "Param applied";
    }
  } catch (const std::exception &) {
    status_message = "Invalid param value";
  }

  cancelEdit();
}

void OnlineParamsSession::moveSelection(int delta, const std::vector<ParamField> & fields)
{
  if (fields.empty()) {
    selected_index_ = 0;
    return;
  }

  const int item_count = static_cast<int>(fields.size());
  selected_index_ = static_cast<std::size_t>(
    (static_cast<int>(selected_index_) + delta + item_count) % item_count);
}

std::string trim(std::string text)
{
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }

  const auto last = text.find_last_not_of(" \t\r\n");
  return text.substr(first, last - first + 1);
}

std::string formatDouble(double value)
{
  std::ostringstream output;
  output << std::fixed << std::setprecision(3) << value;
  return output.str();
}

}  // namespace navigation::ui
