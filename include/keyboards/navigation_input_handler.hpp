#ifndef NAVIGATION_KEYBOARDS_NAVIGATION_INPUT_HANDLER_HPP_
#define NAVIGATION_KEYBOARDS_NAVIGATION_INPUT_HANDLER_HPP_

#include <functional>
#include <string>

#include "ui/map_ui_types.hpp"

namespace navigation::keyboards
{

enum class TextInputMode
{
  None,
  NewPoints,
  SavePointsAs,
  SaveParamsAs,
  SaveRadarPointAs,
  EventLabel,
  MergePointsAs,
};

bool handleTextInputKey(
  int key,
  TextInputMode input_mode,
  std::string & input_text,
  const std::function<void()> & cancel_input,
  const std::function<void()> & confirm_input);

void moveDropdownSelection(int delta, int item_count, int & selected_index);

bool handleDropdownKey(
  int key,
  navigation::ui::MapDropdownMode dropdown_mode,
  int item_count,
  int & selected_index,
  const std::function<void()> & cancel_dropdown,
  const std::function<void(int)> & select_dropdown);

}  // namespace navigation::keyboards

#endif  // NAVIGATION_KEYBOARDS_NAVIGATION_INPUT_HANDLER_HPP_
