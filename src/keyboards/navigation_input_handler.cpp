#include "keyboards/navigation_input_handler.hpp"

#include "keyboards/navigation_keys.hpp"

namespace navigation::keyboards
{

bool handleTextInputKey(
  int key,
  TextInputMode input_mode,
  std::string & input_text,
  const std::function<void()> & cancel_input,
  const std::function<void()> & confirm_input)
{
  if (input_mode == TextInputMode::None) {
    return false;
  }

  if (key == -1) {
    return true;
  }

  if (isEscKey(key)) {
    cancel_input();
    return true;
  }

  if (isEnterKey(key)) {
    confirm_input();
    return true;
  }

  if (isBackspaceKey(key)) {
    if (!input_text.empty()) {
      input_text.pop_back();
    }
    return true;
  }

  if (isArrowKey(key)) {
    return true;
  }

  const int ascii = keyAscii(key);
  if (ascii >= 32 && ascii <= 126) {
    input_text.push_back(static_cast<char>(ascii));
    return true;
  }

  return true;
}

void moveDropdownSelection(int delta, int item_count, int & selected_index)
{
  if (item_count <= 0) {
    selected_index = -1;
    return;
  }

  if (selected_index < 0) {
    selected_index = 0;
    return;
  }

  selected_index = (selected_index + delta + item_count) % item_count;
}

bool handleDropdownKey(
  int key,
  navigation::ui::MapDropdownMode dropdown_mode,
  int item_count,
  int & selected_index,
  const std::function<void()> & cancel_dropdown,
  const std::function<void(int)> & select_dropdown)
{
  if (dropdown_mode == navigation::ui::MapDropdownMode::None) {
    return false;
  }

  if (key == -1) {
    return true;
  }

  if (isEscKey(key)) {
    cancel_dropdown();
    return true;
  }

  if (isEnterKey(key)) {
    select_dropdown(selected_index);
    return true;
  }

  if (isUpKey(key)) {
    moveDropdownSelection(-1, item_count, selected_index);
    return true;
  }

  if (isDownKey(key)) {
    moveDropdownSelection(1, item_count, selected_index);
    return true;
  }

  return true;
}

}  // namespace navigation::keyboards
