#ifndef NAVIGATION_UI_WINDOW_SCROLL_CONTROLLER_HPP_
#define NAVIGATION_UI_WINDOW_SCROLL_CONTROLLER_HPP_

#include <functional>
#include <memory>
#include <string>

namespace navigation::ui
{

class WindowScrollController
{
public:
  using ScrollCallback = std::function<void(int delta, int x, int y)>;

  WindowScrollController(std::string window_name, ScrollCallback callback);
  ~WindowScrollController();

  bool installQtEventFilter();
  void installGtkScrollController();
  void handleScrollDelta(int delta, int x, int y);
  const std::string & windowName() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace navigation::ui

#endif  // NAVIGATION_UI_WINDOW_SCROLL_CONTROLLER_HPP_
