#include "ui/window_scroll_controller.hpp"

#include <cmath>
#include <string>
#include <utility>

#include <gtk/gtk.h>
// Qt's "signals" macro clashes with GTK struct field names
#ifdef signals
#undef signals
#endif
#include <QCoreApplication>
#include <QEvent>
#include <QPoint>
#include <QString>
#include <QWheelEvent>
#include <QWidget>

namespace navigation::ui
{
namespace
{

class WheelEventFilter final : public QObject
{
public:
  explicit WheelEventFilter(WindowScrollController & controller)
  : QObject(nullptr), controller_(controller)
  {
  }

protected:
  bool eventFilter(QObject * watched, QEvent * event) override
  {
    if (event == nullptr || event->type() != QEvent::Wheel) {
      return QObject::eventFilter(watched, event);
    }

    auto * widget = qobject_cast<QWidget *>(watched);
    if (widget == nullptr || widget->window() == nullptr ||
      widget->window()->windowTitle() != QString::fromStdString(controller_.windowName()))
    {
      return QObject::eventFilter(watched, event);
    }

    auto * wheel_event = static_cast<QWheelEvent *>(event);
    int delta = wheel_event->angleDelta().y();
    if (delta == 0) {
      delta = wheel_event->pixelDelta().y();
    }
    if (delta == 0) {
      return QObject::eventFilter(watched, event);
    }

    const QPoint position = wheel_event->position().toPoint();
    controller_.handleScrollDelta(delta, position.x(), position.y());
    event->accept();
    return true;
  }

private:
  WindowScrollController & controller_;
};

static gboolean onGtkScrollEvent(GtkWidget * /*widget*/,
                                 GdkEventScroll * event,
                                 gpointer user_data)
{
  auto * controller = static_cast<WindowScrollController *>(user_data);
  if (controller == nullptr) {
    return GDK_EVENT_PROPAGATE;
  }

  int delta = 0;
#if GTK_CHECK_VERSION(3, 4, 0)
  if (event->direction == GDK_SCROLL_SMOOTH) {
    double dx = 0.0;
    double dy = 0.0;
    if (gdk_event_get_scroll_deltas(reinterpret_cast<GdkEvent *>(event), &dx, &dy)) {
      if (std::abs(dy) >= std::abs(dx)) {
        delta = static_cast<int>(std::round(dy * 120.0));
      } else {
        delta = static_cast<int>(std::round(dx * 120.0));
      }
    }
  } else
#endif
  {
    switch (event->direction) {
      case GDK_SCROLL_UP:    delta =  120; break;
      case GDK_SCROLL_DOWN:  delta = -120; break;
      case GDK_SCROLL_LEFT:  delta = -120; break;
      case GDK_SCROLL_RIGHT: delta =  120; break;
      default: break;
    }
  }

  if (delta == 0) {
    return GDK_EVENT_PROPAGATE;
  }

  controller->handleScrollDelta(
    delta,
    static_cast<int>(static_cast<gint>(event->x)),
    static_cast<int>(static_cast<gint>(event->y)));
  return GDK_EVENT_STOP;
}

}  // namespace

struct WindowScrollController::Impl
{
  std::string window_name;
  ScrollCallback callback;
  GtkWidget * gtk_image_widget{nullptr};
  bool gtk_scroll_installed{false};
  std::unique_ptr<QObject> wheel_event_filter;
};

WindowScrollController::WindowScrollController(std::string window_name, ScrollCallback callback)
: impl_(std::make_unique<Impl>())
{
  impl_->window_name = std::move(window_name);
  impl_->callback = std::move(callback);
}

WindowScrollController::~WindowScrollController()
{
  if (impl_->wheel_event_filter != nullptr) {
    auto * app = QCoreApplication::instance();
    if (app != nullptr) {
      app->removeEventFilter(impl_->wheel_event_filter.get());
    }
  }
}

bool WindowScrollController::installQtEventFilter()
{
  auto * app = QCoreApplication::instance();
  if (app == nullptr) {
    return false;
  }

  impl_->wheel_event_filter = std::make_unique<WheelEventFilter>(*this);
  app->installEventFilter(impl_->wheel_event_filter.get());
  return true;
}

void WindowScrollController::installGtkScrollController()
{
  if (impl_->gtk_scroll_installed) {
    return;
  }

  GList * toplevels = gtk_window_list_toplevels();
  for (GList * l = toplevels; l != nullptr; l = l->next) {
    if (!GTK_IS_WINDOW(l->data)) {
      continue;
    }

    const gchar * title = gtk_window_get_title(GTK_WINDOW(l->data));
    if (title == nullptr || impl_->window_name != std::string(title)) {
      continue;
    }

    GtkWidget * child = gtk_bin_get_child(GTK_BIN(l->data));
    if (child == nullptr) {
      continue;
    }

    impl_->gtk_image_widget = child;
    g_signal_connect(child, "scroll-event", G_CALLBACK(onGtkScrollEvent), this);
    impl_->gtk_scroll_installed = true;
    break;
  }
  g_list_free(toplevels);
}

void WindowScrollController::handleScrollDelta(int delta, int x, int y)
{
  if (impl_->callback != nullptr) {
    impl_->callback(delta, x, y);
  }
}

const std::string & WindowScrollController::windowName() const
{
  return impl_->window_name;
}

}  // namespace navigation::ui
