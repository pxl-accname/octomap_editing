#ifndef OEPANEL_HPP
#define OEPANEL_HPP

#include <QWidget>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

namespace rviz
{
  class Display;
  class RenderPanel;
  class VisualizationManager;
}

namespace octomap_editing
{
  class OEPanel: public QWidget
  {
    Q_OBJECT

    public:
      OEPanel(QWidget* parent = 0);
      // virtual ~OEPanel();

    private Q_SLOTS:
      void setThickness(int thickness_percent);
      void setCellSize(int cell_size_percent);

    private:
      QSharedPointer<rviz::VisualizationManager> manager_;
      QSharedPointer<rviz::RenderPanel> render_panel_;
      QSharedPointer<rviz::Display> grid_;
  };
}

#endif // OEPANEL_HPP
