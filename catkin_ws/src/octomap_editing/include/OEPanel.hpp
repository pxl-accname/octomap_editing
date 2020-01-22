#ifndef OEPANEL_HPP
#define OEPANEL_HPP

#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QSize>
#include <iostream>
#include <OEServer.hpp>
#include <QCheckBox>

#include "rviz/panel.h"

namespace octomap_editing
{
  class OEPanel: public rviz::Panel
  {
    // This class uses Qt slots and is a subclass of QObject, so it needs
    // the Q_OBJECT macro.
    Q_OBJECT

    public:
      OEPanel(QWidget* parent = 0);
      void addServer(std::shared_ptr<octomap_editing::OEServer> server);

    // Next come a couple of public Qt slots.
    private Q_SLOTS:
      void save();

    private:
      std::shared_ptr<octomap_editing::OEServer> _server;
      QPushButton* _button_save;
      QCheckBox* _checkbox_delete;
      QMetaObject::Connection _button_save_connection;
  };
}

#endif // OEPANEL_HPP
