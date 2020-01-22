#include <OEPanel.hpp>

namespace octomap_editing
{
  OEPanel::OEPanel(QWidget* parent)
    : rviz::Panel(parent)
  {
    // creates a push button with the "Q" button als hotkey
    _button_save = new QPushButton("&Save Octomap", this);
    _checkbox_delete = new QCheckBox("&Delete Points in box", this);
    _checkbox_delete->setChecked(false);
    QVBoxLayout* main_layout = new QVBoxLayout;
    main_layout->addWidget(_button_save);
    main_layout->addWidget(_checkbox_delete);
    this->setFixedSize(QSize(250, 100));

    setLayout(main_layout);
    _button_save_connection = connect(_button_save, SIGNAL(clicked()), this, SLOT(save()));
  }

  void
  OEPanel::save()
  {
    // see if the checkbox is checked
    _server->saveMap(_checkbox_delete->isChecked());
  }

  void
  OEPanel::addServer(std::shared_ptr<octomap_editing::OEServer> server)
  {
    _server = server;
  }
}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_editing::OEPanel,rviz::Panel)
