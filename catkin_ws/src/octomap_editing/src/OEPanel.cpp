#include <OEPanel.hpp>

namespace octomap_editing
{
  OEPanel::OEPanel(QWidget* parent)
    : QWidget(parent)
  {
    // START TEST
    QSharedPointer<QLabel> thickness_label =  QSharedPointer<QLabel>(new QLabel("Line Thickness"));
    QSharedPointer<QSlider> thickness_slider = QSharedPointer<QSlider>(new QSlider(Qt::Horizontal));
    thickness_slider->setMinimum( 1 );
    thickness_slider->setMaximum( 100 );
    QSharedPointer<QLabel> cell_size_label = QSharedPointer<QLabel>(new QLabel("Cell Size"));
    QSharedPointer<QSlider> cell_size_slider = QSharedPointer<QSlider>(new QSlider(Qt::Horizontal));
    cell_size_slider->setMinimum( 1 );
    cell_size_slider->setMaximum( 100 );
    QSharedPointer<QGridLayout> controls_layout = QSharedPointer<QGridLayout>(new QGridLayout());
    controls_layout->addWidget( thickness_label.data(), 0, 0 );
    controls_layout->addWidget( thickness_slider.data(), 0, 1 );
    controls_layout->addWidget( cell_size_label.data(), 1, 0 );
    controls_layout->addWidget( cell_size_slider.data(), 1, 1 );

    render_panel_ = QSharedPointer<rviz::RenderPanel>(new rviz::RenderPanel());
    QSharedPointer<QVBoxLayout> main_layout = QSharedPointer<QVBoxLayout>(new QVBoxLayout());
    main_layout->addLayout( controls_layout.data());
    main_layout->addWidget( render_panel_ .data());

    setLayout( main_layout.data());

    connect( thickness_slider.data(), SIGNAL( valueChanged( int )), this, SLOT( setThickness( int )));
    connect( cell_size_slider.data(), SIGNAL( valueChanged( int )), this, SLOT( setCellSize( int )));

    manager_ = QSharedPointer<rviz::VisualizationManager>(new rviz::VisualizationManager(render_panel_.data()));
    render_panel_->initialize(manager_->getSceneManager(), manager_.data());
    manager_->initialize();
    manager_->startUpdate();

    grid_ = QSharedPointer<rviz::Display>(manager_->createDisplay( "rviz/Grid", "adjustable grid", true ));
    ROS_ASSERT( grid_ != NULL );


    grid_->subProp( "Line Style" )->setValue( "Billboards" );
    grid_->subProp( "Color" )->setValue( QColor( Qt::yellow ) );

    thickness_slider->setValue( 25 );
    cell_size_slider->setValue( 10 );

    // ENDE TEST
  }

  // START TEST
//  OEPanel::~OEPanel()
//  {
//    delete manager_;
//  }

  void
  OEPanel::setThickness( int thickness_percent )
  {
    if( grid_ != NULL )
    {
      grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( thickness_percent / 100.0f );
    }
  }

  void
  OEPanel::setCellSize( int cell_size_percent )
  {
    if( grid_ != NULL )
    {
      grid_->subProp( "Cell Size" )->setValue( cell_size_percent / 10.0f );
    }
  }

  // ENDE TEST
}
