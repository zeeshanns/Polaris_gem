
#include <chrono>
#include <memory>
#include <string>

#include <ddiplus_core_planner_msgs/action/command_motion.hpp>
#include <ddiplus_core_planner_msgs/msg/planner_interface_names.hpp>
#include <ddiplus_core_planner_msgs/msg/switch.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <trident/tactical_constructs/singleton/logger.hpp>

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QSlider>
#include <QString>
#include <QVBoxLayout>
#include <QWidget>

using custom_msgs::msg::DroneMode;
using ddiplus_core_planner_msgs::action::CommandMotion;
using ddiplus_core_planner_msgs::msg::PlannerCoreResult;
using ddiplus_core_planner_msgs::msg::PlannerInterfaceNames;
using ddiplus_core_planner_msgs::msg::PlannerSuite;
using ddiplus_core_planner_msgs::msg::RouteMinds;
using ddiplus_core_planner_msgs::msg::Switch;
using trident::logger::logger;

using GoalHandle = rclcpp_action::ClientGoalHandle<CommandMotion>;

class MotionWidget : public QWidget
{
  QVBoxLayout * layout;

public:
  MotionWidget(std::shared_ptr<rclcpp::Node> node)
  : node_(node)
  {
    layout = new QVBoxLayout(this);

    initUI();

    action_client_ = rclcpp_action::create_client<CommandMotion>(
      node_, PlannerInterfaceNames::ACTION_NAME);
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_action::Client<CommandMotion>::SharedPtr action_client_;

  QLineEdit * max_velocity;
  QLineEdit * x_input;
  QLineEdit * y_input;
  QLineEdit * z_input;
  QLineEdit * command_posture;
  QSlider * engagement_radius_slider;
  QLabel * engagement_radius_value_label;
  QComboBox * mode_combo;
  QCheckBox * mission_override_checkbox;
  QLabel * feedback_label;
  QLabel * result_label;
  QLabel * error_label;
  QComboBox * planner_suite_combo;
  QComboBox * route_mind_combo;

  QLineEdit * xmin_input;
  QLineEdit * ymin_input;
  QLineEdit * xmax_input;
  QLineEdit * ymax_input;

  QSlider * velocity_slider;
  QLabel * velocity_value_label;

  QPushButton * trigger_button;
  bool trigger_exploration = false;

  GoalHandle::SharedPtr last_goal_handle_;

  void initUI()
  {
    // Mission setup group
    QGroupBox * mission_group = new QGroupBox("Mission Setup");
    QVBoxLayout * mission_layout = new QVBoxLayout();
    mission_group->setLayout(mission_layout);

    // Enable mission override
    mission_override_checkbox = new QCheckBox("Enable Mission Override");
    mission_override_checkbox->setChecked(false);
    mission_layout->addWidget(mission_override_checkbox);

    // Planner suite
    planner_suite_combo = new QComboBox();
    planner_suite_combo->addItem("UNKNOWN", PlannerSuite::UNKNOWN);
    planner_suite_combo->addItem("EXPLORATION", PlannerSuite::EXPLORATION);
    planner_suite_combo->addItem("GOTO", PlannerSuite::GOTO);
    planner_suite_combo->addItem("VKS", PlannerSuite::VKS);
    planner_suite_combo->addItem("STOP", PlannerSuite::STOP);
    mission_layout->addWidget(new QLabel("Planner Suite:"));
    mission_layout->addWidget(planner_suite_combo);

    // Route minds
    route_mind_combo = new QComboBox();
    route_mind_combo->addItem("UNKNOWN", RouteMinds::UNKNOWN);
    route_mind_combo->addItem("DDI_GOTO", RouteMinds::DDI_GOTO);
    route_mind_combo->addItem("VEL_PLANNER", RouteMinds::VEL_PLANNER);
    route_mind_combo->addItem("VKS", RouteMinds::VKS);
    route_mind_combo->addItem("EGO", RouteMinds::EGO);
    route_mind_combo->addItem("FUEL_EXPLORATION", RouteMinds::FUEL_EXPLORATION);
    mission_layout->addWidget(new QLabel("Route Mind:"));
    mission_layout->addWidget(route_mind_combo);

    // Mode
    mode_combo = new QComboBox();
    mode_combo->addItem("IDLE", DroneMode::MODE_IDLE);
    mode_combo->addItem("FOLLOW", DroneMode::MODE_FOLLOW);
    mode_combo->addItem("KSTRIKE", DroneMode::MODE_KSTRIKE);
    mode_combo->addItem("ABORT", DroneMode::MODE_ABORT);
    mode_combo->addItem("GR_IDLE", DroneMode::MODE_GR_IDLE);
    mode_combo->addItem("GR_IP_NAV", DroneMode::MODE_GR_IP_NAV);
    mode_combo->addItem("GRSTRIKE", DroneMode::MODE_GRSTRIKE);
    mode_combo->addItem("GR_ABORT", DroneMode::MODE_GR_ABORT);
    mode_combo->addItem("SKIP_NAV", DroneMode::MODE_SKIP_NAV);
    mode_combo->addItem("LOOKAT", DroneMode::MODE_LOOKAT);
    mode_combo->addItem("FLYABOVE", DroneMode::MODE_FLYABOVE);
    mission_layout->addWidget(new QLabel("Mode:"));
    mission_layout->addWidget(mode_combo);

    // Max velocity
    mission_layout->addWidget(new QLabel("Max Velocity"));
    velocity_slider = new QSlider(Qt::Horizontal);
    velocity_slider->setRange(0, 40);
    velocity_slider->setTickInterval(10);
    velocity_slider->setTickPosition(QSlider::TicksBelow);
    velocity_slider->setValue(40);
    mission_layout->addWidget(velocity_slider);

    velocity_value_label = new QLabel("4.0");
    mission_layout->addWidget(velocity_value_label);

    max_velocity = new QLineEdit();
    max_velocity->setText("4.0");
    max_velocity->setVisible(false);
    mission_layout->addWidget(max_velocity);

    connect(
      velocity_slider, &QSlider::valueChanged, this, [this](int value) {
        double scaled = value / 10.0;
        velocity_value_label->setText(QString::number(scaled, 'f', 1));
        max_velocity->setText(QString::number(scaled, 'f', 1));
      });

    // Goal radius
    mission_layout->addWidget(new QLabel("Goal Radius:"));
    engagement_radius_slider = new QSlider(Qt::Horizontal);
    engagement_radius_slider->setRange(0, 300);
    engagement_radius_slider->setTickInterval(10);
    engagement_radius_slider->setTickPosition(QSlider::TicksBelow);
    engagement_radius_slider->setValue(20);
    mission_layout->addWidget(engagement_radius_slider);

    engagement_radius_value_label = new QLabel("2.0");
    mission_layout->addWidget(engagement_radius_value_label);

    connect(
      engagement_radius_slider, &QSlider::valueChanged, this,
      [this](int value) {
        double scaled = value / 10.0;
        engagement_radius_value_label->setText(
          QString::number(scaled, 'f', 1));
      });

    layout->addWidget(mission_group);

    // Target group
    QGroupBox * target_group = new QGroupBox("Target Coordinates & Heading");
    QVBoxLayout * target_layout = new QVBoxLayout();
    target_group->setLayout(target_layout);

    x_input = new QLineEdit();
    y_input = new QLineEdit();
    z_input = new QLineEdit();
    command_posture = new QLineEdit();

    target_layout->addWidget(new QLabel("Target X:"));
    target_layout->addWidget(x_input);
    target_layout->addWidget(new QLabel("Target Y:"));
    target_layout->addWidget(y_input);
    target_layout->addWidget(new QLabel("Target Z:"));
    target_layout->addWidget(z_input);
    target_layout->addWidget(new QLabel("Orientation:"));
    target_layout->addWidget(command_posture);

    layout->addWidget(target_group);

    // Action group
    QGroupBox * action_group = new QGroupBox("Goal Actions");
    QHBoxLayout * action_layout = new QHBoxLayout();
    action_group->setLayout(action_layout);

    QPushButton * send_button = new QPushButton("Send Goal");
    QPushButton * cancel_button = new QPushButton("Cancel Goal");
    action_layout->addWidget(send_button);
    action_layout->addWidget(cancel_button);

    connect(send_button, &QPushButton::clicked, this, &MotionWidget::sendGoal);
    connect(cancel_button, &QPushButton::clicked, this, &MotionWidget::cancelGoal);

    layout->addWidget(action_group);

    // Exploration group
    QGroupBox * exploration_group = new QGroupBox("Exploration Area");
    QHBoxLayout * exploration_layout = new QHBoxLayout();
    exploration_group->setLayout(exploration_layout);

    QVBoxLayout * bbox_layout = new QVBoxLayout();
    bbox_layout->addWidget(new QLabel("BoundingBox xmin:"));
    xmin_input = new QLineEdit();
    bbox_layout->addWidget(xmin_input);

    bbox_layout->addWidget(new QLabel("BoundingBox ymin:"));
    ymin_input = new QLineEdit();
    bbox_layout->addWidget(ymin_input);

    bbox_layout->addWidget(new QLabel("BoundingBox xmax:"));
    xmax_input = new QLineEdit();
    bbox_layout->addWidget(xmax_input);

    bbox_layout->addWidget(new QLabel("BoundingBox ymax:"));
    ymax_input = new QLineEdit();
    bbox_layout->addWidget(ymax_input);

    exploration_layout->addLayout(bbox_layout);

    trigger_button = new QPushButton("Trigger Exploration");
    exploration_layout->addWidget(trigger_button);

    connect(trigger_button, &QPushButton::clicked, this, [this]() {
      trigger_exploration = true;
      QMetaObject::invokeMethod(
        error_label, "setText", Qt::QueuedConnection,
        Q_ARG(QString, "<font color='orange'>Exploration Trigger set</font>"));
    });

    layout->addWidget(exploration_group);

    createFeedbackResultUI();
  }

  void createFeedbackResultUI()
  {
    feedback_label = new QLabel("Feedback:");
    result_label = new QLabel("Result:");
    error_label = new QLabel("Error:");

    layout->addWidget(feedback_label);
    layout->addWidget(result_label);

    QFrame * line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);

    layout->addWidget(error_label);
  }

  std::string fromPathFate(PlannerCoreResult fate)
  {
    switch (fate.result) {
      case PlannerCoreResult::SUCCESS:
        return "SUCCESS";
      case PlannerCoreResult::FAILURE:
        return "FAILURE";
      case PlannerCoreResult::ABORTED:
        return "ABORTED";
      case PlannerCoreResult::INVALID:
        return "INVALID";
      case PlannerCoreResult::TIMEOUT:
        return "TIMEOUT";
      case PlannerCoreResult::UNREACHABLE:
        return "UNREACHABLE";
      case PlannerCoreResult::OCCUPIED:
        return "OCCUPIED";
      default:
        return "UNKNOWN";
    }
  }

  void sendGoal()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      logger()->warn("Action server unavailable");
      QMetaObject::invokeMethod(error_label, "clear", Qt::QueuedConnection);
      QMetaObject::invokeMethod(
        error_label, "setText", Qt::QueuedConnection,
        Q_ARG(
          QString,
          "<font color='red'>Error: Action server unavailable</font>"));
      return;
    }

    auto goal_msg = CommandMotion::Goal();
    goal_msg.command_posture = command_posture->text().toDouble();
    goal_msg.drone_mode.mode = mode_combo->currentData().toInt();

    // Set mission_override based on checkbox state
    if (mission_override_checkbox->isChecked()) {
      goal_msg.mission_override.mode = Switch::AUTOMATIC;
    } else {
      goal_msg.mission_override.mode = Switch::MANUAL;
    }

    goal_msg.engagement_radius = engagement_radius_slider->value() / 1.0;
    goal_msg.planner_suite.suite = planner_suite_combo->currentData().toInt();
    goal_msg.route_mind.algorithm = route_mind_combo->currentData().toInt();
    goal_msg.set_max_velocity = max_velocity->text().toDouble();
    goal_msg.target_x = x_input->text().toDouble();
    goal_msg.target_y = y_input->text().toDouble();
    goal_msg.target_z = z_input->text().toDouble();

    // Get bounding box values
    goal_msg.bound.xmin = xmin_input->text().toDouble();
    goal_msg.bound.ymin = ymin_input->text().toDouble();
    goal_msg.bound.xmax = xmax_input->text().toDouble();
    goal_msg.bound.ymax = ymax_input->text().toDouble();

    // // Set trigger field for exploration mission
    goal_msg.trigger = trigger_exploration;
    trigger_exploration = false;

    auto send_goal_options =
      rclcpp_action::Client<CommandMotion>::SendGoalOptions();

    send_goal_options.feedback_callback =
      [this](GoalHandle::SharedPtr,
        const std::shared_ptr<const CommandMotion::Feedback> feedback) {
        QMetaObject::invokeMethod(error_label, "clear", Qt::QueuedConnection);
        QMetaObject::invokeMethod(
          feedback_label, "setText", Qt::QueuedConnection,
          Q_ARG(
            QString,
            QString("<font color='blue'>Feedback received: %1</font>")
            .arg(feedback->path_ops_readout)));
        logger()->info("Feedback received: {}", feedback->path_ops_readout);
      };

    send_goal_options.result_callback =
      [this](const GoalHandle::WrappedResult & result) {
        QMetaObject::invokeMethod(error_label, "clear", Qt::QueuedConnection);
        QMetaObject::invokeMethod(
          result_label, "setText", Qt::QueuedConnection,
          Q_ARG(
            QString, QString("<font color='green'>%1</font>")
            .arg(
              QString::fromStdString(
                fromPathFate(result.result->path_fate)))));
        last_goal_handle_ = nullptr;
      };

    send_goal_options.goal_response_callback =
      [this](GoalHandle::SharedPtr goal_handle) {
        last_goal_handle_ = goal_handle;
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancelGoal()
  {
    if (!last_goal_handle_) {
      logger()->warn("No active goal to cancel");
      QMetaObject::invokeMethod(error_label, "clear", Qt::QueuedConnection);
      QMetaObject::invokeMethod(
        error_label, "setText", Qt::QueuedConnection,
        Q_ARG(QString, "<font color='red'>No active goal to cancel</font>"));
      return;
    }

    action_client_->async_cancel_goal(last_goal_handle_);
    logger()->info("Goal cancelled");

    QMetaObject::invokeMethod(error_label, "clear", Qt::QueuedConnection);
    QMetaObject::invokeMethod(
      error_label, "setText", Qt::QueuedConnection,
      Q_ARG(QString, "<font color='green'>Goal cancelled</font>"));
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto gui_node = rclcpp::Node::make_shared("ironpad");
  MotionWidget widget(gui_node);
  widget.setWindowTitle("IronPAD");
  widget.show();

  std::thread ros_spin([&]() {rclcpp::spin(gui_node);});

  int result = app.exec();
  rclcpp::shutdown();
  ros_spin.join();
  return result;
}
