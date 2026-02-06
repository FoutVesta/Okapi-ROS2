#include "rviz_panel/rviz_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// RViz / ROS integration
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>

#include <rclcpp/qos.hpp>
#include <QMetaObject>

#include <iostream>
#include <fstream>
#include <filesystem>

using namespace std;

PLUGINLIB_EXPORT_CLASS(rviz_panel::simplePanel, rviz_common::Panel)

namespace rviz_panel
{
simplePanel::simplePanel(QWidget * parent)
: rviz_common::Panel(parent),
  ui_(std::make_shared<Ui::two_button>())
{
  // Extend the widget with all attributes and children from UI file
  ui_->setupUi(this);

  // Only do Qt wiring here. DO NOT touch getDisplayContext() in constructor.
  connect(ui_->start_reader, SIGNAL(clicked()), this, SLOT(start_inventory()));      // button1
  connect(ui_->save_tags, SIGNAL(clicked()), this, SLOT(save2File()));              // button2
  connect(ui_->loc_all_tag, SIGNAL(clicked()), this, SLOT(localizeAllTags()));      // button3
  connect(ui_->stop_reader, SIGNAL(clicked()), this, SLOT(stop_inventory()));       // button4
  connect(ui_->read_tags, SIGNAL(clicked()), this, SLOT(readFromFile()));           // button5
  connect(ui_->inv_report, SIGNAL(clicked()), this, SLOT(reportUniquetags()));      // button6

  connect(ui_->epcListWidget, &QListWidget::itemClicked,
          this, &simplePanel::handleListItemClicked);

  // Default message value
  msg_.data = true;
}

void simplePanel::onInitialize()
{
  // RViz has now set up DisplayContext, safe to use
  auto ros_node_abstraction =
    this->getDisplayContext()->getRosNodeAbstraction().lock();

  if (!ros_node_abstraction) {
    // Don't segfault RViz; log and return gracefully
    // (throwing here can still kill RViz depending on build)
    fprintf(stderr, "[rviz_panel] ERROR: Failed to get RViz RosNodeAbstraction.\n");
    return;
  }

  node_ = ros_node_abstraction->get_raw_node();
  if (!node_) {
    fprintf(stderr, "[rviz_panel] ERROR: RViz raw node is null.\n");
    return;
  }

  // Publishers
  start_reader_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/resume_inventory_enable_new_pos", 1);
  save_tags_pub_    = node_->create_publisher<std_msgs::msg::Bool>("/save_raw_data", 1);
  loc_all_tag_pub_  = node_->create_publisher<std_msgs::msg::Bool>("/loc_all_tag", 1);
  stop_reader_pub_  = node_->create_publisher<std_msgs::msg::Bool>("/pause_inventory_delete_pos", 1);
  read_tags_pub_    = node_->create_publisher<std_msgs::msg::String>("/read_raw_data", 1);

  RFD8500_pub_        = node_->create_publisher<std_msgs::msg::Int16>("/set_rfd8500", 1);
  zebraReaderInv_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/FX9600inventory", 1);
  send_EPC_pub_       = node_->create_publisher<std_msgs::msg::String>("/loc_a_tag", 10);

  // Subscriber
  // Use explicit QoS (tags are event-like, reliable is fine)
  rfidTagsSub_ = node_->create_subscription<rfidbot_tags_interfaces::msg::TagReader>(
    "/rfid_tags",
    rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&simplePanel::handleRfidTags, this, std::placeholders::_1)
  );

  RCLCPP_INFO(node_->get_logger(), "rviz_panel initialized: subscribed to /rfid_tags");
}

// button 1
void simplePanel::start_inventory()
{
  if (!node_) return;
  RCLCPP_WARN(node_->get_logger(), "start inventory.");

  std_msgs::msg::Int16 RFD8500_msg;
  RFD8500_msg.data = RFD8500_START_INV;
  RFD8500_pub_->publish(RFD8500_msg);

  std_msgs::msg::Bool start_reader_msg;
  start_reader_msg.data = true;
  start_reader_pub_->publish(start_reader_msg);

  std_msgs::msg::Bool zebra_msg;
  zebra_msg.data = true;
  zebraReaderInv_pub_->publish(zebra_msg);
}

void simplePanel::save2File()
{
  if (!node_) return;
  RCLCPP_WARN(node_->get_logger(), "save rawdata to file");
  save_tags_pub_->publish(msg_);
}

// button 3
void simplePanel::localizeAllTags()
{
  if (!node_) return;
  RCLCPP_WARN(node_->get_logger(), "localize all tags.");
  loc_all_tag_pub_->publish(msg_);
}

// button 4
void simplePanel::stop_inventory()
{
  if (!node_) return;
  RCLCPP_WARN(node_->get_logger(), "stop inventory");

  std_msgs::msg::Int16 RFD8500_msg;
  RFD8500_msg.data = RFD8500_STOP_INV;
  RFD8500_pub_->publish(RFD8500_msg);

  std_msgs::msg::Bool stop_reader_msg;
  stop_reader_msg.data = true;
  stop_reader_pub_->publish(stop_reader_msg);

  std_msgs::msg::Bool zebra_msg;
  zebra_msg.data = false;
  zebraReaderInv_pub_->publish(zebra_msg);
}

void simplePanel::readFromFile()
{
  if (!node_) return;
  RCLCPP_WARN(node_->get_logger(), "read rawdata from file (default file)");
  std_msgs::msg::String msg;
  msg.data = "";  // empty string => use default path on subscriber side
  read_tags_pub_->publish(msg);
}

void simplePanel::reportUniquetags()
{
  if (!node_) return;
  RCLCPP_WARN(node_->get_logger(), "Generating inventory report.");
  std::string current_directory = ament_index_cpp::get_package_share_directory("rviz_panel");

  // get the time string
  time_t now = time(0);
  struct tm tstruct;
  char tstring[80];
  tstruct = *localtime(&now);
  strftime(tstring, sizeof(tstring), "%Y_%m_%d_%H_%M", &tstruct);

  std::string reports_dir = current_directory + "/inventoryreports";
  std::string filename = reports_dir + "/inventory_" + tstring + ".txt";

  // Ensure reports directory exists
  std::error_code ec;
  std::filesystem::create_directories(reports_dir, ec);

  RCLCPP_WARN(node_->get_logger(), "Saving file: %s", filename.c_str());

  ofstream MyFile(filename);
  if (!MyFile) {
    RCLCPP_WARN(node_->get_logger(), "Error: Unable to open the file.");
    return;
  }

  for (const auto& epc : uniqueEPCs) {
    MyFile << epc << std::endl;
  }

  MyFile.close();
  RCLCPP_WARN(node_->get_logger(), "saved raw data to file.");
}

void simplePanel::handleListItemClicked(QListWidgetItem* item)
{
  if (!node_) return;

  if (item) {
    std_msgs::msg::String EPC_msg;
    EPC_msg.data = item->text().toStdString();
    RCLCPP_WARN(node_->get_logger(), "Clicked item: %s", EPC_msg.data.c_str());
    send_EPC_pub_->publish(EPC_msg);
  }
}

void simplePanel::handleRfidTags(const rfidbot_tags_interfaces::msg::TagReader::ConstSharedPtr msg)
{
  if (!msg) return;

  // IMPORTANT: update Qt UI on the GUI thread to avoid crashes.
  const QString epc_q = QString::fromStdString(msg->epc);

  QMetaObject::invokeMethod(
    this,
    [this, epc_q]() {
      // total reads
      ui_->tagsReadNumber->display(ui_->tagsReadNumber->intValue() + 1);

      // unique reads
      const std::string epc = epc_q.toStdString();
      if (uniqueEPCs.find(epc) == uniqueEPCs.end()) {
        uniqueEPCs.insert(epc);
        ui_->epcListWidget->addItem(epc_q);
        ui_->uniqueTagsReadNumber->display(ui_->uniqueTagsReadNumber->intValue() + 1);
      }
    },
    Qt::QueuedConnection
  );
}

void simplePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void simplePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

} // namespace rviz_panel
