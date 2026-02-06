#ifndef rviz_panel_H_
#define rviz_panel_H_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rclcpp/logger.hpp>

#include <set>
#include <string>
#include <QListWidgetItem>

// ROS2 message headers (generated from .msg)
#include <rfidbot_tags_interfaces/msg/tag_reader.hpp>

// Qt UI header generated from ui file
#include <ui_simple_panel.h>

// Other ROS dependencies
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>

#define RFD8500_START_INV       1
#define RFD8500_STOP_INV       -1
#define RFD8500_STOP_EXIT       0

namespace rviz_panel
{
  class simplePanel : public rviz_common::Panel
  {
    Q_OBJECT

  public:
    simplePanel(QWidget * parent = 0);

    virtual void save(rviz_common::Config config) const;
    virtual void load(const rviz_common::Config & config);

  public Q_SLOTS:
    // (none)

  private Q_SLOTS:
    void start_inventory();
    void save2File();
    void localizeAllTags();
    void stop_inventory();
    void readFromFile();
    void reportUniquetags();
    void handleListItemClicked(QListWidgetItem* item);
    void handleRfidTags(const rfidbot_tags_interfaces::msg::TagReader::ConstSharedPtr msg);

  protected:
    void onInitialize() override;   // ✅ ADD THIS

    std::shared_ptr<Ui::two_button> ui_;
    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_reader_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr save_tags_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr loc_all_tag_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_reader_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr read_tags_pub_;

    rclcpp::Subscription<rfidbot_tags_interfaces::msg::TagReader>::SharedPtr rfidTagsSub_;
    std_msgs::msg::Bool msg_;
    std::set<std::string> uniqueEPCs;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr RFD8500_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zebraReaderInv_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr send_EPC_pub_;
  };
} // namespace rviz_panel

#endif
