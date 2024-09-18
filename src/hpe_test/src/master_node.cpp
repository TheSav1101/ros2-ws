#include <hpe_test/master_node.hpp>

namespace hpe_test{

    MasterNode::MasterNode(std::string name, int slave_n_) : Node(name){
        slave_n = slave_n_;
        subscribers_ = {};
        slaves_feedback = {};
        for(int i = 0; i < slave_n; i++){
            subscribers_.push_back(this->create_subscription<hpe_msgs::msg::Slave>("/slave_" + std::to_string(i), 10, std::bind(&MasterNode::callback, this, _1)));
            slaves_feedback.push_back(hpe_msgs::msg::Slave()); 
        }

    }

    MasterNode::~MasterNode(){

        
        
    }

};