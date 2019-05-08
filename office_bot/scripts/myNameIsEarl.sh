#!/bin/bash
rostopic pub /set_ascii std_msgs/UInt8MultiArray "layout:
  dim:
  - label: ''
    size: 4
    stride: 0
  data_offset: 0
data: 'earl'" 
