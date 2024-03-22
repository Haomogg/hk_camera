//
// Created by zihan on 2022/6/2.
//
#include <pluginlib/class_list_macros.h>
#include <hk_camera.h>
#include <utility>
#include <ros/time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>
#include <string>

namespace hk_camera
{
PLUGINLIB_EXPORT_CLASS(hk_camera::HKCameraNodelet, nodelet::Nodelet) //PLUGINLIB_EXPORT_CLASS 宏，它将 hk_camera::HKCameraNodelet 类导出为一个名为 nodelet::Nodelet 的类
HKCameraNodelet::HKCameraNodelet()
{
}

void HKCameraNodelet::onInit()
{
  nh_ = this->getPrivateNodeHandle(); //获取了一个私有的节点句柄
  image_transport::ImageTransport it(nh_);
  pub_ = it.advertiseCamera("image_raw", 1);

  nh_.param("camera_frame_id", image_.header.frame_id, std::string("camera_optical_frame")); //相机框架 ID
  nh_.param("camera_name", camera_name_, std::string("camera"));
  nh_.param("camera_info_url", camera_info_url_, std::string("")); //相机信息 URL
  nh_.param("image_width", image_width_, 1440);
  nh_.param("image_height", image_height_, 1080);
  nh_.param("image_offset_x", image_offset_x_, 0); //图像偏移量
  nh_.param("image_offset_y", image_offset_y_, 0);
  nh_.param("pixel_format", pixel_format_, std::string("bgr8")); //图像像素格式，默认为 "bgr8"
  nh_.param("frame_id", frame_id_, std::string("camera_optical_frame")); //帧 ID，表示相机数据所在的坐标系，默认为 "camera_optical_frame"
  nh_.param("camera_sn", camera_sn_, std::string("")); //相机序列号
  nh_.param("frame_rate", frame_rate_, 200.0); //相机帧率
  nh_.param("sleep_time", sleep_time_, 0);
  nh_.param("enable_imu_trigger", enable_imu_trigger_, false); //是否启用 IMU 触发
  nh_.param("imu_name", imu_name_, std::string("gimbal_imu"));
  nh_.param("gain_value", gain_value_, 15.0); //增益值
  nh_.param("gain_auto", gain_auto_, false); //是否开启增益自动调节
  nh_.param("gamma_selector", gamma_selector_, 2);
  nh_.param("gamma_value", gamma_value_, 0.5);
  nh_.param("exposure_auto", exposure_auto_, true);
  nh_.param("exposure_value", exposure_value_, 20.0);
  nh_.param("exposure_max", exposure_max_, 3000.0);
  nh_.param("exposure_min", exposure_min_, 20.0);
  nh_.param("white_auto", white_auto_, true); //是否开启白平衡自动调节
  nh_.param("white_selector", white_selector_, 0); //白平衡选项
  nh_.param("enable_resolution", enable_resolution_, false); //是否启用分辨率调节
  nh_.param("resolution_ratio_width", resolution_ratio_width_, 1440); //分辨率的宽度和高度比例
  nh_.param("resolution_ratio_height", resolution_ratio_height_, 1080);
  nh_.param("stop_grab", stop_grab_, false); //停止数据采集的标志

  info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_)); // reset:重置 info_manager_ 智能指针的指向，使其指向一个新的对象

  // check for default camera info
  if (!info_manager_->isCalibrated()) //isCalibrated() 方法用于判断相机是否已经完成了校准
  {
    info_manager_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = image_.header.frame_id; //将 camera_info 对象中的帧 ID 设置为与图像消息 image_ 的帧 ID 相同
    camera_info.width = image_width_; //设置 camera_info 对象的宽度为图像宽度image_width_
    camera_info.height = image_height_;
    info_manager_->setCameraInfo(camera_info); //将包含相机信息的 camera_info 对象传递给 CameraInfoManager 对象
  }
  ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(), image_width_, image_height_);
  info_ = std::move(info_manager_->getCameraInfo()); //使用 info_manager_ 获取相机的信息，并通过 std::move 移动赋值给 info_
  info_.header.frame_id = frame_id_; //设置相机信息中的 header 的 frame_id 为之前从参数服务器中获取的 frame_id_
  image_.header.frame_id = frame_id_;
  image_.height = image_height_; //设置图像消息的高度和宽度为从参数服务器中获取的值
  image_.width = image_width_;
  image_.step = image_width_ * 3; //计算图像数据每行的字节数，通常是图像宽度乘以通道数（这里假设每个像素有 3 个通道）
  image_.data.resize(image_.height * image_.step); //调整图像数据的大小，确保能够容纳整个图像数据
  image_.encoding = pixel_format_; //设置图像消息的编码格式为之前从参数服务器中获取的 pixel_format_
  img_ = new unsigned char[image_.height * image_.step];

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST)); //使用 memset 函数将 stDeviceList 清零，确保数据初始化为 0
  try
  {
    int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList); //调用 MV_CC_EnumDevices 举支持 GigE 接口和 USB 接口的设备，并将设备列表信息存储在 stDeviceList 结构体中
    if (nRet != MV_OK)
      throw(nRet);
  }
  catch (int nRet)
  {
    std::cout << "MV_CC_EnumDevices fail! nRet " << std::hex << nRet << std::endl;
    exit(-1);
  }
  //  assert(MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList) == MV_OK);
  assert(stDeviceList.nDeviceNum > 0);

  // Opens the device.
  unsigned int nIndex = 0;
  MVCC_STRINGVALUE dev_sn; //定义一个结构体 MVCC_STRINGVALUE 类型的变量 dev_sn
  memset(&dev_sn, 0, sizeof(MVCC_STRINGVALUE)); //使用 memset 函数将 dev_sn 变量清零
  ros::Duration(sleep_time_).sleep();
  if (stDeviceList.nDeviceNum > 1)
  {
    for (; nIndex < stDeviceList.nDeviceNum; nIndex++)
    {
      assert(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]) == MV_OK); //使用相机 SDK 创建设备句柄，并进行断言检查是否成功
      MV_CC_OpenDevice(dev_handle_); //打开相机设备
      MV_CC_GetStringValue(dev_handle_, "DeviceSerialNumber", &dev_sn); //获取相机设备的序列号信息
      if (strcmp(dev_sn.chCurValue, (char*)camera_sn_.data()) == 0) //比较相机设备的序列号与指定序列号 camera_sn_ 是否相同
      {
        break;
      }
      else
      {
        MV_CC_DestroyHandle(dev_handle_);
        if (nIndex == stDeviceList.nDeviceNum - 1)
          ROS_INFO("The serial number is false!");
      }
    }
  }
  else
  {
    assert(MV_CC_CreateHandle(&dev_handle_, stDeviceList.pDeviceInfo[nIndex]) == MV_OK);
    assert(MV_CC_OpenDevice(dev_handle_) == MV_OK);
  }

  MvGvspPixelType format;
  if (pixel_format_ == "mono8")
    format = PixelType_Gvsp_Mono8;
  if (pixel_format_ == "mono16")
    format = PixelType_Gvsp_Mono16;
  if (pixel_format_ == "bgra8")
    format = PixelType_Gvsp_BayerBG8;
  if (pixel_format_ == "rgb8")
    format = PixelType_Gvsp_BayerRG8;
  if (pixel_format_ == "bgr8")
    format = PixelType_Gvsp_BayerGB8;
  if (format == 0)
    static_assert(true, "Illegal format");

  //  assert(MV_CC_SetEnumValue(dev_handle_,"PixelFormat",format) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "Width", image_width_) == MV_OK); //设置图像宽高、偏移等参数
  assert(MV_CC_SetIntValue(dev_handle_, "Height", image_height_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "OffsetX", image_offset_x_) == MV_OK);
  assert(MV_CC_SetIntValue(dev_handle_, "OffsetY", image_offset_y_) == MV_OK);
  //  AcquisitionLineRate ,LineRate can't be set
  //  assert(MV_CC_SetBoolValue(dev_handle_,"AcquisitionLineRateEnable", true)== MV_OK);
  //  assert(MV_CC_SetIntValue(dev_handle_,"AcquisitionLineRate", 10)== MV_OK);

  _MVCC_FLOATVALUE_T frame_rate;
  MV_CC_SetFrameRate(dev_handle_, frame_rate_); //设置相机的帧率
  MV_CC_GetFrameRate(dev_handle_, &frame_rate);
  ROS_INFO("Frame rate is: %f", frame_rate.fCurValue); //输出当前帧率信息

  if (enable_imu_trigger_)
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerMode", 1) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerSource", MV_TRIGGER_SOURCE_LINE2) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerActivation", 2) == MV_OK);
    //      Raising_filter_value Setting haven't been realized

    trigger_sub_ =
        nh_.subscribe("/rm_hw/" + imu_name_ + "/trigger_time", 50, &hk_camera::HKCameraNodelet::triggerCB, this); //订阅 IMU 触发时间的话题
  }
  else
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "TriggerMode", 0) == MV_OK);
  }
  MV_CC_RegisterImageCallBackEx(dev_handle_, onFrameCB, dev_handle_);

  if (MV_CC_StartGrabbing(dev_handle_) == MV_OK) //如果开始视频流捕获成功，则输出信息 "Stream On."
  {
    ROS_INFO("Stream On.");
  }

  ros::NodeHandle p_nh(nh_, "hk_camera_reconfig");
  pub_rect_ = p_nh.advertise<sensor_msgs::Image>("/image_rect", 1);
  srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh); //创建动态参数配置服务器 srv_
  dynamic_reconfigure::Server<CameraConfig>::CallbackType cb = boost::bind(&HKCameraNodelet::reconfigCB, this, _1, _2); //定义回调函数 cb，用于处理参数配置更新
  srv_->setCallback(cb);
  if (enable_imu_trigger_)
  {
    imu_trigger_client_ = nh_.serviceClient<rm_msgs::EnableImuTrigger>("imu_trigger"); //创建 IMU 触发服务的客户端
    rm_msgs::EnableImuTrigger imu_trigger_srv;
    imu_trigger_srv.request.imu_name = imu_name_; //设置 IMU 触发请求的 IMU 名称
    imu_trigger_srv.request.enable_trigger = true;
    while (!imu_trigger_client_.call(imu_trigger_srv)) //循环调用 IMU 触发服务，直到成功为止
    {
      ROS_WARN("Failed to call service enable_imu_trigger. Retry now.");
      ros::Duration(1).sleep();
    }
    if (imu_trigger_srv.response.is_success)
      ROS_INFO("Enable imu %s trigger camera successfully", imu_name_.c_str());
    else
      ROS_ERROR("Failed to enable imu %s trigger camera", imu_name_.c_str());
    enable_trigger_timer_ = nh_.createTimer(ros::Duration(0.5), &HKCameraNodelet::enableTriggerCB, this); //创建定时器，定时执行启用触发的回调函数
  }

  camera_change_sub = nh_.subscribe("/camera_name", 50, &hk_camera::HKCameraNodelet::cameraChange, this); //订阅相机名称变化的话题，并指定回调函数。
}

void HKCameraNodelet::cameraChange(const std_msgs::String camera_change) //根据传入的字符串判断是否需要开始或停止抓取摄像头图像数据
{
  if (strcmp(camera_change.data.c_str(), "hk_camera") == 0)
    MV_CC_StartGrabbing(dev_handle_); //开始抓取摄像头图像数据
  else
    MV_CC_StopGrabbing(dev_handle_); //停止抓取摄像头图像数据
}

void HKCameraNodelet::triggerCB(const sensor_msgs::TimeReference::ConstPtr& time_ref) //根据传入的时间引用对象，将其中的时间信息和序列号封装到 hk_camera::TriggerPacket 对象中，并将该对象写入到 FIFO 队列中
{
  last_trigger_time_ = time_ref->time_ref; //将传入时间引用的时间戳赋值给类成员变量 last_trigger_time_
  hk_camera::TriggerPacket pkt;
  pkt.trigger_time_ = time_ref->time_ref;
  pkt.trigger_counter_ = time_ref->header.seq;
  fifoWrite(pkt); //调用 fifoWrite 函数将 pkt 对象写入到 FIFO（First-In-First-Out）队列中
}

void HKCameraNodelet::enableTriggerCB(const ros::TimerEvent&) //在满足一定条件时（距离上一次触发时间超过1秒），向 IMU 触发服务发送使能触发信号的请求，并根据需要更新 trigger_not_sync_ 标志位
{
  if ((ros::Time::now() - last_trigger_time_).toSec() > 1.0) //算当前时间与上一次触发时间 last_trigger_time_ 的时间差，如果时间差大于1.0秒
  {
    ROS_INFO("Try to enable imu %s to trigger camera.", imu_name_.c_str());
    rm_msgs::EnableImuTrigger imu_trigger_srv;
    imu_trigger_srv.request.imu_name = imu_name_;
    imu_trigger_srv.request.enable_trigger = true;
    imu_trigger_client_.call(imu_trigger_srv);
    if (trigger_not_sync_) //如果 trigger_not_sync_ 为 true，则将其设置为 false。
      trigger_not_sync_ = false;
  }
}

void HKCameraNodelet::fifoWrite(TriggerPacket pkt) //一个 FIFO（先进先出）队列的写入操作，主要用于将触发包（TriggerPacket）写入队列中
{
  if (fifo_front_ == (fifo_rear_ + 1) % FIFO_SIZE) //判断 FIFO 队列是否已满
  {
    ROS_WARN("FIFO overflow!");
    return;
  }
  fifo_[fifo_rear_] = pkt; //将触发包 pkt 写入到 FIFO 队列中的位置 fifo_rear_
  fifo_rear_ = (fifo_rear_ + 1) % FIFO_SIZE; //更新 fifo_rear_ 的值，使其指向下一个可写入的位置
}

bool HKCameraNodelet::fifoRead(TriggerPacket& pkt) //从一个循环队列中安全地读取一个元素。如果队列中有元素可读，它就读取队列头部的元素到pkt中，并更新队列头部的索引，然后返回true。如果队列为空，即没有元素可读，它就直接返回false
{
  if (fifo_front_ == fifo_rear_) //fifo_front_表示队列头部的索引，而fifo_rear_表示队列尾部的索引。如果这两个索引相等，说明队列为空（没有元素可以读取）
    return false;
  pkt = fifo_[fifo_front_]; //获取队列头部的元素，赋值给pkt
  fifo_front_ = (fifo_front_ + 1) % FIFO_SIZE; //更新了队列头部的索引fifo_front_,取模操作确保当索引增加到队列末尾之后能够循环回到队列的开始位置
  return true;
}

void HKCameraNodelet::onFrameCB(unsigned char* pData, MV_FRAME_OUT_INFO_EX* pFrameInfo, void* pUser) //相机帧回调函数，处理从相机接收到的每一帧图像数据
{
  if (pFrameInfo)
  {
    ros::Time now = ros::Time::now();
    if (enable_imu_trigger_)
    {
      if (!trigger_not_sync_)
      {
        TriggerPacket pkt;
        while (!fifoRead(pkt)) //循环调用fifoRead函数尝试从FIFO队列中读取一个触发包，直到成功读取
        {
          ros::Duration(0.001).sleep();
        }
        //        ROS_INFO("imu:%f", now.toSec() - pkt.trigger_time_.toSec());
        if (pkt.trigger_counter_ != receive_trigger_counter_++) //检查从FIFO队列读取的触发包计数器是否与期望的接收触发计数器匹配。如果不匹配，表示触发信号可能丢失或顺序错乱
        {
          ROS_WARN("Trigger not in sync!");
          trigger_not_sync_ = true;
        }
//        else if ((now - pkt.trigger_time_).toSec() < 0)
//        {
//          ROS_WARN("Trigger not in sync! Maybe any CAN frames have be dropped?");
//          trigger_not_sync_ = true;
//        }
        else if ((now - pkt.trigger_time_).toSec() > 0.013) //检查从触发到现在的时间差是否超过了13毫秒，如果超过，可能意味着IMU没有真正触发相机
        {
          ROS_WARN("Trigger not in sync! Maybe imu %s does not actually trigger camera?", imu_name_.c_str());
          trigger_not_sync_ = true;
        }
        else
        {
          image_.header.stamp = pkt.trigger_time_; //如果触发包处理正常，没有不同步的问题，则将图像和信息的头部时间戳设置为触发包的时间戳
          info_.header.stamp = pkt.trigger_time_;
        }
      }
      if (trigger_not_sync_) //如果触发不同步，则进入此分支处理
      {
        fifo_front_ = fifo_rear_; //直接将FIFO队列的头部索引设置为尾部索引，清空队列
        rm_msgs::EnableImuTrigger imu_trigger_srv; //声明一个服务消息类型的变量imu_trigger_srv，用于调用服务禁用IMU触发
        imu_trigger_srv.request.imu_name = imu_name_; //设置服务请求参数，包括IMU名称和禁用触发标志
        imu_trigger_srv.request.enable_trigger = false;
        imu_trigger_client_.call(imu_trigger_srv); //调用服务，发送禁用IMU触发的请求
        ROS_INFO("Disable imu %s from triggering camera.", imu_name_.c_str());
        receive_trigger_counter_ = fifo_[fifo_rear_ - 1].trigger_counter_ + 1; //将接收触发计数器设置为FIFO队列中最后一个触发包的计数器加1，为下一次触发做准备
        return;
      }
    }
    else
    {
      ros::Time now = ros::Time::now(); //如果没有启用IMU触发模式，则直接使用当前时间作为图像和信息的时间戳
      image_.header.stamp = now;
      info_.header.stamp = now;
    }

    MV_CC_PIXEL_CONVERT_PARAM stConvertParam = { 0 }; //声明并初始化一个像素转换参数结构体stConvertParam，用于后续图像格式转换
    // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format,
    // destination pixel format, output data buffer, provided output buffer size
    stConvertParam.nWidth = pFrameInfo->nWidth; //设置像素转换参数，包括图像宽度、高度、源数据缓冲区、源数据大小、源像素格式、目标像素格式、目标数据缓冲区和目标缓冲区大小
    stConvertParam.nHeight = pFrameInfo->nHeight; //这里目标格式被设置为BGR8，适用于大多数图像处理和显示需求。
    stConvertParam.pSrcData = pData;
    stConvertParam.nSrcDataLen = pFrameInfo->nFrameLen;
    stConvertParam.enSrcPixelType = pFrameInfo->enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    stConvertParam.pDstBuffer = img_;
    stConvertParam.nDstBufferSize = pFrameInfo->nWidth * pFrameInfo->nHeight * 3;
    MV_CC_ConvertPixelType(dev_handle_, &stConvertParam); //调用像素转换函数，根据提供的参数将原始像素数据转换为指定的格式
    memcpy((char*)(&image_.data[0]), img_, image_.step * image_.height); //将转换后的图像数据复制到image_成员变量的数据缓冲区中

    //      if(take_photo_)
    //      {
    //          std::string str;
    //          str = std::to_string(count_);
    //          ROS_INFO("ok");
    //          cv_bridge::CvImagePtr cv_ptr1;
    //          cv_ptr1 = cv_bridge::toCvCopy(image_, "bgr8");
    //          cv::Mat cv_img1;
    //          cv_ptr1->image.copyTo(cv_img1);
    //          cv::imwrite("/home/irving/carphoto/"+str+".jpg",cv_img1);
    //          count_++;
    //      }

    if (enable_resolution_) //检查是否启用了分辨率调整功能
    {
      cv_bridge::CvImagePtr cv_ptr; //使用cv_bridge将ROS图像消息转换为OpenCV图像格式，以便进行图像处理
      cv_ptr = cv_bridge::toCvCopy(image_, "bgr8");
      cv::Mat cv_img;
      cv_ptr->image.copyTo(cv_img); //将转换后的图像数据复制到一个新的cv::Mat对象中
      sensor_msgs::ImagePtr image_rect_ptr;

      cv::resize(cv_img, cv_img, cvSize(resolution_ratio_width_, resolution_ratio_height_));
      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg(); //将调整分辨率后的图像转换回ROS图像消息格式
      pub_rect_.publish(image_rect_ptr);

      //    if (strcmp(camera_name_.data(), "hk_right"))
      //    {
      //      cv::Rect rect(0, 0, 1440 - width_, 1080);
      //      cv_img = cv_img(rect);
      //      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
      //      pub_rect_.publish(image_rect_ptr);
      //    }
      //    if (strcmp(camera_name_.data(), "hk_left"))
      //    {
      //      cv::Rect rect(width_, 0, 1440 - width_, 1080);
      //      cv_img = cv_img(rect);
      //      image_rect_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
      //      pub_rect_.publish(image_rect_ptr);
      //    }
    }
    pub_.publish(image_, info_);
  }
  else
    ROS_ERROR("Grab image failed!");
}

void HKCameraNodelet::reconfigCB(CameraConfig& config, uint32_t level) //相机节点的回调函数，用于根据传入的相机配置参数来设置相机的各种属性，并控制相机的操作
{
  (void)level;

  // Launch setting
  if (initialize_flag_) //如果初始化标志位为真，表示需要进行初始化设置
  {
    config.exposure_auto = exposure_auto_; //将保存的曝光自动模式值赋给配置参数中的曝光自动模式
    config.exposure_value = exposure_value_; //曝光值
    config.exposure_max = exposure_max_; //最大曝光值
    config.exposure_min = exposure_min_; //最小曝光值
    config.gain_auto = gain_auto_; //增益自动调节。如果设置为 true，相机将自动调整增益以适应场景亮度。如果设置为 false，需要手动设置增益值
    config.gain_value = gain_value_; //增益值
    config.gamma_selector = gamma_selector_; //伽马值选择
    config.gamma_value = gamma_value_; //伽马值。用于调整相机图像的伽马校正，以改变图像的对比度
    config.white_auto = white_auto_; //白平衡自动调节。如果设置为 true，相机将尝试自动调整白平衡以适应场景的颜色温度。如果设置为 false，需要手动设置白平衡参数
    config.white_selector = white_selector_; //白平衡选择
    config.stop_grab = stop_grab_; //停止图像抓取
    initialize_flag_ = false; //将初始化标志位置为假，表示已完成初始化设置
  }

  // Switch camera
  if (!config.stop_grab) //如果停止采集的标志为假，开始相机采集
    MV_CC_StartGrabbing(dev_handle_); //调用相机 SDK 函数开始采集图像
  else
    MV_CC_StopGrabbing(dev_handle_); //调用相机 SDK 函数停止采集图像

  // Exposure
  //根据配置参数中的曝光自动模式来设置曝光时间的上下限和曝光模式的设定
  if (config.exposure_auto) //检查配置参数中的曝光自动模式是否为真
  {
    _MVCC_FLOATVALUE_T exposure_time; //声明一个名为exposure_time的结构体变量，用于存储相机的曝光时间信息
//    assert(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min) == MV_OK); //设置相机的自动曝光时间的下限为配置参数中设定的config.exposure_min。使用MV_CC_SetIntValue函数调用相机SDK提供的接口，并通过assert来确保设置操作成功
    MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeLowerLimit", config.exposure_min); //设置相机的自动曝光时间的下限为配置参数中设定的config.exposure_min。使用MV_CC_SetIntValue函数调用相机SDK提供的接口，并通过assert来确保设置操作成功
    assert(MV_CC_SetIntValue(dev_handle_, "AutoExposureTimeUpperLimit", config.exposure_max) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS) == MV_OK);
    assert(MV_CC_GetFloatValue(dev_handle_, "ExposureTime", &exposure_time) == MV_OK); //获取当前相机的曝光时间值，并将其保存到之前声明的exposure_time结构体变量中。
    config.exposure_value = exposure_time.fCurValue; //将获取到的曝光时间值赋给配置参数config.exposure_value
  }
  else //如果配置参数中的曝光自动模式为假
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF) == MV_OK); //置相机的曝光模式为关闭自动曝光模式。使用MV_CC_SetEnumValue函数调用相机SDK提供的接口
    assert(MV_CC_SetFloatValue(dev_handle_, "ExposureTime", config.exposure_value) == MV_OK); //设置相机的曝光时间为配置参数中的config.exposure_value
  }

  // Gain
  //根据配置参数中的增益自动模式来设置增益的上下限和增益模式的设定
  if (config.gain_auto) //检查配置参数中的增益自动模式是否为真
  {
    _MVCC_FLOATVALUE_T gain_value;
    assert(MV_CC_SetFloatValue(dev_handle_, "AutoGainLowerLimit", config.gain_min) == MV_OK);
    assert(MV_CC_SetFloatValue(dev_handle_, "AutoGainUpperLimit", config.gain_max) == MV_OK);
    assert(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS) == MV_OK); //设置相机的增益模式为连续增益模式
    assert(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value) == MV_OK); //获取当前相机的增益值，并将其保存到之前声明的gain_value结构体变量中
    config.gain_value = gain_value.fCurValue; //将获取到的增益值赋给配置参数config.gain_value
  }
  else //如果配置参数中的增益自动模式为假
  {
    _MVCC_FLOATVALUE_T gain_value;
    assert(MV_CC_SetEnumValue(dev_handle_, "GainAuto", MV_GAIN_MODE_OFF) == MV_OK); //设置相机的增益模式为关闭自动增益模式
    assert(MV_CC_SetFloatValue(dev_handle_, "Gain", config.gain_value) == MV_OK); //设置相机的增益值为配置参数中的config.gain_value
    assert(MV_CC_GetFloatValue(dev_handle_, "Gain", &gain_value) == MV_OK); //再次获取当前相机的增益值，并将其保存到gain_value结构体变量中
    config.gain_value = gain_value.fCurValue; //将获取到的增益值赋给配置参数config.gain_value
  }

  // Black level
  // Can not be used!
  //根据白平衡选择和白平衡自动模式来设置白平衡模式和值
  assert(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF) == MV_OK); //MV_CC_SetEnumValue 来设置白平衡模式为手动模式（即关闭白平衡自动调节）
                                                                                                                    //MV_BALANCEWHITE_AUTO_OFF 是表示关闭白平衡自动模式的枚举值
  switch (config.white_selector)
  {
    case 0:
      assert(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 0) == MV_OK);
      break;
    case 1:
      assert(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 1) == MV_OK);
      break;
    case 2:
      assert(MV_CC_SetEnumValue(dev_handle_, "BalanceRatioSelector", 2) == MV_OK);
      break;
  }

  _MVCC_INTVALUE_T white_value;
  if (config.white_auto) //如果白平衡自动模式为真
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS) == MV_OK); //MV_CC_SetEnumValue 来设置白平衡模式为手动模式（即关闭白平衡自动调节）
    assert(MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value) == MV_OK);
    config.white_value = white_value.nCurValue;
  }
  else
  {
    assert(MV_CC_SetEnumValue(dev_handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF) == MV_OK); //MV_BALANCEWHITE_AUTO_OFF 是表示关闭白平衡自动模式的枚举值
    assert(MV_CC_GetIntValue(dev_handle_, "BalanceRatio", &white_value) == MV_OK);
    config.white_value = white_value.nCurValue;
  }

  //根据配置参数中的伽马选择器来动态设置相机的伽马值和伽马模式
  switch (config.gamma_selector) //据配置参数中的伽马选择器的值进行切换
  {
    case 0:
      assert(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true) == MV_OK); //设置相机的伽马使能为真，即开启伽马校正功能
      assert(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_SRGB) == MV_OK); //设置相机的伽马选择器为sRGB模式
      break;
    case 1:
      assert(MV_CC_SetBoolValue(dev_handle_, "GammaEnable", true) == MV_OK); //设置相机的伽马使能为真，即开启伽马校正功能
      assert(MV_CC_SetEnumValue(dev_handle_, "GammaSelector", MV_GAMMA_SELECTOR_USER) == MV_OK); //设置相机的伽马选择器为用户自定义模式
      assert(MV_CC_SetGamma(dev_handle_, config.gamma_value) == MV_OK);
      break;
    case 2:
      MV_CC_SetBoolValue(dev_handle_, "GammaEnable", false); //设置相机的伽马使能为假，即关闭伽马校正功能
      break;
  }

  take_photo_ = config.take_photo; //将配置参数中的拍照标志值赋给类成员变量 take_photo_，用于确定是否需要进行拍照操作
  //  Width offset of image
  //  width_ = config.width_offset;
}

HKCameraNodelet::~HKCameraNodelet() //用于销毁相机相关资源
{
  MV_CC_StopGrabbing(dev_handle_); //调用相机SDK提供的函数MV_CC_StopGrabbing停止相机的取流操作
  MV_CC_DestroyHandle(dev_handle_); //用相机SDK提供的函数MV_CC_DestroyHandle销毁相机句柄
}

void* HKCameraNodelet::dev_handle_; //dev_handle_ 是一个静态指针变量，用于保存相机的句柄
unsigned char* HKCameraNodelet::img_; //img_ 是一个静态指针变量，用于保存图像数据的指针
sensor_msgs::Image HKCameraNodelet::image_; //image_ 是一个静态对象，表示原始图像
sensor_msgs::Image HKCameraNodelet::image_rect; //image_rect 是一个静态对象，表示经过矫正后的图像
image_transport::CameraPublisher HKCameraNodelet::pub_; //pub_ 是一个静态对象，用于发布图像消息
ros::Publisher HKCameraNodelet::pub_rect_; //pub_rect_ 是一个静态对象，用于发布矫正后的图像消息
sensor_msgs::CameraInfo HKCameraNodelet::info_; //info_ 是一个静态对象，表示相机的相机信息
int HKCameraNodelet::width_{};
std::string HKCameraNodelet::imu_name_; //imu_name_是静态字符串变量，表示 IMU 设备的名称
std::string HKCameraNodelet::camera_name_;
ros::ServiceClient HKCameraNodelet::imu_trigger_client_; //imu_trigger_client_ 是一个静态对象，用于与 IMU 触发服务进行通信
bool HKCameraNodelet::enable_imu_trigger_; //enable_imu_trigger_ 是一个静态布尔变量，表示是否启用 IMU 触发
bool HKCameraNodelet::trigger_not_sync_ = false; //trigger_not_sync_ 是一个静态布尔变量，表示是否触发未同步
const int HKCameraNodelet::FIFO_SIZE = 1023; //FIFO_SIZE 是一个静态常量整数，表示触发包队列的大小
int HKCameraNodelet::count_ = 837; //count_ 是一个静态整数变量，表示当前的计数
int HKCameraNodelet::fifo_front_ = 0; //fifo_front_ 和 fifo_rear_ 是两个静态整数变量，分别表示触发包队列的前端和后端
int HKCameraNodelet::fifo_rear_ = 0;
bool HKCameraNodelet::device_open_ = true; //device_open_ 是一个静态布尔变量，表示设备是否已经打开
bool HKCameraNodelet::take_photo_{}; //take_photo_ 是一个静态布尔变量，表示是否拍摄照片
struct TriggerPacket HKCameraNodelet::fifo_[FIFO_SIZE]; //fifo_ 是一个静态数组，存储触发包队列的元素
uint32_t HKCameraNodelet::receive_trigger_counter_ = 0; //receive_trigger_counter_ 是一个静态无符号整数变量，表示收到的触发计数
bool HKCameraNodelet::enable_resolution_ = false; //enable_resolution_ 是一个静态布尔变量，表示是否启用分辨率
int HKCameraNodelet::resolution_ratio_width_ = 1440; //resolution_ratio_width_ 和 resolution_ratio_height_ 是两个静态整数变量，分别表示分辨率的宽度和高度
int HKCameraNodelet::resolution_ratio_height_ = 1080;
}  // namespace hk_camera
