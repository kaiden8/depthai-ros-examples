
#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <cstdio>
#include <tuple>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <functional>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/DisparityConverter.hpp>


std::tuple<dai::Pipeline, int, int> createPipeline(bool withDepth, bool lrcheck, bool extended, bool subpixel, int confidence, int LRchecktresh, std::string resolution, int fps, bool colorPointcloud){
    dai::Pipeline pipeline;

        auto monoLeft    = pipeline.create<dai::node::MonoCamera>();
        auto monoRight   = pipeline.create<dai::node::MonoCamera>();
        // auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
        // auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
        auto stereo      = pipeline.create<dai::node::StereoDepth>();
        auto xoutDepth   = pipeline.create<dai::node::XLinkOut>();

        // StereoDepth
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
        stereo->initialConfig.setConfidenceThreshold(confidence);
        stereo->initialConfig.setLeftRightCheckThreshold(LRchecktresh);
        stereo->setRectifyEdgeFillColor(0); // black, to better see the cutout

        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        if (colorPointcloud) {
            stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
        }
        auto config = stereo->initialConfig.get();

        if (colorPointcloud) {
            auto camRgb               = pipeline.create<dai::node::ColorCamera>();
            auto xoutRgb              = pipeline.create<dai::node::XLinkOut>();
            xoutRgb->setStreamName("rgb");
            camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
            camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            camRgb->setFps(10);
            //TODO set scale based on rosParam and decimation of depth image
            // would video work better for this?
            camRgb->setIspScale(1, 3);
            camRgb->initialControl.setManualFocus(135);
            camRgb->isp.link(xoutRgb->input);

            // TODO add filters to be enabled and controlled from rosParam
            config.postProcessing.speckleFilter.enable = true;
            config.postProcessing.speckleFilter.speckleRange = 100;
            // config.postProcessing.temporalFilter.enable = true;
            config.postProcessing.spatialFilter.enable = true;
            config.postProcessing.spatialFilter.holeFillingRadius = 2;
            config.postProcessing.spatialFilter.numIterations = 1;
            config.postProcessing.thresholdFilter.minRange = 0;
            config.postProcessing.thresholdFilter.maxRange = 15000;
            config.postProcessing.decimationFilter.decimationFactor = 2;
        } else {
            // XLinkOut
            auto xoutLeft    = pipeline.create<dai::node::XLinkOut>();
            auto xoutRight   = pipeline.create<dai::node::XLinkOut>();
            xoutLeft->setStreamName("left");
            xoutRight->setStreamName("right");

            stereo->syncedLeft.link(xoutLeft->input);
            stereo->syncedRight.link(xoutRight->input);

            // TODO add filters to be enabled and controlled from rosParam
            config.postProcessing.speckleFilter.enable = true;
            config.postProcessing.speckleFilter.speckleRange = 100;
            // config.postProcessing.temporalFilter.enable = true;
            config.postProcessing.spatialFilter.enable = true;
            config.postProcessing.spatialFilter.holeFillingRadius = 2;
            config.postProcessing.spatialFilter.numIterations = 1;
            config.postProcessing.thresholdFilter.minRange = 0;
            config.postProcessing.thresholdFilter.maxRange = 15000;
            config.postProcessing.decimationFilter.decimationFactor = 1;
        }

        stereo->initialConfig.set(config);

        if (withDepth) {
            xoutDepth->setStreamName("depth");
        }
        else {
            xoutDepth->setStreamName("disparity");
        }

        int width, height;
        dai::node::MonoCamera::Properties::SensorResolution monoResolution;
        if(resolution == "720p"){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_720_P; 
            width  = 1280;
            height = 720;
        }else if(resolution == "400p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_400_P; 
            width  = 640;
            height = 400;
        }else if(resolution == "800p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_800_P; 
            width  = 1280;
            height = 800;
        }else if(resolution == "480p" ){
            monoResolution = dai::node::MonoCamera::Properties::SensorResolution::THE_480_P; 
            width  = 640;
            height = 480;
        }else{
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Invalid parameter. -> monoResolution: %s", resolution.c_str());
            throw std::runtime_error("Invalid mono camera resolution.");
        }

        // MonoCamera
        monoLeft->setResolution(monoResolution);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        monoLeft->setFps(fps);
        monoRight->setResolution(monoResolution);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        monoRight->setFps(fps);

        // int maxDisp = 96;
        // if (extended) maxDisp *= 2;
        // if (subpixel) maxDisp *= 32; // 5 bits fractional disparity

        // Link plugins CAM -> STEREO -> XLINK
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        if(withDepth){
            stereo->depth.link(xoutDepth->input);
        }
        else{
            stereo->disparity.link(xoutDepth->input);
        }

    return std::make_tuple(pipeline, width, height);
}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("stereo_node");
    
    std::string tfPrefix, mode, monoResolution;
    bool lrcheck, extended, subpixel, enableDepth;
    int confidence, LRchecktresh;
    int monoWidth, monoHeight;
    int fps;
    bool publishPointcloud, colorPointcloud;
    dai::Pipeline pipeline;

    // *** Voxel Filter Params ***//
    bool useVoxelFilter;
    float voxelLeafX, voxelLeafY, voxelLeafZ;
    int minPointsPerVoxel;
    std::string voxelFilterFieldName;
    double voxelFilterMinLimit, voxelFilterMaxLimit;

    // *** Statistical Outlier Removal params ***//
    bool useSorFilter, sorKeepOrganized;
    int  sorMeanK;
    double sorStddevMulThresh;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("mode", "depth");
    node->declare_parameter("lrcheck", true);
    node->declare_parameter("extended", false);
    node->declare_parameter("subpixel", true);
    node->declare_parameter("confidence",  200);
    node->declare_parameter("LRchecktresh",  5);
    node->declare_parameter("monoResolution",  "720p");
    node->declare_parameter("fps",  fps);
    node->declare_parameter("publishPointcloud",  publishPointcloud);
    node->declare_parameter("colorPointcloud",  colorPointcloud);
    node->declare_parameter("useVoxelFilter",  useVoxelFilter);
    node->declare_parameter("voxelLeafX",  voxelLeafX);
    node->declare_parameter("voxelLeafY",  voxelLeafY);
    node->declare_parameter("voxelLeafZ",  voxelLeafZ);
    node->declare_parameter("minPointsPerVoxel",  minPointsPerVoxel);
    node->declare_parameter("voxelFilterFieldName", voxelFilterFieldName);
    node->declare_parameter("voxelFilterMinLimit",  voxelFilterMinLimit);
    node->declare_parameter("voxelFilterMaxLimit",  voxelFilterMaxLimit);
    node->declare_parameter("useSorFilter",  useSorFilter);
    node->declare_parameter("sorKeepOrganized",  sorKeepOrganized);
    node->declare_parameter("sorMeanK",  sorMeanK);
    node->declare_parameter("sorStddevMulThresh",  sorStddevMulThresh);

    node->get_parameter("tf_prefix",      tfPrefix);
    node->get_parameter("mode",           mode);
    node->get_parameter("lrcheck",        lrcheck);
    node->get_parameter("extended",       extended);
    node->get_parameter("subpixel",       subpixel);
    node->get_parameter("confidence",     confidence);
    node->get_parameter("LRchecktresh",   LRchecktresh);
    node->get_parameter("monoResolution", monoResolution);
    node->get_parameter("fps",  fps);
    node->get_parameter("publishPointcloud",  publishPointcloud);
    node->get_parameter("colorPointcloud",  colorPointcloud);
    node->get_parameter("useVoxelFilter",  useVoxelFilter);
    node->get_parameter("voxelLeafX",  voxelLeafX);
    node->get_parameter("voxelLeafY",  voxelLeafY);
    node->get_parameter("voxelLeafZ",  voxelLeafZ);
    node->get_parameter("minPointsPerVoxel",  minPointsPerVoxel);
    node->get_parameter("voxelFilterFieldName", voxelFilterFieldName);
    node->get_parameter("voxelFilterMinLimit",  voxelFilterMinLimit);
    node->get_parameter("voxelFilterMaxLimit",  voxelFilterMaxLimit);
    node->get_parameter("useSorFilter",  useSorFilter);
    node->get_parameter("sorKeepOrganized",  sorKeepOrganized);
    node->get_parameter("sorMeanK",  sorMeanK);
    node->get_parameter("sorStddevMulThresh",  sorStddevMulThresh);

    //TODO add disparity mode back
    if(mode == "depth"){
        enableDepth = true;
    }
    else{
        enableDepth = false;
    }

    std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, fps, colorPointcloud);
    dai::Device device(pipeline);
    
    std::shared_ptr<dai::DataOutputQueue> stereoQueue;
    if (enableDepth) {
        stereoQueue = device.getOutputQueue("depth", 30, false);
    }else{
        stereoQueue = device.getOutputQueue("disparity", 30, false);
    }

    // *** Pointcloud Filter Setup *** // 

    std::shared_ptr<dai::rosBridge::ImageConverter::Filters> filters = std::make_shared<dai::rosBridge::ImageConverter::Filters>();

    // Voxel
    filters->voxelGrid.leafX = voxelLeafX;
    filters->voxelGrid.leafY = voxelLeafY;
    filters->voxelGrid.leafZ = voxelLeafZ;
    filters->voxelGrid.minPointsPerVoxel = minPointsPerVoxel;
    filters->voxelGrid.filterFieldName = voxelFilterFieldName;
    filters->voxelGrid.filterMinLimit = voxelFilterMinLimit;
    filters->voxelGrid.filterMaxLimit = voxelFilterMaxLimit;
    filters->voxelGrid.useFilter = useVoxelFilter;

    // StatisticalOutlierRemoval
    filters->statisticalOutlierRemoval.useFilter = useSorFilter;
    filters->statisticalOutlierRemoval.keepOrganized = sorKeepOrganized;
    filters->statisticalOutlierRemoval.meanK = sorMeanK;
    filters->statisticalOutlierRemoval.stddevMulThresh = sorStddevMulThresh;

    // END*** Pointcloud Filter Setup *** //

    auto calibrationHandler = device.readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if (monoHeight > 480 && boardName == "OAK-D-LITE") {
        monoWidth = 640;
        monoHeight = 480;
    }

    if (colorPointcloud) {
        auto imgQueue = device.getOutputQueue("rgb", 30, false);
         //TODO determine size of color image based on rosParam
        int colorWidth = 640, colorHeight = 360;
        dai::rosBridge::ImageConverter rgbconverter(tfPrefix + "_rgb_camera_optical_frame", false, filters);
        auto rgbCameraInfo = rgbconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight);  

        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(imgQueue,
                                                                                     node, 
                                                                                     std::string("color/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rgbconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rgbCameraInfo,
                                                                                     "color",
                                                                                     publishPointcloud,
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsgRGB, 
                                                                                     &rgbconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2,
                                                                                     std::placeholders::_3));

        rightPublish.addPublisherCallback();

        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rgbconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rgbCameraInfo,
                                                                                     "stereo",
                                                                                     publishPointcloud,
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsgRGB, 
                                                                                     &rgbconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2,
                                                                                     std::placeholders::_3));
        depthPublish.addPublisherCallback();
        rclcpp::spin(node);
    } else {
        auto leftQueue = device.getOutputQueue("left", 30, false);
        auto rightQueue = device.getOutputQueue("right", 30, false);

        dai::rosBridge::ImageConverter rightconverter(tfPrefix + "_right_camera_optical_frame", false, filters);
        auto rightCameraInfo = rightconverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight);  

        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> rightPublish(rightQueue,
                                                                                     node, 
                                                                                     std::string("right/image"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, 
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "right",
                                                                                     publishPointcloud,
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsg, 
                                                                                     &rightconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2,
                                                                                     std::placeholders::_3));

        rightPublish.addPublisherCallback();

        dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> depthPublish(stereoQueue,
                                                                                     node, 
                                                                                     std::string("stereo/depth"),
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                     &rightconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2) , 
                                                                                     30,
                                                                                     rightCameraInfo,
                                                                                     "stereo",
                                                                                     publishPointcloud,
                                                                                     std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsg, 
                                                                                     &rightconverter, // since the converter has the same frame name
                                                                                                      // and image type is also same we can reuse it
                                                                                     std::placeholders::_1, 
                                                                                     std::placeholders::_2,
                                                                                     std::placeholders::_3));
        depthPublish.addPublisherCallback();
        rclcpp::spin(node);
    }

    return 0;
}
