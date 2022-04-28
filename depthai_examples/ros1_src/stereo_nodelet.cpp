#include "ros/ros.h"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "sensor_msgs/Image.h"
#include <camera_info_manager/camera_info_manager.h>
#include <functional>
#include <tuple>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>

namespace depthai_examples{


 class StereoNodelet : public nodelet::Nodelet
{

    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>> leftPublish, rightPublish, depthPublish, rgbPublish;
    std::unique_ptr<dai::rosBridge::ImageConverter> leftConverter, rightConverter, rgbConverter;
    std::unique_ptr<dai::Device> _dev;

    public:
        virtual void onInit() override {

            auto& pnh = getPrivateNodeHandle();
            
            std::string tfPrefix, mode;
            std::string cameraParamUri;
            std::string monoResolution = "720p";
            int badParams = 0;
            bool lrcheck, extended, subpixel, enableDepth, publishPointcloud, colorPointcloud;
            int confidence = 200;
            int LRchecktresh = 5;
            int fps;

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

            badParams += !pnh.getParam("tf_prefix", tfPrefix);
            badParams += !pnh.getParam("camera_param_uri", cameraParamUri);
            badParams += !pnh.getParam("mode", mode);
            badParams += !pnh.getParam("lrcheck",  lrcheck);
            badParams += !pnh.getParam("extended",  extended);
            badParams += !pnh.getParam("subpixel",  subpixel);
            badParams += !pnh.getParam("confidence",  confidence);
            badParams += !pnh.getParam("LRchecktresh",  LRchecktresh);
            badParams += !pnh.getParam("monoResolution",  monoResolution);
            badParams += !pnh.getParam("fps",  fps);
            badParams += !pnh.getParam("publishPointcloud",  publishPointcloud);
            badParams += !pnh.getParam("colorPointcloud",  colorPointcloud);
            badParams += !pnh.getParam("useVoxelFilter",  useVoxelFilter);
            badParams += !pnh.getParam("voxelLeafX",  voxelLeafX);
            badParams += !pnh.getParam("voxelLeafY",  voxelLeafY);
            badParams += !pnh.getParam("voxelLeafZ",  voxelLeafZ);
            badParams += !pnh.getParam("minPointsPerVoxel",  minPointsPerVoxel);
            badParams += !pnh.getParam("voxelFilterFieldName", voxelFilterFieldName);
            badParams += !pnh.getParam("voxelFilterMinLimit",  voxelFilterMinLimit);
            badParams += !pnh.getParam("voxelFilterMaxLimit",  voxelFilterMaxLimit);
            badParams += !pnh.getParam("useSorFilter",  useSorFilter);
            badParams += !pnh.getParam("sorKeepOrganized",  sorKeepOrganized);
            badParams += !pnh.getParam("sorMeanK",  sorMeanK);
            badParams += !pnh.getParam("sorStddevMulThresh",  sorStddevMulThresh);
            
            if (badParams > 0)
            {   
                std::cout << " Bad parameters -> " << badParams << std::endl;
                throw std::runtime_error("Couldn't find %d of the parameters");
            }

            if(mode == "depth"){
                enableDepth = true;
            }
            else{
                enableDepth = false;
            }

            dai::Pipeline pipeline;
            int monoWidth, monoHeight;
            std::tie(pipeline, monoWidth, monoHeight) = createPipeline(enableDepth, lrcheck, extended, subpixel, confidence, LRchecktresh, monoResolution, fps, colorPointcloud);
            _dev = std::make_unique<dai::Device>(pipeline);

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

            // auto leftQueue = _dev->getOutputQueue("left", 30, false);
            // auto rightQueue = _dev->getOutputQueue("right", 30, false);

            std::shared_ptr<dai::DataOutputQueue> stereoQueue;
            if (enableDepth) {
                stereoQueue = _dev->getOutputQueue("depth", 30, false);
            }else{
                stereoQueue = _dev->getOutputQueue("disparity", 30, false);
            }
            auto calibrationHandler = _dev->readCalibration();

            // this part would be removed once we have calibration-api
            /*             
            std::string left_uri = camera_param_uri +"/" + "left.yaml";

            std::string right_uri = camera_param_uri + "/" + "right.yaml";

            std::string stereo_uri = camera_param_uri + "/" + "right.yaml"; 
            */

            auto boardName = calibrationHandler.getEepromData().boardName;
            if (monoHeight > 480 && boardName == "OAK-D-LITE") {
                monoWidth = 640;
                monoHeight = 480;
            }

            if (colorPointcloud) {
                auto imgQueue = _dev->getOutputQueue("rgb", 30, false);

                //TODO determine size of color image based on rosParam
                int colorWidth = 640, colorHeight = 360;
                rgbConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", false, filters);
                auto rgbCameraInfo = rgbConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight);
                
                rgbPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                (imgQueue,
                                                                                 pnh, 
                                                                                 std::string("color/image"),
                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                 rgbConverter.get(), // since the converter has the same frame name
                                                                                                 // and image type is also same we can reuse it
                                                                                 std::placeholders::_1, 
                                                                                 std::placeholders::_2) , 
                                                                                 30,
                                                                                 rgbCameraInfo,
                                                                                 "color",
                                                                                 publishPointcloud,
                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsgRGB, 
                                                                                 rgbConverter.get(), // since the converter has the same frame name
                                                                                                  // and image type is also same we can reuse it
                                                                                 std::placeholders::_1, 
                                                                                 std::placeholders::_2,
                                                                                 std::placeholders::_3));

                rgbPublish->addPublisherCallback();


                // depthconverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_rgb_camera_optical_frame", false);
                // auto depthCameraInfo = depthconverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, colorWidth, colorHeight);
                depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                (stereoQueue,
                                                                                 pnh, 
                                                                                 std::string("stereo/depth"),
                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                 rgbConverter.get(), // since the converter has the same frame name
                                                                                                 // and image type is also same we can reuse it
                                                                                 std::placeholders::_1, 
                                                                                 std::placeholders::_2) , 
                                                                                 30,
                                                                                 rgbCameraInfo,
                                                                                 "stereo",
                                                                                 publishPointcloud,
                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsgRGB, 
                                                                                 rgbConverter.get(), // since the converter has the same frame name
                                                                                                  // and image type is also same we can reuse it
                                                                                 std::placeholders::_1, 
                                                                                 std::placeholders::_2,
                                                                                 std::placeholders::_3));

                depthPublish->addPublisherCallback();
            } else {
                auto leftQueue = _dev->getOutputQueue("left", 30, false);
                auto rightQueue = _dev->getOutputQueue("right", 30, false);

                leftConverter = std::make_unique<dai::rosBridge::ImageConverter>(tfPrefix + "_left_camera_optical_frame", true);
                auto leftCameraInfo = leftConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, monoWidth, monoHeight); 

                leftPublish  = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                                (leftQueue,
                                                                                                 pnh, 
                                                                                                 std::string("left/image"),
                                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                                 leftConverter.get(),
                                                                                                 std::placeholders::_1, 
                                                                                                 std::placeholders::_2) , 
                                                                                                 30,
                                                                                                 leftCameraInfo,
                                                                                                 "left");

                // bridgePublish.startPublisherThread();
                leftPublish->addPublisherCallback();

                rightConverter = std::make_unique<dai::rosBridge::ImageConverter >(tfPrefix + "_right_camera_optical_frame", true, filters);
                auto rightCameraInfo = rightConverter->calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, monoWidth, monoHeight); 

                rightPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                                (rightQueue,
                                                                                                 pnh, 
                                                                                                 std::string("right/image"),
                                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                                 rightConverter.get(), 
                                                                                                 std::placeholders::_1, 
                                                                                                 std::placeholders::_2) , 
                                                                                                 30,
                                                                                                 rightCameraInfo,
                                                                                                 "right",
                                                                                                 publishPointcloud,
                                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsg, 
                                                                                                 rightConverter.get(), // since the converter has the same frame name
                                                                                                                  // and image type is also same we can reuse it
                                                                                                 std::placeholders::_1, 
                                                                                                 std::placeholders::_2,
                                                                                                 std::placeholders::_3));

                rightPublish->addPublisherCallback();

                // dai::rosBridge::ImageConverter depthConverter(tfPrefix + "_right_camera_optical_frame");
                depthPublish = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame>>
                                                                                (stereoQueue,
                                                                                 pnh, 
                                                                                 std::string("stereo/depth"),
                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosMsg, 
                                                                                 rightConverter.get(), // since the converter has the same frame name
                                                                                                 // and image type is also same we can reuse it
                                                                                 std::placeholders::_1, 
                                                                                 std::placeholders::_2) , 
                                                                                 30,
                                                                                 rightCameraInfo,
                                                                                 "stereo",
                                                                                 publishPointcloud,
                                                                                 std::bind(&dai::rosBridge::ImageConverter::toRosPointcloudMsg, 
                                                                                 rightConverter.get(), // since the converter has the same frame name
                                                                                                  // and image type is also same we can reuse it
                                                                                 std::placeholders::_1, 
                                                                                 std::placeholders::_2,
                                                                                 std::placeholders::_3));

                depthPublish->addPublisherCallback();
            }

            // We can add the rectified frames also similar to these publishers. 
            // Left them out so that users can play with it by adding and removing
        }


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
            ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
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
};

PLUGINLIB_EXPORT_CLASS(depthai_examples::StereoNodelet, nodelet::Nodelet)
}   // namespace depthai_examples
