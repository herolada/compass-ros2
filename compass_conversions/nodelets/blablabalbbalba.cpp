{
    cras::Nodelet::onInit();

    const auto params = this->privateParams();

    // Start reading params

    const auto queue_size = params->getParam("queue_size", 10_sz, "messages");
    auto nh = this->getNodeHandle();
    auto pnh = this->getPrivateNodeHandle();

    const auto targetUnit = params->getParam<decltype(Az::unit), std::string>("target_unit", Az::UNIT_RAD, "");
      // cras::GetParamConvertingOptions<decltype(Az::unit), std::string>(
      //   &compass_interfaces::msg::unitToString, &compass_interfaces::msg::parseUnit));

    const auto targetOrientation = params->getParam<decltype(Az::orientation), std::string>(
      "target_orientation", Az::ORIENTATION_ENU, "");
      // cras::GetParamConvertingOptions<decltype(Az::orientation), std::string>(
      //   &compass_interfaces::msg::orientationToString, &compass_interfaces::msg::parseOrientation));

    const auto targetReference = params->getParam<decltype(Az::reference), std::string>(
      "target_reference", Az::REFERENCE_GEOGRAPHIC, "");
      // cras::GetParamConvertingOptions<decltype(Az::reference), std::string>(
      //  &compass_interfaces::msg::referenceToString, &compass_interfaces::msg::parseReference));

    this->targetType = params->getParam<OutputType, std::string>("target_type", this->targetType, "");
      // cras::GetParamConvertingOptions<OutputType, std::string>(&outputTypeToString, &parseOutputType));

    const auto targetAppendSuffix = params->getParam("target_append_suffix", false);

    this->targetFrame = params->getParam("target_frame", std::string());

    const auto subscribeFix = params->getParam("subscribe_fix", true);
    const auto subscribeUTMZone = params->getParam("subscribe_utm", true);

    // End reading params

    const auto log = this->getLogger();

    this->converter = std::make_shared<CompassConverter>(log, params->getParam("strict", true));
    this->converter->configFromParams(*params);

    auto outputNh = targetAppendSuffix ? ros::NodeHandle(pnh, "azimuth_out") : pnh;

    std::string outputTopicSuffix;
    switch (this->targetType)
    {
      case OutputType::Imu:
        outputTopicSuffix = getAzimuthTopicSuffix<sensor_msgs::msg::Imu>(targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<sensor_msgs::msg::Imu>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      case OutputType::Pose:
        outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::msg::PoseWithCovarianceStamped>(
          targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<geometry_msgs::msg::PoseWithCovarianceStamped>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      case OutputType::Quaternion:
        outputTopicSuffix = getAzimuthTopicSuffix<geometry_msgs::msg::QuaternionStamped>(
          targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<geometry_msgs::msg::QuaternionStamped>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
      default:
        outputTopicSuffix = getAzimuthTopicSuffix<Az>(targetUnit, targetOrientation, targetReference);
        this->pub = outputNh.advertise<Az>(
          targetAppendSuffix ? outputTopicSuffix :  "azimuth_out", queue_size);
        break;
    }

    this->azimuthInput = std::make_unique<UniversalAzimuthSubscriber>(this->get_logger(), pnh, "azimuth_in", queue_size);
    this->azimuthInput->configFromParams(*params);

    this->compassFilter = std::make_unique<CompassFilter>(
      log, this->converter, *this->azimuthInput, targetUnit, targetOrientation, targetReference);

    if (subscribeFix)
    {
      this->fixInput = std::make_unique<message_filters::Subscriber<Fix>>(nh, "fix", queue_size);
      this->compassFilter->connectFixInput(*this->fixInput);
    }

    if (subscribeUTMZone)
    {
      this->utmZoneInput = std::make_unique<message_filters::Subscriber<std_msgs::msg::Int32>>(nh, "utm_zone", queue_size);
      this->compassFilter->connectUTMZoneInput(*this->utmZoneInput);
    }

    if (targetFrame.empty())
    {
      this->compassFilter->registerCallback(&CompassTransformerNodelet::publish, this);
    }
    else
    {
      this->tfFilter = std::make_unique<tf2_ros::MessageFilter<Az>>(
        log, *this->compassFilter, this->getBuffer().getRawBuffer(), targetFrame, queue_size, nh);
      this->tfFilter->registerCallback(&CompassTransformerNodelet::transformAndPublish, this);
      this->tfFilter->registerFailureCallback(std::bind_front(&CompassTransformerNodelet::failedCb, this));
    }

    RCLCPP_INFO(log, "Publishing azimuth to topic %s (type %s).",
      ros::names::resolve(this->pub.getTopic()).c_str(), outputTypeToString(this->targetType).c_str());
  }