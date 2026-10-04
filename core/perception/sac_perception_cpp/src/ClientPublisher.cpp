#include "ClientPublisher.hpp"

ClientPublisher::ClientPublisher(rclcpp::Node &node, int id, std::mutex &mtx) : running(false)
{
    id_ = id;
    node_ = &node;
    mtx_ = &mtx;
}

ClientPublisher::~ClientPublisher()
{
    zed.close();
}

bool ClientPublisher::open(sl::FusionConfiguration conf)
{
    // already running

    if (runner.joinable())
        return false;
    sl::InitParameters init_parameters;
    init_parameters.input = conf.input_type;
    init_parameters.depth_mode = sl::DEPTH_MODE::ULTRA;
    // init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;

    init_parameters.coordinate_units = sl::UNIT::METER;
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;
    init_parameters.depth_maximum_distance = 8.;
    init_parameters.camera_fps = 30;

    auto state = zed.open(init_parameters);
    if (state != sl::ERROR_CODE::SUCCESS)
    {
        std::cout << "Error: " << state << std::endl;
        return false;
    }

    // translation
    translations = conf.pose.getTranslation();
    vector<float> data_trans;
    for (int i = 0; i < 3; i++)
        data_trans.push_back(translations(i));

    trans_mat = torch::from_blob(data_trans.data(), {3}).to(torch::kCUDA);
    // cout << "trans" << endl;

    // cout << trans_mat.sizes() << endl;
    // cout << trans_mat[0] <<" "<<trans_mat[1] <<" "<< trans_mat[2] << endl;
    // rotation matrix

    // auto angles = conf.pose.getEulerAngles();
    // sl::float3 angle(0,angles[1],0);
    // // sl::float3 angle(angles[0],angles[1],angles[2]);

    // conf.pose.setEulerAngles(angle);
    auto sl_rot = conf.pose.getRotationMatrix();

    vector<float> data_rot;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            data_rot.push_back(sl_rot(i, j));

    rot_mat = torch::from_blob(data_rot.data(), {3, 3}).to(torch::kCUDA);

    // cout <<endl <<rot_mat <<" " << sl_rot.getInfos()<< endl;

    // publishers
    // base_frame = "zed_";
    // base_frame += std::to_string(id_);
    base_frame = "base_link";
    string marker_topic = "/zed/visulize_3d";

    auto qos = rclcpp::QoS(1000);
    marker_publisher = node_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, qos);

    string laserscan_topic = "/zed/scan_";
    laserscan_topic += std::to_string(id_);
    laserscan_publisher = node_->create_publisher<sensor_msgs::msg::LaserScan>(laserscan_topic, qos);

    string pose_topic = "/zed/pose_";
    pose_topic += std::to_string(id_);
    pose_publisher = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, qos);

    string odom_topic = "/zed/odom_";
    odom_topic += std::to_string(id_);
    odom_publisher = node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, qos);


    string pointcloud_topic = "/zed/pointcloud_";
    pointcloud_topic += std::to_string(id_);
    pointcloud_publisher = node_->create_publisher<sensor_msgs::msg::PointCloud2>(pointcloud_topic, qos);
    
    string imu_topic = "/zed/imu_";
    imu_topic += std::to_string(id_);
    imu_publisher = node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, qos);


    // laser scan msg
    laser_msg.header.frame_id = base_frame;
    laser_msg.angle_max = 0.7176;
    laser_msg.angle_min = -0.7176;
    laser_msg.range_max = 40.0;
    laser_msg.range_min = 0.1;
    laser_msg.angle_increment = 0.003;
    laser_msg.scan_time = 0.03;
    ind_len = int((laser_msg.angle_max - laser_msg.angle_min) / laser_msg.angle_increment);

    // marker msg
    marker.header.frame_id = base_frame; // Replace with your desired frame ID
    marker.header.stamp = node_->get_clock()->now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS; // Set type to POINTS

    // pose stamped msg
    string odom_frame = "odom";
    pose_msg.header.frame_id = odom_frame;//base_frame;

    // odometry msg
    odom_msg.header.frame_id = odom_frame;//base_frame;
    odom_msg.child_frame_id = base_frame;

    // pointcloud msg
    pointcloud_msg.header.frame_id = base_frame;
    pointcloud_msg.header.stamp = node_->get_clock()->now();
    pointcloud_msg.height = 1;
    pointcloud_msg.width = 0; // number of points
    pointcloud_msg.point_step = (int)sizeof(float) * 3;
    sensor_msgs::msg::PointField field;
    field.name = "x";
    field.offset = 0;
    field.datatype = field.FLOAT32;
    field.count = 1;
    pointcloud_msg.fields.push_back(field);
    field.name = "y";
    field.offset = sizeof(float);
    field.datatype = field.FLOAT32;
    field.count = 1;
    pointcloud_msg.fields.push_back(field);
    field.name = "z";
    field.offset = sizeof(float) * 2;
    field.datatype = field.FLOAT32;
    field.count = 1;
    pointcloud_msg.fields.push_back(field);
    pointcloud_msg.row_step = 0; // point_step * width
    pointcloud_msg.is_dense = true;

    imu_msg.header.frame_id = base_frame;



    // enable Positional Tracking
    if (position_tracking_open)
    {
        // Set parameters for Positional Tracking
        sl::PositionalTrackingParameters positional_tracking_param;
        positional_tracking_param.enable_imu_fusion = true;
        // positional_tracking_param.enable_area_memory = true;
        auto returned_state = zed.enablePositionalTracking(positional_tracking_param);
        if (returned_state != sl::ERROR_CODE::SUCCESS)
        {
            zed.close();
            return false;
        }
    }
    return true;
}

void ClientPublisher::start()
{
    if (zed.isOpened())
    {
        running = true;
        // the thread can start to process the camera grab in background
        runner = std::thread(&ClientPublisher::work, this);
        
        sensors_runner = std::thread(&ClientPublisher::publish_sensors, this);


        // work();
    }
}

void ClientPublisher::stop()
{
    running = false;
    if (runner.joinable())
        runner.join();

    if (sensors_runner.joinable())
        sensors_runner.join();
    zed.close();
}

void ClientPublisher::work()
{

    auto camera_infos = zed.getCameraInformation();
    auto resolution = camera_infos.camera_configuration.resolution;

    // Define display resolution and check that it fit at least the image resolution
    float image_aspect_ratio = resolution.width / (1.f * resolution.height);
    int requested_low_res_w = min(720, (int)resolution.width);
    sl::Resolution display_resolution(requested_low_res_w, requested_low_res_w / image_aspect_ratio);

    // Mat image(display_resolution, MAT_TYPE::U8_C4, sl::MEM::GPU);
    sl::Mat point_cloud(display_resolution, sl::MAT_TYPE::F32_C4, sl::MEM::GPU);

    sl::RuntimeParameters runtime_parameters;

    // Use low depth confidence to avoid introducing noise in the constructed model
    runtime_parameters.confidence_threshold = 50;

    torch::nn::AvgPool2d avg_pool(torch::nn::AvgPool2dOptions(3).stride(3));

    cout << "zed : " << id_ << " starting.." << endl;

    while (running)
    {

        

        sprintf(memory_string, "%d lock", id_);
        nvtxRangePushA(memory_string);

        mtx_->lock();
        bool publish_flag = false;
        
        if(zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS)
        { 
            publish_flag = true;
            // auto ts_last = chrono::high_resolution_clock::now();
            sprintf(memory_string, "%d retrieve measurment", id_);
            nvtxRangePushA(memory_string);
            
            zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZ, sl::MEM::GPU);

            pointcloud_timestamp_ns = point_cloud.timestamp.getNanoseconds();
            pointcloud_timestamp_s = point_cloud.timestamp.getSeconds();

            // Get current ROS time
            // rclcpp::Time now = node_->now();

            // Get nanoseconds and milliseconds
            //int64_t nanoseconds = now.nanoseconds();
            //int64_t milliseconds = nanoseconds / 1e9;  // Convert ns to ms
            //pointcloud_timestamp_ns = nanoseconds % 1e9;
            //pointcloud_timestamp_s = milliseconds;
            
            
            
            to_tensor(point_cloud, tensor);
            
            // cudaDeviceSynchronize();
            nvtxRangePop();

            // cout << id_ << endl;
            sprintf(memory_string, "%d tensor calc", id_);
            nvtxRangePushA(memory_string);

            tensor = avg_pool(tensor);
            tensor = avg_pool(tensor);
            flatten_tensor(tensor);
            tensor = torch::matmul(rot_mat, tensor.t()).t();
            tensor = tensor + trans_mat;
            // cout << tensor.sizes() << endl;

            auto z_tensor = tensor.select(1, 2);

            auto cond1 = z_tensor.lt(2.0).to(torch::kCUDA);
            auto cond2 = z_tensor.gt(0.0).to(torch::kCUDA);
            auto cond3 = z_tensor.isnan().to(torch::kCUDA);

            auto cond = cond1.bitwise_and(cond2);
            cond = cond.bitwise_and(cond3.bitwise_not());

            tensor = tensor.index({cond});

            tensor = torch::div(torch::mul(tensor, 100), voxel_size);
            tensor = torch::floor(tensor);
            tensor = torch::mul(torch::div(tensor, 100), voxel_size);
            tensor = std::get<0>(torch::unique_dim(tensor, 0));

            // cout << tensor.sizes() <<endl;
            cpu_tensor = tensor.to(torch::kCPU);
            // cudaDeviceSynchronize();
            nvtxRangePop();
            // cudaDeviceSynchronize();
            // auto duration = chrono::duration_cast<chrono::nanoseconds>( chrono::high_resolution_clock::now() - ts_last).count();
            // cout <<  duration / 1000000<< "ms" << endl;

            // cout << id_ << endl;

            if (publish_laser_scan_open) // publish laser scan
            {
                sprintf(memory_string, "%d laser scan calculation", id_);
                nvtxRangePushA(memory_string);

                auto angles = torch::atan2(tensor.select(1, 1), tensor.select(1, 0)).to(torch::kCUDA);

                auto pcl_dist = torch::hypot(tensor.select(1, 1), tensor.select(1, 0)).to(torch::kCUDA);

                auto ranges = torch::full({ind_len}, std::numeric_limits<float>::max()).to(torch::kCUDA);

                auto inds = ((angles - laser_msg.angle_min) / laser_msg.angle_increment).to(torch::kInt).to(torch::kCUDA);

                auto index_tensor = torch::arange(inds.sizes()[0]).to(torch::kCUDA);

                ranges.index_put_({inds}, torch::min(ranges.index({inds}), pcl_dist.index({index_tensor})));

                ranges_tensor_cpu = ranges.to(torch::kCPU);
                nvtxRangePop();
            }

            // auto duration = chrono::duration_cast<chrono::nanoseconds>( chrono::high_resolution_clock::now() - ts_last).count();
            // cout<< id_ << " " << duration / 1000000<< "ms" << endl;
        }
        mtx_->unlock();
        nvtxRangePop();
        if (publish_flag)
        {
            //time stamps 
            visulize_marker(publish_marker_open);
            publish_laser_scan(publish_laser_scan_open);
            publish_pointcloud(publish_point_cloud_open);
            publish_pose(publish_pose_open || publish_odom_open);
            
        }
        usleep(5);
    }
}

void deleter(void *arg) {}
void ClientPublisher::to_tensor(sl::Mat &point_cloud, torch::Tensor &tensor)
{

    //  std::vector<int64_t> sizes = {static_cast<int64_t>(point_cloud.getWidth()),
    //                             static_cast<int64_t>(point_cloud.getHeight()),
    //                             static_cast<int64_t>(point_cloud.getChannels())
    //                           };

    // long long step = point_cloud.getStep();

    // std::vector<int64_t> strides = {static_cast<int64_t>(point_cloud.getChannels()) , step*static_cast<int64_t>(point_cloud.getChannels()), 1};

    // tensor = torch::from_blob(point_cloud.getPtr<sl::uchar1>(MEM::GPU), sizes, strides, deleter,  torch::kCUDA);

    std::vector<int64_t> sizes = {
        static_cast<int64_t>(point_cloud.getChannels() - 1),
        static_cast<int64_t>(point_cloud.getHeight()),
        static_cast<int64_t>(point_cloud.getWidth())};

    long long step = point_cloud.getStep();

    std::vector<int64_t> strides = {1, step * static_cast<int64_t>(point_cloud.getChannels()), static_cast<int64_t>(point_cloud.getChannels())};

    tensor = torch::from_blob(
        point_cloud.getPtr<sl::uchar1>(sl::MEM::GPU), sizes, strides, [](void *arg) {}, torch::kCUDA);
    tensor.set_requires_grad(0);
    // std::cout << tensor_image << std::endl;
}

void ClientPublisher::flatten_tensor(torch::Tensor &tensor)
{
    auto size = tensor.sizes();
    std::vector<int64_t> sizes = {
        static_cast<int64_t>(size[1] * size[2]),
        static_cast<int64_t>(size[0])};

    std::vector<int64_t> strides = {1, size[1] * size[2]};

    tensor = torch::from_blob(
        tensor.data_ptr<float>(), sizes, strides, [](void *arg) {}, torch::kCUDA);
    tensor.set_requires_grad(0);
    // std::cout << tensor_image << std::endl;
}
void ClientPublisher::visulize_marker(bool is_open)
{
    if (is_open) // visulize as marker
    {
        sprintf(memory_string, "%d publish marker", id_);
        nvtxRangePushA(memory_string);

        // Define your points as a vector of geometry_msgs::msg::Point
        std::vector<geometry_msgs::msg::Point> points;
        // cudaDeviceSynchronize();
        // duration = chrono::duration_cast<chrono::nanoseconds>( chrono::high_resolution_clock::now() - ts_last).count();
        // cout <<  duration / 1000000<< "ms" << endl  ;
        for (int i = 0; i < cpu_tensor.sizes()[0]; i += 10)
        {

            geometry_msgs::msg::Point point;

            point.x = cpu_tensor[i][0].item<float>();
            point.y = cpu_tensor[i][1].item<float>();
            point.z = cpu_tensor[i][2].item<float>();

            // cout << point.x << " " << point.y << " " << point.z << endl;

            if (isnan(point.x) || isnan(point.y) || isnan(point.z) && (point.x == 0 && point.y == 0 && point.z == 0))
                continue;
            // cout << "ASDasdsad" << endl;
            points.push_back(point);
        }

        // Set marker points
        marker.points = points;

        // Set other marker properties (scale, color, etc.) as needed
        marker.scale.x = voxel_size / 100.0; // Replace with desired scale
        marker.scale.y = voxel_size / 100.0; // Replace with desired scale
        marker.scale.z = voxel_size / 100.0; // Replace with desired scale
        marker.color.r = 1.0;                // Red
        marker.color.g = 0.0;                //[3] Green
        marker.color.b = 0.0;                // Blue
        marker.color.a = 1.0;                // Fully opaque

        // Publish the marker message
        marker.header.stamp.set__nanosec(pointcloud_timestamp_ns); 
        marker.header.stamp.set__sec(pointcloud_timestamp_s);

        marker_publisher->publish(marker);
        nvtxRangePop();
    }
}

void ClientPublisher::publish_laser_scan(bool is_open)
{

    if (is_open)
    {
        sprintf(memory_string, "%d publish laser scan", id_);
        nvtxRangePushA(memory_string);

        auto ptr = ranges_tensor_cpu.data_ptr<float>();
        vector<float> ranges_vector(ptr, ptr + ranges_tensor_cpu.numel());

        laser_msg.ranges = ranges_vector;

        // laser_msg.header.stamp = node_->get_clock()->now();
        laser_msg.header.stamp.set__nanosec(pointcloud_timestamp_ns); 
        laser_msg.header.stamp.set__sec(pointcloud_timestamp_s);

        laserscan_publisher->publish(laser_msg);
        nvtxRangePop();
    }
}

void ClientPublisher::publish_pointcloud(bool is_open)
{
    if (is_open)
    {
        sprintf(memory_string, "%d publish laser scan", id_);
        nvtxRangePushA(memory_string);
        auto size = cpu_tensor.sizes(); // nxm tensor
        pointcloud_msg.height = 1;
        pointcloud_msg.width = size[0];
        pointcloud_msg.row_step = size[0] * pointcloud_msg.point_step;

        // cout << cpu_tensor.strides()<< " " << cpu_tensor.sizes() << " " << cpu_tensor.is_contiguous()<< endl;

        // cpu_tensor = cpu_tensor.view(size[0]*size[1]);
        // cpu_tensor = cpu_tensor.contiguous();

        // cout << cpu_tensor.strides()<< " " << cpu_tensor.sizes() << " " << cpu_tensor.is_contiguous() << endl;
        // cout <<endl;
        // vector<uint8_t> data_vector;
        // for(int i = 0 ; i < cpu_tensor.sizes()[0]; i += 1)
        // {
        //     float a = cpu_tensor[i][0].item<float>();
        //     unsigned char *pFloat = reinterpret_cast<unsigned char *>(&a);
        //     data_vector.push_back(pFloat[0]);
        //     data_vector.push_back(pFloat[1]);
        //     data_vector.push_back(pFloat[2]);
        //     data_vector.push_back(pFloat[3]);
        //     a = cpu_tensor[i][1].item<float>();
        //     pFloat = reinterpret_cast<unsignezed.close();d char *>(&a);
        //     data_vector.push_back(pFloat[0]);
        //     data_vector.push_back(pFloat[1]);
        //     data_vector.push_back(pFloat[2]);
        //     data_vector.push_back(pFloat[3]);
        //     a = cpu_tensor[i][2].item<float>();
        //     pFloat = reinterpret_cast<unsigned char *>(&a);
        //     data_vector.push_back(pFloat[0]);
        //     data_vector.push_back(pFloat[1]);
        //     data_vector.push_back(pFloat[2]);
        //     data_vector.push_back(pFloat[3]);
        // }
        auto ptr = cpu_tensor.data_ptr<float>();
        uint8_t *ibegin = reinterpret_cast<uint8_t *>(ptr);
        vector<uint8_t> data_vector(ibegin, ibegin + sizeof(float) * cpu_tensor.numel());

        pointcloud_msg.data = data_vector;

        // pointcloud_msg.header.stamp = node_->get_clock()->now();
        pointcloud_msg.header.stamp.set__nanosec(pointcloud_timestamp_ns); 
        pointcloud_msg.header.stamp.set__sec(pointcloud_timestamp_s);
        pointcloud_publisher->publish(pointcloud_msg);

        nvtxRangePop();
    }
}

void ClientPublisher::publish_pose(bool is_open)
{
    if (zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD) == sl::POSITIONAL_TRACKING_STATE::OK && is_open)
    {
        sprintf(memory_string, "%d publish pose", id_);
        nvtxRangePushA(memory_string);

        auto trans = pose.getTranslation();
        auto orient = pose.getOrientation();
        auto covariance = pose.pose_covariance;

        for (int i = 0; i < 36; i++)
            pose_msg.pose.covariance[i] = (double)covariance[i];

        pose_msg.pose.pose.position.x = trans(0);
        pose_msg.pose.pose.position.y = trans(1);
        pose_msg.pose.pose.position.z = trans(2);

        pose_msg.pose.pose.orientation.x = orient(0);
        pose_msg.pose.pose.orientation.y = orient(1);
        pose_msg.pose.pose.orientation.z = orient(2);
        pose_msg.pose.pose.orientation.w = orient(3);
        pose_msg.header.stamp = node_->get_clock()->now();
        // pose_msg.header.stamp.set__nanosec(pointcloud_timestamp_ns);
        // pose_msg.header.stamp.set__sec(pointcloud_timestamp_s);
        if(publish_pose_open)
            pose_publisher->publish(pose_msg);

        publish_odom(publish_odom_open);

        nvtxRangePop();
    }
}


void ClientPublisher::publish_odom(bool is_open)
{
    if (zed.getPosition(pose, sl::REFERENCE_FRAME::WORLD) == sl::POSITIONAL_TRACKING_STATE::OK && is_open)
    {
        sprintf(memory_string, "%d publish odom", id_);
        nvtxRangePushA(memory_string);
        odom_msg.set__pose(pose_msg.pose);
        odom_msg.header.stamp.set__nanosec(pointcloud_timestamp_ns);
        odom_msg.header.stamp.set__sec(pointcloud_timestamp_s);
        odom_publisher->publish(odom_msg);
       
        nvtxRangePop();
    }
}

void ClientPublisher::publish_sensors()
{
    
    while(running)
    {
        
        publish_imu(publish_imu_open);
 
        
    }
    
}
void ClientPublisher::publish_imu(bool is_open)
{
    if(is_open)
    {
        if (zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) 
        {
            if (ts.isNew(sensors_data.imu))
            { 
                imu_msg.linear_acceleration.x = (double)sensors_data.imu.linear_acceleration.x;
                imu_msg.linear_acceleration.y = (double)sensors_data.imu.linear_acceleration.y;
                imu_msg.linear_acceleration.z = (double)sensors_data.imu.linear_acceleration.z;

                imu_msg.angular_velocity.x = (double)sensors_data.imu.angular_velocity.x * 3.141592653589793 /180;
                imu_msg.angular_velocity.y = (double)sensors_data.imu.angular_velocity.y * 3.141592653589793 /180;
                imu_msg.angular_velocity.z = (double)sensors_data.imu.angular_velocity.z * 3.141592653589793 /180;
                auto Orientation = sensors_data.imu.pose.getOrientation();
                imu_msg.orientation.x = Orientation(0); 
                imu_msg.orientation.y = Orientation(1); 
                imu_msg.orientation.z = Orientation(2); 
                imu_msg.orientation.w = Orientation(3); 

                for (int i = 0; i < 9; i++)
                {
                    imu_msg.linear_acceleration_covariance[i] = sensors_data.imu.linear_acceleration_covariance.r[i];
                    imu_msg.angular_velocity_covariance[i] = sensors_data.imu.angular_velocity_covariance.r[i];
                    imu_msg.orientation_covariance[i] = sensors_data.imu.pose_covariance.r[i];
                }
                imu_msg.header.stamp.set__nanosec(sensors_data.imu.timestamp.getNanoseconds()); 
                imu_msg.header.stamp.set__sec(sensors_data.imu.timestamp.getSeconds());
                imu_publisher->publish(imu_msg);
                // if (ts.isNew(sensors_data.magnetometer))
                //     cout << " - Magnetometer\n \t Magnetic Field: {" << sensors_data.magnetometer.magnetic_field_calibrated << "} [uT]\n";
                // cout << " \t Orientation: {" << sensors_data.imu.pose.getOrientation() << "}\n";
                // = node_->get_clock()->now();
                // cout << "Linear Acceleration: " << sensors_data.imu.linear_acceleration << " ";
                // cout << "Angular Velocity: " << sensors_data.imu.angular_velocity << endl;
            }
        }
    }


}



