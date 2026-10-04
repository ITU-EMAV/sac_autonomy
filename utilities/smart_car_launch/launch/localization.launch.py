"""Autoware tabanli lokalizasyon yigini (NDT + EKF).

Ayni dosya hem canli sensorlerle hem bag kaydiyla calisir; fark sadece
launch argumanlarindadir. Varsayilanlar canli arac icindir.

Dugumler:
  * ndt_scan_matcher      : LiDAR + PCD harita -> NDT pozu
  * ekf_localizer         : NDT pozu + GNSS pozu + twist -> filtrelenmis poz, map->base_link TF
  * pose_initializer      : Ilk pozu hizalar, NDT ve EKF'i tetikler
  * sac_pose_initializer  : RViz ilk pozunu map frame'ine tasir
  * pointcloud_map_loader : PCD haritayi NDT'ye servis eder   (launch_map_loader)
  * lanelet2_map_loader   : Publishes the OSM map on /map/lanelet2_map
  * lanelet visualizer    : Converts the OSM map to an RViz MarkerArray
  * twist zinciri         : twist_source argumanina gore secilir
  * statik TF'ler         : publish_static_tf ile acilir/kapanir
  * rviz2                 : launch_rviz ile acilir/kapanir (varsayilan acik)

Veri akisi:
    <pointcloud> ─> ndt_scan_matcher ─> .../ndt_localizer/pose_with_cov ─┐
                          ^                                              │
                          └── .../ekf_localizer/pose_with_cov <── ekf_localizer
    <twist> ────────────────────────────────────────────────────────────┘

twist_source secenekleri:
    gyro_odometer : arac hizi (Float32) + IMU -> gyro_odometer   [varsayilan]
    eagleye       : /eagleye/twist_with_covariance dogrudan kullanilir
    topic         : input_twist_topic dogrudan EKF'e verilir

Ornek — canli arac:
    ros2 launch smart_car_launch localization.launch.py

Ornek — bag kaydi (TF cakismasini onlemek icin bag'in /tf'i oynatilmaz):
    ros2 launch smart_car_launch localization.launch.py use_sim_time:=true
    ros2 bag play without_mag --clock \\
        --topics /velodyne_points /encoder_speed /zed/zed_node/imu/data
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# Yigin ici sabit topic / servis isimleri
NDT_POSE_COV = '/localization/ndt_localizer/pose_with_cov'
EKF_POSE_COV = '/localization/ekf_localizer/pose_with_cov'
EKF_TRIGGER = '/localization/ekf_localizer/trigger_node_service'
NDT_TRIGGER = '/localization/ndt_localizer/trigger_node_service'
NDT_ALIGN = '/localization/ndt_localizer/align_service'
MAP_SERVICE = '/map/get_differential_pointcloud_map'
VEHICLE_TWIST = '/vehicle/twist_with_covariance'
NDT_DEBUG = '/localization/ndt_localizer/debug'



def _default_use_sim_time():
    """Default for use_sim_time, from smart_car_launch/config/use_sim_time.yaml."""
    path = os.path.join(get_package_share_directory('smart_car_launch'),
                        'config', 'use_sim_time.yaml')
    with open(path, encoding='utf-8') as config_file:
        return str(bool((yaml.safe_load(config_file) or {})['use_sim_time'])).lower()

def _bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('true', '1', 'yes')


def _setup(context, *_, **__):
    share = get_package_share_directory('smart_car_launch')
    config_dir = os.path.join(share, 'config', 'localization')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar_frame = LaunchConfiguration('lidar_frame')
    twist_source = LaunchConfiguration('twist_source').perform(context)
    use_gnss = _bool(context, 'use_gnss')
    map_z = LaunchConfiguration('map_z').perform(context)
    auto_init = _bool(context, 'auto_init')

    nodes = []

    # ------------------------------------------------------------------ #
    # Aracin sabit TF agaci URDF'den robot_state_publisher ile yayinlanir.
    # Bag /tf_static iceriyorsa cakismayi onlemek icin kapatilabilir.
    # ------------------------------------------------------------------ #
    if _bool(context, 'publish_static_tf'):
        robot_xacro = os.path.join(share, 'urdf', 'smart_car.urdf.xacro')
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='smart_car_state_publisher',
            output='both',
            parameters=[{
                'robot_description': Command([
                    'xacro ', robot_xacro,
                    ' lidar_frame:=', lidar_frame,
                    ' imu_frame:=', LaunchConfiguration('imu_frame'),
                    # Bosluk iceren deger tirnaklanmali, yoksa xacro onu
                    # ayri girdi dosyalari sanip hata verir.
                    ' lidar_rpy:="', LaunchConfiguration('lidar_rpy'), '"',
                ]),
                'use_sim_time': use_sim_time,
            }],
        ))

    # ------------------------------------------------------------------ #
    # Harita yukleyici
    # Tek PCD verildiginde map_loader metadata'yi kendi uretir.
    # Dugum adi 'pointcloud_map_loader' olmali (map_height_fitter bunu arar).
    # ------------------------------------------------------------------ #
    if _bool(context, 'launch_map_loader'):
        # pcd_paths_or_directory string_array'dir. [LaunchConfiguration(...)]
        # yazilirsa launch bunu substitution listesi sayip tek string'e
        # birlestirir ve dugum InvalidParameterTypeException ile olur.
        # OpaqueFunction icindeyiz; degeri cozup gercek liste veriyoruz.
        map_path = LaunchConfiguration('pointcloud_map_path').perform(context)
        map_metadata = LaunchConfiguration('pointcloud_map_metadata_path').perform(context)
        nodes.append(Node(
            package='autoware_map_loader', executable='autoware_pointcloud_map_loader',
            name='pointcloud_map_loader', output='both',
            parameters=[
                {
                    # NDT haritayi partial_load SERVISI uzerinden alir; asagidaki
                    # topic yayinlari sadece gorsellestirme / map_height_fitter icin.
                    'enable_whole_load': True,

                    # RViz'e seyreltilmis harita ver. SADECE gorsellestirme;
                    # NDT bu topic'i kullanmaz, haritayi
                    # get_differential_pointcloud_map servisinden alir.
                    #
                    # NDT'ye giden harita artik GlobalMap_ds05.pcd (0.5 m
                    # centroid, 1.766.814 nokta) -- bkz. config/maps/map_config.yaml.
                    # Buradaki leaf de 0.5 yapildi, yani RViz pratikte ayni
                    # bulutu goruyor. Ayri bir sayi tutmak, gecmiste oldugu
                    # gibi ikisinin birbirinden kaymasina yol aciyordu.
                    #
                    # RViz agirlasirsa BURAYI buyutun (0.7 -> ~971k nokta,
                    # 1.0 -> ~494k); NDT'yi etkilemez.
                    'enable_downsampled_whole_load': True,
                    'leaf_size': 0.5,

                    'enable_partial_load': True,
                    'enable_selected_load': False,
                    'pcd_paths_or_directory': [map_path],
                    'pcd_metadata_path': map_metadata,
                    'use_sim_time': use_sim_time,
                },
            ],
            remappings=[
                ('output/pointcloud_map', '/map/pointcloud_map'),
                ('output/debug/downsampled_pointcloud_map', '/map/downsampled_pointcloud_map'),
                ('service/get_partial_pcd_map', '/map/get_partial_pointcloud_map'),
                ('service/get_differential_pcd_map', MAP_SERVICE),
            ],
        ))

    # Vector-map nodes are owned by map.launch.py so the same map stack can be
    # launched independently or as part of localization.
    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'map.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_pointcloud_map': 'false',
            'launch_lanelet_map': LaunchConfiguration('launch_lanelet_map'),
            'lanelet2_map_path': LaunchConfiguration('lanelet2_map_path'),
            'map_projector_info_path': LaunchConfiguration('map_projector_info_path'),
            'map_projector_info_topic': LaunchConfiguration('map_projector_info_topic'),
            'lanelet2_map_topic': LaunchConfiguration('lanelet2_map_topic'),
            'lanelet2_map_marker_topic': LaunchConfiguration('lanelet2_map_marker_topic'),
        }.items(),
    ))

    if auto_init:
        pose_file = LaunchConfiguration('initial_pose_file').perform(context)
        if not os.path.isfile(pose_file):
            raise RuntimeError(f'Saved initial pose file not found: {pose_file}')
        nodes.append(Node(
            package='smart_car_launch', executable='saved_initial_pose.py',
            name='saved_initial_pose', output='both',
            arguments=['--pose-file', pose_file,
                       '--scan-topic', LaunchConfiguration('filtered_pointcloud_topic')],
            parameters=[{'use_sim_time': use_sim_time}],
        ))

    # ------------------------------------------------------------------ #
    # LiDAR on isleme
    # Ham velodyne bulutu organize ve is_dense=False, yani NaN iceriyor.
    # autoware_ndt_scan_matcher bunlari filtrelemeden PCL'e verdigi icin
    # ndt_align sirasinda "Invalid (NaN, Inf) ... radiusSearch" ile cokuyor.
    # Autoware'de bu is pointcloud_preprocessor zincirinde yapilir.
    # ------------------------------------------------------------------ #
    ndt_input_topic = LaunchConfiguration('input_pointcloud_topic')
    if _bool(context, 'filter_pointcloud'):
        filtered_topic = LaunchConfiguration('filtered_pointcloud_topic').perform(context)
        nodes.append(Node(
            package='sac_pointcloud_filter', executable='pointcloud_filter',
            name='pointcloud_filter', output='both',
            parameters=[{
                'input_topic': ndt_input_topic,
                'output_topic': filtered_topic,
                'min_range': LaunchConfiguration('filter_min_range'),
                'max_range': LaunchConfiguration('filter_max_range'),
                'voxel_leaf_size': LaunchConfiguration('filter_voxel_leaf_size'),
                'use_sim_time': use_sim_time,
            }],
        ))
        ndt_input_topic = filtered_topic

    # ------------------------------------------------------------------ #
    # Twist kaynagi
    # ------------------------------------------------------------------ #
    if twist_source == 'gyro_odometer':
        twist_topic = '/localization/twist_with_covariance'
        # Arac boylamsal hizi (Float32) -> TwistWithCovarianceStamped
        nodes.append(Node(
            package='sac_utils', executable='vehicle_twist_converter',
            name='vehicle_twist_converter', output='both',
            parameters=[{
                'input_type': 'float32',
                'input_topic': LaunchConfiguration('input_vehicle_speed_topic'),
                'output_topic': VEHICLE_TWIST,
                'output_frame_id': 'base_link',
                'use_sim_time': use_sim_time,
            }],
        ))
        # vx (arac) + wz (IMU) -> EKF twist'i
        nodes.append(Node(
            package='gyro_odometer', executable='gyro_odometer_node',
            name='gyro_odometer', output='both',
            parameters=[
                os.path.join(get_package_share_directory('gyro_odometer'),
                             'config', 'gyro_odometer.param.yaml'),
                # Config'te 'lidar_front' yaziyor; bu kurulumda base_link dogru.
                {'output_frame': 'base_link', 'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('vehicle/twist_with_covariance', VEHICLE_TWIST),
                ('imu', LaunchConfiguration('input_imu_topic')),
                ('twist_raw', '/localization/gyro_twist_raw'),
                ('twist_with_covariance_raw', '/localization/gyro_twist_with_covariance_raw'),
                ('twist', '/localization/gyro_twist'),
                ('twist_with_covariance', twist_topic),
            ],
        ))
    elif twist_source == 'eagleye':
        # eagleye ayrica calistirilmalidir (eagleye_rt.launch.xml)
        twist_topic = '/eagleye/twist_with_covariance'
    elif twist_source == 'topic':
        twist_topic = LaunchConfiguration('input_twist_topic').perform(context)
    else:
        raise RuntimeError(
            f"twist_source '{twist_source}' gecersiz; "
            "'gyro_odometer', 'eagleye' veya 'topic' olmali")

    # ------------------------------------------------------------------ #
    # NDT scan matcher
    # ------------------------------------------------------------------ #
    nodes.append(Node(
        package='autoware_ndt_scan_matcher', executable='autoware_ndt_scan_matcher_node',
        name='ndt_scan_matcher', output='both',
        parameters=[
            # 1) apt taban config  2) bu arac/donanim icin ayarlar  3) launch override
            '/opt/ros/humble/share/autoware_ndt_scan_matcher/config/ndt_scan_matcher.param.yaml',
            os.path.join(config_dir, 'ndt_scan_matcher_tuning.param.yaml'),
            # NDT result is a map -> base_link pose. Publish its optional TF
            # under a dedicated debug child so it cannot give velodyne a
            # second parent or compete with EKF's map -> base_link TF.
            # Sadece launch'a ozgu / calisma aninda degisen degerler burada.
            # Sayisal ayarlar ndt_scan_matcher_tuning.param.yaml icinde.
            {
                'frame.ndt_base_frame': LaunchConfiguration('ndt_debug_frame'),
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('points_raw', ndt_input_topic),
            ('ekf_pose_with_covariance', EKF_POSE_COV),
            ('regularization_pose_with_covariance',
             LaunchConfiguration('input_gnss_pose_with_cov_topic')),
            ('trigger_node_srv', NDT_TRIGGER),
            ('ndt_align_srv', NDT_ALIGN),
            ('pcd_loader_service', MAP_SERVICE),
            ('ndt_pose', '/localization/ndt_localizer/pose'),
            ('ndt_pose_with_covariance', NDT_POSE_COV),
            ('initial_pose_with_covariance', '/localization/ndt_localizer/initial_pose'),
            ('ndt_marker', '/localization/ndt_localizer/marker'),
            ('debug/loaded_pointcloud_map', f'{NDT_DEBUG}/loaded_map'),
            ('exe_time_ms', f'{NDT_DEBUG}/exe_time_ms'),
            ('initial_to_result_distance', f'{NDT_DEBUG}/initial_to_result_distance'),
            ('initial_to_result_distance_new', f'{NDT_DEBUG}/initial_to_result_distance_new'),
            ('initial_to_result_distance_old', f'{NDT_DEBUG}/initial_to_result_distance_old'),
            ('initial_to_result_relative_pose', f'{NDT_DEBUG}/initial_to_result_relative_pose'),
            ('iteration_num', f'{NDT_DEBUG}/iteration_num'),
            ('monte_carlo_initial_pose_marker', f'{NDT_DEBUG}/monte_carlo_initial_pose_marker'),
            ('multi_initial_pose', f'{NDT_DEBUG}/multi_initial_pose'),
            ('multi_ndt_pose', f'{NDT_DEBUG}/multi_ndt_pose'),
            ('nearest_voxel_transformation_likelihood',
             f'{NDT_DEBUG}/nearest_voxel_transformation_likelihood'),
            ('no_ground_nearest_voxel_transformation_likelihood',
             f'{NDT_DEBUG}/no_ground_nearest_voxel_transformation_likelihood'),
            ('no_ground_transform_probability',
             f'{NDT_DEBUG}/no_ground_transform_probability'),
            ('points_aligned', f'{NDT_DEBUG}/points_aligned'),
            ('points_aligned_no_ground', f'{NDT_DEBUG}/points_aligned_no_ground'),
            ('transform_probability', f'{NDT_DEBUG}/transform_probability'),
        ],
    ))

    # ------------------------------------------------------------------ #
    # EKF localizer  (map -> base_link TF'ini bu dugum yayinlar)
    # ------------------------------------------------------------------ #
    nodes.append(Node(
        package='autoware_ekf_localizer', executable='autoware_ekf_localizer_node',
        name='ekf_localizer', output='both',
        parameters=[
            os.path.join(get_package_share_directory('autoware_ekf_localizer'),
                         'config', 'ekf_localizer.param.yaml'),
            os.path.join(config_dir, 'ekf_localizer_tuning.param.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('initialpose', 'initialpose3d'),
            ('trigger_node_srv', EKF_TRIGGER),
            ('in_pose_with_covariance', NDT_POSE_COV),
            ('in_twist_with_covariance', twist_topic),
            ('in_gnss_pose_with_covariance',
             LaunchConfiguration('input_gnss_pose_with_cov_topic')),
            ('ekf_odom', '/localization/ekf_localizer/odom'),
            ('ekf_pose', '/localization/ekf_localizer/pose'),
            ('ekf_pose_with_covariance', EKF_POSE_COV),
            ('ekf_biased_pose', '/localization/ekf_localizer/biased_pose'),
            ('ekf_biased_pose_with_covariance',
             '/localization/ekf_localizer/biased_pose_with_cov'),
            ('ekf_twist', '/localization/ekf_localizer/twist'),
            ('ekf_twist_with_covariance', '/localization/ekf_localizer/twist_with_cov'),
            ('estimated_yaw_bias', '/localization/ekf_localizer/estimated_yaw_bias'),
        ],
    ))

    # ------------------------------------------------------------------ #
    # Poz baslaticilar
    # use_gnss=false iken ilk poz RViz "2D Pose Estimate" ile verilir.
    # ------------------------------------------------------------------ #
    nodes.append(Node(
        package='autoware_pose_initializer', executable='autoware_pose_initializer_node',
        name='pose_initializer', output='both',
        parameters=[
            os.path.join(config_dir, 'core19_pose_initializer.param.yaml'),
            {'gnss_enabled': use_gnss, 'use_sim_time': use_sim_time},
        ],
        remappings=[
            ('ndt_align', NDT_ALIGN),
            ('gnss_pose_cov', LaunchConfiguration('input_gnss_pose_with_cov_topic')),
            ('pose_reset', 'initialpose3d'),
            ('ekf_trigger_node', EKF_TRIGGER),
            ('ndt_trigger_node', NDT_TRIGGER),
            ('~/pointcloud_map', '/map/pointcloud_map'),
            ('~/partial_map_load', '/map/get_partial_pointcloud_map'),
            ('~/vector_map', LaunchConfiguration('lanelet2_map_topic')),
        ],
    ))

    nodes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sac_pose_initializer'),
                         'launch', 'sac_pose_initializer.launch.py')),
        launch_arguments={
            'param_file': os.path.join(config_dir, 'sac_pose_initializer.param.yaml'),
            'use_sim_time': use_sim_time,
            'map_z': map_z,
        }.items(),
    ))

    # ------------------------------------------------------------------ #
    # NDT kurtarma watchdog'u
    # NDT bir kez yayin yapmayi birakinca kendini kilitliyor (yayin yok ->
    # EKF olu hesaba duser -> baslangic tahmini kayar -> hic converge olmaz).
    # Bu dugum skipping_publish_num'i izler ve kilitlenmede mevcut EKF pozu
    # etrafinda genis yaricapli otomatik yeniden hizalama tetikler.
    # ------------------------------------------------------------------ #
    if _bool(context, 'launch_ndt_watchdog'):
        nodes.append(Node(
            package='sac_pose_initializer', executable='ndt_recovery_watchdog_node',
            name='ndt_recovery_watchdog', output='both',
            parameters=[{
                'enabled': LaunchConfiguration('ndt_watchdog_enabled'),
                'skipping_publish_threshold': LaunchConfiguration('ndt_watchdog_threshold'),
                'hold_sec': LaunchConfiguration('ndt_watchdog_hold_sec'),
                'cooldown_sec': LaunchConfiguration('ndt_watchdog_cooldown_sec'),
                'search_stddev_m': LaunchConfiguration('ndt_watchdog_search_m'),
                'ekf_pose_topic': EKF_POSE_COV,
                'use_sim_time': use_sim_time,
            }],
        ))

    # ------------------------------------------------------------------ #
    # RViz
    # ------------------------------------------------------------------ #
    rviz_arg = LaunchConfiguration('launch_rviz').perform(context).lower()
    launch_rviz = not auto_init if rviz_arg == 'auto' else rviz_arg in ('true', '1', 'yes')
    if launch_rviz:
        rviz_config = LaunchConfiguration('rviz_config').perform(context)
        if not rviz_config:
            rviz_config = os.path.join(share, 'rviz', 'localization.rviz')
        nodes.append(Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='log',
        ))

    return nodes


def generate_launch_description():
    share = get_package_share_directory('smart_car_launch')
    map_config_path = os.path.join(share, 'config', 'maps', 'map_config.yaml')
    with open(map_config_path, encoding='utf-8') as config_file:
        map_config = yaml.safe_load(config_file) or {}

    def map_path_from_config(key, fallback=''):
        value = map_config.get(key, fallback)
        if not value or os.path.isabs(value):
            return value
        return os.path.join(share, 'maps', value)

    default_map = map_path_from_config(
        'pointcloud_map_path', 'generated/lio_sam/GlobalMap.pcd')
    default_map_metadata = map_path_from_config(
        'pointcloud_map_metadata_path')
    default_lanelet_map = map_path_from_config('lanelet2_map_path')
    default_map_projector_info = map_path_from_config('map_projector_info_path')

    args = [
        # --- Harita ---
        DeclareLaunchArgument('pointcloud_map_path', default_value=default_map,
                              description='NDT icin PCD harita'),
        DeclareLaunchArgument('pointcloud_map_metadata_path',
                              default_value=default_map_metadata,
                              description='PCD metadata; bos ise tek PCD icin otomatik uretilir'),
        DeclareLaunchArgument('launch_map_loader', default_value='true',
                              description='map.launch.py ayrica calisiyorsa false yapin'),
        DeclareLaunchArgument('lanelet2_map_path', default_value=default_lanelet_map,
                              description='Lanelet2 OSM map'),
        DeclareLaunchArgument('map_projector_info_path',
                              default_value=default_map_projector_info,
                              description='Map projector information YAML'),
        DeclareLaunchArgument('map_projector_info_topic',
                              default_value='/map/map_projector_info'),
        DeclareLaunchArgument('lanelet2_map_topic', default_value='/map/vector_map'),
        DeclareLaunchArgument('lanelet2_map_marker_topic',
                              default_value='/map/lanelet2_map_marker'),
        DeclareLaunchArgument('launch_lanelet_map', default_value='true',
                              description='Start the Lanelet2 loader and RViz marker publisher'),

        # --- Sensor girisleri ---
        DeclareLaunchArgument('input_pointcloud_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('filter_pointcloud', default_value='true',
                              description='Ham buluttan NaN temizle (NDT icin gerekli)'),
        DeclareLaunchArgument('filtered_pointcloud_topic',
                              default_value='/sensing/points_filtered'),
        DeclareLaunchArgument('filter_min_range', default_value='1.0'),
        DeclareLaunchArgument('filter_max_range', default_value='80.0',
                              description='NDT girdisinin ust menzili [m]. '
                                          '120 m idi: VLP-16 orada zaten cok az donus '
                                          'veriyor ve o noktalar map_radius=150 / '
                                          'lidar_radius=100 yerel haritasinin disina '
                                          'dustugu icin skora katki vermeden iterasyon '
                                          'maliyeti yaratiyordu. 80 m, exe_time\'i '
                                          '100 ms LiDAR periyodunun altinda tutar.'),
        DeclareLaunchArgument('filter_voxel_leaf_size', default_value='0.5',
                              description='NDT girdisine giden KAYNAK taramayi seyreltir [m]. '
                                          '0 = seyreltme yok. ndt.resolution ILE AYNI DEGIL, '
                                          'bilerek: esitlemek (2.5) olculerek elendi. '
                                          'without_mag olcumu, tarama basina (1-80 m, NaN '
                                          'atilmis, ham 21.307 nokta) -- ikinci kolon x/y/yaw\'a '
                                          'bilgi katan noktalar (>20 m VE zemin disi): '
                                          'leaf 0.3 -> 9.042 / 4.200 | 0.5 -> 6.333 / 3.287 | '
                                          '1.0 -> 3.433 / 2.120 | 2.5 -> 1.136 / 821. '
                                          'Voxel filtresi bulutu saflastiriyor (ham %27 -> '
                                          '2.5\'te %72) ama MUTLAK bilgi tasiyan nokta 4x '
                                          'dusuyordu; yaw varyansi ~1/N ile gittigi icin bu '
                                          '~2x kotu yaw std demekti. Yakin alan zemin '
                                          'noktalari yaw cozumunu BOZMUYOR, sadece yaw\'a '
                                          'duyarsiz terim ekliyor -- maliyeti var, hatasi yok. '
                                          '0.5 olculerek uygun: res 2.0 / 8 thread ile '
                                          'hizalama medyan 28 ms, p95 113 ms, maks 144 ms '
                                          '(100 ms LiDAR periyodu icinde). 0.3 denenmedi, '
                                          'p95 butceyi asabilir.'),
        DeclareLaunchArgument('input_vehicle_speed_topic', default_value='/encoder_speed',
                              description='Arac boylamsal hizi (std_msgs/Float32, m/s)'),
        DeclareLaunchArgument('input_imu_topic', default_value='/zed/zed_node/imu/data'),
        DeclareLaunchArgument('input_gnss_pose_with_cov_topic',
                              default_value='/sensing/gnss/gnss_pose_with_cov'),
        DeclareLaunchArgument('input_twist_topic',
                              default_value='/localization/twist_with_covariance',
                              description="twist_source:=topic iken kullanilir"),

        # --- Frame'ler ---
        DeclareLaunchArgument('lidar_frame', default_value='velodyne'),
        DeclareLaunchArgument('ndt_debug_frame', default_value='ndt_base_link',
                              description='Ham NDT sonucu icin ayri debug TF child frame'),
        DeclareLaunchArgument('imu_frame', default_value='zed_imu_link'),
        DeclareLaunchArgument('lidar_rpy', default_value='0 0 0',
                              description="base_link -> LiDAR montaj acisi (roll pitch yaw). "
                                          "LIO-SAM haritasi ham velodyne frame'inde kuruldugu "
                                          "icin o haritayla '0 0 0' kullanilmalidir."),
        DeclareLaunchArgument('publish_static_tf', default_value='true',
                              description='URDF TF agacini robot_state_publisher ile yayinla'),

        # --- Davranis ---
        DeclareLaunchArgument('twist_source', default_value='gyro_odometer',
                              description='gyro_odometer | eagleye | topic'),
        DeclareLaunchArgument('use_gnss', default_value='false',
                              description='true ise ilk poz GNSS ile verilir'),
        DeclareLaunchArgument('auto_init', default_value='false',
                              description='Publish the saved RViz pose after the map and first scan are ready'),
        DeclareLaunchArgument('initial_pose_file', default_value=os.path.join(
            share, 'config', 'localization', 'saved_initial_pose.yaml')),
        DeclareLaunchArgument('map_z', default_value='0.0',
                              description='sac_pose_initializer icin YEDEK zemin yuksekligi [m]. '
                                          'Artik birincil kaynak degil: height_source="map" ile '
                                          'z, harita bulutundan (x,y) cevresindeki zeminden '
                                          'okunuyor. Bu deger sadece harita henuz gelmediyse '
                                          'veya poz haritanin disindaysa kullanilir ve WARN '
                                          'basilir. Bos ise config\'teki deger.'),
        DeclareLaunchArgument('use_sim_time', default_value=_default_use_sim_time(),
                              description='Bag oynatirken true'),

        # --- Gorsellestirme ---
        # Kapali: realign sirasinda initialize servisi EKF'i ~2 s deaktive
        # ediyor (TF donuyor) ve EKF'i hizalamanin basladigi andaki eski pozla
        # baslatiyor; seyir halinde EKF geride kalip Mahalanobis ile NDT'yi
        # reddediyor ve watchdog tekrar tetikleniyordu.
        DeclareLaunchArgument('launch_ndt_watchdog', default_value='false',
                              description='NDT kilitlenme kurtarma watchdogunu baslat'),
        DeclareLaunchArgument('ndt_watchdog_enabled', default_value='true',
                              description='false: sadece raporlar, initialize servisini CAGIRMAZ '
                                          '(esik ayarlarken guvenli mod)'),
        DeclareLaunchArgument('ndt_watchdog_threshold', default_value='20',
                              description='Kurtarma dusunulmeden onceki ardisik reddedilen '
                                          'NDT tahmini sayisi. NDT ~7 Hz -> 20 ~= 3 s. '
                                          'Bag olcumunde saglikli suruste bu sayac 1i gecmedi.'),
        DeclareLaunchArgument('ndt_watchdog_hold_sec', default_value='3.0',
                              description='Esik asildiktan sonra tetiklemeden once beklenen sure'),
        DeclareLaunchArgument('ndt_watchdog_cooldown_sec', default_value='30.0',
                              description='Iki kurtarma arasi asgari sure. Align ~5 s surer ve '
                                          'o sure boyunca lokalizasyon YOKTUR.'),
        DeclareLaunchArgument('ndt_watchdog_search_m', default_value='5.0',
                              description='Alignin arayacagi yaricap [m]. Olculen kayma ~5.25 m.'),
        DeclareLaunchArgument('launch_rviz', default_value='auto',
                              description='auto: open RViz when auto_init is false; close it when true'),
        DeclareLaunchArgument('rviz_config', default_value='',
                              description="Bos ise smart_car_launch/rviz/localization.rviz"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
