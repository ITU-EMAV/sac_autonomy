# Jetson perception modelleri

Planlama ekibine gönderilecek kamera, LiDAR küme ve occupancy grid topic'leri,
mesaj alanları ve sınırlar özeti:
[Planlama için perception arayüzü](docs/PLANNING_HANDOFF.md).

Bu paket, ZED2'nin `/zed/zed_node/left/image_rect_color` görüntüsünü üç ROS 2
node'unda dört modelle işler. Varsayılan `perception.launch.py` yol/şerit,
levha, genel nesne ve yol tehlikesi algılamasını birlikte açar. Modeller
Jetson AGX Orin üzerinde TensorRT FP16 engine ile çalışır. YOLOPv2 engine
dosyası yoksa launch `yolopv2.pt` dosyasına döner.

## Modeller ve ölçülen hız

| Model | Varsayılan dosya | Ne bulur? | Node | Ölçülen hız |
| --- | --- | --- | --- | ---: |
| YOLOPv2 | `models/yolopv2_road_masks_fp16.engine` | Sürülebilir yol alanı ve şerit çizgileri | `yolopv2_road_node` | yaklaşık 13–15 FPS |
| Traffic sign, 22 sınıf | `models/traffic_sign_22cls.engine` | Trafik levhaları ve ışıkları | `traffic_sign_node` | yaklaşık 11–12 FPS |
| YOLOv8n COCO | `models/yolov8n.engine` | İnsan, otomobil, motosiklet, otobüs ve kamyon | `traffic_sign_node` | yaklaşık 11–12 FPS |
| Road hazard | `models/road_hazard/road_hazard_fp16.engine` | Çukur (`Pothole`) ve kasis (`Speed-Bump`) | `road_hazard_node` | yaklaşık 11,6 Hz |

**Ölçüm koşulu:** 26 Eylül 2026'da `new_bag` oynatılırken dört model birlikte
çalıştırıldı. Görsel önizleme ve GPU paylaşım kilidi açıktı. Yol/şerit ve
levha/nesne hızları çalışan node'ların 30 karelik log pencerelerinden; hazard
hızı `/perception/road_hazards/timing` topic ölçümünden alındı. Bunlar sabit
performans garantisi değildir. `traffic_sign_node` iki modeli sırayla
çalıştırdığı için toplam 23–25 FPS hızı model başına yaklaşık 11–12 FPS'tir.
Yol ve birleşik levha/nesne düğümlerinde yapay FPS sınırı kaldırılmıştır
(`max_fps: 0.0`); hazard üst sınırı 15 FPS'tir. Önceki 10/15/5 FPS sınırlarıyla
aynı bag'de yaklaşık 8,5/11,7/4,6 Hz ölçülmüştü.

Bu hızlar 20 km/sa sürüş için yeterlilik kanıtı değildir. Araç bu hızda
yaklaşık 5,56 m/s ilerler; 11,6 Hz hazard çıktıları arasında yaklaşık 0,48 m
yol alır. Karar ve fren gecikmesi ayrıca ölçülmelidir. `new_bag` kayıtlı depth
içermediğinden bu testte hazard mesafesi hesaplanmaz.

YOLOPv2'nin 30 bag karesindeki yalnız model çıkarımı ortalama 24,8 ms (`.pt`)
ve 8,3 ms (`.engine`) ölçüldü. ROS logundaki `infer` süresi GPU kilidinde
beklemeyi de kapsar; bu nedenle model süresi ile canlı FPS doğrudan aynı şey
değildir. Ayrıntılar [TensorRT dokümanında](docs/05_tensorrt_fp16.md) ve
[ölçüm raporunda](reports/yolopv2_engine_validation.json) bulunur.

## Üretilen ROS topic'leri

| Topic | Mesaj türü | İçerik |
| --- | --- | --- |
| `/perception/road/drivable_mask` | `sensor_msgs/msg/Image` | `mono8` sürülebilir alan maskesi; 255 alanı, 0 arka planı gösterir. |
| `/perception/road/lane_mask` | `sensor_msgs/msg/Image` | `mono8` şerit çizgisi maskesi. |
| `/perception/road/overlay` | `sensor_msgs/msg/Image` | RGB üzerine yol ve şerit çizilmiş önizleme; `publish_visualization:=true` iken yayınlanır. |
| `/perception/signs/detections_json` | `std_msgs/msg/String` | O karede çalışan levha **veya** COCO modelinin sınıfı, güveni ve kutusu; `source` alanı modeli belirtir. |
| `/perception/signs/overlay` | `sensor_msgs/msg/Image` | O karedeki levha veya nesne kutularının çizildiği önizleme. |
| `/yolo_detections` | `sac_interfaces/msg/CameraDetectionArray` | Yalnız levha modelinin işlediği karelerde tipli levha tespitleri; geçerli senkron depth varsa mesafe içerir. |
| `/perception/road_hazards` | `sac_interfaces/msg/RoadHazardArray` | Çukur/kasis türü, güven, kutu, zaman damgası ve geçerliyse 3B konum ile metre cinsinden mesafe. Tespit olmasa da boş dizi yayınlanır. |
| `/perception/road_hazards/annotated` | `sensor_msgs/msg/Image` | Çukur/kasis kutularının çizildiği önizleme; `publish_visualization:=true` iken yayınlanır. |
| `/perception/road/timing`, `/perception/signs/timing`, `/perception/road_hazards/timing` | `sac_interfaces/msg/PipelineTiming` | İşlenen, alınan ve atılan kare sayıları ile süreler. |

Mesafe için `/zed/zed_node/depth/depth_registered` ve
`/zed/zed_node/left/camera_info` gerekir. Mevcut `new_bag` depth içermediği
için hazard RGB önizleme modundadır: `distance_m=-1` ve
`position_valid=false`. Levha mesafesi de geçerli depth yoksa `-1` olur.
Hazard mesajındaki `path_assessed` algılama aşamasında `false` kalır; aracın
beklenen yoluyla kesişimi ve hedef hız ayrı planlama node'unda değerlendirilir.
Perception doğrudan gaz veya fren komutu üretmez.

## Docker içinde çalıştırma

```bash
docker exec -it sac bash
source /opt/ros/humble/setup.bash
source /smart_car_ws/install/setup.bash
ros2 launch jetson_perception perception.launch.py
```

Bag başka terminalde oynatılabilir:

```bash
docker exec -it sac bash
source /opt/ros/humble/setup.bash
ros2 bag play /smart_car_ws/new_bag --clock
```

Hızı ve çıktıları kontrol etmek için:

```bash
ros2 topic hz /perception/road/timing
ros2 topic hz /perception/signs/timing
ros2 topic hz /perception/road_hazards/timing
ros2 topic echo /perception/road_hazards --once
ros2 topic echo /perception/signs/detections_json --once
```

Launch seçenekleri: `road_weights:=/path/to/model.pt` veya `.engine`,
`sign_model:=/path/to/model.engine`, `hazard_model:=/path/to/model.engine`,
`enable_road_hazards:=true/false`, `publish_visualization:=true/false`.
`perception.launch.py` defaults to `hazard_auto_depth_preview:=true`:
without registered depth it still publishes RGB hazard detections, with
`position_valid=false`. The RGB-only `/smart_car_ws/new_bag` cannot provide a
metric hazard speed limit; the standalone planner therefore leaves metric
hazard and dynamic-object speed constraints disabled by default. Live launch
files enable them explicitly when registered depth is available.
Kayıtlı bag için `use_sim_time:=true`; canlı kamera için `use_sim_time:=false`
seçilmelidir. Giriş görüntüsü ve maske publisher'ları BEST_EFFORT, VOLATILE,
KEEP_LAST depth 1 QoS kullanır.

## Model dosyaları ve kurulum

Engine dosyaları Git'e eklenmez. Özel Release'deki modelleri indirmek için
[model dosyaları yönergesine](models/README.md) bakın. YOLOPv2 engine'i
Jetson üzerinde [bu adımlarla](docs/05_tensorrt_fp16.md) yeniden oluşturulur.
Hazard modelinin depth ve hız planlama bağlantısı
[burada](docs/ROAD_HAZARD_INTEGRATION.md) belgelenmiştir.

```bash
cd /smart_car_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select jetson_perception
```
