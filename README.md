# sac_autonomy

SAC aracının ROS 2 paketleri. Güncel araç kodları ROS 2 **Humble / Jetson**
ortamından aktarılmıştır; önceki simülasyon paketleri de korunmuştur.

```text
core/
  perception/       algılama ve nokta bulutu işleme
  localization/     GNSS, EKF, LiDAR lokalizasyonu ve SLAM
  planning/         rota ve yörünge planlama
  control/          takip ve kontrol güvenliği
  autoware/         ortak Autoware paketleri ve harita yükleyicileri
interfaces/         SAC mesaj tanımları
utilities/          launch/config, araç modeli, yardımcı araçlar ve sürücüler
```

## Tek komutla kurulum

Git, curl ve Python 3 kurulu bir bilgisayarda:

```bash
curl -fsSL https://raw.githubusercontent.com/ITU-EMAV/sac_autonomy/main/setup.sh | bash -s -- ~/sac_autonomy
```

Bu betik depoyu klonlar, `runtime-assets-v1` Release'indeki model ve harita arşivini
indirir, SHA-256 değerlerini doğrular ve dosyaları paketlerin altına yerleştirir.
Kurulum başka bir dizinde de çalışır. GitHub hesabıyla giriş veya rclone gerekmez.

Düz `git clone` yalnızca Git'teki dosyaları getirir. Böyle klonladıysanız:

```bash
cd sac_autonomy
./utilities/tools/download_assets.sh
```

İndirme kesilirse aynı komutla devam edebilirsiniz. Doğrulanmış arşiv yerel
önbellekte tutulur; tekrar kurulumda yeniden indirilmez.

## Büyük dosyalar

Kodlar, launch/config ve küçük harita tanımları Git'te tutulur. Model ağırlıkları,
ONNX/TensorRT dosyaları, PCD haritaları ve coğrafi veri tablosu GitHub Release'te
saklanır. Büyük dosyalar `.gitignore` kapsamındadır.

İndirilen dosyaların konumları:

- `core/perception/jetson_perception/models/`
- `utilities/smart_car_launch/maps/`
- `core/localization/autoware/llh_converter/data/`

Model ve harita dosyaları derlemeden önce indirilmelidir. `jetson_perception`
modelleri ve YOLOPv2 yardımcı kodlarını paket share dizinine kurar; düğümler ve
launch dosyaları modelleri ROS paket yollarıyla bulur. TensorRT engine dosyaları
kaynak Jetson ortamına aittir; farklı GPU/TensorRT sürümünde yeniden üretilmelidir.

Yeni bir varlık sürümü yayınlamak için GitHub CLI ile giriş yaptıktan sonra:

```bash
SAC_ASSET_RELEASE=runtime-assets-v2 ./utilities/tools/upload_assets.sh
```

Ardından `download_assets.sh` içindeki varsayılan release sürümünü ve
`utilities/tools/runtime_assets.sha256` dosyasını yeni arşivin checksum'u ile
birlikte güncelleyin. Var olan release dosyaları betik tarafından değiştirilmez.

## Derleme

ROS ve sistem bağımlılıkları kurulu Humble ortamında:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Araç başlangıçları `utilities/smart_car_launch/launch/` altındadır.
Önceki `sac_localization_adapters` yeni lokalizasyon paketinin API'siyle uyumlu
olmadığından `COLCON_IGNORE` ile derleme dışında tutulur; kaynak kodu korunmuştur.
Bu aktarım sırasında tüm ROS yığını için derleme veya araç üzerinde sürüş testi
henüz yapılmamıştır.
