# テンキーで色々操作するためのノード
## ビルド方法
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/sato-susumu/susumu_tenkey_controller.git
cd ..
colcon build --packages-select susumu_tenkey_controller --symlink-install
```

## 実行方法
```
source ~/ros2_ws/install/setup.bash
ros2 launch susumu_tenkey_controller tenkey_controller.launch.py
```
