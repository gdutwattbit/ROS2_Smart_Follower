# ROS2 Smart Follower

闈㈠悜鏍戣帗娲?/ ROS 2 Humble 鐨勪綆浣嶆櫤鑳借窡闅忚溅椤圭洰銆?

褰撳墠鍥哄畾鎶€鏈矾绾匡細
- 鎰熺煡锛歒OLO + ReID + Tracker + Lock Manager
- 瀹氫綅锛欰stra 褰╄壊 + 娣卞害杈撳叆锛宒epth compare 涓婚摼璺?
- 鎺у埗锛?0Hz 璺熼殢鎺у埗 + 鐭椂棰勬祴琛ュ抚 + 瓒呭０娉㈤伩闅?+ 鎸囦护浠茶

> 褰撳墠鍙戝竷鏍囩锛歚beta-0.3.2`

---

## 0. beta-0.3.2 本轮更新

这一轮主线改动收口到 **beta-0.3.2**，重点是把跟随控制和现场调参进一步收实：
- follower 转向侧从简单平滑改为轻量卡尔曼滤波，支持 `steering_kalman.*` 在线调参
- 保留 `v_max / w_max / dv_max / dw_max` 这组输出硬限制，但删除额外的 `theta_deadzone`、大转角自动降速和目标速度估计硬截断
- `try.md` 重写为现场调参与排查速查，补齐 `ros2 param get/set`、话题观测和推荐调参顺序
- VM 工作区已同步并完成 `smart_follower_control` 编译验证，便于后续直接围绕实车手感继续细调

---

## 1. 褰撳墠閾捐矾姒傝

```text
/camera/color/image_raw + /camera/depth/image_raw
                    鈫?
                  YOLO
                    鈫?
                  ReID
                    鈫?
                 Tracker
                    鈫?
               Lock Manager
                    鈫?
      depth compare 瀹氫綅锛坆box 涓嬪崐閮ㄧ獥鍙?+ median锛?
                    鈫?
                /person_pose
                    鈫?
Follower / Obstacle / Arbiter
                    鈫?
                  /cmd_vel
```

褰撳墠璁捐瑕佺偣锛?
- 浣跨敤杞婚噺妯″瀷缁勫悎锛歚yolo26n_static_256x320_simplify_e2e_int8.onnx + osnet_x0_5_512.onnx`
- `TrackedPerson.position` 鐢卞榻愭繁搴﹀浘鍙栨牱寰楀埌
- `/person_pose`銆佹帶鍒朵晶鎺ュ彛銆佺敓鍛藉懆鏈熻涓轰繚鎸佺ǔ瀹?
- perception 浼氳姹?Astra 鐨?`/camera/get_camera_info` 浣滀负鐪熷疄鍐呭弬鏉ユ簮

---

## 2. 浠撳簱缁撴瀯

```text
ros2_smart_follower/
鈹溾攢 src/
鈹? 鈹溾攢 smart_follower_msgs/
鈹? 鈹溾攢 smart_follower_perception/
鈹? 鈹溾攢 smart_follower_control/
鈹? 鈹斺攢 smart_follower_bringup/
鈹溾攢 docs/
鈹溾攢 models/
鈹溾攢 scripts/
鈹溾攢 README.md
鈹溾攢 CHANGELOG.md
鈹溾攢 DEPENDENCIES.md
鈹斺攢 try.md
```

---

## 3. 鍚勫寘鑱岃矗

### `smart_follower_msgs`
瀹氫箟椤圭洰娑堟伅锛?
- `TrackedPerson.msg`
- `PersonPoseArray.msg`
- `FollowCommand.msg`

### `smart_follower_perception`
璐熻矗锛?
- 褰╄壊鍥惧儚 + 娣卞害鍥炬帴鍏?
- YOLO 妫€娴?
- ReID 鐗瑰緛鎻愬彇
- 澶氱洰鏍囪窡韪?
- 鐩爣閿佸畾 / 鍒囦汉绛栫暐
- depth compare 瀹氫綅
- 鍙戝竷 `/person_pose`

### `smart_follower_control`
璐熻矗锛?
- 璺熼殢鎺у埗
- 20Hz 鎺у埗琛ュ抚棰勬祴
- 宸﹀彸瓒呭０娉㈤噰鏍?
- 瓒呭０娉㈤伩闅?
- 鎸囦护浠茶
- 閿洏鎺у埗

### `smart_follower_bringup`
璐熻矗锛?
- launch 缁勭粐
- 榛樿 YAML 鍙傛暟
- 妯″瀷璺緞瑕嗙洊

---

## 4. 褰撳墠鎺ㄨ崘妯″瀷涓庤繍琛岀粍鍚?

榛樿妯″瀷缁勫悎锛?
- `models/yolo26n_static_256x320_simplify_e2e_int8.onnx`
- `models/osnet_x0_5_512.onnx`

褰撳墠鎺ㄨ崘绾跨▼鍙傛暟锛?
- YOLO ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`
- ReID ORT: `intra_op_num_threads=1`, `inter_op_num_threads=1`, `sequential`

褰撳墠鎺ㄨ崘鑺傚锛?
- 鐩告満杈撳叆锛氱害 30fps
- perception 澶勭悊锛歚process_every_n_frames=2`
- follower 鎺у埗杈撳嚭锛?0Hz
- 涓棿渚濋潬鎺у埗渚х煭鏃跺父閫熷害棰勬祴琛ュ抚
- follower 榛樿鐩爣璺濈锛歚0.6m`

---

## 5. 鍚姩鏂瑰紡

### 灏忚溅瀹瑰櫒鍐呯涓夋柟渚濊禆瀹為檯璺緞

鍦ㄥ皬杞?`ros2` 瀹瑰櫒鍐呭疄鏌ュ埌锛?

- **ONNX Runtime 鏍圭洰褰?*
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3`
- **ONNX Runtime 澶存枃浠?*
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/include`
- **ONNX Runtime 鍔ㄦ€佸簱**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/lib/libonnxruntime.so`
- **ONNX Runtime CMake 閰嶇疆**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/lib/cmake/onnxruntime`
- **ONNX Runtime pkg-config**
  - `/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3/lib/pkgconfig/libonnxruntime.pc`

- **libgpiod 瀹夎鏍圭洰褰?*
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod`
- **libgpiod 澶存枃浠?*
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod/include/gpiod.h`
- **libgpiod 鍔ㄦ€佸簱**
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod/lib/libgpiod.so`
- **libgpiod 婧愮爜鐩綍**
  - `/home/wheeltec/wheeltec_ros2/third_party/libgpiod-2.1.3`

> 娉ㄦ剰锛氳繖涓ゅ搴撶洰鍓嶅湪瀹瑰櫒閲屾槸瀛樺湪鐨勶紝浣?*涓嶅湪鏈粨搴撶殑 `third_party/` 涓?*銆?
> 褰撳墠涓荤嚎浠ｇ爜宸茬粡鍚屾椂琛ヤ笂锛?
> - 鏄惧紡缁濆璺緞 `/home/wheeltec/wheeltec_ros2/third_party/...`
> - 鐜鍙橀噺璺緞 `$HOME/wheeltec_ros2/third_party/...`
> 杩欐牱鍗充娇瀹瑰櫒閲岀敤 `root` 缂栬瘧锛屼篃涓嶄細鍐嶅洜涓?`$HOME=/root` 鑰屾紡妫€銆?

濡傞渶鍦ㄥ皬杞﹀鍣ㄥ唴鏄惧紡鎸囧畾渚濊禆璺緞锛屽缓璁厛鎵ц锛?

```bash
export ONNXRUNTIME_ROOT=/home/wheeltec/wheeltec_ros2/third_party/onnxruntime-linux-aarch64-1.24.3
export LIBGPIOD_ROOT=/home/wheeltec/wheeltec_ros2/third_party/libgpiod
export LD_LIBRARY_PATH=$ONNXRUNTIME_ROOT/lib:$LIBGPIOD_ROOT/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=$ONNXRUNTIME_ROOT/lib/pkgconfig:$PKG_CONFIG_PATH
```

### 鍚姩瀹屾暣 bringup
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py
```

### 娌℃湁搴曠洏椹卞姩鍖呮椂锛屼粎鍚姩鏈」鐩摼璺?
```bash
ros2 launch smart_follower_bringup smart_follower.launch.py \
  robot_ns:=robot1 \
  bringup_robot:=false
```

---

## 6. 褰撳墠鍊煎緱浼樺厛闃呰鐨勬枃浠?

寤鸿鍏堢湅锛?
- `src/smart_follower_bringup/config/perception_params.yaml`
- `src/smart_follower_control/config/control_params.yaml`
- `src/smart_follower_perception/src/perception_node.cpp`
- `src/smart_follower_perception/src/perception_pipeline.cpp`
- `src/smart_follower_perception/src/tracker.cpp`
- `src/smart_follower_control/src/follower_runtime.cpp`
- `src/smart_follower_control/src/obstacle_runtime.cpp`
- `try.md`

鍏朵腑锛?
- `try.md`锛氬弬鏁颁綔鐢?/ 鍗曚綅 / 璋冨弬寤鸿

---

## 7. 褰撳墠鐘舵€?

褰撳墠涓荤嚎宸茬粡瀹屾垚骞堕獙璇佽繃锛?
- `smart_follower_msgs`
- `smart_follower_perception`
- `smart_follower_control`
- `smart_follower_bringup`

鍚庣画宸ヤ綔閲嶇偣灏嗚浆鍚戯細
- 璺嚎鏀舵暃娓呯悊
- 鍙傛暟鏁寸悊
- 瀹炶溅璋冨弬涓庣ǔ瀹氭€ч獙璇?



