# CHANGELOG

鏈枃妗ｈ褰?`ROS2 Smart Follower` 鐨勪富瑕佺増鏈彉鏇淬€?
## Unreleased

### Performance
- 缁х画鏀跺彛 Raspberry Pi 5B 涓婄殑鏁存満鎬ц兘闂锛屽洿缁?`640x480 color+depth + YOLO + ReID + full stack` 鍋氫簡涓€杞粨鏋勭骇浼樺寲锛岃€屼笉鏄彧鍋滅暀鍦ㄥ弬鏁板井璋?- 鏄庣‘鍖哄垎浜嗕袱绫昏妭鎷?
  - `process_every_n_frames`: 鍐冲畾鍚屾鍚庣殑甯ф槸鍚﹁繘鍏ユ劅鐭ヤ富绾?  - `detect_every_n_frames`: 鍐冲畾杩涘叆涓荤嚎鐨勫抚閲岋紝鍝簺甯х湡姝ｆ墽琛?YOLO + ReID
- 缁忚繃瀹炴祴锛岀‘璁ゆ洿鍚堢悊鐨勬柟鍚戜笉鏄户缁妸 `process_every_n_frames` 鎻愬ぇ锛岃€屾槸鏀规垚:
  - `process_every_n_frames=1`
  - `detect_every_n_frames=2`
  杩欐牱鍙互璁┾€滆窡韪?閿佸畾/娣卞害瀹氫綅/鍙戝竷鈥濅繚鎸侀珮棰戯紝鑰屾妸鏈€閲嶇殑 `YOLO + ReID` 鍘嬪埌绾︿竴鍗婇鐜?
### Pipeline Fixes
- 淇浜嗘劅鐭ョ绾夸腑鐨勭涓€涓悶鍚愮摱棰?
  - 鏃у疄鐜伴噷 `PerceptionPipeline` 鍙湁涓€涓?`pending_work_`
  - 褰?detection worker 蹇欐椂锛屾柊鍒拌揪鐨勫悓姝ュ抚浼氱洿鎺ヨ鐩栨棫甯?  - 缁撴灉鏄ぇ閲忓凡缁忓悓姝ュソ鐨?color/depth 甯у湪杩涘叆 worker 涔嬪墠灏辫涓㈠純
- 灏嗗崟妲?`pending_work_` 鏀逛负鏈夌晫 `pending_work_queue_`
  - 淇濈暀鏈€杩戝嚑甯у緟澶勭悊浠诲姟
  - 闃熷垪婊℃椂鏄惧紡涓㈠純鏈€鏃т换鍔?  - 鏂板 diagnostics 瀛楁 `dropped_pending_work_count`锛屾妸杩欓儴鍒嗕涪甯т粠鈥滈粦鐩掆€濆彉鎴愬彲瑙傛祴鎸囨爣
- 淇浜嗘劅鐭ョ绾夸腑鐨勭浜屼釜鍚炲悙鐡堕:
  - 鏃у疄鐜伴噷 worker 缁撴灉渚у彧鏈変竴涓?`latest_result_`
  - 鍗充究 worker 宸茬粡鏇村揩鍦颁骇鐢熶簡缁撴灉锛屾柊鐨?ready result 浠嶄細瑕嗙洊鏃х粨鏋?  - 涓荤嚎绋?`result_timer` 姣忔鍙秷璐逛竴鏉＄粨鏋滐紝瀵艰嚧渚垮疁鐨勨€滈潪妫€娴嬭窡韪抚鈥濅粛鐒舵棤娉曞畬鏁村彂甯?- 灏嗗崟妲?`latest_result_` 鏀逛负鏈夌晫 `ready_result_queue_`
  - ready result 涓嶅啀浜掔浉瑕嗙洊
  - `result_timer` 鏀逛负姣忔瑙﹀彂鏃舵寔缁?drain 褰撳墠鎵€鏈?ready result
  - 鏂板 diagnostics 瀛楁 `dropped_ready_result_count`
- 杩欎竴杞紭鍖栫殑鏍稿績涓嶆槸鈥滆鍗曞抚鎺ㄧ悊鏇村揩鈥濓紝鑰屾槸鈥滃噺灏戠绾垮唴閮ㄦ棤鎰忎箟瑕嗙洊锛岃宸茬粡绠楀嚭鏉ユ垨宸茬粡鍚屾濂界殑甯х湡姝ｈ蛋鍒板彂甯冪鈥?
### Measured Result
- 浼樺寲鍓嶅熀绾匡紝鍏ㄦ爤銆乣process_every_n_frames=2`:
  - `/camera/color/image_raw`: `18.017 Hz`
  - `/camera/depth/image_raw`: `19.638 Hz`
  - `/robot1/person_pose`: `4.576 Hz`
- 绗竴闃舵锛屽彧鍋氬弬鏁伴噸鎺掞紝鏀规垚鍏ㄦ爤 `process_every_n_frames=1` + `detect_every_n_frames=2`:
  - `/robot1/person_pose`: `5.018 Hz`
  - 璇存槑鏂瑰悜姝ｇ‘锛屼絾浠嶆湁鏄庢樉鍐呴儴瑕嗙洊
- 绗簩闃舵锛屽姞鍏?`pending_work_queue_`:
  - `/robot1/person_pose`: `6.100 Hz`
  - diagnostics 鏄剧ず `dropped_pending_work_count` 寰堥珮锛岀‘璁?worker 鍓嶄粛鍦ㄤ涪鍚屾甯?- 绗笁闃舵锛岀户缁姞鍏?`ready_result_queue_` 骞惰 timer drain:
  - `/camera/color/image_raw`: `22.708 Hz`
  - `/camera/depth/image_raw`: `21.442 Hz`
  - `/robot1/person_pose`: `9.015 Hz`
  - `dropped_ready_result_count=0`
  - `processed_frame_count=755`
  - `person_pose_publish_count=755`
- 杩欒鏄庡綋鍓嶇増鏈凡缁忔妸鏁存満鎰熺煡杈撳嚭浠庣害 `4.6 Hz` 鎻愬崌鍒扮害 `9.0 Hz`
- 鍚屾椂锛屽僵鑹?娣卞害杈撳叆瑙傛祴棰戠巼涔熼娆＄ǔ瀹氳创杩?`21 Hz` 鐩爣鍖洪棿

### Notes
- 褰撳墠鍓╀綑鐡堕宸茬粡鏇撮泦涓?
  - `perception_node` 浠嶉暱鏈熸帴杩?`100% CPU`
  - detect 甯т笂鐨?`YOLO + ReID` 浠嶇劧鏄富鑰楁椂
  - 鐜伴樁娈垫渶澶у墿浣欐崯澶变富瑕佹潵鑷緭鍏ヤ晶 `pending_work_queue_` 浠嶄細婊★紝璇存槑 worker 澶勭悊鑳藉姏渚濇棫鏄笂闄?- 涓嬩竴姝ユ洿鍊煎緱鍋氱殑涓嶆槸鍐嶈皟鎺у埗鑺傜偣锛岃€屾槸缁х画鍘嬬缉 detect 甯ф垚鏈?
  - 闄嶄綆绋冲畾閿佸畾鐘舵€佷笅鐨?ReID 瑙﹀彂棰戠巼
  - 浠呭湪鎭㈠閿佸畾鎴栧鐩爣姝т箟鏃跺己鍒惰窇 ReID
  - 缁х画妫€鏌ュ僵鑹查摼璺槸鍚﹁繕鏈夐澶栬皟搴︽姈鍔?
## beta-0.3.2 - 2026-03-30

### Changed
- follower 转向侧从简单一阶平滑升级为轻量卡尔曼滤波，并开放 `steering_kalman.process_noise`、`steering_kalman.measurement_noise`、`steering_kalman.initial_covariance` 供现场热调
- 保留 `limits.v_max`、`limits.w_max`、`limits.dv_max`、`limits.dw_max` 作为最终输出硬限制，同时删除 `theta_deadzone`、大转角自动降速和 `max_target_speed_mps` 目标速度硬截断，减少控制链上与当前路线无关的额外束缚
- `try.md` 重写为现场调参与排查速查，补齐当前 `/robot1/follower_controller_node` 参数解释、在线调参命令和观测命令

### Verified
- VM 工作区 `/home/wheeltec/ros2_smart_follower` 已同步最新控制模块改动，并完成 `smart_follower_control` 单包编译通过
## beta-0.3.1 - 2026-03-29

### Changed
- 鍦?`beta-0.3.0` 绋冲畾鍙窇鐨勫熀纭€涓婏紝缁х画鍋氭妧鏈矾绾挎敹鍙ｏ細绉婚櫎鏃ф枃妗ｃ€佹棫妯″瀷涓庡綋鍓嶄富绾挎棤鍏崇殑鍘嗗彶璇存槑锛屼粨搴撳彧淇濈暀 Astra color+depth + YOLO/ReID/Tracker/Lock Manager 鐨勫浐瀹氳矾绾?- perception 鏋勫缓鏀逛负寮哄埗渚濊禆 `astra_camera_msgs`锛屽垹闄ょ己鍖呮椂鐨勬棫鍏煎閫昏緫锛岃繍琛岄摼璺粺涓€浠ョ湡瀹炵浉鏈哄唴鍙傛湇鍔′负鍑?- 鍘嗗彶 `monocular` 鍛藉悕缁熶竴鏇挎崲涓?`camera` 鍛藉悕锛涘寘鎷弬鏁伴敭銆佺被鍨嬪悕銆佸嚱鏁板悕鍜屽唴閮ㄦ垚鍛橈紝鍑忓皯褰撳墠 RGB-D 涓荤嚎涓嬬殑鐞嗚В鎴愭湰
- README銆佷緷璧栬鏄庛€佽皟鍙傝鏄庡拰鏂颁汉鏂囨。鍚屾鎻愬崌鍒?`beta-0.3.1`锛岃ˉ榻愭湰杞矾绾挎敹鏁涗笌鍛藉悕鏀跺彛璇存槑

### Removed
- 鍒犻櫎浠撳簱鏍圭洰褰曚腑涓庡綋鍓嶅浐瀹氫富绾挎棤鍏崇殑鏃ц鏄庢枃浠讹細`test.md`銆乣install.md`銆乣褰撳墠鎺ㄨ崘杩愯缁勫悎.md`銆乣鎶€鏈矾绾挎敹鏁涗唬鐮佸鏌ユ姤鍛奯beta-0.2.0.md`
- 鍒犻櫎宸蹭笉鍐嶄娇鐢ㄧ殑鏃?YOLO 妯″瀷 `models/yolo26n_static_256x320_simplify_e2e.onnx`

### Verified
- VM 宸ヤ綔鍖哄凡瀹屾垚鍚屾骞堕噸鏂扮紪璇戦€氳繃锛歚smart_follower_msgs`銆乣smart_follower_perception`銆乣smart_follower_control`銆乣smart_follower_bringup` 4 涓寘鍏ㄩ儴鏋勫缓鎴愬姛
## beta-0.3.0 - 2026-03-29

### Changed
- 榛樿杩愯妯″瀷鍒囨崲涓?`yolo26n_static_256x320_simplify_e2e_int8.onnx + osnet_x0_5_512.onnx`锛屽苟鍚屾鏇存柊 launch / README / try 鏂囨。
- perception 榛樿澶勭悊鑺傚浠?`process_every_n_frames=3` 璋冩暣涓?`2`锛屾洿璐磋繎褰撳墠杞︾杈撳叆鑺傚
- `depth_compare` 榛樿閲囨牱鍙傛暟璋冩暣涓?`sample_window_px=9`銆乣min_valid_samples=5`
- 閿佸畾鐩爣鐨勬繁搴﹀畾浣嶄粠鍗曠獥閲囨牱鏀跺彛涓?**lower-body 澶氱獥鍙ｉ噰鏍?+ median**锛屽彇娑堟棫鍏煎閲囨牱閾捐矾
- follower 榛樿 `target_timeout` 浠?`0.3s` 鎻愬崌鍒?`1.2s`锛岀煭鏃舵棤鏁堟繁搴︽垨婕忔牱鏈椂鏇村鏄撶画涓婅窡闅?- 鏍圭洰褰?README銆佷緷璧栬鏄庛€佽皟鍙傝鏄庛€佹帹鑽愯繍琛岀粍鍚堜笌鏂颁汉鏂囨。缁熶竴鎻愬崌鍒?`beta-0.3.0`

### Added
- perception 澧炲姞閿佸畾鐩爣瀹氫綅澶辫触鏃ュ織锛屽尯鍒?`depth_frame_unavailable`銆乣depth_window_no_valid_samples`銆乣bbox_near_image_edge`銆乣depth_projection_rejected`
- follower 澧炲姞鐩爣澶辨晥鍘熷洜鏃ュ織鍜?diagnostics 瀛楁锛屼究浜庡畾浣?`target_timeout`銆乣locked_track_position_nan`銆乣locked_track_not_confirmed` 绛夌姸鎬?- 鏂板 Textual / 杞婚噺 live dashboard 璋冭瘯宸ュ叿锛屽苟琛ラ綈鐩稿叧鍙€変緷璧栬鏄?- 鏂板 `璋冭瘯鍛戒护琛?md`锛屾眹鎬荤紪璇戙€佸惎鍔ㄣ€佽瘽棰樻帓鏌ュ拰瀹炶溅鑱旇皟甯哥敤鍛戒护

### Fixed
- 璺熻釜妯″潡琛ラ綈寮傚父鎵撳嵃锛屽苟鍦?tracker 鎶涘嚭寮傚父鏃惰嚜鍔?reset锛岄伩鍏?perception 杩涚▼鐩存帴閫€鍑?- perception 鍚姩闃舵瀵?`/camera/get_camera_info` 鐨勭瓑寰呮敼涓哄娆￠噸璇曪紝闄嶄綆鐩告満鑺傜偣鐣ユ櫄鍚姩鏃剁殑 configure 澶辫触姒傜巼
- 灏忚溅瀹瑰櫒鍐呮瀯寤烘槑纭吋瀹?`/home/wheeltec/wheeltec_ros2/third_party/...` 渚濊禆璺緞锛岄伩鍏嶅洜 `$HOME=/root` 瀵艰嚧 ONNX Runtime / libgpiod 婕忔


## beta-0.2.0 - 2026-03-24

### Changed
- 浠?`dev-0.1.8` 鏀跺彛鍒?`beta-0.2.0`锛氫繚鐣欒交閲忔ā鍨嬨€佹帶鍒朵慨澶嶄笌璋冭瘯宸ュ叿閾撅紝鍚屾椂鎭㈠涓婚摼璺?color+depth 瀹氫綅鐢ㄤ簬瀹炶溅瀵圭収
- perception 鎭㈠ `/camera/depth/image_raw` 杈撳叆銆乧olor/depth 鏃堕棿鍚屾涓?depth 鍙栨牱瀹氫綅锛沗/person_pose` 澶栭儴鎺ュ彛淇濇寔涓嶅彉
- `smart_follower.launch.py` 榛樿鎵撳紑 depth 涓?depth registration锛屽綋鍓嶉粯璁よ繍琛岀粍鍚堜负 `yolo26n_static_256x320_simplify_e2e.onnx + osnet_x0_5_512.onnx`
- perception 鑺傜偣淇 Astra `/camera/get_camera_info` 鍐呭弬鍒ゅ畾閫昏緫锛岀幇鍙纭姞杞界湡瀹?`fx/fy/cx/cy`锛屼笉鍐嶈鍥為€€鍒?fallback

### Added
- 鏂板 `depth_compare.*` 閲囨牱鍙傛暟涓?depth diagnostics锛坄depth_ready`銆乣depth_samples_valid`銆乣last_valid_depth_m`銆乣depth_position_ms` 绛夛級

### Verified
- VM 绔畬鏁寸紪璇戜笌娴嬭瘯閫氳繃锛?3 tests, 0 failures锛?- 灏忚溅瀹瑰櫒绔凡瀹屾垚閲嶆柊鍚屾銆侀噸鏂扮紪璇戜笌鍚姩鐑熸祴锛涚‘璁?ONNX Runtime 鐪熸鐢熸晥銆丄stra color/depth 涓婚摼璺彲姝ｅ父鍚姩
- 瀹炶溅鑱旇皟纭褰撳墠鐗堟湰宸插彲杩愯璺熼殢锛沋OLO `intra=2` 瀹炴祴涓鸿礋浼樺寲锛屽凡鎭㈠榛樿 `intra=1`

## dev-0.1.8 - 2026-03-24

### Changed
- 娓呯悊绾?RGB 涓荤嚎涓嬬殑鍏煎娈嬬暀锛氬垹闄?`monocular.person_height_m` 鍙傛暟涓庢棤鏁堟秷鎭瓧娈?`TrackedPerson.velocity/depth_m`
- 灏?ReID / `appearance_feature` 鎺ュ彛浠庡巻鍙插吋瀹圭殑 `2048` 缁存敹鍙ｅ埌褰撳墠鐪熷疄浣跨敤鐨?`512` 缁达紝绉婚櫎杩愯鏃?padding 鍏煎閾?- `smart_follower.launch.py` / `smart_follower_only.launch.py` 榛樿妯″瀷鍒囨崲涓?`yolo26n_static_256x320_simplify_e2e.onnx + osnet_x0_5_512.onnx`锛屽苟缁熶竴 color-only Astra 鍚姩璺緞
- 鎺у埗渚ч粯璁よ窡闅忚窛绂昏皟鏁翠负 `0.6m`锛屽悓鏃惰ˉ榻?`/robot1/follower_controller_node` 鐨?namespaced YAML锛岄伩鍏嶈繍琛屾椂鍥為€€鍒伴粯璁?`1.0m`
- `ArbiterRuntime` 绉婚櫎鈥滅洰鏍囪秴鏃跺悗姘镐箙 stop_latch鈥濊涓猴紱鐜板湪浠?`ESTOP` 浼氶攣鍋滐紝鐩爣鎭㈠鍚庡彲鑷姩鎭㈠璺熼殢

### Added
- 鏍圭洰褰?`test.md` 琛ュ厖 320x256 妯″瀷銆佺嚎绋嬬粍鍚堜笌杞︾鑱旇皟璁板綍
- 鏍圭洰褰?`try.md` / `褰撳墠鎺ㄨ崘杩愯缁勫悎.md` 鍚屾鏇存柊褰撳墠鎺ㄨ崘妯″瀷銆佺嚎绋嬩笌璋冨弬璇存槑

## alpha-0.1.7 - 2026-03-23

### Changed
- 鍗曠洰浣嶇疆浼拌浠庘€渂box 楂樺害 + 浜洪珮鍋囪鈥濆垏鎹负 **bbox 搴曠偣鍦伴潰鎶曞奖**
- perception 鑺傜偣鏂板 Astra 鍐呭弬鍒濆鍖栭€昏緫锛?  - `configure()` / 鐑洿鏂版椂浼樺厛璋冪敤 `/camera/get_camera_info`
  - 鏈嶅姟澶辫触鏃惰嚜鍔?fallback 鍒?`horizontal_fov_deg` 杩戜技鍐呭弬
- `PerceptionPipeline` / `pipeline_utils` 鏀逛负鏄惧紡浼犻€?`MonocularCameraIntrinsics`
- diagnostics 澧炲姞锛?  - `intrinsics_ready`
  - `intrinsics_source`
  - `camera_fx / camera_fy / camera_cx / camera_cy`
  - `position_projection_ms`
  - `position_valid_count / position_invalid_count`
- `perception_params.yaml` 鏂板骞跺惎鐢細
  - `monocular.camera_info_service`
  - `monocular.camera_height_m`
  - `monocular.camera_pitch_deg`
  - `monocular.camera_x_offset_m`
  - `monocular.camera_y_offset_m`
  - `monocular.min_downward_angle_deg`
- 鏍圭洰褰曟柊澧?`try.md`锛屾暣鐞嗗綋鍓?YAML 鍙傛暟鐨勫崟浣嶃€佷綔鐢ㄤ笌璋冨弬寤鸿
- 杩愯鏃剁増鏈瓧绗︿覆缁熶竴鎻愬崌鍒?`alpha-0.1.7`
- 鍚勫寘 `package.xml` 鐗堟湰缁熶竴鎻愬崌鍒?`0.1.7`

### Added
- `smart_follower_perception/test/test_geometry_utils.cpp`

### Verified
- VM 绔暣搴撻噸鏂拌鐩栭儴缃插畬鎴?- VM 绔?`smart_follower_msgs + smart_follower_perception + smart_follower_control + smart_follower_bringup` 缂栬瘧閫氳繃
- VM 绔祴璇曢€氳繃锛歚39 tests, 0 errors, 0 failures, 0 skipped`

## alpha-0.1.6 - 2026-03-23

### Changed
- 鎰熺煡涓婚摼璺粠 RGB-D 鏀逛负绾?RGB锛岀Щ闄?`depth_topic` / `camera_info_topic`
- `FrameSynchronizer` 鏀逛负鍙紦瀛樺僵鑹插浘鍍?- `PerceptionPipeline` 鏀逛负绾僵鑹叉娴嬮摼璺?- `TrackedPerson.position` 鏀圭敱 bbox 鍋氬崟鐩綅缃及璁＄敓鎴?- `Tracker` 鍘绘帀娣卞害浠ｄ环涓庢繁搴?gating锛岀姸鎬佺淮搴︾敱 10 缁存敹缂╁埌 8 缁?- `obstacle_avoidance_node` / `ObstacleRuntime` 绉婚櫎娣卞害鍥句緷璧栵紝浠呬繚鐣欏乏鍙宠秴澹版尝閬块殰
- perception / control / bringup 鐨?YAML銆乨iagnostics銆佹祴璇曘€丷EADME 鍚屾娓呯悊
- 缁熶竴杩愯鏃剁増鏈瓧绗︿覆鍒?`alpha-0.1.6`锛屽苟灏嗗悇鍖?`package.xml` 鐗堟湰鎻愬崌鍒?`0.1.6`

### Verified
- VM 绔?`smart_follower_msgs + smart_follower_perception + smart_follower_control + smart_follower_bringup` 缂栬瘧閫氳繃
- VM 绔叏閲忕浉鍏虫祴璇曢€氳繃锛歚42 tests, 0 failures`
- `smart_follower_only.launch.py` 鐑熸祴鍙惎鍔?
## alpha-0.1.5 - 2026-03-23

### Changed
- 鍥為€€缁熶竴缂撳啿鍖?/ 澶?worker 瀹為獙锛屾仮澶?`alpha-0.1.4` 鐨勫崟 worker 鍗曞抚缂撳啿涓荤嚎
- 淇濈暀 `runtime` 鐑矾寰勫璞″鐢ㄣ€乊OLO 棰勫鐞嗗噺鎷疯礉涓?profiling 鎷嗗垎
- 榛樿鎺ㄨ崘 YOLO ORT 绾跨▼鍙傛暟璋冩暣涓?`intra_op_num_threads = 3`
- 琛ュ厖娴嬭瘯璁板綍涓庡綋鍓嶆帹鑽愯繍琛岀粍鍚堟枃妗?
## alpha-0.1.4 - 2026-03-22

### Added
- 鎰熺煡渚ф媶鍑?`perception_pipeline.*`
- 鎺у埗渚ф媶鍑?`control_node_common.*`
- 鍖呯骇 README锛?  - `src/smart_follower_perception/README.md`
  - `src/smart_follower_control/README.md`
  - `src/smart_follower_bringup/README.md`
- `docs/鏂颁汉涓婃墜鎸囧崡.md`
- diagnostics 杈呭姪瀹炵幇鎷嗗垎鍒?`.cpp`

### Changed
- `perception_node.cpp` 鏀剁缉涓烘洿钖勭殑 Lifecycle 鑺傜偣鍏ュ彛
- arbiter / follower / obstacle / ultrasonic 鑺傜偣鍋氫簡绗竴杞彲璇绘€ф媶鍒?- 缁熶竴浜嗕竴鎵归噸澶嶇殑鐢熷懡鍛ㄦ湡 / 鍙傛暟鐑洿鏂版牱鏉?
## alpha-0.1.3 - 2026-03-22

### Added
- 鏇寸粏绮掑害 profiling锛歚tf_lookup_ms`銆乣tf_transform_ms`銆乣message_fill_ms` 绛?- 琛ュ厖娴嬭瘯璁板綍涓?ORT / YOLO 浼樺寲鏂囨。

### Changed
- 榛樿妯″瀷缁勫悎鍒囧埌锛?  - `models/yolo26n_static_480x640_simplify_e2e.onnx`
  - `models/osnet_x0_5_512.onnx`
- YOLO 杈撳叆鍥哄畾鍒?`640x480`
- 灏嗘秷鎭粍瑁呬笌 TF 澶勭悊杩涗竴姝ユ媶鍒?`pipeline_utils.*`

## alpha-0.1.2 - 2026-03-18

### Added
- `smart_follower_bringup` 琛ラ綈 `ament_index_python` 渚濊禆
- diagnostics 澧炲姞鏇村 ready / timeout / publish 瑙傛祴

### Changed
- `ReidExtractor::extract()` 澧炲姞 ONNX Runtime 寮傚父鍏滃簳
- diagnostics 绛夌骇鏀逛负 `OK / WARN / ERROR`
- CMake / package.xml 琛ラ綈 lint 鐩稿叧閰嶇疆

## alpha-0.1.1 - 2026-03-17

### Changed
- 淇绗竴杞増鏈榻愰棶棰?- 鎺у埗渚у拰鎰熺煡渚х粺涓€鍒?`alpha-0.1.1` 鍙ｅ緞

## alpha-0.1.0 - 2026-03-16

### Added
- 瀹屾垚 `perception_node.cpp` 鐨勮亴璐ｆ媶鍒嗛噸鏋勭涓€杞氦浠?- 澧炲姞 `runtime / frame_sync / tracker / lock_manager / geometry / params / diagnostics` 妯″潡鍖栫粨鏋?
