# av_safety_monitor 鈥?瀹夊叏鐩戞帶涓庢晠闅滄敞鍏?
TTC 纰版挒棰勮/绱ф€ュ埗鍔ㄣ€佺鎾炰簨浠跺鐞嗐€佸绾ф姤璀︿笌鏁呴殰娉ㄥ叆宸ュ叿銆?
## 鐩綍缁撴瀯

```
av_safety_monitor/
鈹溾攢鈹€ setup.py / package.xml
鈹溾攢鈹€ config/safety_params.yaml
鈹溾攢鈹€ resource/av_safety_monitor            # (鏈琛ュ缓, 淇 colcon 瀹夎澶辫触)
鈹溾攢鈹€ av_safety_monitor/
鈹?  鈹溾攢鈹€ safety_monitor.py    # TTC涓夌骇棰勮(WARNING/CRITICAL/EMERGENCY) + AEB
鈹?  鈹斺攢鈹€ fault_injector.py    # drop/noise/bias/latency/stall 鏁呴殰娉ㄥ叆
鈹斺攢鈹€ test/test_safety_monitor.py
```

## 瀹夎涓庣紪璇?
```bash
cd <宸ヤ綔绌洪棿鏍圭洰褰?
colcon build --packages-select av_carla_interfaces av_safety_monitor
source install/setup.bash
```

## 杩愯鏂规硶

```bash
ros2 run av_safety_monitor safety_monitor --ros-args \
    -p ttc_threshold_warning:=4.0 -p ttc_threshold_critical:=2.5 -p ttc_threshold_emergency:=1.5

ros2 run av_safety_monitor fault_injector --ros-args \
    -p fault_type:=drop -p target_topic:=/plan -p fault_probability:=0.1
```

璇濋: 璁㈤槄 `/ego_state`(TwistStamped)銆乣/perception_objects`(PerceptionObjectArray)銆?`/plan`銆乣/carla/ego_vehicle/collision`(CollisionEvent);
鍙戝竷 `/safety_status`(String)銆乣/safety_markers`銆乣/emergency_stop`(Bool)銆?
## 娴嬭瘯鏂规硶

```bash
cd src/av_safety_monitor
python -m pytest test -q
```

## 杩愯缁撴灉

```text
$ cd src/av_safety_monitor && python -m pytest test -q
.................                                                        [100%]
17 passed in 0.07s
```

瑕嗙洊: TTC 璁＄畻鍥涜薄闄?鎺ヨ繎/闈欐/杩滅/闆惰窛绂?銆佷笁绾ф姤璀﹂槇鍊笺€佺鎾炰簨浠跺己鍒?绱ф€ュ埗鍔ㄣ€佹劅鐭ュ洖璋冩渶杩戦殰纰嶆洿鏂般€佹晠闅滄敞鍏?drop/noise/bias)杞彂璇箟銆?
> 璇存槑: 鏈満(Windows)鏈畨瑁?ROS2/CARLA, 鏃犳硶鎴彇浠跨湡杩愯鐢婚潰,
> 杩愯缁撴灉浠?*鐪熷疄缁堢杈撳嚭**浠ｆ浛鎴浘; 鍏ㄩ儴杈撳嚭鍧囧彲鎸変笂杩板懡浠ゅ鐜般€?
## 鏈淇璁板綍

1. `resource/av_safety_monitor` 缂哄け, `setup.py` 鐨?data_files 寮曠敤浼氬鑷?   colcon 鏋勫缓瀹夎澶辫触 鈫?琛ュ缓 ament 璧勬簮鏍囪;
2. `/perception_objects` 璁㈤槄绫诲瀷璇敤 `PointCloud2`(鍙戝竷鏂逛负
   `PerceptionObjectArray`, 鍥炶皟姘歌繙鏀朵笉鍒版秷鎭? 鈫?淇绫诲瀷骞跺湪鎺ュ彛鍙敤鏃惰闃?
3. `_perception_callback` 涓虹┖瀹炵幇, 鏈€杩戦殰纰嶈窛绂绘亽涓?inf銆乀TC 姘镐笉瑙﹀彂 鈫?   瀹炵幇鏈€杩戦殰纰嶈窛绂?閫熷害璁＄畻;
4. 绉婚櫎浠庢湭琚皟鐢ㄧ殑姝讳唬鐮?`_aeb_trigger`(鍏惰亴璐ｅ凡鐢?`_monitor_loop` 瑕嗙洊);
5. 鏂板 17 涓崟鍏冩祴璇曘€?
