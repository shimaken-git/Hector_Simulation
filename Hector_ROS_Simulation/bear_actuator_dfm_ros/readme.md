# about bear_actuator_dfm_ros driver
bear_actuator_dfm_rosはwestwood robotics Koala Bear actuator をROS上で、Direct Force Mode(DFM)で使用するためのドライバーです。
ただし、SteadyWin性GIM4010ドライバーを併用できます。

## Configuration
bear_actuator_dfm_ros driverは複合関節に対応しています。複合関節とはベベルギアやベルト、リンクなどを用いた複合関節構造を指します。

### hardware.yaml
hardware.yamlはアクチュエータ設定と関節設定を行います。
#### actuator setting
記述例
```
row_joint:
  joint1:              <= actuator name 関節設定で呼び出すアクチュエータ名
    ID: 1              <= actuator id アクチュエータ固有のID
    type: bear         <= actuator type
    limit_i_max: 10.0  <= actuator setting parameter
    mode: 3            <= actuator mode  0:torque 1:speed 2:position 3:direct force mode
  joint5:
    ID: 2
    type: gim
    cal_rpm: -7        <= GIM actuator は起動時に原点出しが必要。原点探索時の回転速度をrpmで指定する。
    limit: -1.30       <= 原点探索点の角度　原点探索により基準出し点の角度　これにより原点設定を行う。
```

### joint setting
記述例
```
robot_joint:
  L_hip_joint:           <= 関節名　ros driver で使用する関節名
    joint:
      - name: joint1     <= actuator指定 この関節で使用するアクチュエータを指定する。
        coeff: -1.0      <= actuator比率 この関節でのこのアクチュエータの支配率　極性を含める
    limit_p_angle: 0.2   <= トルク制御時の保護境界　limit_p_angleはプラストルクに対する境界
    limit_m_angle: -0.2  <= トルク制御時の保護境界　limit_m_angleはマイナストルクに対する境界
    protection_kd: 0.0   <= 保護動作時の速度ゼロ制御フィードバック係数
  L_hip2_joint:
    joint:
      - name: joint2     <= 複合関節の場合は複数のアクチュエータを指定する。
        coeff: -0.5
      - name: joint3
        coeff: 0.5
    limit_p_angle: 0.7
    limit_m_angle: -0.7
    protection_kd: 0.0

```

### lambda_leg_bear_effort_controllers.yaml
lambda_leg_bear_effort_controllers.yamlはros driver の設定を行う
```
lambda_leg:
    # Publish all joint states -----------------------------------
    joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 1000  

    # L Controllers ---------------------------------------
    L_hip_controller:                                              <= hardware.yamlで定義した関節名
        type: bear_effort_controllers/JointPositionController      <= ros driver このドライバーしか実装されていない。
        joint: L_hip_joint                                         <= hardware.yamlで定義した関節名
        pid: {p: 100.0, i: 0.0, d: 5.0, i_clamp_min: -1.0, i_clamp_max: 1.0}  <= driver parameter 未実装

```

## connection
Koala Bear アクチュエータはBEAR専用RS485アダプタ(8Mbps)を使用。
デバイス名はbear_actuator_ros.launchで指定する。
GM4010 はCANインターフェイスを使用。
デバイス名はcan0固定。#こちらもlaunchにて指定できるように変更予定

## Usage
### 立ち上げ
term1
```
roslaunch bear_actuator_dfm_ros bear_actuator_ros 
```
term2
```
roslaunch bear_description wwlambda_r2_rviz_dfm.launch
```

#### command
コマンドトピックによりトルクON/OFF　リセットなどを行うことができる
```
rostopic pub -1 /pwr_cmd std_msgs/String "data: 'on'"       <= torque ff
rostopic pub -1 /pwr_cmd std_msgs/String "data: 'off'"      <= torque off
rostopic pub -1 /pwr_cmd std_msgs/String "data: 'reset'"    <= reset
```
ドライバーが起動した時点で前アクチュエータはトルクゼロでトルクオン状態となります。
GIMアクチュエータが含まれている場合は角度キャリブレーションを実行します。
関節角度がhardware.yamlで指定した角度範囲を超えた場合、エラーとなり、以降のトルク指定は無視されます。
ただし、トルクゼロで指定される位置制御指令は実行されます。

リセットコマンドを送ることでエラーは解消します。

