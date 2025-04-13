# About hector_control
このプログラムはhector_simulationリポジトリからforkし、本家Hectorとは異なるハードウェア、関節構成を持つ二足歩行ロボットの制御プログラムとして作られたものである。
hector_simulationリポジトリではGazeboシミュレエータ上で動かすことを前提としたプログラムとなっているが、本プログラムは実ロボットを制御することを前提としている。

## configuration
本プログラムのconfig/robot.yamlは現在未実装でロボット構造の指定、各種制御パラメータはプログラム内に直接記述されている。
今後、外部設定により各種設定ができるようにしていきたいと思ってはいる。

## Usage

## Finite State Machine (FSM)
このプログラムの状態遷移は以下

- Passive
    プログラム開始状態　また、ロボットが転倒した場合もこの状態になる
- Standing
    ただ立っている状態。制御は行われず、関節は位置制御
- TO
- Walikng
    歩行状態

### 遷移
状態の遷移は以下の条件で行われる。操作はキーボードまたはコントローラで行われるが、現状、コントローラの接続は未実装

- Passive
    9 Standing
- Standing
    0 Walking
- Walking
    1 Passive
    9 Standing
    0 Walking
    転倒 Passiveへ

## Programing

### include/commmon/robot_select.h

'''
//#define _HECTOR_     <= 本家Hector
//#define _LAMBDA_     <= 旧ロボット設定　事実上不要
#define _LAMBDA_R2_    <= 現状のロボット設定

#ifdef _LAMBDA_R2_
#define BEAR_REAL          <= 実機設定 コメントアウトするとGazebo連携となる
#define TORQUE_RESTRICT    <= トルク制限設定 デバッグ用設定。
#define debug              <= デバッグ設定
#endif

#define FOOTSENSOR         <= 足裏センサー使用　足裏センサーを起点に自己位置計算を行う。本日現在まともに動かない
#define HUMAN              <= ヒト足モード 本家Hectorは鳥足構造を取っている。_LAMBDA_R2_設定時のみ有効　コメントアウトすると鳥足になる。

'''
### About Robot Setting
ロボットの構造や制御設定はできるだけinclude/common/Biped.hに集約するようにしているが完全ではない。
以下に注意点を示す。
#### include/common/Biped.h
ロボットの構造や制御設定はできるだけここに集約している。ただし、完全ではない。
- mass MPCの設定に使われている。
    src/FSM/FSMState_Walking.cpp の FSMState_Walking::FSMState_Walking()
    '''
    Cmpc(0.001, 40, data->_biped->height, data->_biped->mass)
    '''
    Biped.h -> ConvexLocomotion::ConvexLocomotion() -> update_problem_data() -> solver_mpc() -> ct_ss_mats()　と最終的にct_ss_mats()にて適用される。
- height MPCの設定に使われている(上述)、ここで設定が完結していない。
    - gazebo
        unitree_ros/unitree_gazebo/launch/wwlambda_rs.launch内でz値が指定されている
- LegController.cpp
    src/common/LegController.cppのcomputeLegJacobianAndPosition()ではロボット構造値を独自で設定している
- LegIk.cpp
    src/common/LegIk.cppのcomputeIK_()では一部構造値をコード上で記述している。
- footHeight 遊脚の足上げ高さ
    include/common/SwingLegController.h
    '''
    #ifdef _LAMBDA_R2_
    #ifdef debug
            const double footHeight = 0.06;        //足上げ高さ
    #else
            const double footHeight = 0.15;        //足上げ高さ
    #endif
    #endif
    '''
### About MPC Setting
- MPC Weights
    src/ConvexMPC/ConvexMPCLocomotion.cppのConvexMPCLocomotion::updateMPCIfNeeded()
    '''
    //MPC Weights
    double Q[12] = {100, 100, 250,  1, 200, 300,  1, 1, 1,  1, 1, 1}; // roll pitch yaw x y z droll dpitch dyaw dx dy dz
    double Alpha[12] = {1e-4, 1e-4, 5e-4, 1e-4, 1e-4, 5e-4,   1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};
    '''
- horizonLength, mu, f_max
    src/ConvexMPC/ConvexMPCLocomotion.cppのConvexMPCLocomotion::ConvexMPCLocomotion()
    '''
    horizonLength(10),
    '''

    '''
    #ifdef _HECTOR_
    f_max = 500;
    mu = 0.25;
    #else
    #ifdef _LAMBDA_
    f_max = 100;
    mu = 0.25;
    #else
    #ifdef _LAMBDA_R2_
    f_max = 200;
    mu = 0.25;
    #endif
    #endif
    #endif
    '''
- 足裏摩擦　lt, lh
    src/ConvexMPC/SolverMPC.cppのsolve_mpc()
    '''
    // Initalization of Line Contact Constraint Parameters
    fpt mu = setup->mu;
    #if defined(_HECTOR_)
    fpt lt = 0.09;
    fpt lh = 0.06;
    #else
    #if defined(_LAMBDA_) || defined(_LAMBDA_R2_)
    #ifdef HUMAN
    fpt lt = 0.02;        //gazeboPluginでの設定
    fpt lh = 0.04;        //gazeboPluginでの設定
    #else
    fpt lt = 0.04;
    fpt lh = 0.02;
    #endif
    #endif
    #endif
    '''
    足裏摩擦に関する設定　うまく設定しないと足首がめくれるようになってしまう。
- dt, iterationsBetweenMPC
    上述したが、ConvexMPCLocomotion::Cmpcの初期化時に dt, iterationsBetweenMPC を設定している。
    src/FSM/FSMState_Walking.cpp の FSMState_Walking::FSMState_Walking()
    '''
    Cmpc(0.001, 40, data->_biped->height, data->_biped->mass)
    '''

    SwingLegController.h の class swingLegController に、_dt がある。統合すべきだが、できていない。dtを変更する場合は注意。

- 慣性モーメント I_body
    src/ConvexMPC/RobotState.cpp の RobotState::set()
    '''
    #ifdef _LAMBDA_R2_
        I_body << 50320829.484e-9, -788.372e-9, -5101.664e-9,
                -788.372e-9,  44848336.284e-9, -156655.156e-9,
                -5101.664e-9, -156655.156e-9, 17863828.962e-9;
    #endif
    '''

### 関節パラメータ設定(Kp, Kd)
関節の制御パラメータ、アクチュエータの制御パラメータは歩行制御にとって重要な要素であるが、これらは以下の個所で設定されている。
- Standing Leg
    src/common/StandLegController.cppのsetDesiredJointState()
- Swing Leg
    src/common/SwingLegController.cppのsetDesiredJointState()
- Stance Leg(支持脚)
### About IMU
### 自己位置について
### About Swing Leg
このプログラムでロボットが歩けるのはSwingLegControllerにちょっとした仕掛けがあることが大きなポイントとなっている。
それは支持脚が遊脚(Swing Leg)に切り替わる際、一瞬だけ蹴り足となるようになっていることである。
本家Hector Simulationではrobot descriptionの基準姿勢とhector_control上のロボットの基準姿勢に差異があり、シミュレータ上のロボットは自動的に蹴り足となるようになっていたため（バグかも）であるが、本プログラムは実機適用を前提とするため、実機とコントローラのモデル差異はなくし、蹴り足をプログラムしている。
以下のその個所を示す。
#### src/common/swingLegController.cpp
'''
void swingLegController::computeFootDesiredPosition(){
..
..
#ifdef debug
            pDesFootWorld[2] -=0.0;
#else
            pDesFootWorld[2] -=0.02;
#endif
..
..
}

'''
ここで、遊脚の初期位置を地面から-2cmと設定することで蹴り足を実現している。（デバッグ設定では蹴り足を無効になる）

