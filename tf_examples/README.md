# TFの説明のための例プログラム 

## 概要

- PythonでROS2のTF（TF2）を扱うための例プログラム．
- Ubuntu 20.04, ROS Foxyで作成・確認

## インストール

- ROSのワークスペースを`~/airobot_ws`とする．
  ```
  cd ~/airobot_ws/src
  ```

- このパッケージとsimple_armを含むリポジトリを入手．
  ```
  git clone https://github.com/AI-Robot-Book/appendixE
  git clone https://github.com/AI-Robot-Book/chapter6
  ```
- パッケージのビルド．
  ```
  sudo apt -y install ros-foxy-tf-transformations
  pip3 install transforms3d
  cd ~/airobot_ws
  colcon build --packages-select simple_arm tf_examples
  ```

## 実行

### TFのブロードキャスタ・リスナの単純な例

- 以下で使う端末では，次のコマンドを実行済みとする．
  ```
  source ~/airobot_ws/install/setup.bash
  ```
- 端末1
  ```
  ros2 run tf_examples planet_broadcaster
  ```

- 端末2
  - RVizの過去の設定を削除
    ```
    rm ~/.rviz2/default.rviz
    ```
  - RVizを起動
    ```
    rviz2
    ```

- 端末3
  ```
  ros2 run tf_examples satellite_broadcaster
  ```

- 端末4
  ```
  ros2 run tf_examples satellite_listener
  ```

- 端末5
  ```
  ros2 run tf_examples planet_broadcaster planet2 1 8
  ```

- 端末6
  - トピック
  ```
  ros2 topic list
  ros2 topic echo /tf
  ros2 topic echo /tf_static
  ros2 topic echo /pose
  ```
  - ノードグラフ
  ```
  rqt_graph
  ```
- 端末4
  - 実行中の`satellite_listener`をCtrl+cで終了させる．
  - 以下を実行．
    ```
    ros2 run tf_examples satellite_listener planet2
    ```

- ローンチファイル
  - ここまでの内容をまとめて実行する．
    ```
    ros2 launch tf_examples solar_system.launch.py 
    ```

### URDFやローンチとの組み合わせ・センサデータの座標変換

- 端末を開き，以下を実行．
  ```
  source ~/airobot_ws/install/local_setup.bash
  ros2 launch tf_examples simple_arm.launch.py
  ```

## ヘルプ

- tfに関係する操作をしているとRVizが時々落ちる．
- ROS2のPythonでは，C++の`tf2_ros::MessageFilter`に相当するものが用意されていない．また，`tf2_ros.Buffer.transform()`も使えない．

## 著者

升谷 保博

## 履歴

- 2022-08-23: ライセンス・ドキュメントの整備

## ライセンス

Copyright (c) 2022, MASUTANI Yasuhiro  
All rights reserved.  
This project is licensed under the Apache License 2.0 license found in the LICENSE file in the root directory of this project.

## 参考文献
