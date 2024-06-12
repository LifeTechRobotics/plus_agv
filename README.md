Ubuntu 22.04でROS2（humble）の環境を構築します。
python3.xxの環境が必要です。

m5moverのパッケージ(Python)をROS2のワークスペースに作成します。
m5mover_msgのパッケージ(C/C++)をROS2のワークスペースに作成します。

各パッケージのソースコードやpackage.xml、setup.cfg、CMakeLists.txtを参照して環境にあわせてください。

m5mover pkg(python)はM5Stackとの通信をするためのパッケージです。
m5mover/joctlがジョイスティックのデータをm5moverに通知します。
m5mover/joctlはm5moverから台車のデータを受信します。
m5mover/joyctlはROS2のjoy_nodeを使用しています。
m5mover/m5moverはjoyctlからデータを受けてM5StackとTCP/IPで通信します
m5mover_msg pkg(C/C++)は専用のカスタムメッセージ用です。

M5StackはEsptochアプリ（スマートフォン専用）の2.4GHz通信を使用してAPのSSIDやキーを設定して使用します。
M5Stackが正常にAPと通信できると液晶にM5StackのIPアドレスが表示されます。
m5mover/m5moverには制御するM5StackのIPを設定する必要があります。


起動方法）
Ubuntu
１．ジョイスティックノード起動
1)Terminalを起動してROS2のワークスペースへ移動します。
cd ROS2ワークスペース
2)環境設定をします。
source install/setup.bash
3)ノードを起動します。
ros2 run joy joy_node

２．ジョイスティックコントローラノードを起動
1)Terminalを起動してROS2のワークスペースへ移動します。
cd ROS2ワークスペース
2)環境設定をします。
source install/setup.bash
3)ノードを起動します。
ros2 run m5mover joyctl

３．M5Stackノードを起動
1)Terminalを起動してROS2のワークスペースへ移動します。
cd ROS2ワークスペース
2)環境設定をします。
source install/setup.bash
3)ノードを起動します。
ros2 run m5mover m5mover

AGVを起動します。
