jetson xavier nxの環境構築

1. セットアップ
https://qiita.com/futakuchi0117/items/b815fa64563e98602bfc

2. jetson-ioでpwmを使えるように設定
https://tiryoh.hateblo.jp/entry/2019/12/24/221411
設定画面でpwm32,pwm33を使えるようにして保存し再起動．

3. rosの設定
熊野さんの作ったパワポ参考

４．apriltagの設定
川上先輩が使用していたimage_position_dataを得られるプログラムを使用．
https://github.com/soramame1625/apriltag_ros

build errorが出るときはエラーファイルのpythonのバージョンを変更


・jtopによる負荷状況の表示
https://www.fabshop.jp/jetson-nano-jtop/

・ファンモードをquietからcoolへ
$ sudo /usr/sbin/nvpmodel -d cool
現在のファンモードを確認
$ sudo /usr/sbin/nvpmodel -q

・ファンモード最大
$ sudo sh -c 'echo 255 > /sys/devices/pwm-fan/target_pwm'

・パフォーマンスの最大化
https://developer.ridgerun.com/wiki/index.php/Xavier/JetPack_5.0.2/Performance_Tuning/Maximizing_Performance
jtopの5CTRLでも変更可能
jetson_clocks大事
