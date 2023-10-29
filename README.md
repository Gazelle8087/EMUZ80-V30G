# EMUZ80-V30G

奥江聡さんが開発されたEMUZ80+メザニンボードMEZV20の
ファームウエアをベースとして、T1サイクル中にしか出力されない
アドレス値をPICから供給されるクロックを一時停止して
取得する方法を模索したファームウエアです。

元のファームウエア同様
リセットにてUniversal monitor が立ち上がり、
Bコマンドにて Nascom Basic 起動となります。

現在のところPIC18F47Q84 + V30 8MHz限定です。
ソース中 #define emulation8080 を有効にすると
8086basicに代わって 8080エミュレーションモードを
利用したSBCV20用の V20BASICが組み込まれます。
V20BASICの起動はmonitorから g0:814b と入力ください

![v30_chart](https://github.com/Gazelle8087/EMUZ80-V30G/assets/148423174/df5f935c-bb4b-45b3-a1ef-f7e2c9843aa0)
![ALE_CLOCK_STOP](https://github.com/Gazelle8087/EMUZ80-V30G/assets/148423174/3478d1fb-3a6a-4b35-9ffa-09ba50ee4e11)
