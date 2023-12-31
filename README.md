# EMUZ80-V30G

奥江聡さんが開発されたEMUZ80+メザニンボードMEZV20の
ファームウエアをベースとして、T1サイクル中にしか出力されない
アドレス値をPICから供給されるクロックを一時停止して
取得する方法を模索したファームウエアです。

V30用のソフトウエアは奥江さんと電脳伝説さんの成果物をそのままありがたく流用させていただいています。

奥江さんのEMUZ80-V20はこちらです
https://github.com/satoshiokue/EMUZ80-V20

電脳伝説さんのSBCV20はこちらです
https://vintagechips.wordpress.com/2020/12/27/sbcv20_software/

元のファームウエア同様
リセットにてUniversal monitor が立ち上がり、
Bコマンドにて Nascom Basic 起動となります。

V20用ソース emuz80-V20G.c

V30用ソース emuz80-V30G.c

私の手元の 8086-2はV30用ファームウエア（クロックデューティー1：1のまま）で動作しました。

ソース中 #define emulation8080 を有効にすると
8086basicに代わって 8080エミュレーションモードを
利用したSBCV20用の V20BASICが組み込まれます。
V20BASICの起動はmonitorから g0:814b と入力ください。

![v30_chart](https://github.com/Gazelle8087/EMUZ80-V30G/assets/148423174/df5f935c-bb4b-45b3-a1ef-f7e2c9843aa0)
![ALE_CLOCK_STOP](https://github.com/Gazelle8087/EMUZ80-V30G/assets/148423174/3478d1fb-3a6a-4b35-9ffa-09ba50ee4e11)

EMUZ80+MEZV20と全く同じ回路の1枚基板の回路図と基板設計データもここに置きます。
EMUV30_1103.zip でそのまま基板メーカーに発注できます。JLCPCBで実績ありです。
![DSC_0210](https://github.com/Gazelle8087/EMUZ80-V30G/assets/148423174/a4835a3f-18fd-4da1-a3cf-35c56b1e8c0b)
