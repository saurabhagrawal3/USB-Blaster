USB-Blasterもどき
マイクロチップのUSBファームウェアVer.2.1-Genericをベースに作っています。
ピン配置変更、セラロック周波数変更等で再コンパイルする場合下記の環境が必要です。
・MPLAB IDE
・C18 Compiler
・Microchip USB Firmware Ver.2.1(必要ファイルを同梱しています)
上記ツールをインストールした上でプロジェクトファイルを開き、
Configure->SelectDevice画面でPIC18F2550を選択してください。
クロック周波数変更時は、「main2.c」ファイルの60行目「#pragma config PLLDIV = 5」
をクロック周波数/4MHzの値に書き換えてください。
ピンアサインは110-121行目のdefine文で行っています。他は変更しなくてよいはずです。