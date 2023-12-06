# P410
def get_alarm_name(code):

    alarm_list = \
          [False for i in range(16)] \
        + ["位置偏差過大"] + [False for i in range(15)] \
        + ["過電流", "主回路加熱", "過電圧", False, False, "不足電圧", "モーター過熱", False, "エンコーダ異常", "内部回路異常", "エンコーダ通信異常", False, False, "モーター接続異常", False, False] \
        + ["過負荷", "過速度"] + [False for i in range(14)] \
        + [False, "EEPROM異常",  "初期時エンコーダ異常", False, "エンコーダEEPROM異常",  "モーター組合せ異常",  False,  False,  False,  False,  "原点復帰未完了", False, False, False, False, False] \
        + ["電磁ブレーキ過電流",  False, False, "HWTO入力回路異常", False, "電磁ブレーキ接続異常"] + [False for i in range(10)] \
        + ["±LS同時入力", "±LS逆接続", "原点復帰運転異常", "HOMES未検出", "Z, SLIT信号異常", False, "ハードウェアオーバートラベル", "ソフトウェアオーバートラベル", "HWTO入力検出", False, "原点復帰追加運転異常", False, False, False, "ユーザーアラーム", False] \
        + ["運転データ異常", "単位設定異常"] + [False for i in range(14)] \
        + [False, "ネットワークパス異常", False, False, "RS-485通信異常", "RS-485通信タイムアウト", False, False, False, False, False, False, "読込み失敗"] 
 
    if code<8*16+13:
        return alarm_list[code]
    elif code==15*16:
        return "CPU異常"
    elif code==15*16+3:
        return "CPU過負荷"
    else:
        return False



if __name__ == "__main__":

    tx = ""
    for i in range(15*16+10):
        n = get_alarm_name(i)
        if n:
            tx += "%03d,%02x,"%(i,i)+n+"\n"
    print(tx)
    with open("alarm.csv", "w") as f:
        f.write(tx)

