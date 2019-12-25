// TWELITEの親機にシリアルでデータを送って無線で子機に転送
// 
// TWELITEのファームウェアは公式配布の「シリアル通信アプリ」
// 書式モード・バイナリー形式（AI2ピンLo）
// 115200bps，8bit parity none, ストップビット1

use std::io;
use std::thread;
use std::io::prelude::*;
use std::time::Duration;
use std::fs;
use std::io::{Write, BufWriter, Read, BufRead, BufReader};
use serial;
use serial::prelude::*;

const SLAVE_ID: u8 = 0x78;  // ブロードキャストアドレス
const RESPONSE_ID: u8 = 0x13;  // 応答ID（任意に指定可能）

fn main() {
    let mut port = serial::open("/dev/ttyUSB1").expect("Couldn't open serial port.");
    port_config(&mut port).expect("Port configration.");

    thread::sleep(Duration::from_millis(1000));  // 少し待ってあげないとコケる


    // 送信データ
    let mut data = vec![0x12, 0x34, 0x56];
    let tx = make_packet(&mut data).unwrap();

    let mut rx_buf: Vec<u8> = vec![0; 30];
    loop {
        port.write(&tx[..]).unwrap();
        let _byte_num = port.read(&mut rx_buf[..]).unwrap();

        let values = match parser(&rx_buf) {
            Ok(val) => val,
            Err(err) => {
                println!("parser: {}", err);
                (0, 0, 0, 0, 0, vec![0])  // 適当な値を返しておく
            },
        };

        // パーサで取り出したデータをHexで表示
        print!("[");
        for i in 0..(values.5).len() {
            print!("0x{:X}, ", (values.5)[i]);
        }
        print!("]\n");

        // 受信データ表示（デバッグ用）
        //for i in 0..30 {
        //    print!("0x{:X}, ", rx_buf[i]);
        //}
        //print!("\n");

        thread::sleep(Duration::from_millis(800));
    }
}

// 任意データ部を入力して，TWELITEに入力するためのデータ構造を作る．
// TWELITEの最大送信バイト数は640Byteだが，パケットが大きすぎると送信段階で
// 複数に分かれてしまうため80Byte以下にすることが推奨されている．
// この実装で扱える任意データの最大サイズは，オプション列を考慮して620Byte
fn make_packet(data: &mut Vec<u8>) -> Result<Vec<u8>, &'static str> {
    if data.len() > 620 {
        return Err("Data size exceeds 620Byte.");
    }

    let mut packet: Vec<u8> = vec![0xA5, 0x5A];  // ヘッダ
    // データ長を付加
    let data_len = data.len() as u16;
    let tmp_h = (data_len >> 8) as u8;
    let tmp_l = (data_len & 0xFF) as u8;
    packet.push(0x80 | tmp_h);
    packet.push(tmp_l);

    // ----- データ部を作る（拡張形式の送信コマンド）----- //
    packet.append(&mut vec![SLAVE_ID, 0xA0, RESPONSE_ID]);
    packet.append(&mut vec![0x01, 0xFF]);  // オプション列（0x01はMAC ACKの設定）

    // データを入れて最後にチェックサムを付加
    let checksum = calc_checksum(data);
    packet.append(data);
    packet.push(checksum);
    Ok(packet)
}

/// データ部の各バイトのXORを計算する
fn calc_checksum(data: &Vec<u8>) -> u8 {
    let mut num: u8 = data[0];
    for i in 1..data.len() {
        num ^= data[i];
    }
    num
}

/// 受信バッファの値から任意データ部その他諸々の値を取り出す
/// 返り値：(送信元ID: u8, 応答ID: u8, 送信元拡張アドレス: u32, 
///          送信先（受信側）拡張アドレス: u32, LQI値: u8, 任意データ: Vec<u8>)
fn parser(rx: &Vec<u8>) -> Result<(u8, u8, u32, u32, u8, Vec<u8>), &'static str> {
    let mut optinal_data: Vec<u8> = Vec::new();
    let mut header_flag = false;
    let mut i: usize = 0;

    // ヘッダを探す
    for _ in 0..(rx.len() - 3) {  // 3引いておかないとデータ長を読むときにオーバーランする
        if rx[i] == 0xA5 {
            i += 1;
            if rx[i] == 0x5A {
                header_flag = true;
                break;
            }
        }
        i += 1;
    }

    // ヘッダを読み出せずに最後まで行ってしまった場合の処理
    if header_flag == false {
        return Err("Header does not exist.");
    }

    // データ長を読む
    i += 1;
    let data_len: usize;
    if (rx[i] & 0x80) == 0x80 {
        let tmp_h = ((rx[i] & 0x7F) as usize) << 8;
        i += 1;
        let tmp_l = rx[i] as usize;
        data_len = tmp_h | tmp_l;
    } else {
        return Err("Syntax error (The 3rd byte MSB is not 1).");
    }

    // データ長が残りのバッファサイズを超えていた場合の処理
    if data_len >= (rx.len() - (i + 2)) {
        return Err("Data length exceeds packet size.");
    }

    // 送信元ID
    i += 1;
    let sender_id = rx[i];
    let mut checksum = rx[i];  // ここからデータ部の全てのByteのXORをとっていく．
    // 固定値を見て構文の整合性を確認
    i += 1;
    if rx[i] != 0xA0 {
        return Err("Syntax error (The sixth of the packet is not 0xA0).");
    }
    checksum ^= 0xA0;

    // 応答ID
    i += 1;
    let response_id = rx[i];
    checksum ^= rx[i];
    // 送信元拡張アドレス
    let mut sender_extend_addr: u32 = 0;
    for j in 0..4 {
        i += 1;
        sender_extend_addr |= (rx[i] as u32) << ( (3 - j) * 8 );
        checksum ^= rx[i];
    }
    // 送信先拡張アドレス（自分の拡張アドレスのこと）
    // 未設定の場合は0xFFが4Byte入っている
    let mut self_extend_addr: u32 = 0;
    for j in 0..4 {
        i += 1;
        self_extend_addr |= (rx[i] as u32) << ( (3 - j) * 8 );
        checksum ^= rx[i];
    }

    // LQI(電解強度)値
    i += 1;
    let lqi = rx[i];
    checksum ^= rx[i];

    // 続く任意データ領域のByte数
    let mut optional_data_len: usize = 0;
    i += 1;
    checksum ^= rx[i];
    optional_data_len |= (rx[i] as usize) << 8;
    i += 1;
    checksum ^= rx[i];
    optional_data_len |= rx[i] as usize;

    // 任意データ領域が残りのバッファサイズを超えていた場合の処理
    if optional_data_len >= (rx.len() - (i + 2)) {
        return Err("Optional data does not fit in Rx buffer.");
    }

    // 任意データ
    i += 1;
    for j in i..(i + optional_data_len) {
        checksum ^= rx[j];
        optinal_data.push( rx[j] );
    }
    i += optional_data_len;

    // チェックサムで整合性を確認
    if rx[i] != checksum {
        return Err("Checksum mismatch.");
    }

    // フッタを確認
    i += 1;
    if rx[i] != 0x04 {
        return Err("Footer does not exist.");
    }

    Ok((
        sender_id,           // 送信元ID
        response_id,         // 応答ID
        sender_extend_addr,  // 送信元拡張アドレス
        self_extend_addr,    // 送信先拡張アドレス
        lqi,                 // 通信LQI値
        optinal_data         // 任意送信データ
    ))
}

/// 通信速度など，シリアルポートの各種設定
fn port_config(port: &mut SerialPort) -> io::Result<()> {
    port.reconfigure(&|settings| {
        settings.set_baud_rate(serial::Baud115200)?;
        settings.set_char_size(serial::Bits8);
        settings.set_parity(serial::ParityNone);
        settings.set_stop_bits(serial::Stop1);
        settings.set_flow_control(serial::FlowNone);
        Ok(())
    })?;
    port.set_timeout(Duration::from_millis(5000))?;
    Ok(())
}


#[test]
fn test_parser() {
    // データ部（拡張形式）
    let mut data: Vec<u8> = vec![0x00, 0xA0, 0x13, 
                             0x1A, 0x1B, 0x1C, 0x1D, 
                             0x2A, 0x2B, 0x2C, 0x2D, 
                             0xFF, 0x00, 0x03, 0x12, 
                             0x34, 0x56];
    let checksum = calc_checksum(&data);
    let mut rx_buf = vec![0xA5, 0x5A, 0x80, 0x11];
    rx_buf.append(&mut data);
    rx_buf.push(checksum);
    rx_buf.push(0x04);
    
    let vals = parser(&rx_buf).unwrap();
    assert_eq!(vals.0, 0x00);  // 送信元ID
    assert_eq!(vals.1, 0x13);  // 応答ID
    assert_eq!(vals.2, 0x1A1B1C1D);  // 送信元拡張アドレス
    assert_eq!(vals.3, 0x2A2B2C2D);  // 送信先拡張アドレス
    assert_eq!(vals.4, 0xFF);  // LQI値
    assert_eq!(vals.5, vec![0x12, 0x34, 0x56]);  // 任意データ
}