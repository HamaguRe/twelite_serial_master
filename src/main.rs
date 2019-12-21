// TWELITEの親機にシリアルでデータを送って無線で子機に転送
// 
// TWELITEのファームウェアは公式配布の「シリアル通信アプリ」
// 書式モード，バイナリー形式
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

fn main() {
    let mut port = serial::open("/dev/ttyUSB1").expect("Couldn't open serial port.");
    port_config(&mut port).expect("Port configration.");

    thread::sleep(Duration::from_millis(1000));  // 少し待ってあげないとコケる

    // 送信データ（拡張形式）
    // 子機から親機に ACK 付きで 123456 を送信する
    let mut data = vec![SLAVE_ID, 0xA0, 0x13, 0x01, 0xFF, 0x12, 0x34, 0x56];
    //let mut data = vec![0x12, 0x34, 0x56];
    let tx = make_packet(&mut data).unwrap();


    let mut rx_buf: Vec<u8> = vec![0; 30];
    loop {
        port.write(&tx[..]).unwrap();
        let byte_num = port.read(&mut rx_buf[..]).unwrap();

        for i in 0..30 {
            print!("{:X}, ", rx_buf[i]);
        }
        print!("\n");

        thread::sleep(Duration::from_millis(800));
    }
}

// データ部を入力して，TWELITEに入力するためのデータ構造を作る．
// TWELITEの最大送信バイト数は640Byteだが，パケットが大きすぎると送信段階で
// 複数に分かれてしまうため80Byte以下にすることが推奨されている．
fn make_packet(data: &mut Vec<u8>) -> Result<Vec<u8>, &'static str> {
    if data.len() > 640 {
        return Err("Data size exceeds 640Byte.");
    }

    let mut packet: Vec<u8> = vec![0xA5, 0x5A];  // ヘッダ
    // データ長を付加
    let length = data.len() as u16;


    let length = data.len() as u8;
    packet.push(0x80);
    packet.push(length);
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
fn parser(rx: Vec<u8>) -> Result<(u8, u8, u32, u32, u8, Vec<u8>), &'static str> {
    let mut optinal_data: Vec<u8> = Vec::new();

    let mut i: usize = 0;
    // ヘッダを探す
    for _ in 0..(rx.len() - 1) {
        if rx[i] == 0xA5 {
            i += 1;
            if rx[i] == 0x5A {
                break;
            } else {
                return Err("Header does not exist.");
            }
        }
        i += 1;
    }

    // データ長を読む
    let data_len: usize;
    if rx[i] == 0x80 {
        i += 1;
        data_len = rx[i] as usize;
    } else {
        return Err("Syntax error (The 3rd byte MSB is not 1).");
    }

    // データ長が残りのバッファサイズを超えていた場合の処理
    if data_len >= (rx.len() - i) {
        return Err("Data length exceeds packet size.");
    }

    // 送信元ID
    i += 1;
    let sender_id = rx[i];
    let mut checksum = sender_id.clone();  // ここから全てのByteのXORをとっていく．
    // 固定値を見て構文の整合性を確認
    i += 1;
    if rx[i] != 0xA0 {
        return Err("Syntax error(The sixth of the packet is not 0xA0).");
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
    // 送信先拡張アドレス（自分のアドレスのこと）
    // 未設定の場合は0xFFが4Byte入っている
    let mut self_extend_addr: u32 = 0;
    for j in 0..4 {
        i += 1;
        self_extend_addr |= (rx[i] as u32) << ( (3 - j) * 8 );
        checksum ^= rx[i];
    }

    // 通信品質LQI値
    i += 1;
    let lqi = rx[i];
    checksum ^= rx[i];

    // 続く任意データ領域のByte数
    // u16で十分だけど，for式で使う都合上usizeにする
    let mut optional_data_len: usize = 0;
    i += 1;
    optional_data_len |= (rx[i] as usize) << 8;
    checksum ^= rx[i];
    i += 1;
    optional_data_len |= rx[i] as usize;

    // データ長が残りのバッファサイズを超えていた場合の処理
    if optional_data_len >= rx.len() {
        return Err("Optional data does not fit in Rx buffer.");
    }

    // 任意データ
    let mut num = 0;
    for j in i..(i + optional_data_len) {
        checksum ^= rx[j];
        optinal_data.push(rx[j]);
        num += 1;
    }
    i += num - 1;

    // チェックサムで整合性を確認
    i += 1;
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