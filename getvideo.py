import socket
import subprocess
import time
from collections import deque
from datetime import datetime
from pathlib import Path

INPUT_PORT = 1234
VLC_PORT = 1235

KEEP_SECONDS = 60.0
IDLE_TIMEOUT = 2.0

VLC_EXE = Path(r"D:\Program Files\VideoLAN\VLC\vlc.exe")
OUTPUT_DIR = Path(r"D:\record")


def save_recording(buffer, recording_start, recording_end):
    if not buffer:
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    temporary_path = OUTPUT_DIR / f"record_{timestamp}.ts.part"
    final_path = OUTPUT_DIR / f"record_{timestamp}.ts"

    total_size = 0

    with temporary_path.open("wb", buffering=1024 * 1024) as file:
        for _, data in buffer:
            file.write(data)
            total_size += len(data)

    temporary_path.replace(final_path)

    complete_duration = recording_end - recording_start
    saved_duration = buffer[-1][0] - buffer[0][0]

    print(f"录制结束：{final_path}")
    print(f"本次视频时长：{complete_duration:.1f} 秒")
    print(f"实际保存时长：{saved_duration:.1f} 秒")
    print(f"文件大小：{total_size / 1024 / 1024:.2f} MB")


def main():
    if not VLC_EXE.exists():
        raise FileNotFoundError(f"找不到 VLC：{VLC_EXE}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_RCVBUF,
        8 * 1024 * 1024
    )
    receiver.bind(("0.0.0.0", INPUT_PORT))

    # 每0.2秒检查一次是否已经超时
    receiver.settimeout(0.2)

    forwarder = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # VLC播放Python转发到1235端口的视频
    vlc = subprocess.Popen([
        str(VLC_EXE),
        f"udp://@:{VLC_PORT}",
        "--no-video-title-show",
        "--no-one-instance",
    ])

    # 保存(timestamp, UDP payload)
    buffer = deque()

    recording = False
    recording_start = None
    last_packet_time = None
    buffered_size = 0

    print(f"正在监听 udp://@:{INPUT_PORT}")
    print("收到视频流时自动开始录制")
    print("内存中只保留最后 60 秒")
    print("连续 2 秒没有数据时自动保存")
    print("按 Ctrl+C 退出")

    try:
        while True:
            try:
                data, _ = receiver.recvfrom(65535)
                now = time.monotonic()

                # 将视频转发给VLC实时播放
                forwarder.sendto(data, ("127.0.0.1", VLC_PORT))

                if not recording:
                    recording = True
                    recording_start = now
                    buffered_size = 0
                    buffer.clear()

                    print(
                        "\n开始录制："
                        f"{datetime.now():%Y-%m-%d %H:%M:%S}"
                    )

                buffer.append((now, data))
                buffered_size += len(data)
                last_packet_time = now

                # 删除超过最近60秒的数据
                cutoff = now - KEEP_SECONDS

                while buffer and buffer[0][0] < cutoff:
                    _, expired_data = buffer.popleft()
                    buffered_size -= len(expired_data)

            except socket.timeout:
                if (
                    recording
                    and last_packet_time is not None
                    and time.monotonic() - last_packet_time
                    >= IDLE_TIMEOUT
                ):
                    # 使用最后一个数据包的时间作为视频结束时间，
                    # 不把两秒静默时间计算在视频时长内
                    save_recording(
                        buffer,
                        recording_start,
                        last_packet_time
                    )

                    buffer.clear()
                    recording = False
                    recording_start = None
                    last_packet_time = None
                    buffered_size = 0

                    print("\n继续等待下一次视频流……")

    except KeyboardInterrupt:
        print("\n正在退出……")

        # 手动退出时保存当前缓存
        if recording and buffer:
            save_recording(
                buffer,
                recording_start,
                last_packet_time
            )

    finally:
        receiver.close()
        forwarder.close()

        if vlc.poll() is None:
            vlc.terminate()


if __name__ == "__main__":
    main()