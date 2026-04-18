
import time
try:
    from .driver import SkydroidDriver
except ImportError:  # pragma: no cover - supports running files directly
    from driver import SkydroidDriver

SKY_IP   = "192.168.144.108"
LOCAL_IP = "192.168.144.10"

def main():
    print(">>> Skydroid Driver Test <<<")

    with SkydroidDriver(sky_ip=SKY_IP, local_ip=LOCAL_IP) as drv:

        # Test 1: connection check
        print("\n=== Test 1: Connection ===")
        version = drv.check_connection()
        if version is None:
            print("Connection failed")
            return
        print(f"Connection OK — Firmware: {version}")

        # Test 2: PTZ
        print("\n=== Test 2: PTZ ===")
        for action, label in [
            ("up",     "tilt up"),
            ("stop",   "stop"),
            ("down",   "tilt down"),
            ("stop",   "stop"),
            ("center", "center"),
        ]:
            print(f"  -> {label}")
            drv.g_ptz(action)
            time.sleep(0.8)
        print("PTZ OK")

        drv.g_speed(pitch=-90)
        time.sleep(1.0)
        drv.g_speed(pitch= 90)
        time.sleep(1.0)

        drv.g_speed(yaw=120)
        time.sleep(1.0)
        drv.g_speed(yaw=-120)
        time.sleep(1.0)

        drv.g_speed(yaw=120, pitch=120)
        time.sleep(1.0)
        drv.g_speed(yaw=-120, pitch=-120)
        time.sleep(1.0)

        drv.g_speed()     # stop both axes


        drv.g_angle(pitch=0)
        drv.g_angle(pitch=-30.0)                          # pitch, default speed 3.0 deg/s
        time.sleep(1.0)
        drv.g_angle(pitch=30.0, pitch_speed_dps=13.0)     # pitch faster
        drv.g_angle(pitch=0)

        drv.g_angle(yaw=0, yaw_speed_dps=10.0)
        time.sleep(1.0)
        drv.g_angle(yaw=45.0, yaw_speed_dps=10.0)         # yaw fast
        time.sleep(1.0)
        drv.g_angle(yaw=-45.0, yaw_speed_dps=10.0)        # yaw fast reverse
        time.sleep(1.0)
        drv.g_angle(yaw=0, yaw_speed_dps=10.0)
        time.sleep(1.0)

        drv.g_angle(yaw=30.0, pitch=-30.0,
                    yaw_speed_dps=5.0,
                    pitch_speed_dps=2.0)                  # independent speed per axis
        time.sleep(1.0)
        drv.g_angle(yaw=0.0, pitch=0.0,
            yaw_speed_dps=5.0,
            pitch_speed_dps=2.0)

        # Test 4: attitude push
        print("\n=== Test 4: Attitude push (5 seconds) ===")
        drv.g_enable_attitude_push(freq_hz=1)
        deadline = time.time() + 5.0
        count = 0
        while time.time() < deadline:
            att = drv.recv_attitude(timeout=0.5)
            if att is None:
                continue

            count += 1
            print(f"  Y={att['yaw_deg']:7.2f}  "
                  f"P={att['pitch_deg']:7.2f}  "
                  f"R={att['roll_deg']:7.2f}")
        drv.g_enable_attitude_push(freq_hz=0)
        print(f"Attitude OK — {count} packets received")

        # Return to center before exiting
        drv.g_ptz("center")
        time.sleep(1.0)

    print("\n>>> Done <<<")

if __name__ == "__main__":
    main()
