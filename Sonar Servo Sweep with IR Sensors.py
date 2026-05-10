import sensors
import time

print("=== FULL SENSOR TEST ===")
print("Ctrl+C to stop\n")

while True:
    d1  = sensors.sonar1()
    i1  = sensors.ir1_val()
    i2  = sensors.ir2_val()
    il  = sensors.ir_left_val()
    ir  = sensors.ir_right_val()
    imu = sensors.read_imu()

    print(f"S1:{d1:6.1f}cm | "
          f"IR1:{i1} IR2:{i2} IR_L:{il} IR_R:{ir} | "
          f"Gz:{imu['gz']:+.2f}°/s")
    print()
    sensors.print_sweep(step=10)
    print()
    time.sleep_ms(500)