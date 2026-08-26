# =====================
# Test LkMotor
# =====================

import time
import numpy as np
import os
# stable version for motor's state feedback
from motor_library import LKMotor, prime_watchdog
import motor_gains as param

def main():

    ## ========== File Settings ==========
    log_dir = "log"

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    filename_only = f"Test_log_{time.strftime('%Y%m%d_%H%M%S')}.txt"
    log_filename = os.path.join(log_dir, filename_only)
    log_file = open(log_filename, "w", encoding="utf-8")

    ## ========== Motor Settings ==========
    # m1: hip motor; m2: knee motor
    # Only one motor for Test
    m1 = LKMotor(bus_interface="socketcan", bus_channel="can0", motor_id=2, bitrate=1_000_000)
    # m2 = LKMotor(bus_interface="socketcan", bus_channel="can0", motor_id=2, bitrate=1_000_000)

    print("Connected to motors with CanBus")

    # Feed the MCU comms watchdog through power-on: start THIS script first, then
    # switch on motor power. Without it the watchdog trips ~500 ms after power-on
    # (latched, needs a power cycle). See motor_library.prime_watchdog().
    try:
        if not prime_watchdog(m1):
            print("Motor not ready (watchdog) -- aborting.")
            m1.motor_release()
            return
    except KeyboardInterrupt:
        print("\nAborted before start.")
        m1.motor_release()
        return

    m1.motor_run()
    # m2.motor_run()

    try:
        ## ========== Preparing ==========
        print(f"Moving to start position: {param.th1_begin_deg} deg")
        m1.multi_turn_position_control(
            int(param.th1_begin_deg * param.pos_gain * param.dir_1),
            int(param.max_speed_pos),
        )
        # m2.multi_turn_position_control(int(param.th2_jump_begin_deg * param.pos_gain * param.dir_2), int(param.max_speed_pos))
        time.sleep(5)

        target_positions = [param.th1_pos_a_deg, param.th1_pos_b_deg]
        target_index = 0

        while True:
            target_deg = target_positions[target_index]
            target_index = (target_index + 1) % len(target_positions)

            res1 = m1.multi_turn_position_control(
                int(target_deg * param.pos_gain * param.dir_1),
                int(param.max_speed_pos),
            )
            # res2 = m2.multi_turn_position_control(int(target_deg * param.pos_gain * param.dir_2), int(param.max_speed_pos))

            if res1 is None:
                print("====== Lost Packet ======")
                time.sleep(param.position_hold_time)
                continue

            temperature1, iq1_raw, spd1_raw, enc1 = res1
            # temperature2, iq2_raw, spd2_raw, enc2 = res2

            th1_rad = np.deg2rad(enc1 * param.dir_1 * param.encoder_gain)
            # th2_rad = np.deg2rad(enc2 * param.dir_2 * param.encoder_gain)

            tau_iq1 = iq1_raw * param.dir_1 / param.torque_gain
            # tau_iq2 = iq2_raw * param.dir_2 / param.torque_gain

            dth1_rad = np.deg2rad(spd1_raw * param.dir_1 / param.vel_state_gain)
            # dth2_rad = np.deg2rad(spd2_raw * param.dir_2 / param.vel_state_gain)

            # ------ Saving Data ------
            print(
                f"target={target_deg:+.2f} "
                f"th1={np.rad2deg(th1_rad):+.2f} "
                f"tau_iq1={tau_iq1:+.2f} "
                f"temp1={temperature1}"
            )

            log_file.write(
                f"{time.time():.4f},"
                f"{target_deg:.4f},"
                f"{th1_rad:.4f},"
                f"{dth1_rad:.4f},"
                f"{tau_iq1:.4f},"
                f"{temperature1}\n"
            )
            log_file.flush()

            time.sleep(param.position_hold_time)

    except KeyboardInterrupt:
        print("Stopped by user.")

    finally:
        ## ========== Release Motors ==========
        m1.motor_release()
        # m2.motor_release()
        log_file.close()   # For Debug


if __name__ == '__main__':

    print('Start Test.')

    main()

    print('End Test.')
