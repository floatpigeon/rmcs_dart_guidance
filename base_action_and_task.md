##　Task and Action　列表

- slider_init
    - belt_up

- launch_prepare
    - (fire_count == 0) belt_down(hold) -> trigger_lock -> belt_up
    - (fire_count > 0) belt_down(hold) -> trigger_lock -> filling_lift_down -> belt_up

- launch_cancel
    - belt_down(hold) -> trigger_free -> belt_up

- fire_preload
    - (fire_count == 0) trigger_free
    - (fire_count > 0) trigger_free -> filling_lift_up -> filling_limit_servo
