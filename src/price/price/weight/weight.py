import time
from hx711 import HX711

class WeightSensor:

    def __init__(self, node):
        self.get_logger = node.get_logger

        # HX711 setup
        self.hx = HX711(5, 6)
        self.hx.reset()

        # calibration 
        self.calibration_value = -117.50

        self.minimum_item_weight = 5
        self.stability_samples = 5
        self.stability_tolerance = 5.0

        self.readings_buffer = []
        self.change_detected = False
        self.start_weight = None
        self.change_start_time = None

        time.sleep(0.5)

        # for taring=
        raw_data = self.hx.get_raw_data(times=5)
        if raw_data:
            self.tare_value = sum(raw_data) / len(raw_data)
        else:
            self.tare_value = 0

        self.prev_weight = self.get_weight()

    def get_weight(self):
        raw_data = self.hx.get_raw_data(times=5)
        if raw_data:
            average = sum(raw_data) / len(raw_data)
            weight = (average - self.tare_value) / self.calibration_value
            return weight
        return 0

    def read_weight(self):
        current_weight = self.get_weight()
        self.readings_buffer.append(current_weight)

        if len(self.readings_buffer) > self.stability_samples:
            self.readings_buffer.pop(0)

        # initial big change
        delta_weight = current_weight - self.prev_weight
        result = None
        
        if not self.change_detected and abs(delta_weight) > self.minimum_item_weight:
            self.change_start_time = time.time()
            self.change_detected = True
            self.start_weight = self.prev_weight
            self.get_logger().info('Weight change detected. Waiting to stabilize...')
            
        if self.change_detected and len(self.readings_buffer) == self.stability_samples:
            max_w = max(self.readings_buffer)
            min_w = min(self.readings_buffer)
            stability = abs(max_w - min_w)
            
            if stability < self.stability_tolerance:
                elapsed = time.time() - self.change_start_time
                self.get_logger().info(f'Weight stabilized in {elapsed}s')
                
                final_weight = sum(self.readings_buffer) / len(self.readings_buffer)
                total_delta = final_weight - self.start_weight

                result = {
                    "action": "added" if total_delta > 0 else "removed",
                    "weight": abs(total_delta),
                }

                self.change_detected = False
                self.readings_buffer.clear()
            
        self.prev_weight = current_weight

        # reset detection
        self.hx.power_down()
        self.hx.power_up()

        return result
