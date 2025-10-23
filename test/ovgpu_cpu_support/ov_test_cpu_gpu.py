import openvino as ov

def test_cpu_gpu_device_availability():
    """Test to check the availability of CPU and GPU devices in OpenVINO."""
    available_devices = ov.Core().get_available_devices()

    print("Available devices:", available_devices)

if __name__ == "__main__":
    test_cpu_gpu_device_availability()