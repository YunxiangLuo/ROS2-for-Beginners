from sensor_interfaces.msg import SensorData


def test_sensor_data_fields_and_defaults():
    message = SensorData()
    message.temperature = 25.5
    message.humidity = 60.0
    message.pressure = 1013.25
    message.device_id = 'sensor_01'

    assert message.temperature == 25.5
    assert message.humidity == 60.0
    assert message.pressure == 1013.25
    assert message.device_id == 'sensor_01'
