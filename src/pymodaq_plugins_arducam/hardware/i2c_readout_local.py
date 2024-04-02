import pigpio

pi = pigpio.pi()
h = pi.i2c_open(10, 0x10, 0)

pi.i2c_write_device(h, [0x01, 0x40, 0x80])

pi.i2c_write_device(h, [0x01, 0x40])
count, data = pi.i2c_read_device(h, 1)

print(int.from_bytes(data, 'big') - 128)

pi.i2c_close(h)