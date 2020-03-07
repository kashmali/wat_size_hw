
with open('test_cap.txt') as cam_file:
  data = cam_file.read()

data = data.split(',')
data = data[:-1]
print(len(data))
print(data[:10])
data = [int(i,16) for i in data]
print(data[:10])
bdata = bytearray(data)

with open('test_cap.jpg', 'wb') as cam_out:
  cam_out.write(bdata)

print("done")
