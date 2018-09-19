import serial
#st = serial.Serial('COM3',115200, timeout=None,parity=serial.PARITY_NONE, rtscts=0)
st = serial.Serial('/dev/ttyACM0',115200, timeout=None,parity=serial.PARITY_NONE, rtscts=0)

message="abcdefghi\n"
# payload=chr(0b01100001)+chr(len(message))+message
payload=chr(0b01100010)+chr(0)+"asdfghjk"
size=len(payload)
print(size)

out=chr(size)+payload
# out=list(out)
# for n in range(1,size+1):
#     out.append(payload[n-1])
print(out)
st.write(out.encode())
# st.write(chr(1).encode('utf-8'))
# size=st.read(1)
# back=st.read(1)
# print(back)
back=st.read(size)
print(back)