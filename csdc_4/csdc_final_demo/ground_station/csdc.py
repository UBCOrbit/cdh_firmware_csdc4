#from serial import serial
import time
import serial
memfile="payload.jpeg"
open(memfile, 'w').close()#update physical memory
st = serial.serial('COM15',115200)
#st = serial.Serial('/dev/ttyACM0',115200, timeout=None,parity=serial.PARITY_NONE, rtscts=0)

# payload=chr(0b01100001)+chr(len(message))+message
################################################################
timep=10
payload=chr(0b01100010)+chr(0)*8+chr(timep)+"000fire0"
size=len(payload)
print(size)

out=chr(size)+payload
print(out)
st.write(out.encode())
# time.sleep(3)

size=st.read(1)
size=ord(size)
if (size==0): size = 256
print(size)
cmd=st.read(size)
print(cmd)
time.sleep(3)

################################################################
payload=chr(0b01100010)+chr(1)+"000fire0"
size=len(payload)
print(size)

out=chr(size)+payload
print(out)
st.write(out.encode())
# time.sleep(3)

st.reset_input_buffer()
size=st.read(1)
size=ord(size)
print(size)
if (size==0): size = 256
cmd=st.read(size)
print(cmd)
time.sleep(3)
################################################################
payload=chr(0b01100010)+chr(2)+"000fire0"+chr(1)
size=len(payload)
print(size)

out=chr(size)+payload
print(out)
st.write(out.encode())
# time.sleep(3)

st.reset_input_buffer()
size=st.read(1)
size=ord(size)
print(size)
if (size==0): size = 256
cmd=st.read(size)
print(cmd)
################################################################
# st.write(chr(1).encode('utf-8'))
# back=st.read(1)
# print(back)
print("commands done\n")
while(1):

    st.reset_input_buffer()
    size=st.read(1)
    size=ord(size)
    print(size)
    if (size==0): size = 256
    cmd=st.read(size)
    print(cmd)

    # for n in range(0,len(cmd)-1):
    #     memory[address+n]=cmd[n]#modify instantiated memory

    # upstr=''.join(memory)
    open(memfile, 'ab').close()#update physical memory
    txtmem=open(memfile,"ab")
    txtmem.write(cmd[2:])
    txtmem.close()


# while 1:
#     print("waiting for command")
#     st.reset_input_buffer()
#     cmd=st.read(1)

#     if(cmd=='w'):
#         print("write received")
#         size=st.read(2)
#         size=ord(size[0])*256+ord(size[1])
#         print(size)
#         cmd=st.read(size)
#         print(cmd)
        
#         txtmem.write(cmd)
#         txtmem.write('\n')

#     elif(cmd=='b'):
#         print("binary received")
#         size=st.read(2)#wait for command
#         size=ord(size[0])*256+ord(size[1])
#         print(size)
#         ser=st.read(size)
#         ser=list(ser)
#         for n in range(0,size):
#             hexout.append('')
#             binout.append('')

#         for n in range(0,size):
#             ser[n]=ord(ser[n])
#             hexout[n]=hex(ser[n])
#             binout[n]='{0:08b}'.format(ser[n])
        
#         hexoutf=' '.join(hexout)
#         binoutf=' '.join(binout)
#         txtmem.write(hexoutf)
#         txtmem.write('\n')
#         txtmem.write(binoutf)
#         txtmem.write('\n')
#         hexoutf=None
#         binoutf=None
#         hexout=[]
#         binout=[]


#     elif(cmd=='d'):
#         print("exiting")
#         txtmem.close()
#         exit()

    # if (cmd[0]=='r'):
    #     print("read received")
    #     cmd=st.read(3)
    #     address=ord(cmd[0])*256+ord(cmd[1])
    #     size=ord(cmd[2])
    #     if (size==0): size = 256

    #     #setup pkt to be sent    
    #     pkt=memory[address]
    #     for n in range(1,size):
    #         pkt=pkt+memory[address+n]
        
    #     #send pkt
    #     st.write(pkt)


    # elif(cmd[0]=='w'):
    #     print("write received")
    #     cmd=st.read(3)
    #     address=ord(cmd[0])*256+ord(cmd[1])
    #     size=ord(cmd[2])
    #     if (size==0): size = 256
    #     cmd=st.read(size)
        
    #     for n in range(0,len(cmd)-1):
    #         memory[address+n]=cmd[n]#modify instantiated memory
        

    #     upstr=''.join(memory)
    #     open(memfile, 'r+').close()#update physical memory
    #     txtmem=open(memfile,"r+")
    #     txtmem.write(upstr)
    #     txtmem.close()

    # elif(cmd[0]=='p'):#use this command to print to console directly, for debud purposes
    #     print("print received")
    #     cmd=st.read(1)
    #     size=ord(cmd[0])
    #     if (size==0): size = 256
    #     cmd=st.read(size)
    #     print(cmd)
        
    # else:
    #     print("invalid command\n")

