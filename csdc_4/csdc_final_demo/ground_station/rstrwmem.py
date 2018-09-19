##RUN THIS TO RESET THE RWMEM
open("rwmem.txt", 'w').close()
txtmem=open("rwmem.txt","w")
memory=[]
for n in range(0,65536):
    txtmem.write('\n')
txtmem.close()
