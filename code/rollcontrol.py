import socket
import time
import pandas as pd

# IP address and port for the socket server
host = '192.168.89.239'
port = 1234

# Set up the socket server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((host, port))
server_socket.listen(1)
print("Socket server listening on {}:{}".format(host, port))

# Accept incoming connections
client_socket, addr = server_socket.accept()
print("Connected to ESP32:", addr)

def sendReset():
    response="RESET\n"
    client_socket.sendall(response.encode())
    data = client_socket.recv(1024).decode().strip()
    while(data!="done"):
        data = client_socket.recv(1024).decode().strip()
        pass
    print("reset successful")

# def process(data):
#     i=0
#     data_=np.zeros(len(data.split())/2,2)
#     while(i<len(data)):
#         data_[i//2][i%2]=data[i]
#         i+=1
#     return data_
#     pass

df=pd.DataFrame(columns=['time','ref','roll','pitch', 'c1', 'c2', 'c3','c4','angle','height'])
start=time.time()

def rn():
    return time.time()-start
while True:
    # Receive sensor data from the ESP32
    data = client_socket.recv(1024).decode('ascii').strip()
    print(data)
    data_=[]
    if data:
        row=[]
        arr=data.split()
        count = 0
        for i in arr:
            if count%6==0:
                row=[]
                row.append(i)
                count+=1
            else:
                row.append(i)
                data_.append(row)
                count+=1
            # print(data_)
    # print(data_)
    
    for i in data_:
        print(i)
        print(data)
        i=[rn()]+i
        if len(i)>6:
                i=i[:6]
        df_=pd.DataFrame([i],columns=['time','ref','roll','pitch', 'c1', 'c2', 'c3','c4','angle','height'])
        df=pd.concat([df,df_])
    # print(df)
    if rn()>130:
        df.to_csv('rollcontrol_6_20.csv')
        exit()

    # time.sleep(100)
    
    # response = "Response message"
    # client_socket.send(response.encode())
    # sendReset()