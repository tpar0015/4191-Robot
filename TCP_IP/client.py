import socket

# Set up a socket using AF_INET interface, streaming protocol (TCP)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

host_address = '192.168.1.5' # Replace this with other group's IP address
port = 12345 # Replace with a suitable port

# Try to connect to server
s.connect((host_address,port))
while(1):
	text = []	# need to change this to the target we aiming and send to other groups
	# if text == 'quit':
	# 	break
	
    # Send text to server
	s.send(bytes(text,'UTF-8'))
	
    # Receive message in response
	data = s.recv(1024)
	
	print ('I received', data)

# Close connection
s.close()
