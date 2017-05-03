import communicator

def l(data):
	print(data)

c = communicator.Communicator(38400, "COM3", l)
