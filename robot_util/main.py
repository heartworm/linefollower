import communicator as com
import tkinter as tk
from struct import *

def listener(data):
	print(data)
	print(len(data))
	if len(data) >= 1:
		header = data[0]
		print(header)
		body = data[1:]
		if (header == 0x00):
			unpacked = unpack("<Hhhhhhhh", body)
			print(unpacked)
			strAvg.set(str(unpacked[1]))

def on_closing():
	c.close()
	win.destroy()
	
def refresh():
	c.write(b'\x00')

def sendAverage():
	newAvg = int(strNewAvg.get())
	print("new motor val ", newAvg)
	bytes_out = pack("<Bh", 0x01, newAvg)
	c.write(bytes_out)
	
	
win = tk.Tk()
frame = tk.Frame(win)
frame.pack(expand = True)
	
strLblAvg = tk.StringVar()
strLblAvg.set("Desired Speed")
strAvg = tk.StringVar()
strAvg.set("--")

strNewAvg = tk.StringVar()
strNewAvg.set("100")


c = com.Communicator(38400, "COM3", listener) 
c.open()

tk.Label(frame, textvariable = strLblAvg).grid(column = 1, row = 1, sticky = "W")
tk.Label(frame, textvariable = strAvg).grid(column = 2, row = 1, sticky = "E")

tk.Entry(frame, textvariable = strNewAvg).grid(column = 1, row = 2)
tk.Button(frame, text = "Send", command = refresh).grid(column = 2, row = 2)

tk.Button(frame, text = "Refresh", command = refresh).grid(column = 1, row = 3, columnspan = 2)

win.protocol("WM_DELETE_WINDOW", on_closing)

win.mainloop()
