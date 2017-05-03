import communicator as com
import tkinter as tk
from consts import * 
from struct import *

def packet_to_dict(pkt, fmt):
    return {key: pkt[ind] for key, ind in fmt.items()}

def listener(data):
    if len(data) >= 1:
        header = data[0]
        body = data[1:]
        if (header == HDR_GETSTATUS):
            unpacked = packet_to_dict(unpack("<Hhhhhhhh", body), FMT_GETSTATUS)
            for key, val in unpacked.items():
                coreStatusLabels[key].set(val)
            if boolAutoPoll.get():
                refresh()

def on_closing():
    c.close()
    win.destroy()
    
def refresh():
    c.write(b'\x00')

def send_average():
    newAvg = int(strNewAvg.get())
    print("new motor val ", newAvg)
    bytes_out = pack("<Bh", 0x01, newAvg)
    c.write(bytes_out)
    
    
win = tk.Tk()
win.title("Dick Bot Controller")

frame = tk.Frame(win)
frame.pack(fill = tk.BOTH, expand = 1)
    
coreStatusLabels = {key: tk.StringVar() for key in FMT_GETSTATUS.keys()}
for sVar in coreStatusLabels.values():
    sVar.set("---")

    
strNewAvg = tk.StringVar()
strNewAvg.set("100")

boolAutoPoll = tk.BooleanVar()
boolAutoPoll.set(False)
             
c = com.Communicator(57600, "COM3", listener) 
c.open()

tk.Label(frame, text = "Center of Line").grid(column = 0, row = 1, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["centerOfLine"]).grid(column = 1, row = 1, sticky = "WE")
tk.Label(frame, text = "Desired Speed").grid(column = 0, row = 2, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["avgSpeed"]).grid(column = 1, row = 2, sticky = "WE")
tk.Label(frame, text = "Duty Cycle A").grid(column = 0, row = 3, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["valA"]).grid(column = 1, row = 3, sticky = "WE")
tk.Label(frame, text = "Target Speed A").grid(column = 0, row = 4, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["speedA"]).grid(column = 1, row = 4, sticky = "WE")
tk.Label(frame, text = "Real Speed A").grid(column = 0, row = 5, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["actualA"]).grid(column = 1, row = 5, sticky = "WE")
tk.Label(frame, text = "Duty Cycle B").grid(column = 0, row = 6, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["valB"]).grid(column = 1, row = 6, sticky = "WE")
tk.Label(frame, text = "Target Seped B").grid(column = 0, row = 7, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["speedB"]).grid(column = 1, row = 7, sticky = "WE")
tk.Label(frame, text = "Real Speed B").grid(column = 0, row = 8, sticky = "WE")
tk.Label(frame, textvariable = coreStatusLabels["actualB"]).grid(column = 1, row = 8, sticky = "WE")

tk.Entry(frame, textvariable = strNewAvg).grid(column = 0, row = 9, sticky="WE")
tk.Button(frame, text = "Send Average Speed", command = send_average).grid(column = 1, row = 9, sticky="WE")

tk.Checkbutton(frame, text="Auto?", variable = boolAutoPoll).grid(column = 0, row = 10, sticky="WE")
tk.Button(frame, text = "Refresh", command = refresh).grid(column = 1, row = 10, sticky="WE")


for row in range(10):
    tk.Grid.rowconfigure(frame, row, weight=1)

for col in range(2):
    tk.Grid.columnconfigure(frame, col, weight=1)


win.protocol("WM_DELETE_WINDOW", on_closing)

win.mainloop()