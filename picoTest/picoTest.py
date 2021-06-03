from picoTestLib import Pico,findComPort
import numpy as np
import matplotlib.pyplot as plt

"""
Pico test script.

Run this script, will scan for connecte pico ports.
If the pico is connected, 
the script will perform an simple scan.
The result will be ploted with matplotlib.
"""

 
port = findComPort()

if not port:
    print('Pico is not connected!')
    exit(0)



pico = Pico(port=port)
pico.initialize()




print(f"Pico is connected ? {pico.isConnected}" )


sc = """e
var c
var p
var f
var r
set_pgstat_chan 0
set_pgstat_mode 3
set_max_bandwidth 400
set_pot_range -600m 0m
set_autoranging 100u 100u
cell_on
meas_loop_swv p c f r -600m 0m 2m 50m 100
	pck_start
	pck_add p
	pck_add c
	pck_add f    
	pck_add r
	pck_end
endloop
on_finished:
cell_off

"""

res = pico.runScript(sc,parseValue=True)

pico.close()

p = np.array([i[0] for i in res[1][0]])
c = np.array([i[1]*1e6 for i in res[1][0]])
f = np.array([i[2]*1e6 for i in res[1][0]])
r = np.array([i[3]*1e6 for i in res[1][0]])


fig,ax = plt.subplots()
ax.plot(p,r,'.',label='Reverse Current')
ax.plot(p,f,'x',label='Forward Current')
ax.plot(p,c,'-',label='Delta Current')
ax.legend()
ax.set_ylabel('Current / uA')
ax.set_xlabel('Voltage / V')
# plt.show()

plt.savefig('./test.png')

